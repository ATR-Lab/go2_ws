#!/usr/bin/env python3
"""
Sui QR Processor Node

ROS2 node that processes QR codes from camera feed and executes Sui blockchain transactions.
Subscribes to camera image topic, detects QR codes, parses transaction data, and executes
transactions on the Sui network with duplicate prevention.
"""

import asyncio
import json
import hashlib
import time
from typing import Optional, Dict, Any, Set
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_srvs.srv import SetBool
from cv_bridge import CvBridge

import cv2
import numpy as np

try:
    from pyzbar import pyzbar
    QR_DETECTION_AVAILABLE = True
except ImportError:
    QR_DETECTION_AVAILABLE = False

try:
    from sui_py import SuiClient, SuiError, TransactionBlockResponseOptions
    SUI_PY_AVAILABLE = True
except ImportError:
    SUI_PY_AVAILABLE = False


class TransactionCache:
    """Manages duplicate prevention for QR code transactions."""
    
    def __init__(self, max_size: int = 1000, cooldown_seconds: float = 30.0):
        self.max_size = max_size
        self.cooldown_seconds = cooldown_seconds
        
        # Tracking sets and dictionaries
        self.processed_hashes: Set[str] = set()
        self.processing_queue: Set[str] = set()
        self.successful_digests: Set[str] = set()
        self.last_execution_time: float = 0.0
        
        # Cleanup tracking
        self.hash_timestamps: Dict[str, float] = {}
        self.last_cleanup: float = time.time()
        self.cleanup_interval: float = 300.0  # 5 minutes
    
    def get_qr_hash(self, qr_data: str) -> str:
        """Generate SHA-256 hash of QR data for duplicate detection."""
        return hashlib.sha256(qr_data.encode('utf-8')).hexdigest()
    
    def is_duplicate(self, qr_data: str) -> bool:
        """Check if QR data has already been processed."""
        qr_hash = self.get_qr_hash(qr_data)
        return qr_hash in self.processed_hashes
    
    def is_processing(self, qr_data: str) -> bool:
        """Check if QR data is currently being processed."""
        qr_hash = self.get_qr_hash(qr_data)
        return qr_hash in self.processing_queue
    
    def is_in_cooldown(self) -> bool:
        """Check if we're in cooldown period after last execution."""
        return (time.time() - self.last_execution_time) < self.cooldown_seconds
    
    def start_processing(self, qr_data: str) -> bool:
        """Mark QR data as being processed. Returns False if already processing."""
        qr_hash = self.get_qr_hash(qr_data)
        
        if qr_hash in self.processing_queue:
            return False
        
        self.processing_queue.add(qr_hash)
        return True
    
    def finish_processing(self, qr_data: str, success: bool = False, digest: Optional[str] = None):
        """Mark QR data processing as complete."""
        qr_hash = self.get_qr_hash(qr_data)
        
        # Remove from processing queue
        self.processing_queue.discard(qr_hash)
        
        if success:
            # Mark as processed
            self.processed_hashes.add(qr_hash)
            self.hash_timestamps[qr_hash] = time.time()
            self.last_execution_time = time.time()
            
            # Store successful digest
            if digest:
                self.successful_digests.add(digest)
        
        # Cleanup if needed
        self._cleanup_if_needed()
    
    def _cleanup_if_needed(self):
        """Clean up old entries to prevent memory growth."""
        current_time = time.time()
        
        if (current_time - self.last_cleanup) < self.cleanup_interval:
            return
        
        # Remove old hashes (older than 1 hour)
        cutoff_time = current_time - 3600
        old_hashes = [h for h, t in self.hash_timestamps.items() if t < cutoff_time]
        
        for old_hash in old_hashes:
            self.processed_hashes.discard(old_hash)
            del self.hash_timestamps[old_hash]
        
        # Limit total size
        if len(self.processed_hashes) > self.max_size:
            # Remove oldest entries
            sorted_hashes = sorted(self.hash_timestamps.items(), key=lambda x: x[1])
            to_remove = len(self.processed_hashes) - self.max_size
            
            for hash_to_remove, _ in sorted_hashes[:to_remove]:
                self.processed_hashes.discard(hash_to_remove)
                del self.hash_timestamps[hash_to_remove]
        
        self.last_cleanup = current_time
    
    def reset(self):
        """Reset all caches (for testing or manual reset)."""
        self.processed_hashes.clear()
        self.processing_queue.clear()
        self.successful_digests.clear()
        self.hash_timestamps.clear()
        self.last_execution_time = 0.0


class SuiQrProcessorNode(Node):
    """
    ROS2 node for processing QR codes and executing Sui transactions.
    
    Subscribes to camera image topic, detects QR codes containing Sui transaction
    data, and executes transactions on the configured Sui network with comprehensive
    duplicate prevention.
    """
    
    def __init__(self):
        super().__init__('sui_qr_processor_node')
        
        # Check dependencies
        if not QR_DETECTION_AVAILABLE:
            self.get_logger().error('pyzbar not available - QR detection disabled')
            return
        
        if not SUI_PY_AVAILABLE:
            self.get_logger().error('sui-py not available - Sui integration disabled')
            return
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize components
        self.bridge = CvBridge()
        self.transaction_cache = TransactionCache(
            max_size=self.get_parameter('cache_max_size').value,
            cooldown_seconds=self.get_parameter('execution_cooldown').value
        )
        
        # QR detection state
        self.last_qr_detection_time = 0.0
        self.qr_detection_interval = self.get_parameter('qr_detection_interval').value
        
        # Sui client (will be initialized async)
        self.sui_client: Optional[SuiClient] = None
        self.client_lock = threading.Lock()
        
        # Set up QoS profile for camera stream
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscription
        camera_topic = self.get_parameter('camera_topic').value
        self.camera_sub = self.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            self.qos_profile
        )
        
        # Create publishers
        self.status_pub = self.create_publisher(String, '~/transaction_status', 10)
        self.qr_detection_pub = self.create_publisher(String, '~/qr_detection_status', 10)
        
        # Create services
        self.reset_cache_srv = self.create_service(
            SetBool, '~/reset_cache', self._reset_cache_callback
        )
        
        # Initialize Sui client asynchronously
        self.create_timer(0.1, self._initialize_sui_client_once)
        
        self.get_logger().info(f'Sui QR Processor initialized')
        self.get_logger().info(f'Subscribing to camera topic: {camera_topic}')
        self.get_logger().info(f'Network: {self.get_parameter("default_network").value}')
    
    def _declare_parameters(self):
        """Declare ROS2 parameters with default values."""
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('default_network', 'testnet')
        self.declare_parameter('allowed_networks', ['testnet', 'mainnet', 'devnet'])
        self.declare_parameter('network_source', 'qr_with_fallback')  # qr_with_fallback, config_only, qr_only
        self.declare_parameter('qr_detection_interval', 0.2)
        self.declare_parameter('execution_cooldown', 30.0)
        self.declare_parameter('cache_max_size', 1000)
        self.declare_parameter('enable_visualization', True)
    
    def _initialize_sui_client_once(self):
        """Initialize Sui client once (called by timer)."""
        if self.sui_client is not None:
            return
        
        try:
            network = self.get_parameter('default_network').value
            self.sui_client = SuiClient(network)
            self.get_logger().info(f'Sui client initialized for {network}')
            
            # Destroy the timer since we only need to run this once
            for timer in self._timers:
                if timer.callback == self._initialize_sui_client_once:
                    timer.destroy()
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Sui client: {e}')
    
    def _camera_callback(self, msg: Image):
        """Process incoming camera images for QR detection."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process frame for QR detection
            self._process_frame_for_qr(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera frame: {e}')
    
    def _process_frame_for_qr(self, frame: np.ndarray):
        """Process frame for QR code detection and transaction execution."""
        current_time = time.time()
        
        # Only process QR detection at intervals to save CPU
        if current_time - self.last_qr_detection_time < self.qr_detection_interval:
            return
        
        self.last_qr_detection_time = current_time
        
        # Convert to grayscale for QR detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect QR codes
        qr_codes = pyzbar.decode(gray)
        
        for qr_code in qr_codes:
            try:
                # Extract QR code data
                qr_data = qr_code.data.decode('utf-8')
                
                # Get bounding box for visualization
                points = qr_code.polygon
                if len(points) == 4:
                    center_x = int(np.mean([p.x for p in points]))
                    center_y = int(np.mean([p.y for p in points]))
                    
                    self.get_logger().info(f'QR Code detected at ({center_x}, {center_y})')
                    
                    # Publish QR detection event
                    detection_msg = String()
                    detection_msg.data = f'QR detected at ({center_x}, {center_y}): {qr_data[:50]}...'
                    self.qr_detection_pub.publish(detection_msg)
                    
                    # Process the QR code data
                    self._process_qr_data(qr_data)
                
            except Exception as e:
                self.get_logger().error(f'Error processing QR code: {e}')
    
    def _process_qr_data(self, qr_data: str):
        """Process QR code data and execute Sui transaction if valid."""
        try:
            # Check for duplicates and processing state
            if self.transaction_cache.is_duplicate(qr_data):
                self.get_logger().info('QR code already processed - skipping')
                return
            
            if self.transaction_cache.is_processing(qr_data):
                self.get_logger().info('QR code currently being processed - skipping')
                return
            
            if self.transaction_cache.is_in_cooldown():
                remaining = self.transaction_cache.cooldown_seconds - (time.time() - self.transaction_cache.last_execution_time)
                self.get_logger().info(f'In cooldown period - {remaining:.1f}s remaining')
                return
            
            # Start processing
            if not self.transaction_cache.start_processing(qr_data):
                self.get_logger().warn('Failed to start processing - already in progress')
                return
            
            # Parse QR data
            try:
                qr_json = json.loads(qr_data)
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Invalid QR JSON format: {e}')
                self.transaction_cache.finish_processing(qr_data, success=False)
                return
            
            # Validate QR data structure
            if not self._validate_qr_data(qr_json):
                self.transaction_cache.finish_processing(qr_data, success=False)
                return
            
            # Determine network
            network = self._determine_network(qr_json.get('chain', ''))
            if not network:
                self.transaction_cache.finish_processing(qr_data, success=False)
                return
            
            # Execute transaction asynchronously
            self._execute_transaction_async(qr_json, network, qr_data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing QR data: {e}')
            self.transaction_cache.finish_processing(qr_data, success=False)
    
    def _validate_qr_data(self, qr_json: Dict[str, Any]) -> bool:
        """Validate QR code JSON structure."""
        required_fields = ['bytes', 'signature']
        
        for field in required_fields:
            if field not in qr_json:
                self.get_logger().error(f'Missing required field in QR data: {field}')
                return False
            
            if not isinstance(qr_json[field], str) or not qr_json[field].strip():
                self.get_logger().error(f'Invalid {field} in QR data')
                return False
        
        return True
    
    def _determine_network(self, chain: str) -> Optional[str]:
        """Determine which Sui network to use based on configuration and QR data."""
        network_source = self.get_parameter('network_source').value
        default_network = self.get_parameter('default_network').value
        allowed_networks = self.get_parameter('allowed_networks').value
        
        # Extract network from chain field (e.g., "sui:testnet" -> "testnet")
        qr_network = None
        if chain and ':' in chain:
            parts = chain.split(':')
            if len(parts) > 1:
                qr_network = parts[1]
        
        if network_source == 'config_only':
            return default_network
        
        elif network_source == 'qr_only':
            if qr_network and qr_network in allowed_networks:
                return qr_network
            else:
                self.get_logger().error(f'QR network {qr_network} not in allowed list: {allowed_networks}')
                return None
        
        else:  # qr_with_fallback
            if qr_network and qr_network in allowed_networks:
                return qr_network
            return default_network
    
    def _execute_transaction_async(self, qr_json: Dict[str, Any], network: str, qr_data: str):
        """Execute Sui transaction asynchronously."""
        def run_async():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._execute_transaction(qr_json, network, qr_data))
            finally:
                loop.close()
        
        # Run in separate thread to avoid blocking ROS callbacks
        thread = threading.Thread(target=run_async)
        thread.daemon = True
        thread.start()
    
    async def _execute_transaction(self, qr_json: Dict[str, Any], network: str, qr_data: str):
        """Execute Sui transaction."""
        try:
            # Initialize client for specific network if needed
            with self.client_lock:
                if self.sui_client is None:
                    self.sui_client = SuiClient(network)
                
                # Check if we need to switch networks
                current_network = getattr(self.sui_client, '_network', None)
                if current_network != network:
                    self.sui_client = SuiClient(network)
            
            self.get_logger().info(f'Executing transaction on {network}')
            
            # Prepare transaction data
            tx_bytes = qr_json['bytes']
            signature = qr_json['signature']
            
            # Configure response options
            options = TransactionBlockResponseOptions(
                show_effects=True,
                show_events=True,
                show_object_changes=True,
                show_balance_changes=True
            )
            
            # Execute transaction
            response = await self.sui_client.write_api.execute_transaction_block(
                transaction_block=tx_bytes,
                signature=signature,
                options=options
            )
            
            # Handle successful execution
            digest = response.digest
            self.get_logger().info(f'Transaction executed successfully: {digest}')
            
            # Publish status
            status_msg = String()
            status_msg.data = f'SUCCESS: {digest}'
            self.status_pub.publish(status_msg)
            
            # Mark as successfully processed
            self.transaction_cache.finish_processing(qr_data, success=True, digest=digest)
            
        except SuiError as e:
            self.get_logger().error(f'Sui transaction failed: {e}')
            status_msg = String()
            status_msg.data = f'FAILED: {str(e)}'
            self.status_pub.publish(status_msg)
            self.transaction_cache.finish_processing(qr_data, success=False)
            
        except Exception as e:
            self.get_logger().error(f'Unexpected error executing transaction: {e}')
            status_msg = String()
            status_msg.data = f'ERROR: {str(e)}'
            self.status_pub.publish(status_msg)
            self.transaction_cache.finish_processing(qr_data, success=False)
    
    def _reset_cache_callback(self, request, response):
        """Service callback to reset transaction cache."""
        self.transaction_cache.reset()
        self.get_logger().info('Transaction cache reset')
        response.data = True
        return response


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    try:
        node = SuiQrProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
