# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

# Phase 2A+: Configure FFmpeg environment variables before any imports
import os
os.environ['AV_LOG_LEVEL'] = 'fatal'  # Suppress FFmpeg log messages
os.environ['FFMPEG_LOG_LEVEL'] = 'fatal'  # Alternative FFmpeg log control
os.environ['AV_CODEC_FLAG_ERROR_CONCEALMENT'] = '1'  # Enable error concealment

import asyncio
import collections
import logging
import os
import time
from typing import Dict, Any

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from go2_interfaces.msg import LowState, VoxelMapCompressed, WebRtcReq
from sensor_msgs.msg import PointCloud2, JointState, Joy, Image, CameraInfo
from nav_msgs.msg import Odometry

from ..domain.entities import RobotConfig, RobotData, CameraData
from ..application.services import RobotDataService, RobotControlService
from ..infrastructure.ros2 import ROS2Publisher
from ..infrastructure.webrtc import WebRTCAdapter

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class VideoErrorTracker:
    """Track H.264 error rates for adaptive behavior"""
    
    def __init__(self, window_size=50):
        self.error_window = collections.deque(maxlen=window_size)
        self.total_errors = 0
        self.last_error_time = 0
        
    def record_error(self):
        """Record a video processing error"""
        current_time = time.time()
        self.error_window.append(1)
        self.total_errors += 1
        self.last_error_time = current_time
        
    def record_success(self):
        """Record successful video processing"""
        self.error_window.append(0)
        
    def get_error_rate(self) -> float:
        """Get current error rate (0.0 to 1.0)"""
        if len(self.error_window) < 10:
            return 0.0
        return sum(self.error_window) / len(self.error_window)
        
    def is_error_storm(self) -> bool:
        """Check if we're in an error storm (>50% error rate)"""
        return self.get_error_rate() > 0.5
        
    def get_stats(self) -> dict:
        """Get error statistics for logging"""
        return {
            'error_rate': self.get_error_rate(),
            'total_errors': self.total_errors,
            'window_size': len(self.error_window)
        }


class Go2DriverNode(Node):
    """Main Go2 driver node - entry point to the application"""

    def __init__(self, event_loop=None):
        super().__init__('go2_driver_node')  # Clean architecture main driver
        self.event_loop = event_loop
        
        # Configuration initialization
        self.config = self._setup_configuration()
        
        # Infrastructure initialization
        self.publishers_dict = self._setup_publishers()
        self.broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))
        self.bridge = CvBridge()
        
        # Architecture layers initialization
        self.ros2_publisher = ROS2Publisher(
            node=self,
            config=self.config,
            publishers=self.publishers_dict,
            broadcaster=self.broadcaster
        )
        
        self.robot_data_service = RobotDataService(self.ros2_publisher)
        
        self.webrtc_adapter = WebRTCAdapter(
            config=self.config,
            on_validated_callback=self._on_robot_validated,
            on_video_frame_callback=self._on_video_frame if self.config.enable_video else None,
            event_loop=self.event_loop
        )
        
        self.robot_control_service = RobotControlService(self.webrtc_adapter)
        
        # Set callback for data
        self.webrtc_adapter.set_data_callback(self._on_robot_data_received)
        
        # Phase 2A: Configure aiortc logging to reduce H.264 error spam
        self._configure_aiortc_logging()
        
        # Phase 2A+: Configure FFmpeg error concealment for robust H.264 decoding
        self._configure_ffmpeg_error_concealment()
        
        # Subscribers initialization
        self._setup_subscribers()
        
        # State
        self.joy_state = Joy()

    def _setup_configuration(self) -> RobotConfig:
        """Configuration setup"""
        robot_ip = os.getenv('ROBOT_IP', os.getenv('GO2_IP', ''))
        token = os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN', ''))
        conn_type = os.getenv('CONN_TYPE', '')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', robot_ip),
                ('token', token),
                ('conn_type', conn_type),
                ('enable_video', True),
                ('decode_lidar', True),
                ('publish_raw_voxel', False),
                ('obstacle_avoidance', False),
            ]
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Get parameter values
        config = RobotConfig.from_params(
            robot_ip=self.get_parameter('robot_ip').get_parameter_value().string_value,
            token=self.get_parameter('token').get_parameter_value().string_value,
            conn_type=self.get_parameter('conn_type').get_parameter_value().string_value,
            enable_video=self.get_parameter('enable_video').get_parameter_value().bool_value,
            decode_lidar=self.get_parameter('decode_lidar').get_parameter_value().bool_value,
            publish_raw_voxel=self.get_parameter('publish_raw_voxel').get_parameter_value().bool_value,
            obstacle_avoidance=self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value
        )

        # Log configuration
        self.get_logger().info(f"Robot IPs: {config.robot_ip_list}")
        self.get_logger().info(f"Connection type: {config.conn_type}")
        self.get_logger().info(f"Connection mode: {config.conn_mode}")
        self.get_logger().info(f"Enable video: {config.enable_video}")
        self.get_logger().info(f"Decode lidar: {config.decode_lidar}")
        self.get_logger().info(f"Publish raw voxel: {config.publish_raw_voxel}")
        self.get_logger().info(f"Obstacle avoidance: {config.obstacle_avoidance}")

        return config

    def _setup_publishers(self) -> Dict[str, list]:
        """ROS2 publishers setup"""
        qos_profile = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Camera QoS optimized for smooth video display
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Buffer frames for guaranteed smooth delivery
        )

        publishers = {
            'joint_state': [],
            'robot_state': [],
            'lidar': [],
            'odometry': [],
            'imu': [],
            'camera': [],
            'camera_info': [],
            'voxel': []
        }

        num_robots = len(self.config.robot_ip_list)
        
        for i in range(num_robots):
            # Define topics depending on connection mode
            if self.config.conn_mode == 'single':
                joint_topic = 'joint_states'
                robot_state_topic = 'go2_states'
                lidar_topic = 'point_cloud2'
                odom_topic = 'odom'
                imu_topic = 'imu'
                camera_topic = 'camera/image_raw'
                camera_info_topic = 'camera/camera_info'
                voxel_topic = '/utlidar/voxel_map_compressed'
            else:
                prefix = f'robot{i}'
                joint_topic = f'{prefix}/joint_states'
                robot_state_topic = f'{prefix}/go2_states'
                lidar_topic = f'{prefix}/point_cloud2'
                odom_topic = f'{prefix}/odom'
                imu_topic = f'{prefix}/imu'
                camera_topic = f'{prefix}/camera/image_raw'
                camera_info_topic = f'{prefix}/camera/camera_info'
                voxel_topic = f'{prefix}/utlidar/voxel_map_compressed'

            # Create publishers
            publishers['joint_state'].append(
                self.create_publisher(JointState, joint_topic, qos_profile))
            publishers['robot_state'].append(
                self.create_publisher(Go2State, robot_state_topic, qos_profile))
            publishers['lidar'].append(
                self.create_publisher(
                    PointCloud2, lidar_topic, best_effort_qos,
                    qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            publishers['odometry'].append(
                self.create_publisher(Odometry, odom_topic, qos_profile))
            publishers['imu'].append(
                self.create_publisher(IMU, imu_topic, qos_profile))

            if self.config.enable_video:
                publishers['camera'].append(
                    self.create_publisher(
                        Image, camera_topic, camera_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                publishers['camera_info'].append(
                    self.create_publisher(
                        CameraInfo, camera_info_topic, camera_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))

            if self.config.publish_raw_voxel:
                publishers['voxel'].append(
                    self.create_publisher(VoxelMapCompressed, voxel_topic, best_effort_qos))

        return publishers

    def _setup_subscribers(self) -> None:
        """ROS2 subscribers setup"""
        qos_profile = QoSProfile(depth=10)

        # Command subscribers
        num_robots = len(self.config.robot_ip_list)
        
        if self.config.conn_mode == 'single':
            self.create_subscription(
                Twist, 'cmd_vel_out',
                lambda msg: self._on_cmd_vel(msg, "0"), qos_profile)
            self.create_subscription(
                WebRtcReq, 'webrtc_req',
                lambda msg: self._on_webrtc_req(msg, "0"), qos_profile)
        else:
            for i in range(num_robots):
                self.create_subscription(
                    Twist, f'robot{i}/cmd_vel_out',
                    lambda msg, robot_id=str(i): self._on_cmd_vel(msg, robot_id), qos_profile)
                self.create_subscription(
                    WebRtcReq, f'robot{i}/webrtc_req',
                    lambda msg, robot_id=str(i): self._on_webrtc_req(msg, robot_id), qos_profile)

        # Joystick subscriber
        self.create_subscription(Joy, 'joy', self._on_joy, qos_profile)

        # CycloneDDS support
        if self.config.conn_type == 'cyclonedds':
            self.create_subscription(
                LowState, 'lowstate',
                self._on_cyclonedds_low_state, qos_profile)
            self.create_subscription(
                PoseStamped, '/utlidar/robot_pose',
                self._on_cyclonedds_pose, qos_profile)
            self.create_subscription(
                PointCloud2, '/utlidar/cloud',
                self._on_cyclonedds_lidar, qos_profile)

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Callback for parameter changes"""
        result = SetParametersResult(successful=True)

        try:
            for p in params:
                if p.name == 'obstacle_avoidance':
                    self.get_logger().info(f'New obstacle_avoidance value: {p.value}')
                    self.config.obstacle_avoidance = p.value
                    
                    try:
                        self.robot_control_service.set_obstacle_avoidance(p.value, "0")
                    except Exception as e:
                        self.get_logger().error(f"Failed to set obstacle avoidance: {e}")
                        result.successful = False
                        result.reason = str(e)
                        break
                    
                    result.successful = True
                    result.reason = 'Updated obstacle_avoidance'
                    break
        except Exception as e:
            self.get_logger().error(f"Error setting parameters: {e}")
            result.successful = False
            result.reason = str(e)
            
        return result

    def _on_cmd_vel(self, msg: Twist, robot_id: str) -> None:
        """Callback for movement commands"""
        self.robot_control_service.handle_cmd_vel(
            msg.linear.x, msg.linear.y, msg.angular.z, 
            robot_id, self.config.obstacle_avoidance
        )

    def _on_webrtc_req(self, msg: WebRtcReq, robot_id: str) -> None:
        """Callback for WebRTC requests"""
        self.robot_control_service.handle_webrtc_request(
            msg.api_id, msg.parameter, msg.topic, msg.id, robot_id
        )

    def _on_joy(self, msg: Joy) -> None:
        """Callback for joystick"""
        self.joy_state = msg

    def _on_robot_validated(self, robot_id: str) -> None:
        """Callback after robot validation"""
        self.get_logger().info(f"Robot {robot_id} validated and ready")

    def _on_robot_data_received(self, msg: Dict[str, Any], robot_id: str) -> None:
        """Callback for receiving data from robot"""
        self.robot_data_service.process_webrtc_message(msg, robot_id)

    def _configure_aiortc_logging(self) -> None:
        """Configure aiortc logging to reduce H.264 error spam"""
        logging.getLogger('aiortc.codecs.h264').setLevel(logging.ERROR)
        logging.getLogger('aiortc.codecs').setLevel(logging.ERROR)
        logging.getLogger('aiortc').setLevel(logging.WARNING)
        logger.info("Configured aiortc logging to reduce H.264 error spam")

    def _configure_ffmpeg_error_concealment(self) -> None:
        """Configure FFmpeg error concealment and log suppression for robust H.264 decoding"""
        try:
            import av
            import av.logging
            
            # Primary approach: PyAV log level control (most reliable)
            av.logging.set_level(av.logging.QUIET)  # Complete suppression of FFmpeg messages
            logger.info("Configured PyAV to completely suppress FFmpeg H.264 decoder messages")
            
            # Secondary approach: Try to configure codec options (may not work in all versions)
            try:
                # Attempt to set global decoder options for error concealment
                h264_codec = av.codec.Codec('h264', 'r')
                if hasattr(h264_codec, 'options'):
                    h264_codec.options.update({
                        'error_concealment': '3',  # All error concealment methods
                        'ec': '3',  # Alternative parameter name
                    })
                    logger.info("Configured H.264 decoder error concealment options")
            except Exception as codec_error:
                logger.debug(f"Could not configure codec options (this is normal): {codec_error}")
                
        except ImportError:
            logger.warning("PyAV not available - cannot suppress FFmpeg logging")
        except Exception as e:
            logger.warning(f"Could not configure PyAV logging: {e}")
            
        # Environment variables were set at module import time
        logger.info("FFmpeg environment variables configured at module import (AV_LOG_LEVEL=fatal)")

    async def _on_video_frame(self, track: MediaStreamTrack, robot_id: str) -> None:
        """Callback for processing video frames"""
        # Phase 1 optimizations: Frame rate limiting, resolution downscaling, 
        # subscription checking, and enhanced error handling
        max_fps = 15  # Limit to 15 FPS for CPU efficiency
        frame_interval = 1.0 / max_fps
        last_frame_time = 0
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        # Phase 2A: Error tracking for adaptive behavior and diagnostics
        error_tracker = VideoErrorTracker()
        last_stats_log = 0

        while True:
            try:
                # Phase 2A: Add timeout to prevent indefinite blocking during H.264 decoder recovery
                frame = await asyncio.wait_for(track.recv(), timeout=0.1)
                
                # Phase 1: Frame rate limiting - skip frames if processing too fast
                current_time = time.time()
                if current_time - last_frame_time < frame_interval:
                    continue  # Skip frame to maintain target FPS
                
                # Phase 1: Enhanced frame skipping - more aggressive under load
                if consecutive_errors > 3:  # High stress - skip 75% of frames
                    if int(current_time * 10) % 4 != 0:
                        continue
                elif consecutive_errors > 1:  # Moderate stress - skip 50% of frames  
                    if int(current_time * 10) % 2 == 0:
                        continue
                
                last_frame_time = current_time
                
                # Phase 1: Subscription checking - skip processing if no subscribers
                if not self.ros2_publisher.has_camera_subscribers(robot_id):
                    continue  # Skip frame processing when no one is listening
                
                # Phase 1: Resolution downscaling - 85% data reduction (1920x1080 -> 640x480)
                img = frame.to_ndarray(format="rgb24", width=640, height=480)

                # Create camera data
                camera_data = CameraData(
                    image=img,
                    height=img.shape[0],
                    width=img.shape[1],
                    encoding="rgb8"
                )

                robot_data = RobotData(
                    robot_id=robot_id,
                    timestamp=0.0,
                    camera_data=camera_data
                )

                # Publish via ROS2Publisher
                self.ros2_publisher.publish_camera_data(robot_data)
                
                # Reset error counter on successful processing
                consecutive_errors = 0
                
                # Phase 2A: Record successful frame processing
                error_tracker.record_success()
                
                # Phase 2A: Periodic error statistics logging (every 30 seconds)
                current_time = time.time()
                if current_time - last_stats_log > 30:
                    stats = error_tracker.get_stats()
                    if stats['total_errors'] > 0:
                        logger.info(f"Video error stats: {stats['error_rate']:.1%} error rate, "
                                   f"{stats['total_errors']} total errors")
                    last_stats_log = current_time
                
                await asyncio.sleep(0)

            except asyncio.TimeoutError:
                # Phase 2A: Handle frame receive timeout - skip frame without counting as error
                continue
                
            except Exception as e:
                # Phase 1: Enhanced error handling for video failures
                consecutive_errors += 1
                
                # Phase 2A: Record error for tracking and statistics
                error_tracker.record_error()
                
                logger.error(f"Error processing video frame (attempt {consecutive_errors}): {e}")
                
                # Phase 1: Graceful degradation - break after too many consecutive errors
                if consecutive_errors >= max_consecutive_errors:
                    logger.error(f"Too many consecutive video errors ({consecutive_errors}), stopping video processing for robot {robot_id}")
                    break
                
                # Brief pause before retry to avoid tight error loop
                await asyncio.sleep(0.1)

    # CycloneDDS callbacks
    def _on_cyclonedds_low_state(self, msg: LowState) -> None:
        """Processing LowState for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_pose(self, msg: PoseStamped) -> None:
        """Processing pose for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_lidar(self, msg: PointCloud2) -> None:
        """Processing lidar for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    async def connect_robots(self) -> None:
        """Connect to robots"""
        if self.config.conn_type == 'webrtc':
            for i, robot_ip in enumerate(self.config.robot_ip_list):
                try:
                    await self.webrtc_adapter.connect(str(i))
                except Exception as e:
                    self.get_logger().error(f"Failed to connect to robot {i}: {e}")
                    raise

    async def run_robot_control_loop(self, robot_id: str) -> None:
        """Main robot control loop"""
        while True:
            try:
                # Process joystick commands
                if self.joy_state.buttons:
                    self.robot_control_service.handle_joy_command(
                        self.joy_state.buttons, robot_id
                    )

                # Process WebRTC commands
                self.webrtc_adapter.process_webrtc_commands(robot_id)
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error in control loop for robot {robot_id}: {e}")
                raise 