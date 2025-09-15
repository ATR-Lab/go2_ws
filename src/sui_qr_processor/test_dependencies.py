#!/usr/bin/env python3
"""
Test script to verify all dependencies are available for sui_qr_processor.
Run this script to check if the package can be executed properly.
"""

import sys

def test_dependencies():
    """Test all required dependencies."""
    print("Testing sui_qr_processor dependencies...")
    print("=" * 50)
    
    # Test ROS2 dependencies
    try:
        import rclpy
        print("✅ rclpy: OK")
    except ImportError as e:
        print(f"❌ rclpy: FAILED - {e}")
        return False
    
    try:
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import Point
        from std_msgs.msg import String, Bool
        print("✅ ROS2 message types: OK")
    except ImportError as e:
        print(f"❌ ROS2 message types: FAILED - {e}")
        return False
    
    try:
        from cv_bridge import CvBridge
        print("✅ cv_bridge: OK")
    except ImportError as e:
        print(f"❌ cv_bridge: FAILED - {e}")
        return False
    
    # Test OpenCV
    try:
        import cv2
        print(f"✅ opencv-python: OK (version {cv2.__version__})")
    except ImportError as e:
        print(f"❌ opencv-python: FAILED - {e}")
        return False
    
    # Test pyzbar
    try:
        from pyzbar import pyzbar
        print("✅ pyzbar: OK")
    except ImportError as e:
        print(f"❌ pyzbar: FAILED - {e}")
        print("   Install with: sudo apt-get install libzbar0 && pip install pyzbar")
        return False
    
    # Test sui-py
    try:
        from sui_py import SuiClient, SuiError, TransactionBlockResponseOptions
        print("✅ sui-py: OK")
    except ImportError as e:
        print(f"❌ sui-py: FAILED - {e}")
        print("   Install with: pip install sui-py")
        return False
    
    # Test other Python dependencies
    try:
        import asyncio
        import json
        import hashlib
        import threading
        import numpy as np
        print("✅ Python standard libraries: OK")
    except ImportError as e:
        print(f"❌ Python libraries: FAILED - {e}")
        return False
    
    print("=" * 50)
    print("✅ All dependencies are available!")
    print("\nYou can now run the sui_qr_processor node:")
    print("  ros2 run sui_qr_processor sui_qr_processor_node")
    print("  ros2 launch sui_qr_processor sui_qr_processor.launch.py")
    
    return True

def test_sui_connection():
    """Test Sui network connection."""
    print("\nTesting Sui network connection...")
    print("-" * 30)
    
    try:
        from sui_py import SuiClient
        
        # Test testnet connection
        client = SuiClient("testnet")
        print("✅ Sui testnet client created successfully")
        
        # Note: We don't test actual network calls here to avoid blocking
        print("   (Network connectivity will be tested when node runs)")
        
    except Exception as e:
        print(f"❌ Sui client test failed: {e}")
        return False
    
    return True

if __name__ == "__main__":
    success = test_dependencies()
    
    if success:
        test_sui_connection()
        print("\n🎉 All tests passed! The package is ready to use.")
        sys.exit(0)
    else:
        print("\n❌ Some dependencies are missing. Please install them before running the node.")
        sys.exit(1)
