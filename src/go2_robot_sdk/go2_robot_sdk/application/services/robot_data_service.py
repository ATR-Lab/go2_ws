# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import logging
import math
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Any

from ...domain.entities import RobotData, RobotState, IMUData, OdometryData, JointData, LidarData
from ...domain.interfaces import IRobotDataPublisher
from ...domain.constants import RTC_TOPIC

logger = logging.getLogger(__name__)


class RobotDataService:
    """Service for processing and validating robot data"""

    def __init__(self, publisher: IRobotDataPublisher):
        self.publisher = publisher
        # Thread pool for async LiDAR processing only
        self.lidar_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="lidar")
        # Shared LiDAR decoder instance - initialize once, reuse across threads
        self.lidar_decoder = None
        self._init_lidar_decoder()

    def process_webrtc_message(self, msg: Dict[str, Any], robot_id: str) -> None:
        """Process WebRTC message with async LiDAR processing"""
        try:
            topic = msg.get('topic')

            if topic == RTC_TOPIC["ULIDAR_ARRAY"]:
                # Process LiDAR asynchronously - don't block other messages
                asyncio.create_task(self._process_lidar_async(msg, robot_id))

            else:
                # Process all other messages immediately (they're fast)
                robot_data = RobotData(robot_id=robot_id, timestamp=0.0)

                if topic == RTC_TOPIC["ROBOTODOM"]:
                    self._process_odometry_data(msg, robot_data)
                    self.publisher.publish_odometry(robot_data)

                elif topic == RTC_TOPIC["LF_SPORT_MOD_STATE"]:
                    self._process_sport_mode_state(msg, robot_data)
                    self.publisher.publish_robot_state(robot_data)

                elif topic == RTC_TOPIC["LOW_STATE"]:
                    self._process_low_state(msg, robot_data)
                    self.publisher.publish_joint_state(robot_data)

        except Exception as e:
            logger.error(f"Error processing WebRTC message: {e}")

    def _init_lidar_decoder(self) -> None:
        """Initialize shared LiDAR decoder instance"""
        try:
            from ...infrastructure.sensors.lidar_decoder import LidarDecoder
            self.lidar_decoder = LidarDecoder()
            logger.info("Initialized shared LiDAR decoder instance")
        except Exception as e:
            logger.error(f"Failed to initialize LiDAR decoder: {e}")
            self.lidar_decoder = None

    async def _process_lidar_async(self, msg: Dict[str, Any], robot_id: str) -> None:
        """Process LiDAR data asynchronously to prevent blocking other messages"""
        try:
            if not self.lidar_decoder:
                logger.warning("LiDAR decoder not available, skipping LiDAR processing")
                return
                
            # Move WASM decoding to thread pool using shared decoder
            compressed_data = msg.get("compressed_data")
            data = msg.get("data", {})
            
            if compressed_data and data:
                loop = asyncio.get_event_loop()
                
                # Use shared decoder instance in thread pool
                decoded_result = await loop.run_in_executor(
                    self.lidar_executor,
                    self.lidar_decoder.decode,
                    compressed_data,
                    data
                )
                
                # Add decoded data to message for processing
                msg_with_decoded = msg.copy()
                msg_with_decoded["decoded_data"] = decoded_result
                
                # Process and publish (fast operations)
                robot_data = RobotData(robot_id=robot_id, timestamp=0.0)
                self._process_lidar_data(msg_with_decoded, robot_data)
                
                if robot_data.lidar_data:
                    self.publisher.publish_lidar_data(robot_data)
                    self.publisher.publish_voxel_data(robot_data)
            else:
                logger.warning("No compressed LiDAR data to process")
                
        except Exception as e:
            logger.error(f"Error in async LiDAR processing: {e}")

    def _process_lidar_data(self, msg: Dict[str, Any], robot_data: RobotData) -> None:
        """Process lidar data"""
        try:
            decoded_data = msg.get("decoded_data", {})
            data = msg.get("data", {})
            
            robot_data.lidar_data = LidarData(
                positions=decoded_data.get("positions"),
                uvs=decoded_data.get("uvs"),
                resolution=data.get("resolution", 0.0),
                origin=data.get("origin", [0.0, 0.0, 0.0]),
                stamp=data.get("stamp", 0.0),
                width=data.get("width"),
                src_size=data.get("src_size"),
                compressed_data=msg.get("compressed_data")
            )
        except Exception as e:
            logger.error(f"Error processing lidar data: {e}")

    def _process_odometry_data(self, msg: Dict[str, Any], robot_data: RobotData) -> None:
        """Process odometry data"""
        try:
            pose_data = msg['data']['pose']
            position = pose_data['position']
            orientation = pose_data['orientation']

            # Data validation - Phase 1 optimization: type-check first for faster rejection
            pos_vals = [position['x'], position['y'], position['z']]
            rot_vals = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
            all_vals = pos_vals + rot_vals

            # Fast path: check types first (cheaper operation)
            if not all(isinstance(v, (int, float)) for v in all_vals):
                logger.warning("Invalid odometry data types - skipping")
                return
            
            # Only check finite values if types are correct (more expensive operation)
            if not all(math.isfinite(v) for v in all_vals):
                logger.warning("Invalid odometry data values - skipping")
                return

            robot_data.odometry_data = OdometryData(
                position=position,
                orientation=orientation
            )
        except Exception as e:
            logger.error(f"Error processing odometry data: {e}")

    def _process_sport_mode_state(self, msg: Dict[str, Any], robot_data: RobotData) -> None:
        """Process sport mode state"""
        try:
            data = msg["data"]

            # Data validation - Phase 1 optimization: consolidated validation to reduce function call overhead
            validation_fields = [
                ("position", data.get("position", [])),
                ("range_obstacle", data.get("range_obstacle", [])),
                ("foot_position_body", data.get("foot_position_body", [])),
                ("foot_speed_body", data.get("foot_speed_body", []))
            ]
            
            # Validate all float lists in one pass
            for field_name, field_data in validation_fields:
                if not self._validate_float_list(field_data):
                    logger.warning(f"Invalid sport mode state data in field '{field_name}' - skipping")
                    return
            
            # Validate single float value
            if not self._validate_float(data.get("body_height")):
                logger.warning("Invalid sport mode state body_height - skipping")
                return

            robot_data.robot_state = RobotState(
                mode=data["mode"],
                progress=data["progress"],
                gait_type=data["gait_type"],
                position=data["position"],
                body_height=data["body_height"],
                velocity=data["velocity"],
                range_obstacle=data["range_obstacle"],
                foot_force=data["foot_force"],
                foot_position_body=data["foot_position_body"],
                foot_speed_body=data["foot_speed_body"]
            )

            # Process IMU data
            imu_data = data["imu_state"]
            if (self._validate_float_list(imu_data.get("quaternion", [])) and
                self._validate_float_list(imu_data.get("accelerometer", [])) and
                self._validate_float_list(imu_data.get("gyroscope", [])) and
                self._validate_float_list(imu_data.get("rpy", []))):
                
                robot_data.imu_data = IMUData(
                    quaternion=imu_data["quaternion"],
                    accelerometer=imu_data["accelerometer"],
                    gyroscope=imu_data["gyroscope"],
                    rpy=imu_data["rpy"],
                    temperature=imu_data["temperature"]
                )

        except Exception as e:
            logger.error(f"Error processing sport mode state: {e}")

    def _process_low_state(self, msg: Dict[str, Any], robot_data: RobotData) -> None:
        """Process low state data"""
        try:
            low_state_data = msg['data']
            robot_data.joint_data = JointData(
                motor_state=low_state_data['motor_state']
            )
        except Exception as e:
            logger.error(f"Error processing low state: {e}")

    def _validate_float_list(self, data: list) -> bool:
        """Validate a list of float values - Phase 1 optimization: type-check first"""
        # Fast path: check types first (cheaper operation)
        if not all(isinstance(x, (int, float)) for x in data):
            return False
        # Only check finite values if types are correct (more expensive operation)
        return all(math.isfinite(x) for x in data)

    def _validate_float(self, value: Any) -> bool:
        """Validate a float value - Phase 1 optimization: type-check first"""
        # Fast path: check type first (cheaper operation)
        if not isinstance(value, (int, float)):
            return False
        # Only check finite if type is correct (more expensive operation)
        return math.isfinite(value) 