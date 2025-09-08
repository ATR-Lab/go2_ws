# SPDX-License-Identifier: Apache-2.0

"""
Centralized configuration loader for dance commands.
"""

import logging
import os
from typing import Dict, Optional
from dataclasses import dataclass

import yaml
from ament_index_python.packages import get_package_share_directory


logger = logging.getLogger(__name__)


@dataclass
class CommandConfig:
    """Configuration for a single dance command"""
    command_name: str
    command_id: int
    expected_duration: float
    description: str
    category: str
    completion_detection: Optional[Dict] = None


class DanceCommandConfig:
    """Centralized configuration for dance commands loaded from YAML"""
    
    def __init__(self, config_file_path: Optional[str] = None):
        """Initialize configuration loader"""
        self._commands: Dict[str, CommandConfig] = {}
        self._default_duration = 5.0
        
        # Default config file location relative to package
        if config_file_path is None:
            # Use ROS2 standard method to find package share directory
            package_share_dir = get_package_share_directory('go2_dance_orchestrator')
            config_file_path = os.path.join(package_share_dir, "config", "dance_commands.yaml")
        
        self._config_file_path = config_file_path
        self._load_configuration()
    
    def _load_configuration(self):
        """Load configuration from YAML file"""
        try:
            if not os.path.exists(self._config_file_path):
                logger.error(f"Configuration file not found: {self._config_file_path}")
                return
                
            with open(self._config_file_path, 'r') as file:
                config_data = yaml.safe_load(file)
                
            # Load dance commands
            dance_commands = config_data.get('dance_commands', {})
            for command_name, command_data in dance_commands.items():
                try:
                    command_config = CommandConfig(
                        command_name=command_name,
                        command_id=command_data.get('command_id'),
                        expected_duration=command_data.get('expected_duration', self._default_duration),
                        description=command_data.get('description', ''),
                        category=command_data.get('category', 'unknown'),
                        completion_detection=command_data.get('completion_detection')
                    )
                    self._commands[command_name] = command_config
                except Exception as e:
                    logger.error(f"Error loading command {command_name}: {e}")
                    continue
                    
            logger.info(f"Loaded {len(self._commands)} dance commands from {self._config_file_path}")
            
        except Exception as e:
            logger.error(f"Error loading dance configuration: {e}")
    
    def get_command_id(self, command_name: str) -> Optional[int]:
        """Get command ID for given command name"""
        command = self._commands.get(command_name)
        return command.command_id if command else None
    
    def get_expected_duration(self, command_name: str) -> float:
        """Get expected duration for given command name"""
        command = self._commands.get(command_name)
        return command.expected_duration if command else self._default_duration
    
    def get_command_config(self, command_name: str) -> Optional[CommandConfig]:
        """Get full command configuration"""
        return self._commands.get(command_name)
    
    def get_all_command_names(self) -> list:
        """Get list of all available command names"""
        return list(self._commands.keys())
    
    def get_duration_mapping(self) -> Dict[str, float]:
        """Get dictionary mapping command names to durations"""
        return {name: config.expected_duration for name, config in self._commands.items()}
    
    def get_command_id_mapping(self) -> Dict[str, int]:
        """Get dictionary mapping command names to command IDs"""
        return {name: config.command_id for name, config in self._commands.items()}
    
    def is_valid_command(self, command_name: str) -> bool:
        """Check if command name is valid"""
        return command_name in self._commands
    
    def reload_configuration(self):
        """Reload configuration from file"""
        self._commands.clear()
        self._load_configuration()


# Global singleton instance
_config_instance: Optional[DanceCommandConfig] = None


def get_dance_config() -> DanceCommandConfig:
    """Get singleton instance of dance configuration"""
    global _config_instance
    if _config_instance is None:
        _config_instance = DanceCommandConfig()
    return _config_instance