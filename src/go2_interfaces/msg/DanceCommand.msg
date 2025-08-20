# Single dance move definition
string command_name          # e.g., "Hello", "FrontFlip", "Dance1"  
int32 command_id            # Robot command ID (from robot_commands.py)
float32 expected_duration   # Expected execution time (seconds)
float32 delay_before        # Wait time before executing (seconds)
string parameters           # JSON parameters for the command
uint8 priority             # Command priority (0=normal, 1=high)
builtin_interfaces/Time timestamp