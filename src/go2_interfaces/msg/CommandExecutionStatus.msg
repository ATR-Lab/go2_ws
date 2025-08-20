# Real-time execution status for a single dance command
string command_name
int32 command_id
uint8 status               # 0=pending, 1=executing, 2=completed, 3=failed, 4=timeout
float32 progress_percent   # 0.0 to 100.0  
float32 elapsed_time       # Time since command started
string completion_reason   # "progress_detection", "timeout", "mode_change", etc.
builtin_interfaces/Time start_time
builtin_interfaces/Time completion_time