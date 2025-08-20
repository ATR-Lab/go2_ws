# Real-time execution status for a dance sequence
string routine_name
uint8 sequence_status      # 0=idle, 1=executing, 2=paused, 3=completed, 4=error
int32 current_command_index
CommandExecutionStatus current_command
float32 sequence_progress_percent
string error_message       # If sequence_status == error
int32 commands_completed
int32 total_commands