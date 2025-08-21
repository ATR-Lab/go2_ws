# Dance routine execution status
string routine_name
string[] command_sequence
int32 current_command_index    # -1 = not started, commands.length = completed
string current_command_name
int32 total_commands
float32 routine_progress_percent  # Overall routine progress (0-100)
float32 current_command_progress_percent  # Current command progress (0-100)
float32 routine_elapsed_time
string status                 # "PENDING", "EXECUTING", "COMPLETED", "FAILED", "CANCELLED"
string completion_reason