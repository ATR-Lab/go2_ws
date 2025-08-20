# Complete dance routine definition
string routine_name         # e.g., "birthday_dance", "greeting_sequence"
string description         # Human-readable description
DanceCommand[] commands     # Array of dance commands
float32 total_duration     # Estimated total time
bool loop                  # Whether to repeat the sequence
int32 loop_count          # Number of repetitions (-1 = infinite)