# 硬件状态
string device_name
uint8 state
uint8 STATE_UNKNOWN=0
uint8 STATE_INITIALIZING=1
uint8 STATE_READY=2
uint8 STATE_RUNNING=3
uint8 STATE_ERROR=4
uint8 STATE_EMERGENCY_STOP=5

string error_message
builtin_interfaces/Time last_update