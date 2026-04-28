# 系统整体状态
builtin_interfaces/Time stamp

uint8 overall_status
uint8 STATUS_UNKNOWN=0
uint8 STATUS_INITIALIZING=1
uint8 STATUS_READY=2
uint8 STATUS_RUNNING=3
uint8 STATUS_ERROR=4
uint8 STATUS_EMERGENCY_STOP=5

BatteryStatus battery
HardwareStatus agv_status
HardwareStatus arm_status
SensorStatus[] sensor_statuses
string[] active_tasks
string error_message