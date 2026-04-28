# 跟踪对象
string object_id
string object_class
float32 confidence

# 位姿
geometry_msgs/Pose pose
geometry_msgs/Twist velocity

# 跟踪信息
uint32 track_count
float32 time_since_update