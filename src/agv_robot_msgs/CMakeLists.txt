cmake_minimum_required(VERSION 3.8)
project(agv_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  msg/SystemStatus.msg
  msg/BatteryStatus.msg
  msg/HardwareStatus.msg
  msg/SensorStatus.msg
  msg/DetectionResult.msg
  msg/DetectionResultArray.msg
  msg/UltrasoundReading.msg
  msg/UltrasoundScan.msg
  msg/TrackedObject.msg
  msg/TrackedObjectArray.msg
  
  # Services
  srv/NavigateToPose.srv
  srv/MoveArm.srv
  srv/DetectObject.srv
  srv/GetSystemStatus.srv
  
  # Actions
  action/PickPlace.action
  action/Patrol.action
  action/MoveArm.action
  
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    moveit_msgs
    vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()