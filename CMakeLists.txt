cmake_minimum_required(VERSION 3.8)
project(agv_robot_new)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Add all subdirectories
add_subdirectory(src/agv_robot_common)
add_subdirectory(src/agv_robot_msgs)
add_subdirectory(src/agv_robot_hal)
# add_subdirectory(src/agv_robot_capability)
# add_subdirectory(src/agv_robot_planning)
# add_subdirectory(src/agv_robot_control)
# add_subdirectory(src/agv_robot_perception)
# add_subdirectory(src/agv_robot_task_manager)
# add_subdirectory(src/agv_robot_drivers)
# add_subdirectory(src/agv_robot_bringup)

ament_package()
