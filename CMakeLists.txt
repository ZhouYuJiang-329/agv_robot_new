cmake_minimum_required(VERSION 3.8)
project(agv_robot_new)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# This is a metapackage, no actual build needed
# All sub-packages are built independently by colcon

ament_package()
