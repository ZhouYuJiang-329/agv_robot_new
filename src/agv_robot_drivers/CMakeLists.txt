cmake_minimum_required(VERSION 3.8)
project(agv_robot_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agv_robot_hal REQUIRED)
find_package(agv_robot_common REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加珞石SDK库
add_subdirectory(include/agv_robot_drivers/arm/rokae_cr7/lib)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/arm/rokae_cr7/rokae_cr7_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/agv_robot_drivers/arm/rokae_cr7>
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  Rokae::Rokae
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  agv_robot_hal
  agv_robot_common
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  agv_robot_hal
  agv_robot_common
)
ament_package()
