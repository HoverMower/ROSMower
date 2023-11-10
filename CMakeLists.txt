cmake_minimum_required(VERSION 3.5)
project(rosmower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosmower_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 include/ROSMower
 ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/main.cpp 
  src/ROSMower_joystick.cpp
  src/ROSMower_SafetyController.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosmower_msgs
  )

install(TARGETS
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()
