cmake_minimum_required(VERSION 2.8.3)
project(rosmower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosmower_msgs
  oled_display_node
  ds4_driver  
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
#generate_dynamic_reconfigure_options(
#  config/Hoverboard.cfg
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosmower_msgs
)

include_directories(
 include
 include/ROSMower
 ${catkin_INCLUDE_DIRS}
# ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/main_oled.cpp 
  src/ROSMower_oled.cpp
  src/ROSMower_ds4.cpp
#  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_dependencies(hoverboard_driver ${PROJECT_NAME}_gencfg)

