cmake_minimum_required(VERSION 3.0.2)
project(PF_nav)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_lib
  potbot_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PF_nav.cfg
)

catkin_package(
  LIBRARIES potbot_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/PF_nav.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_Marker src/PF_Marker.cpp)
target_link_libraries(${PROJECT_NAME}_Marker ${catkin_LIBRARIES})

add_executable(class_tutorial src/class_tutorial.cpp)
target_link_libraries(class_tutorial ${catkin_LIBRARIES})

add_definitions(-std=c++14)