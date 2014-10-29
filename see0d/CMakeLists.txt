cmake_minimum_required(VERSION 2.8.3)
project(see0d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  OpenCV
  odomi_path_planner
  message_generation
)

 add_message_files(
  FILES
  buildings.msg

 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
  )

find_package(OpenCV REQUIRED)
find_package(roscpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(odomi_path_planner REQUIRED)

catkin_package(
INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${odomi_path_planner_INCLUDE_DIRS}
 #${PCL_INCLUDE_DIRS}
)

# Declare a cpp executable
add_executable(see0d src/see0d.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(see0d ${roscpp_LIBRARIES}  ${catkin_LIBRARIES} ${odomi_path_planner_INCLUDE_DIRS})
