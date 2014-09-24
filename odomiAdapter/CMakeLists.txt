cmake_minimum_required(VERSION 2.8.3)
project(odomiadapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  OpenCV
  odomi_path_planner
 # pcl_conversions
 # pcl_ros
  
)

find_package(OpenCV REQUIRED)
find_package(roscpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(odomi_path_planner REQUIRED)

#find_package(PCL REQUIRED)

catkin_package()


include_directories( ${roscpp_INCLUDE_DIRS} ${odomi_path_planner_INCLUDE_DIRS}) #${PCL_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(odomiAdapter src/odomiAdapter.cpp )

#add_dependencies(pubstartgoal pubstartgoal_generate_messages_cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(odomiAdapter ${roscpp_LIBRARIES} ${odomi_path_planner_INCLUDE_DIRS}  ${catkin_LIBRARIES})
