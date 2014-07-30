cmake_minimum_required(VERSION 2.8.3)
project(drone)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_package()

add_executable(drone src/drone.cpp)
target_link_libraries(drone ${catkin_LIBRARIES})

