cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

add_executable(mission_planner src/mission_planner.cpp)
target_link_libraries(mission_planner ${catkin_LIBRARIES})

