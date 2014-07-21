cmake_minimum_required(VERSION 2.8.3)
project(odomi_path_planner)


find_package(catkin REQUIRED COMPONENTS cv_bridge tf nav_msgs roscpp std_msgs)
find_package(OpenCV REQUIRED)

catkin_package()

add_executable(odomi_path_planner src/odomi_path_planner.cpp src/Dstar.cpp)
target_link_libraries(odomi_path_planner ${catkin_LIBRARIES})


