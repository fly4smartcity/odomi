cmake_minimum_required(VERSION 2.8.3)
project(odomi_path_planner)

find_package(roscpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS OpenCV cv_bridge tf nav_msgs std_msgs)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(roscpp REQUIRED)


catkin_package()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES  mygeolib
   CATKIN_DEPENDS roscpp
   #DEPENDS system_lib
)

include_directories(include ${roscpp_INCLUDE_DIRS})

# Define a library target called mygeolib
add_library(mygeolib  src/mygeolib.cpp )
target_link_libraries(mygeolib ${roscpp_LIBRARIES})

# Define an executable target called hello_world_node
add_executable(odomi_path_planner src/odomi_path_planner.cpp src/Dstar.cpp )
target_link_libraries(odomi_path_planner ${roscpp_LIBRARIES} mygeolib ${catkin_LIBRARIES})

