cmake_minimum_required(VERSION 2.8.3)
project(odomi_path_planner)


find_package(catkin REQUIRED COMPONENTS cv_bridge tf nav_msgs roscpp std_msgs)
find_package(OpenCV REQUIRED)

catkin_package()

<<<<<<< HEAD
catkin_package(
   #INCLUDE_DIRS include
   LIBRARIES mygeolib.h 
   #CATKIN_DEPENDS roscpp rospy std_msgs
   #DEPENDS system_lib
)

#add_library(mygeolib  src/mygeolib.cpp )

add_executable(odomi_path_planner src/odomi_path_planner.cpp src/Dstar.cpp )
target_link_libraries(odomi_path_planner ${catkin_LIBRARIES})

=======
add_executable(odomi_path_planner src/odomi_path_planner.cpp src/Dstar.cpp)
target_link_libraries(odomi_path_planner ${catkin_LIBRARIES})


>>>>>>> 9ee2ed5823151f80049aa73bcf5d8c240b83eecf
