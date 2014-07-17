cmake_minimum_required(VERSION 2.8.3)
project(open_data_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

FIND_LIBRARY(CURL_LIBRARY curl /usr/bin/curl.so)
FIND_LIBRARY(GDAL_LIBRARY gdal /usr/local/lib/libgdal.so)
FIND_LIBRARY(ZZIP_LIBRARY zzip /usr/lib/libzzip.so)
FIND_LIBRARY(MYSQLCLIENT_LIBRARY mysqlclient /usr/lib/i386-linux-gnu/libmysqlclient.so)
FIND_LIBRARY(MYSQLZ_LIBRARY z /usr/lib/i386-linux-gnu/libz.so)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  open_data_msg
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES open_data_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS curl gdal zzip
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include
  ${catkin_INCLUDE_DIRS} ${curl_INCLUDE_DIRS} ${gdal_INCLUDE_DIRS}${zzip_INCLUDE_DIRS}${mysql_INCLUDE_DIRS}${z_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(open_data_driver
#   src/${PROJECT_NAME}/open_data_driver.cpp
# )




## Declare a cpp executable


# set the path to the library folder
link_directories(/usr/local/lib)


#add_library(geoportale src/Geoportale.cpp src/Oggetto.cpp src/read.cpp src/Json/json_reader.cpp src/Json/json_value.cpp src/Json/json_writer.cpp)
#TARGET_LINK_LIBRARIES(geoportale ${CURL_LIBRARY} ${GDAL_LIBRARY})
#add_library(pot_segnale src/PotenzaSegnale.cpp src/read.cpp src/Json/json_reader.cpp src/Json/json_value.cpp src/Json/json_writer.cpp)
#TARGET_LINK_LIBRARIES(pot_segnale ${CURL_LIBRARY})
#add_library(traffico src/Traffico.cpp)
#add_library(trasporti src/Csv/csv_parser.cpp src/Csv/Trasporti.cpp)
#TARGET_LINK_LIBRARIES(trasporti ${CURL_LIBRARY} ${ZZIP_LIBRARY})

#LINK_LIBRARIES(${catkin_LIBRARIES} ${CURL_LIBRARY} ${ZZIP_LIBRARY} ${ZZIP_LIBRARY})

#add_executable(open_data_driver src/open_data_driver.cpp)


 add_executable(open_data_driver src/open_data_driver.cpp src/read.cpp src/Json/json_reader.cpp src/Json/json_value.cpp src/Json/json_writer.cpp src/Geoportale.cpp  src/Trasporti.cpp src/Csv/csv_parser.cpp  src/PotenzaSegnale.cpp src/Traffico.cpp src/Weather.cpp src/xml/pugixml.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(open_data_driver ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(open_data_driver
   ${catkin_LIBRARIES} 
#${CURL_LIBRARY} ${ZZIP_LIBRARY} ${ZZIP_LIBRARY} 
)  


# link the libraries to the executable
target_link_libraries (open_data_driver curl gdal zzip mysqlclient z)

#TARGET_LINK_LIBRARIES(open_data_driver geoportale pot_segnale traffico trasporti)

add_executable(bounding_box_client src/client.cpp)
target_link_libraries(bounding_box_client ${catkin_LIBRARIES})

add_executable(open_data_sync src/open_data_sync.cpp src/Trasporti.cpp src/Csv/csv_parser.cpp src/read.cpp)
add_dependencies(open_data_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_data_sync ${catkin_LIBRARIES})
target_link_libraries (open_data_sync curl zzip mysqlclient z)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS open_data_driver open_data_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_open_data_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
