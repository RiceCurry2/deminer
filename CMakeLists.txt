cmake_minimum_required(VERSION 2.8.3)
project(deminer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d 
  dynamic_reconfigure 
  roscpp
  visualization_msgs
  move_base_msgs
  geometry_msgs
  std_msgs
  tf
  kobuki_msgs
  actionlib
  actionlib_msgs
#  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)


## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Deminer.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#FILES
#GetNextTask.srv
#)

## Generate actions in the 'action' folder
#add_action_files(
#FILES 
#DeminerTask.action
#)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
#   actionlib_msgs
#   move_base_msgs
#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
catkin_package(INCLUDE_DIRS include
INCLUDE_DIRS include
CATKIN_DEPENDS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  tf
  actionlib
DEPENDS
  PCL
  Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#add_executable(deminer_server src/deminer_server.cpp)
#target_link_libraries(deminer_server ${catkin_LIBRARIES})
#add_dependencies(deminer_server ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(deminer_client src/deminer_client.cpp)
target_link_libraries(deminer_client ${catkin_LIBRARIES})
# add_dependencies(deminer_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(click src/click.cpp)
target_link_libraries(click ${catkin_LIBRARIES})

add_executable(create_mine src/create_mine.cpp)
target_link_libraries(create_mine ${catkin_LIBRARIES})

#############
## Install ##
#############


# all install targets should use catkin DESTINATION variables


install(TARGETS deminer_client #deminer_server 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)