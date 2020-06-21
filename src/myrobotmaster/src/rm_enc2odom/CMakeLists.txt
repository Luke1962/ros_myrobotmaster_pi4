cmake_minimum_required(VERSION 2.8.3)
project(rm_enc2odom)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs 
  std_msgs  
  tf
  tf2_ros 
  )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_msgs std_msgs geometry_msgs tf2  tf2_ros 
 # DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES1  src/enc2odom_node.cpp)

## Build libraries
add_library(${PROJECT_NAME} ${SRC_FILES1} )

## Declare a cpp executable
add_executable(enc2odom ${SRC_FILES1})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(espeak_ros_node espeak_ros_generate_messages_cpp)

add_dependencies(enc2odom
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
)


## Specify libraries to link a library or executable target against
target_link_libraries(enc2odom
   ${catkin_LIBRARIES}
 )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS
  enc2odom
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
