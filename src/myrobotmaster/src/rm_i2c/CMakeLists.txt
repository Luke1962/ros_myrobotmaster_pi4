cmake_minimum_required(VERSION 2.8.3)
project(rm_i2c)
set(CMAKE_CXX_FLAGS "-std=c++11")








# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)

IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")


#senza questo non trova il package
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp  )
find_library(
 
  ## wiringPi REQUIRED
  rt REQUIRED
  pthread REQUIRED
  
)
 
 

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  include
)
## Declare a cpp executable
 
#add_executable(rm_i2c src/rm_i2c.cpp src/ina219.cpp )
add_executable(rm_i2c src/rm_i2c_ina219_ads1115_sx1509.cpp src/ina219.cpp src/sx1509.cpp)
add_dependencies(rm_i2c ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




#link_directories(/opt/ros/melodic/lib)
 #ADD_LIBRARY(/opt/ros/melodic/lib/librosconsole_log4cxx.so STATIC IMPORTED)
target_link_libraries(rm_i2c 
	#rosconsole_log4cxx 
  ${catkin_LIBRARIES} 
  ${mraa_LIBRARIES}
  #wiringPi 
  rt 
  pthread
  mraa
)



 # /opt/ros/melodic/lib/librosconsole_log4cxx.so

