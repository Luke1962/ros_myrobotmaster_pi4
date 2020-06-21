cmake_minimum_required(VERSION 2.8.3)
project(rm_speech)

#-----------------------------
#non uso le librerie ma chiamo espeak a sistema




find_package(catkin REQUIRED COMPONENTS std_msgs roscpp)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS std_msgs roscpp espeak
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  
)

add_executable(rm_speech_node src/speech.cpp)

target_link_libraries(rm_speech_node
  ${catkin_LIBRARIES}
)
