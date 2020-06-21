cmake_minimum_required(VERSION 2.8.3)

project(edge_tpu)

find_package(catkin REQUIRED COMPONENTS message_generation rospy
    dnn_detect #genera l'array delle detection usata da questo nodo
	vision_msgs # packafe che contiene messaggi utilizzati 
)

# Questa sezione va specificata solo quando ci sono nuovi messaggi del package
# add_message_files(
#    FILES
#    DetectedObject.msg
#    DetectedObjectArray.msg
# )

#Questa sezione va usata quando devo usare messaggi di altri pkg e/o del proprio package
# in package.xml vanno poi aggiunti
# <build_depend>message_generation</build_depend>
# <exec_depend>message_runtime</exec_depend>
 generate_messages(
   DEPENDENCIES
   vision_msgs
 )

catkin_package()


#############
## Install ##
#############
