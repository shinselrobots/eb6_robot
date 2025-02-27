cmake_minimum_required(VERSION 2.8.3)
project(eb_servos)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ReturnJointStates.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs   std_msgs
)


catkin_package(
  CATKIN_DEPENDS dynamixel_controllers 
)




install(DIRECTORY launch config udev scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)








