cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  JointState.msg
  JointStateArray.msg
  MotorState.msg
  MotorStateList.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
