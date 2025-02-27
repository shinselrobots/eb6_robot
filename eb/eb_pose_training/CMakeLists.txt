cmake_minimum_required(VERSION 2.8.3)
project(eb_pose_training)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/eb_pose_training.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
