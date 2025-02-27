cmake_minimum_required(VERSION 2.8.3)
project(eb)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


