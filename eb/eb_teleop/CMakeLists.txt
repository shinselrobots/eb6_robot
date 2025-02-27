cmake_minimum_required(VERSION 2.8.3)
project(eb_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs joy
)

add_executable(eb_teleop src/turtlebot_joy.cpp)
target_link_libraries(eb_teleop ${catkin_LIBRARIES})
add_dependencies(eb_teleop ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS eb_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/eb_teleop_key DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
