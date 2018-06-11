cmake_minimum_required(VERSION 2.8.3)
project(rosbridge_push)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/rosbridge_push
	  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

