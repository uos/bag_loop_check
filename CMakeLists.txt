cmake_minimum_required(VERSION 2.8.3)
project(bag_loop_check)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
