cmake_minimum_required(VERSION 3.0.2)
project(Robot)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/move_base.launch)
roslaunch_add_file_check(launch/pat_configuration.launch)

install(
DIRECTORY launch maps params
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)