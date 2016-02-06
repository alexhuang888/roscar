cmake_minimum_required(VERSION 2.8.3)
project(wheels)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	roscpp
	roslib
	rospy
	std_msgs
	sensor_msgs
	message_generation
	genmsg
	actionlib_msgs
	actionlib
	tf2
	tf
	image_transport
)
# for raspicam controller
#find_package(raspicam REQUIRED)
# for OpenCV
find_package(OpenCV REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	wheels_status.msg
	navigator_engine_status.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	cmd_get_one_wheel_status.srv
	cmd_set_car_direction_speed.srv
	cmd_get_navigator_engine_status.srv
	cmd_set_navigator_engine.srv
	cmd_send_manual_instruction.srv
	cmd_set_car_two_wheels_direction_speed.srv
	cmd_ask_navigator_saveimage.srv
)

## Generate actions in the 'action' folder
add_action_files(
	FILES
	set_car_direction_speed.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wheels
#  CATKIN_DEPENDS roscpp rospy std_msg
#  DEPENDS system_lib
	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


## Declare a C++ library
# add_library(wheels
#   src/${PROJECT_NAME}/wheels.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(wheels ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(wheels_node src/wheels_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(wheels_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(wheels_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS wheels wheels_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wheels.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	../include
	../../wiringPi/wiringPi
	${PROJECT_SOURCE_DIR}/src
	${Boost_INCLUDE_DIRS}
	)

#to display wheel status
add_executable(wheel_status_console src/wheelstatusconsole.cpp src/wheelstatusconsolenode.cpp src/myutil.cpp )
target_link_libraries(wheel_status_console ${catkin_LIBRARIES})
add_dependencies(wheel_status_console wheels_generate_messages_cpp)

# the driver of the robot-car, it will listen to cmd_vel and send message to motor controller
add_executable(wheel_driver src/wheeldriver.cpp src/wheeldrivernode.cpp src/myutil.cpp )
target_link_libraries(wheel_driver ${catkin_LIBRARIES})
add_dependencies(wheel_driver wheels_generate_messages_cpp)

# motor controller, will listen to message then call motor controller.
add_executable(wheel_controller src/wheelmotorengine.cpp src/wheelcontroller.cpp src/wheelcontrollernode.cpp src/myutil.cpp )
target_link_libraries(wheel_controller ${catkin_LIBRARIES} wiringPi pthread)
add_dependencies(wheel_controller wheels_generate_messages_cpp)

# navigator engine, will get image from image_raw, process them, then convert to wheel_cmd_vel msg
add_executable(wheel_navigator src/wheelnavigatornode.cpp src/wheelnavigator.cpp src/myutil.cpp src/clinefollowernavigatorengine.cpp src/clanedetectornavigatorengine.cpp src/cnavigatorenginebase.cpp src/clinefollowernavigatorengine2.cpp)
target_link_libraries(wheel_navigator ${catkin_LIBRARIES})
add_dependencies(wheel_navigator wheels_generate_messages_cpp)

# the manual override console of the robot-car, it will listen to cmd_vel and send message to motor controller
add_executable(wheel_manual_console src/wheeldriver.cpp src/wheelmanualconsole.cpp src/myutil.cpp )
target_link_libraries(wheel_manual_console ${catkin_LIBRARIES})
add_dependencies(wheel_manual_console wheels_generate_messages_cpp)

# the manual override console of the robot-car, it will listen to cmd_vel and send message to motor controller
add_executable(testlinefollowing2 src/myutil.cpp src/clinefollowernavigatorengine2.cpp src/cnavigatorenginebase.cpp src/test/testlinefollower2.cpp)
target_link_libraries(testlinefollowing2 ${catkin_LIBRARIES})

# the manual override console of the robot-car, it will listen to cmd_vel and send message to motor controller
add_executable(testlanedetector src/myutil.cpp src/clanedetectornavigatorengine.cpp src/cnavigatorenginebase.cpp src/test/testlanedetector.cpp)
target_link_libraries(testlanedetector ${catkin_LIBRARIES})

# the manual override console of the robot-car, it will listen to cmd_vel and send message to motor controller
add_executable(testlanedetector2 src/myutil.cpp src/clinefollowernavigatorengine3.cpp src/cnavigatorenginebase.cpp src/test/testlanedetector2.cpp)
target_link_libraries(testlanedetector2 ${catkin_LIBRARIES})
