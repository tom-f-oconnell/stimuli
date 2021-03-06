cmake_minimum_required(VERSION 2.8.3)
project(stimuli)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# TODO add genpy*?
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  rosserial_arduino
  rosserial_client
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
  State.msg
  DefaultState.msg
  Transition.msg
  Sequence.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LoadDefaultStates.srv
  LoadSequence.srv
  TestTransportLoadDefaultStatesReq.srv
  TestTransportLoadSequenceReq.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  stimuli
)
# TODO should stimuli be here? i don't think tutorial did this?

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# TODO update?
catkin_package(
# TODO add ros_lib for arduino?
#  INCLUDE_DIRS include
#  LIBRARIES stimuli
#  CATKIN_DEPENDS rosserial_arduino
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

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

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Upload  ##
#############

# TODO make this automatic, conditional on an Arduino being connected that already has the sofware
# installed?
# or prompt if an arduino is connected w/o the software installed (or print message at end
# saying to run the target required for uploading when ready?)

# this set doesn't seem to work globally from here? how to?
#set(ARDUINO_SDK_PATH "~/arduino-1.8.3")
#message(STATUS "SET ARDUINO SDK PATH")
#
## TODO need to set path for libraries for avr build to work?
## (normally i would supply make_libraries.py a path if calling
##  manually... actually, there isn't a make_libraries.py, right?)
#rosserial_generate_ros_lib(
#  PACKAGE rosserial_arduino
#  SCRIPT make_libraries.py
#)
#message(STATUS "AFTER ROSSERIAL_GENERATE_ROS_LIB")
#
## TODO why might cmake not be entering that directory?
#rosserial_configure_client(
#  DIRECTORY firmware
## TODO what is this doing if not replacing sdk?
#  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
#)
#message(STATUS "AFTER ROSSERIAL_CONFIGURE_CLIENT")
#
## TODO document if prefix of upload target needs to be the 
## target above?
#rosserial_add_client_target(firmware stimulus ALL)
#message(STATUS "AFTER ADDING STIMULUS_ARDUINO TARGET")
#rosserial_add_client_target(firmware stimulus-upload)
#message(STATUS "AFTER ADDING UPLOAD TARGET")

