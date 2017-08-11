cmake_minimum_required(VERSION 2.8.3)

message(STATUS "IN FIRMWARE CMAKE FILE")

#set(ARDUINO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../libraries")
#message(STATUS "in sub target csd=${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${ROS_LIB_DIR})

# TODO handle in Arduino cmake based on board argument?
# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

# TODO detect board based on port? vice versa?

# TODO these arguments dont need to be quoted?
# i would rather not have to explicitly include 
# each library file... do i need to?
# TODO in this cmake function definition, modify to also check relative
# to parent directory for source
generate_arduino_firmware(stimulus
  SRCS ../stimuli/stimuli.ino
# TODO why is this failing? is it because path above is wrong?
# TODO should i replace version of cmake stuff here with most 
# recent from the github repo (queezythegreat stuff)
  BOARD uno
  PORT /dev/ttyACM0
)

message(STATUS "END OF FIRMWARE CMAKE FILE")
