cmake_minimum_required(VERSION 2.8.3)
project(ccr_msgs)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  message_generation
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Battery.msg
  Brushes.msg
  Bumper.msg
  IRCharacter.msg
  LEDBoardCommand.msg
  LEDBoardEvent.msg
  IRBumper.msg
  BooleanSensor.msg
  WheelDrop.msg
  Range.msg
  DirtDetect.msg
  Drive.msg
  Slip.msg
  Brushes_cmd.msg
  BrushesPWM_cmd.msg
  AirCondition.msg
  SystemInfo.msg
  DrivePwm.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package()


