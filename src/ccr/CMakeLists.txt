cmake_minimum_required(VERSION 2.8.3)
project(ccr)

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
  serial
  tf
  ccr_msgs
)
# Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#  rulo_msgs
#)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ccr_serial src/SerialDataPicker.cpp src/diversion/odometry.cpp src/Ccr.cpp)
add_dependencies(ccr_serial ccr_gencpp ccr_msgs_generate_messages_cpp)

add_executable(joy_node src/util/joy_node.cpp)
add_dependencies(joy_node ccr_msgs_generate_messages_cpp)

add_executable(keyboard_node src/util/keyboard_node.cpp)
add_dependencies(keyboard_node ccr_msgs_generate_messages_cpp)

target_link_libraries(ccr_serial ${catkin_LIBRARIES})

target_link_libraries(joy_node ${catkin_LIBRARIES})

target_link_libraries(keyboard_node ${catkin_LIBRARIES})
