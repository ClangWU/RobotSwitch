cmake_minimum_required(VERSION 2.8)

project(robotswitch_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
   message_generation
  # genmsg
  # geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  # std_srvs
  nav_msgs
  serial
)

find_package(Eigen3 REQUIRED)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES   ${INCLUDE_DIR}/rs_common/math_utils.hpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)
## Declare a C++ executable
set(SRCS
    src/rs_common/math_utils.cpp)
## Declare a C++ library
add_library(math_utils ${H_FILES} ${SRCS})

add_dependencies(math_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ##${PROJECT_NAME}_generate_messages_cpp
  ##${PROJECT_NAME}_gencfg
)

target_link_libraries(math_utils PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(math_utils SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(math_utils PUBLIC
  include
)

catkin_package(
  CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES xf_mic_api
#  CATKIN_DEPENDS roscpp rospy senor_msgs std_msgs
#  DEPENDS system_lib
)
## ahrs_driver
add_executable(ahrs_driver src/ahrs/ahrs_driver.cpp)
target_link_libraries(ahrs_driver crc_table math_utils ${catkin_LIBRARIES})# ${SERIAL_LIB}

add_executable(imu_tf src/ahrs/imu_tf.cpp)
target_link_libraries( imu_tf ${catkin_LIBRARIES})# ${SERIAL_LIB}

## crc_table
add_library(crc_table src/ahrs/crc_table.cpp)