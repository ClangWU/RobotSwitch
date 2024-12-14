cmake_minimum_required(VERSION 3.0.2)

project(robotswitch_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  nav_msgs
  serial
  geometry_msgs
  message_filters
)

find_package(Eigen3 REQUIRED)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES   
${INCLUDE_DIR}/rs_common/math_utils.hpp
${INCLUDE_DIR}/rs_common/filter_math.h
${INCLUDE_DIR}/rs_common/biquad_filter.h
${INCLUDE_DIR}/rs_common/common_data.h
${INCLUDE_DIR}/rs_common/serial_node.h
${INCLUDE_DIR}/autobot.h
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)
## Declare a C++ executable
set(SRCS
    src/rs_common/math_utils.cpp
    src/rs_common/filter_math.c
    src/rs_common/biquad_filter.c
    )
## Declare a C++ library
add_library(utils ${H_FILES} ${SRCS})

add_dependencies(utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ##${PROJECT_NAME}_generate_messages_cpp
  ##${PROJECT_NAME}_gencfg
)

target_link_libraries(utils PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(utils SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(utils PUBLIC
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
target_link_libraries(ahrs_driver crc_table utils ${catkin_LIBRARIES})# ${SERIAL_LIB}

add_executable(imu_tf src/ahrs/imu_tf.cpp)
target_link_libraries( imu_tf ${catkin_LIBRARIES})# ${SERIAL_LIB}

add_executable(rs_tf src/ahrs/rs_tf.cpp)
target_link_libraries( rs_tf ${catkin_LIBRARIES})# ${SERIAL_LIB}

add_executable(imu_handler src/ahrs//imu_handler.cpp)
target_link_libraries(imu_handler utils ${catkin_LIBRARIES})

add_executable(rs_tf_test src/ahrs/rs_tf_test.cpp)
target_link_libraries( rs_tf_test ${catkin_LIBRARIES})# ${SERIAL_LIB}

add_executable(rs_tf_delta src/ahrs/rs_tf_delta.cpp)
target_link_libraries( rs_tf_delta ${catkin_LIBRARIES})# ${SERIAL_LIB}

add_executable(force_mcu_node 
              src/force_mcu/force_mcu_serial_node.cpp
              src/force_mcu/force_mcu_control.cpp)
target_link_libraries(force_mcu_node ${catkin_LIBRARIES})

add_executable(teleop_mcu_node 
              src/teleop_mcu/teleop_mcu_serial_node.cpp
              src/teleop_mcu/teleop_mcu_control.cpp)
target_link_libraries(teleop_mcu_node ${catkin_LIBRARIES})


add_executable(imu_mcu_node 
              src/imu_mcu/imu_mcu_serial_node.cpp
              src/imu_mcu/imu_mcu_control.cpp)
target_link_libraries(imu_mcu_node ${catkin_LIBRARIES})

add_executable(autobot_node  src/autobot.cpp)
target_link_libraries(autobot_node ${catkin_LIBRARIES})

## crc_table
add_library(crc_table src/ahrs/crc_table.cpp)