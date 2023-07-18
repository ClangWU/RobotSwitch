cmake_minimum_required(VERSION 3.0.2)
project(franka_reactive_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # franka_control
  # franka_description
  control_toolbox
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_gripper
  franka_hw
  geometry_msgs
  # franka_visualization
  # panda_moveit_config
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  tf
  tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_reactive_controller
  CATKIN_DEPENDS
    controller_interface
    control_toolbox
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES   ${INCLUDE_DIR}/franka_cartesian_controller/cartesian_velocity_node_controller.h
              ${INCLUDE_DIR}/franka_motion_generator/libfranka_joint_motion_generator.h)

include_directories(include/franka_cartesian_controller include/franka_motion_generator ${catkin_INCLUDE_DIRS})
## Declare a C++ executable
set(SRCS
    src/franka_cartesian_controller/cartesian_velocity_node_controller.cpp
    src/franka_motion_generator/libfranka_joint_motion_generator.cpp)
## Declare a C++ library
add_library(franka_reactive_controller ${H_FILES} ${SRCS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(franka_reactive_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ##${PROJECT_NAME}_generate_messages_cpp
  ##${PROJECT_NAME}_gencfg
)

target_link_libraries(franka_reactive_controller PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_reactive_controller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_reactive_controller PUBLIC
  include
)

# Executable interface for joint-space goal motion and open/close the gripper
add_executable(franka_gripper_run_node src/franka_gripper_run_node.cpp)
target_link_libraries(franka_gripper_run_node franka_reactive_controller ${catkin_LIBRARIES})

add_executable(libfranka_joint_goal_motion_generator src/libfranka_joint_goal_motion_generator.cpp)
target_link_libraries(libfranka_joint_goal_motion_generator franka_reactive_controller ${catkin_LIBRARIES})

## Installation
install(TARGETS franka_reactive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_reactive_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS
  scripts/franka_to_geometry_messages.py
  scripts/franka_gui_gripper_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
