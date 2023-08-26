#!/bin/bash
gnome-terminal -- bash -c "roscore; exec bash"
sleep 2
gnome-terminal -- bash -c "source devel/setup.bash; rosrun franka_reactive_controller libfranka_joint_goal_motion_generator 2; exec bash"
