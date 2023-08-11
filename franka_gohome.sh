#!/bin/bash
gnome-terminal -- bash -c "roscore; exec bash"
sleep 2
gnome-terminal -- bash -c "rosrun franka_reactive_controller libfranka_joint_goal_motion_generator 1; exec bash"
