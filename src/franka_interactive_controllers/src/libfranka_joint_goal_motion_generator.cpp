// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Easy non-franka_ros executable to send robot to desired joint configurations. 
// Caution: It won't work when franka_ros/franka_control is running!

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

// #include <franka_motion_generators/libfranka_joint_motion_generator.h>
#include <libfranka_joint_motion_generator.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

int main(int argc, char** argv) {

  std::string franka_ip = "192.168.1.100";

  // Check whether the required arguments were passed replace this with rosparam!
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <goal_id>" << std::endl;
    return -1;
  }

  try {
    
    // Connect to robot.
    franka::Robot robot(franka_ip);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});


    // First move the robot to a suitable joint configuration
    int goal_id = std::stod(argv[1]);

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    switch(goal_id) {
       case 1  :
          std::cout << "Selected q_home as goal" << std::endl;
          // q_goal = {{0.009340305111597252, -0.36729619687063647, 0.05388360048542943, -2.3592505386694267, 0.016102603363234352, 2.0088766928513846, 0.8014595653550303}};
          // q_goal = {{0.013076832091468467, 0.05892901891679095, 0.05233293692648427, -2.289458147450497, 0.014325451839091376, 2.3411785166360315, 0.8852154568417203}};
          //q_goal = {{-0.0026496768670445665, -0.09072233400129963, 0.05107856889461215, -2.412414035562883, -0.02818185633089807, 2.3957946495215094, 0.7987457616245822}};
          q_goal = {{-0.04125446628949098, -0.23194385116560415, 0.037441184571944, -2.1041153984069823, 0.004531050744156042, 1.8886842381954192, 0.796802966348411}};         
          break;

       case 2  :
        std::cout << "切前的初始位置" << std::endl;
          q_goal = {{-0.03832298934536793, 0.1998915870014838, 0.03704407827111713, -2.363868214921435, -0.013180064449634314, 2.562447239663866, 0.7951065804693713}};
          break;

       case 3  :
        std::cout << "切前的初始位置 斜" << std::endl;
          q_goal = {{-0.0382498, 0.202102, 0.035896, -2.36421, -0.0171878, 2.55406, 0.885862}};
          break;

       case 5  :
        std::cout << "Selected tele pose as goal" << std::endl;
          q_goal = {{-0.0122613, 0.190203, 0.0354907, -2.37207, -0.0177259, 2.55778, 0.918566}};
        break;
          
       case 6 :
        std::cout << "Selected q_left_table_setting as goal" << std::endl;
        q_goal = {{-0.011755, 0.190072, 0.0365307, -2.36253, -0.0189527, 2.50591, 0.90272}};
        break;
       case 7 :
        std::cout << "Selected q_knife as goal" << std::endl;
        q_goal = {{-0.0133481, -0.128973, -0.0548055, -2.31734, -0.0430293, 2.21409, 0.762126}};
        break;
    }
  
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
