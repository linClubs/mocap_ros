
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>

#include "mocap/readCSV.hpp"
#include "mocap/listen2mocap_v1.hpp"
#include "mocap/sphere2hinge.hpp"

#include "mocap/mocap.hpp"
#include "h1_arm/h1_arm_hand.hpp"


H1Teleop::H1Teleop() : Node("h1_node")
{ 
  next_arm_q_ = std::vector<float>(20, 0.);
  curr_arm_q_ = std::vector<float>(20, 0.);

 

  next_head_q_ = std::vector<float>(4, 0.);
  curr_head_q_ = std::vector<float>(4, 0.);
  

  // pub
  pub_arm_ = this->create_publisher<unitree_go::msg::MotorCmds>("body/cmd", 10);
  pub_hand_ = this->create_publisher<unitree_go::msg::MotorCmds>("hand/cmd", 10);
  pub_head_ = this->create_publisher<unitree_go::msg::MotorCmds>("head/cmd", 10);
  
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&H1Teleop::timer_callback, this));

  while(rclcpp::ok())
  {
    timer_callback();
  }

}

void H1Teleop::timer_callback()
{   // 1 get mocap data
    // 34, 8 + 1(left_hand) + 12(left_fingers) + 1(right_hand) + 12(right_fingers)
    // upJoints[[8], [21]] = 0, upJoints[9->20](left_thumb 9->12, left_index->little 13->20) 
    auto upJoints = mocap->getJoints();

    // 2 finger data process
    // 2.1 left_finger, qcmd[litte, ring, middle, index, thumb_1, thumb_2]
    // qcmd(4, 5) -> upJoints[9, 10, 11, 12]
    qcmd(4) = 1 -  upJoints[11] / 1.2;   
    qcmd(5) = 1 -  upJoints[12] / 0.8;
    //  qcmd[0, 1, 2, 3] -> upJoints[20, 18, 16, 14],  // little -> index    
    for(int i = 0; i < 4; i++)
    { 
      qcmd(i) =1 - upJoints[20 - i * 2] / 1.7; 
    }

    // 2.2 right_finger
    // qcmd[10, 11] -> upJoints[22, 23, 24, 25]
    qcmd(10) = 1 -  upJoints[24] / 1.2;   
    qcmd(11) = 1 -  upJoints[25] / 0.6;   
    // qcmd[6, 7, 8, 9] -> upJoints[33, 31, 29, 27]
    for(int i = 0; i < 4; i++)
    { 
      qcmd(6 + i) =1 - upJoints[33 - i * 2] / 1.7; 
    }
    
      std::cout << "finger  : ";
   

    // 2.3 publish finger signal 
    unitree_go::msg::MotorCmds handCmdsMsg;
    unitree_go::msg::MotorCmd handCmd;

    for(int i = 0; i < 12; ++i)
    { 
      handCmd.mode = 0;
      handCmd.kp = 0;
      handCmd.kd = 0;
      handCmd.q = qcmd(i);
      handCmdsMsg.cmds.push_back(handCmd);
      std::cout << qcmd(i) << "  ";
    }
    std::cout << std::endl;

    



    // 3 arm data process
    // 3.1 update arm data
    std::vector<float> arm_q(20, 0.0);
    for(int i = 0; i < 8; ++i)
    {
      arm_q[i+12] = upJoints[i]; 
    }

    // 3.2 define unitree_go ros2_msg , 20dims
    unitree_go::msg::MotorCmds bodyCmdsMsg;
    unitree_go::msg::MotorCmd bodyCmd;
    // 3.2.1 身体的20个关节赋值kp和kd, initialize 20个关节, 12-19总共8个kp=60，kd = 1.5，id=9的qpos=1
    for (int i = 0; i < 20; ++i) 
    {
        bodyCmd.kp = (i >= 12 && i <20) ? 60 : 0.0;
        bodyCmd.kd = (i >= 12 && i <20) ? 1.5 : 0.0;
        bodyCmdsMsg.cmds.push_back(bodyCmd);
    }
    bodyCmdsMsg.cmds[9].q = 1.0;

   std::cout << "arm  : ";
    for (int j = 12; j < 20; ++j) 
    {
      bodyCmdsMsg.cmds[arm_joints.at(j-12)].q = arm_q.at(j) ;
      std::cout << arm_q.at(j) << "  ";
    }
    std::cout << std::endl;

    // 3.3 publish arm control signal
    // pub_hand_->publish(handCmdsMsg);
    pub_arm_->publish(bodyCmdsMsg);
   
   // std::this_thread::sleep_for(std::chrono::milliseconds(1));
};

int main(int argc, char** argv)
{ 
  
  if (argc < 2)
  {
      std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
      exit(-1);
  }
  printf("start \n");
  mocap = new VDMocap();
  auto t1 = std::thread(&VDMocap::readOnlineData, mocap);  // 更新mocap的数据
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H1Teleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  t1.join();
  return 0;
}