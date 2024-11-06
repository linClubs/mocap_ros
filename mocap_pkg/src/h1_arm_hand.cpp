
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
  
  // inspire hand config
  serial1_ = std::make_shared<SerialPort>("/dev/ttyUSB0", B115200);
  lefthand = std::make_shared<inspire::InspireHand>(serial1_, 2); // ID 2
  righthand = std::make_shared<inspire::InspireHand>(serial2_, 1); // ID 2


  // pub
  pub_arm_ = this->create_publisher<unitree_go::msg::MotorCmds>("body/cmd", 10);
  // pub_hand_ = this->create_publisher<unitree_go::msg::MotorCmds>("hand/cmd", 10);
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
    
    qcmd(4) = 1.2 -  upJoints[11] / 1.2;   
    qcmd(5) = 0.5 -  upJoints[12] / 0.8;   
    //  qcmd[0, 1, 2, 3] -> upJoints[20, 18, 16, 14],  // little -> index    
    for(int i = 0; i < 4; i++)
    { 
      qcmd(i) =1 - upJoints[20 - i * 2] / 1.7; 
    }

    // 2.2 right_finger
    // qcmd[10, 11] -> upJoints[22, 23, 24, 25]
    qcmd(10) = 1 -  upJoints[24] / 1.2;   
    qcmd(11) = 1 -  upJoints[25] / 0.8;   
    // qcmd[6, 7, 8, 9] -> upJoints[33, 31, 29, 27]
    for(int i = 0; i < 4; i++)
    { 
      qcmd(6 + i) =1 - upJoints[33 - i * 2] / 1.7; 
    }
    
      std::cout << "finger  : ";
    for(int i = 0; i < 12; ++i)
    {
      std::cout << qcmd(i) << "  ";
    }
    std::cout << std::endl;

    // 2.3 publish finger signal 
    lefthand->SetPosition(qcmd.block<6, 1>(0, 0));
    righthand->SetPosition(qcmd.block<6, 1>(6, 0));

    // 3 arm data process
    // 3.1 update arm data
    for(int i = 0; i < 8; ++i)
    {
      next_arm_q_[i+12] = upJoints[i]; 
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

  //  std::cout << "arm  : ";
    for (int j = 12; j < 20; ++j) 
    {
      bodyCmdsMsg.cmds[arm_joints.at(j-12)].q = next_arm_q_.at(j) ;
      // std::cout << next_arm_q_.at(j) << "  ";
    }
    // std::cout << std::endl;

    // 3.3 publish arm control signal
    // pub_arm_->publish(bodyCmdsMsg);
   
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