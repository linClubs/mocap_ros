
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
  // serial1_ = std::make_shared<SerialPort>("/dev/ttyUSB0", B115200);
  // lefthand = std::make_shared<inspire::InspireHand>(serial1_, 2); // ID 2
  // righthand = std::make_shared<inspire::InspireHand>(serial2_, 1); // ID 2


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
{   
    // 34, 8 + 1(left_hand) + 12(left_fingers) + 1(right_hand) + 12(right_fingers)
    // upJoints[[8], [21]] = 0, upJoints[9->20](left_thumb 9->12, left_index->little 13->20) 
    // std::cout << "arm:  " ;
    auto upJoints = mocap->getJoints();
    // for (int i = 0; i < 8; ++i)
    // {
    //   std::cout << upJoints[i] << "  ";
    // }
    
    // std::cout << std::endl;
    // 手指部分 更新qcmd
    // lefthand->SetPosition(qcmd.block<6, 1>(0, 0));
    // righthand->SetPosition(qcmd.block<6, 1>(6, 0));

    // 手臂处理
    for(int i = 0; i < 8; ++i)
    {
      next_arm_q_[i+12] = upJoints[i]; 
    }
    
    // if(first_frame_)
    // {
    //   first_frame_ = false;
    //   curr_arm_q_ = next_arm_q_;
    //   curr_head_q_ = next_head_q_;
    //   return;
    // }
    // 4 手臂unitree消息 20维
    unitree_go::msg::MotorCmds bodyCmdsMsg;
    unitree_go::msg::MotorCmd bodyCmd;
    // 4.1 身体的20个关节赋值kp和kd, initialize 20个关节, 12-19总共8个kp=60，kd = 1.5，id=9的qpos=1
    for (int i = 0; i < 20; ++i) 
    {
        bodyCmd.kp = (i >= 12 && i <20) ? 60 : 0.0;
        bodyCmd.kd = (i >= 12 && i <20) ? 1.5 : 0.0;
        bodyCmdsMsg.cmds.push_back(bodyCmd);
    }
    bodyCmdsMsg.cmds[9].q = 1.0;

    // 5.2 arm手臂数据插值 mean.at(i)每次插值数值，插值数
    // std::vector<float>diff(20, 0.0);
    // int INSERT_NUM = 1
    // ;
    // for (int i = 12; i <= 19; ++i) 
    // {
    //     diff.at(i) = (next_arm_q_.at(i) - curr_arm_q_.at(i)) / INSERT_NUM;
    // }

    std::cout << "arm  : ";
    // for(int i = 0; i < INSERT_NUM; ++i)
    { 
      // arm_joints只有8个数字，从left开始
      for (int j = 12; j < 20; ++j) 
      {
        // bodyCmdsMsg.cmds[arm_joints.at(j-12)].q = curr_arm_q_.at(j) + diff.at(j);  // 未填充字段保持之前动作 
        bodyCmdsMsg.cmds[arm_joints.at(j-12)].q = next_arm_q_.at(j) ;
        std::cout << next_arm_q_.at(j) << "  ";
      }
      std::cout << std::endl;
      pub_arm_->publish(bodyCmdsMsg);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }
    // curr_arm_q_ = next_arm_q_;
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