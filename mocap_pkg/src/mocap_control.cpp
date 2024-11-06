
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>

#include "mocap/readCSV.hpp"
#include "mocap/listen2mocap_v1.hpp"
#include "mocap/sphere2hinge.hpp"

#include "mocap/mocap.hpp"
#include "h1_arm/mocap_control.hpp"

#include <mutex>

std::queue<std::vector<float>> QueHand;
std::queue<std::vector<float>> QueArm;

std::mutex mtx_lock_hand, mtx_lock_arm;

H1Teleop::H1Teleop() : Node("h1_node")
{ 

  // pub
  pub_arm_ = this->create_publisher<unitree_go::msg::MotorCmds>("body/cmd", 10);
  pub_hand_ = this->create_publisher<unitree_go::msg::MotorCmds>("hand/cmd", 10);
  pub_head_ = this->create_publisher<unitree_go::msg::MotorCmds>("head/cmd", 10);
  
  timer_hand_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&H1Teleop::hand_timer_callback, this));
  time_arm_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&H1Teleop::arm_timer_callback, this));

}
void H1Teleop::hand_timer_callback()
{
  std::vector<float> hand_q(12, 0.);
  if(QueHand.empty())
  { 
    return;
  }
  
  hand_q = QueHand.front();
  mtx_lock_hand.lock();
  QueHand.pop();
  mtx_lock_hand.unlock();

  unitree_go::msg::MotorCmds handCmdsMsg;
  unitree_go::msg::MotorCmd handCmd;

  for(int i = 0; i < 12; ++i)
  {
    handCmd.kp = 60;
    handCmd.kd = 0.5;
    handCmd.q = hand_q[i];
    handCmdsMsg.cmds.push_back(handCmd);
    std::cout << hand_q[i] << "  ";
  }
  std::cout << std::endl;

  pub_hand_->publish(handCmdsMsg);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
};

void H1Teleop::arm_timer_callback()
{  
    std::vector<float> arm_q(20, 0.);
    if(QueArm.empty())
    { 
      return;
    }
    arm_q = QueHand.front();
    mtx_lock_arm.lock();
    QueHand.pop();
    mtx_lock_arm.unlock();

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
    pub_arm_->publish(bodyCmdsMsg);
   
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
};


void handleData()
{ 
  std::cout << "get_data" << std::endl;
  while (rclcpp::ok())
  { 
    auto upJoints = mocap->getJoints();
    std::cout << upJoints.size() << std::endl;

    std::vector<float>hand_q(12, 0.);
    // qcmd(4, 5) -> upJoints[8, 9, 10, 11]
    hand_q[10] = 1 -  upJoints[11] / 1.2;   
    hand_q[11] = 1 -  upJoints[12] / 0.8;
    //  qcmd[0, 1, 2, 3] -> upJoints[20, 18, 16, 14],  // little -> index    
    for(int i = 0; i < 4; i++)
    { 
      hand_q[i + 6] =1 - upJoints[20 - i * 2] / 1.7; 
    }
    // 2.2 right_finger
    // qcmd[10, 11] -> upJoints[22, 23, 24, 25]
    hand_q[4] = 1 -  upJoints[24] / 1.2;   
    hand_q[5] = 1 -  upJoints[25] / 0.6;   
    // qcmd[6, 7, 8, 9] -> upJoints[33, 31, 29, 27]
    for(int i = 0; i < 4; i++)
    { 
      hand_q[i] = 1 - upJoints[33 - i * 2] / 1.7; 
    }

    mtx_lock_hand.lock();
    QueHand.emplace(hand_q);
    mtx_lock_hand.unlock();

    std::vector<float>arm_q(20, 0.);
    for(int i = 0; i < 8; ++i)
    {
      arm_q[i+12] = upJoints[i]; 
    }
    mtx_lock_arm.lock();
    QueArm.emplace(arm_q);
    mtx_lock_arm.unlock();
    std::cout << QueHand.size() << std::endl;
  }
}

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
  auto t2 = std::thread(handleData);
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H1Teleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  t1.join();
  t2.join();
  
  return 0;
}