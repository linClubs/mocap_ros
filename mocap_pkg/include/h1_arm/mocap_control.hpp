
#include <iomanip>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/motor_cmds.hpp"

#include "inspire/inspire.h"


class H1Teleop : public rclcpp::Node 
{
public : 

    H1Teleop();
    ~H1Teleop(){};
    void arm_timer_callback();
    void hand_timer_callback();

private:
  
    rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr pub_arm_;
    rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr pub_head_;
    rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr pub_hand_;
    // 手部处理
    Eigen::Matrix<double, 12, 1> qcmd;
    rclcpp::TimerBase::SharedPtr timer_hand_, time_arm_;

  enum JointIndex 
  {
    kLeftHipYaw = 7,
    kLeftHipRoll = 3,
    kLeftHipPitch = 4,
    kLeftKnee = 5,
    kLeftAnkle = 10,

    kRightHipYaw = 8,
    kRightHipRoll = 0,
    kRightHipPitch = 1,
    kRightKnee = 2,
    kRightAnkle = 11,

    kWaistYaw = 6,
    kNotUsedJoint = 9,    // 11 

    kRightShoulderPitch = 12,
    kRightShoulderRoll = 13,
    kRightShoulderYaw = 14,
    kRightElbow = 15,

    kLeftShoulderPitch = 16,
    kLeftShoulderRoll = 17,
    kLeftShoulderYaw = 18,
    kLeftElbow = 19,

    kHeadRotate = 20,  //
    kHead2 = 21,
    kHead3 = 22,
    kHead4 = 23,

    // right wrist & fingers
    kRightWrist         = 24,
    kRightPink          = 25,
    kRightRing          = 26,
    kRightMiddle        = 27,
    kRightIndex         = 28,
    kRightThumbBend     = 29,
    kRightThumbRotation = 30,

    // left wrist & fingers
    kLeftWrist          = 31,
    kLeftPink           = 32,
    kLeftRing           = 33,
    kLeftMiddle         = 34,
    kLeftIndex          = 35,
    kLeftThumbBend      = 36,
    kLeftThumbRotation  = 37,
  };

  const std::vector<int>  arm_joints = {
    JointIndex::kLeftShoulderPitch, JointIndex::kLeftShoulderRoll,
    JointIndex::kLeftShoulderYaw, JointIndex::kLeftElbow,
    JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
    JointIndex::kRightShoulderYaw, JointIndex::kRightElbow,
    };
  
  const std::vector<int> control_joints = {
    JointIndex::kLeftShoulderPitch, JointIndex::kLeftShoulderRoll,
    JointIndex::kLeftShoulderYaw, JointIndex::kLeftElbow,
    JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
    JointIndex::kRightShoulderYaw, JointIndex::kRightElbow,

    JointIndex::kLeftThumbBend, JointIndex::kLeftThumbRotation,
    JointIndex::kLeftIndex, JointIndex::kLeftMiddle,
    JointIndex::kLeftRing, JointIndex::kLeftPink,

    JointIndex::kRightThumbBend, JointIndex::kRightThumbRotation,
    JointIndex::kRightIndex, JointIndex::kRightMiddle,
    JointIndex::kRightRing, JointIndex::kRightPink,
  };

  const std::vector<int> hand_joints_ = {
    JointIndex::kLeftThumbBend, JointIndex::kLeftThumbRotation,
    JointIndex::kLeftIndex, JointIndex::kLeftMiddle,
    JointIndex::kLeftRing, JointIndex::kLeftPink,

    JointIndex::kRightThumbBend, JointIndex::kRightThumbRotation,
    JointIndex::kRightIndex, JointIndex::kRightMiddle,
    JointIndex::kRightRing, JointIndex::kRightPink,
  };

  const std::vector<int> head_joints_ = {
    JointIndex::kHeadRotate,  
    JointIndex::kHead2,
    JointIndex::kHead3,
    JointIndex::kHead4,
  };
};