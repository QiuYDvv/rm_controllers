//
// Created by yezi on 2022/11/15.
//

#pragma once

#include <rm_common/lqr.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_msgs/BalanceState.h>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
    BLOCK
  };

public:
  BalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  geometry_msgs::Twist odometry() override;
  static const int STATE_DIM = 10;
  static const int CONTROL_DIM = 4;
  /*
  k_：一个 CONTROL_DIM × STATE_DIM 的增益矩阵，可能是控制增益矩阵（如 LQR 的反馈增益）。
  a_：一个 STATE_DIM × STATE_DIM 的系统矩阵，通常用于描述系统的状态转换，即 xk+1=Axk+Bukxk+1​=Axk​+Buk​。
  q_：一个 STATE_DIM × STATE_DIM 的权重矩阵，通常用于优化（如 LQR 控制中的状态权重矩阵）。
  b_：一个 STATE_DIM × CONTROL_DIM 的输入矩阵，描述输入 uu 如何影响系统状态变化。
  r_：一个 CONTROL_DIM × CONTROL_DIM 的权重矩阵，可能是 LQR 中的控制输入权重矩阵。
  x_：一个 STATE_DIM × 1 的状态向量，表示系统当前的状态。
   */
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;
  double wheel_radius_, wheel_base_;
  double position_des_ = 0;
  double position_offset_ = 0.;
  double position_clear_threshold_ = 0.;
  double yaw_des_ = 0;

  int balance_mode_;
  ros::Time block_time_, last_block_time_;
  double block_angle_, block_duration_, block_velocity_, block_effort_, anti_block_effort_, block_overtime_;
  bool balance_state_changed_ = false, maybe_block_ = false;

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_,
      left_momentum_block_joint_handle_, right_momentum_block_joint_handle_;

  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::BalanceState>> RtpublisherPtr;
  RtpublisherPtr state_pub_;
  geometry_msgs::Vector3 angular_vel_base_;
  double roll_, pitch_, yaw_;
};

}  // namespace rm_chassis_controllers
