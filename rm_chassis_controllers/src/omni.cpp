//
// Created by qiayuan on 2022/7/29.
//

#include <string>
#include <Eigen/QR>

#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

#include "rm_chassis_controllers/omni.h"

namespace rm_chassis_controllers
{
bool OmniController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  XmlRpc::XmlRpcValue wheels;
  controller_nh.getParam("wheels", wheels);
  chassis2joints_.resize(wheels.size(), 3);

  size_t i = 0;
  for (const auto& wheel : wheels)
  {
    ROS_ASSERT(wheel.second.hasMember("pose"));
    ROS_ASSERT(wheel.second["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(wheel.second["pose"].size() == 3);
    ROS_ASSERT(wheel.second.hasMember("roller_angle"));
    ROS_ASSERT(wheel.second.hasMember("radius"));

    // Ref: Modern Robotics, Chapter 13.2: Omnidirectional Wheeled Mobile Robots
    //direction 是一个 1×2 的矩阵（行向量）。
    //in_wheel 是一个 2×2 的矩阵。
    //in_chassis 是一个 2×3 的矩阵。
    Eigen::MatrixXd direction(1, 2), in_wheel(2, 2), in_chassis(2, 3);
    //beta 是轮子的姿态（通常是绕垂直轴的旋转角度，单位是弧度）。
    //roller_angle 是滚轮的角度，也通常是弧度。
    double beta = (double)wheel.second["pose"][2];
    double roller_angle = (double)wheel.second["roller_angle"];
    //这里，direction 被初始化为一个行向量 [1, tan(roller_angle)]。这表示方向向量，其中第二个分量是滚轮角度的正切值。
    direction << 1, tan(roller_angle);
    //in_wheel 是一个 2×22×2 的旋转矩阵，表示一个绕 beta 角旋转的二维旋转矩阵。它将轮子的局部坐标系转换到全局坐标系。
    in_wheel << cos(beta), sin(beta), -sin(beta), cos(beta);
    //in_chassis 是一个 2×32×3 的矩阵，用来表示从底盘坐标系到轮子坐标系的变换。第一行与轮子的 y 坐标相关，第二行与 x 坐标相关。
    //其中 xpos 和 ypos 是从轮子姿态中得到的轮子坐标。
    in_chassis << -(double)wheel.second["pose"][1], 1., 0., (double)wheel.second["pose"][0], 0., 1.;
    /*这一行计算了从底盘到关节空间的变换。
    (double)wheel.second["radius"] 是通过轮子的半径进行缩放，
    通常这个缩放因子用于将线速度转换为角速度（因为轮子半径与速度成正比）。
    direction * in_wheel * in_chassis
    是对之前定义的三个矩阵进行矩阵乘法，计算出从轮子坐标系到底盘坐标系的综合变换，然后再乘以方向向量。*/
    Eigen::MatrixXd chassis2joint = 1. / (double)wheel.second["radius"] * direction * in_wheel * in_chassis;
    //最后，变换矩阵 chassis2joint 被存储到更大的矩阵 chassis2joints_ 中，具体存储在第 i 行，第 0 到第 2 列的位置。
    chassis2joints_.block<1, 3>(i, 0) = chassis2joint;

    ros::NodeHandle nh_wheel = ros::NodeHandle(controller_nh, "wheels/" + wheel.first);
    joints_.push_back(std::make_shared<effort_controllers::JointVelocityController>());
    if (!joints_.back()->init(effort_joint_interface_, nh_wheel))
      return false;
    joint_handles_.push_back(joints_[i]->joint_);

    i++;
  }
  return true;
}

void OmniController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  Eigen::Vector3d vel_chassis;
  vel_chassis << vel_cmd_.z, vel_cmd_.x, vel_cmd_.y;
  //将速度从底盘坐标系转换到关节空间。
  //chassis2joints_ 是一个 3×3 的矩阵，它将底盘坐标系的速度映射到关节空间。
  //vel_chassis 是一个 3×1 的向量，包含了 x、y 和 z 方向的速度。
  //vel_joints 是一个 3×1 的向量，包含了每个关节的速度。
  //这个计算过程使用了矩阵乘法，将底盘坐标系的速度映射到关节空间。
  Eigen::VectorXd vel_joints = chassis2joints_ * vel_chassis;
  for (size_t i = 0; i < joints_.size(); i++)
  {
    joints_[i]->setCommand(vel_joints(i));
    joints_[i]->update(time, period);
  }
}

geometry_msgs::Twist OmniController::odometry()
{
  Eigen::VectorXd vel_joints(joints_.size());
//  遍历所有关节，将每个关节的速度（通过 getVelocity() 获取）存储到 vel_joints 向量中。
  for (size_t i = 0; i < joints_.size(); i++)
    vel_joints[i] = joints_[i]->joint_.getVelocity();
  /*
    chassis2joints_ 是一个矩阵，它描述了底盘到关节的变换关系，通常是通过正向动力学计算得到的。
    该公式通过最小二乘法（普通的线性回归）计算出底盘的速度 vel_chassis。
    具体来说，底盘速度 vel_chassis 是通过以下步骤得到的：
    chassis2joints_.transpose() * chassis2joints_：
    计算矩阵 chassis2joints_ 的 Gram 矩阵（转置矩阵与自身的乘积），
    这有助于“平滑”或“反映”速度与关节速度之间的关系。
    .inverse()：取矩阵的逆，用来求解线性方程。
    chassis2joints_.transpose() * vel_joints：
    计算关节速度与底盘速度的投影。
    这样，vel_chassis 就是一个包含底盘角速度和线速度的向量。
  */
  Eigen::Vector3d vel_chassis =
      (chassis2joints_.transpose() * chassis2joints_).inverse() * chassis2joints_.transpose() * vel_joints;
  geometry_msgs::Twist twist;
  twist.angular.z = vel_chassis(0);
  twist.linear.x = vel_chassis(1);
  twist.linear.y = vel_chassis(2);
  return twist;
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::OmniController, controller_interface::ControllerBase)
