#ifndef SIMPLE_WALKING_ROBOT_HANDLER_HPP
#define SIMPLE_WALKING_ROBOT_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace webots_robot_handler
{
  class SimpleWalkingRobotHandler : public webots_ros2_driver::PluginInterface {
    public:
      void init(
        webots_ros2_driver::WebotsNode *node, 
        std::unordered_map<std::string, std::string> &parameters
      ) override;

      void step() override;

    private:
      webots_ros2_driver::WebotsNode *node_;
      
      // 发布关节状态
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
      
      // 关节名称 (6个: 每腿3个)
      std::array<std::string, 6> joint_names_ = {
        "l_hip_pitch", "l_knee", "l_ankle_pitch",
        "r_hip_pitch", "r_knee", "r_ankle_pitch"
      };
      
      // Webots 设备标签
      WbDeviceTag motorsTag_[6];
      WbDeviceTag positionSensorsTag_[6];
      
      // 关节方向系数 (适配 URDF 轴定义)
      std::array<int, 6> joint_sign_ = {-1, -1, 1, -1, -1, 1};
      
      // 步态参数
      double standing_height_ = 0.16;  // m
      double thigh_length_ = 0.093;    // m
      double shin_length_ = 0.093;     // m
      double step_length_ = 0.02;      // m
      double step_height_ = 0.015;     // m
      double step_period_ = 1.0;       // s
      
      // 仿真状态
      double sim_time_ = 0.0;
      int wait_steps_ = 200;
      
      // 逆运动学
      std::array<double, 3> solveIK(double x, double z);
      
      // 步态生成
      std::array<double, 6> computeJointAngles(double t);
      std::array<double, 6> getStandingAngles();
  };
}

#endif

