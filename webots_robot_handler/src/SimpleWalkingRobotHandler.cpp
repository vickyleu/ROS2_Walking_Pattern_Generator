#include "webots_robot_handler/SimpleWalkingRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

namespace webots_robot_handler
{
  // 逆运动学：计算 hip_pitch, knee, ankle_pitch
  std::array<double, 3> SimpleWalkingRobotHandler::solveIK(double x, double z) {
    double L1 = thigh_length_;
    double L2 = shin_length_;
    
    double r = std::sqrt(x*x + z*z);
    
    // 限制范围
    double max_reach = L1 + L2 - 0.005;
    double min_reach = std::abs(L1 - L2) + 0.005;
    r = std::max(min_reach, std::min(r, max_reach));
    
    // 膝关节角度
    double cos_knee_inner = (L1*L1 + L2*L2 - r*r) / (2 * L1 * L2);
    cos_knee_inner = std::max(-1.0, std::min(1.0, cos_knee_inner));
    double knee_inner = std::acos(cos_knee_inner);
    double knee = knee_inner - M_PI;
    
    // 髋关节角度
    double alpha = std::atan2(x, -z);
    double sin_beta = L2 * std::sin(knee_inner) / r;
    sin_beta = std::max(-1.0, std::min(1.0, sin_beta));
    double beta = std::asin(sin_beta);
    double hip_pitch = alpha - beta;
    
    // 踝关节保持水平
    double ankle_pitch = -(hip_pitch + knee);
    
    return {hip_pitch, knee, ankle_pitch};
  }
  
  // 获取站立姿态
  std::array<double, 6> SimpleWalkingRobotHandler::getStandingAngles() {
    auto angles = solveIK(0, -standing_height_);
    return {angles[0], angles[1], angles[2], angles[0], angles[1], angles[2]};
  }
  
  // 计算步态关节角度
  std::array<double, 6> SimpleWalkingRobotHandler::computeJointAngles(double t) {
    double T = step_period_;
    double H = standing_height_;
    
    double phase = std::fmod(t, T) / T;
    int cycle = static_cast<int>(t / T);
    bool left_swing = (cycle % 2 == 0);
    
    double com_x_offset = 0.005 * std::sin(2 * M_PI * phase);
    
    double l_foot_x, l_foot_z, r_foot_x, r_foot_z;
    
    if (left_swing) {
      r_foot_x = -com_x_offset;
      r_foot_z = -H;
      l_foot_x = step_length_ * (phase - 0.5) - com_x_offset;
      l_foot_z = -H + step_height_ * std::sin(M_PI * phase);
    } else {
      l_foot_x = -com_x_offset;
      l_foot_z = -H;
      r_foot_x = step_length_ * (phase - 0.5) - com_x_offset;
      r_foot_z = -H + step_height_ * std::sin(M_PI * phase);
    }
    
    auto l_angles = solveIK(l_foot_x, l_foot_z);
    auto r_angles = solveIK(r_foot_x, r_foot_z);
    
    return {l_angles[0], l_angles[1], l_angles[2],
            r_angles[0], r_angles[1], r_angles[2]};
  }

  void SimpleWalkingRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    node_ = node;
    (void)parameters;
    
    // 创建发布器
    pub_joint_state_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    
    // Webots 中的电机名称映射
    // 根据 simple_working_robot.urdf 中的关节名称
    std::array<std::string, 6> webots_motor_names = {
      "l_hip_pitch", "l_knee", "l_ankle_pitch",
      "r_hip_pitch", "r_knee", "r_ankle_pitch"
    };
    
    // 获取电机和传感器句柄
    for (int i = 0; i < 6; i++) {
      motorsTag_[i] = wb_robot_get_device(webots_motor_names[i].c_str());
      std::string sensor_name = webots_motor_names[i] + "_sensor";
      positionSensorsTag_[i] = wb_robot_get_device(sensor_name.c_str());
      
      if (positionSensorsTag_[i] != 0) {
        wb_position_sensor_enable(positionSensorsTag_[i], 1);
      }
    }
    
    // 设置初始站立姿态
    auto standing = getStandingAngles();
    for (int i = 0; i < 6; i++) {
      if (motorsTag_[i] != 0) {
        double angle = standing[i] * joint_sign_[i];
        wb_motor_set_position(motorsTag_[i], angle);
        wb_motor_set_velocity(motorsTag_[i], 1.0);
      }
    }
    
    RCLCPP_INFO(node_->get_logger(), "SimpleWalkingRobotHandler 初始化完成");
  }

  void SimpleWalkingRobotHandler::step() {
    // 等待初始姿态稳定
    if (wait_steps_ > 0) {
      wait_steps_--;
      return;
    }
    
    // 计算当前关节角度
    auto angles = computeJointAngles(sim_time_);
    
    // 发布关节状态
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    msg->header.stamp = node_->get_clock()->now();
    
    for (int i = 0; i < 6; i++) {
      msg->name.push_back(joint_names_[i]);
      msg->position.push_back(angles[i]);
      msg->velocity.push_back(0.0);
      
      // 设置电机位置
      if (motorsTag_[i] != 0) {
        double angle = angles[i] * joint_sign_[i];
        wb_motor_set_position(motorsTag_[i], angle);
        wb_motor_set_velocity(motorsTag_[i], 3.0);
      }
    }
    
    pub_joint_state_->publish(*msg);
    
    // 更新时间 (假设 step 周期为 10ms)
    sim_time_ += 0.01;
  }
}

PLUGINLIB_EXPORT_CLASS(
  webots_robot_handler::SimpleWalkingRobotHandler,
  webots_ros2_driver::PluginInterface
)

