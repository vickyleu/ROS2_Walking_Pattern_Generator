#ifndef RDC_WEBOTS_ROBOT_HANDLER_HPP
#define RDC_WEBOTS_ROBOT_HANDLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_messages/msg/feedback.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace rdc_webots_robot_handler
{
  class RDC_WebotsRobotHandler : public webots_ros2_driver::PluginInterface {
    
    public:
      void init(
        webots_ros2_driver::WebotsNode *node,
        std::unordered_map<std::string, std::string> &parameters
      ) override;

      void step() override;

    private:

      void JointStates_Callback(const sensor_msgs::msg::JointState::SharedPtr callback_data);

      webots_ros2_driver::WebotsNode *node_;

      rclcpp::Publisher<robot_messages::msg::Feedback>::SharedPtr pub_feedback_;
      std::shared_ptr<robot_messages::msg::Feedback> pub_feedback_msg_ = std::make_shared<robot_messages::msg::Feedback>();
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

      WbDeviceTag motorsTag_[6];
      WbDeviceTag positionSensorsTag_[6];
      WbDeviceTag gyroTag_;
      WbDeviceTag accelerometerTag_;

      std::array<double, 3> walking_pattern_pos_legR_ = {0, 0, 0};
      std::array<double, 3> walking_pattern_vel_legR_ = {0, 0, 0};
      std::array<double, 3> walking_pattern_pos_legL_ = {0, 0, 0};
      std::array<double, 3> walking_pattern_vel_legL_ = {0, 0, 0};

      double getJointAng_[6];
      const double *accelerometerValue_ = 0;
      const double *gyroValue_ = 0;
      uint32_t wait_step_ = 0;
      uint32_t walking_step_ = 0;

      std::array<std::string, 6> motors_name_ = {
        "l_waist_roll", "l_waist_pitch", "l_knee_pitch", 
        "r_waist_roll", "r_waist_pitch", "r_knee_pitch"
      };
  };
}

#endif