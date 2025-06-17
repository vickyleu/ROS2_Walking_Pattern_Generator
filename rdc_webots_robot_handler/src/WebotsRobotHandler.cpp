#include "rdc_webots_robot_handler/WebotsRobotHandler.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include <fstream>  // Logをファイルに吐くため

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rdc_webots_robot_handler
{
  void RDC_WebotsRobotHandler::JointStates_Callback(const sensor_msgs::msg::JointState::SharedPtr callback_data) {
    walking_pattern_pos_legL_ = {
      callback_data->position[5],
      callback_data->position[6],
      callback_data->position[7] 
    };
    walking_pattern_pos_legR_ = {
      callback_data->position[8],
      callback_data->position[9],
      callback_data->position[10]
    };
    walking_pattern_vel_legL_ = {
      callback_data->velocity[5],
      callback_data->velocity[6],
      callback_data->velocity[7]
    };
    walking_pattern_vel_legR_ = {
      callback_data->velocity[8],
      callback_data->velocity[9],
      callback_data->velocity[10]
    };
  }

  void RDC_WebotsRobotHandler::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters
  ) {
    node_ = node;
    (void)parameters;

    sub_joint_state_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 
      10, 
      std::bind(&RDC_WebotsRobotHandler::JointStates_Callback, this, _1)
    );
    pub_feedback_ = node_->create_publisher<robot_messages::msg::Feedback>("feedback", 10);

    for(uint8_t tag = 0; tag < 6; tag++) {
      motorsTag_[tag] = wb_robot_get_device(motors_name_[tag].c_str());
      positionSensorsTag_[tag] = wb_robot_get_device((motors_name_[tag]+"_sensor").c_str());
      wb_position_sensor_enable(positionSensorsTag_[tag], 1);  
    }

    accelerometerTag_ = wb_robot_get_device("Accelerometer");
    wb_accelerometer_enable(accelerometerTag_, 1);  // enable & sampling_period: 1[ms]
    gyroTag_ = wb_robot_get_device("Gyro");
    wb_gyro_enable(gyroTag_, 1);  // enable & sampling_period: 1[ms]
  }

  void RDC_WebotsRobotHandler::step() {

    // feedback message
    for(uint8_t tag = 0; tag < 3; tag++) {
      pub_feedback_msg_->q_now_leg_l[tag] = wb_position_sensor_get_value(positionSensorsTag_[tag]);
      pub_feedback_msg_->q_now_leg_r[tag] = wb_position_sensor_get_value(positionSensorsTag_[tag + 3]);
    }

    accelerometerValue_ = wb_accelerometer_get_values(accelerometerTag_);
    gyroValue_ = wb_gyro_get_values(gyroTag_);

    pub_feedback_msg_->accelerometer_now[0] = accelerometerValue_[0];
    pub_feedback_msg_->accelerometer_now[1] = accelerometerValue_[1];
    pub_feedback_msg_->accelerometer_now[2] = accelerometerValue_[2];
    pub_feedback_msg_->gyro_now[0] = gyroValue_[0];
    pub_feedback_msg_->gyro_now[1] = gyroValue_[1];
    pub_feedback_msg_->gyro_now[2] = gyroValue_[2];

    pub_feedback_msg_->step_count = walking_step_;

    walking_step_++;

    // add motor angle & ang-vel
    for(uint8_t tag = 0; tag < 3; tag++) {
      wb_motor_set_position(
        motorsTag_[tag], walking_pattern_pos_legL_[tag]);
      wb_motor_set_velocity(
        motorsTag_[tag], walking_pattern_vel_legL_[tag]);
      wb_motor_set_position(
        motorsTag_[tag+3], walking_pattern_pos_legR_[tag]);
      wb_motor_set_velocity(
        motorsTag_[tag+3], walking_pattern_vel_legR_[tag]);
    }

    pub_feedback_->publish(*pub_feedback_msg_);
  }
}

PLUGINLIB_EXPORT_CLASS(
  rdc_webots_robot_handler::RDC_WebotsRobotHandler,
  webots_ros2_driver::PluginInterface
)