#include "foot_step_planner/FootStepPlanner.hpp"

namespace foot_step_planner
{
  std::unique_ptr<control_plugin_base::FootStep> Default_FootStepPlanner::foot_step_planner(void) {
    auto foot_step_ptr = std::make_unique<control_plugin_base::FootStep>();
    
    foot_step_ptr->walking_step_time = WALKING_CYCLE_;  // 歩行周期[s]
    
    if (motion_mode_ == "stand") {
      foot_step_ptr->foot_pos = {
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}
      };
    } else if (motion_mode_ == "step_in_place") {
      // local
      foot_step_ptr->foot_pos = {
        {0.0, 0.0},
        {0.0, 0.037},
        {0.03, -0.037},
        {0.03, 0.037},
        {0.03, -0.037},
        {0.03, 0.037},
        {0.0, 0.0},
        {0.0, 0.0}
      };
    } else {
      RCLCPP_WARN(node_ptr_->get_logger(), "Unknown motion_mode '%s', falling back to step_in_place.", motion_mode_.c_str());
      foot_step_ptr->foot_pos = {
        {0.0, 0.0},
        {0.0, 0.037},
        {0.03, -0.037},
        {0.03, 0.037},
        {0.03, -0.037},
        {0.03, 0.037},
        {0.0, 0.0},
        {0.0, 0.0}
      };
    }
    
    foot_step_ptr->waist_height = WAIST_HEIGHT_;
    //foot_step_ptr->waist_height = 171.856 / 1000;  // 腰高さ

    // std::cout << "Here is default foot_step_controller plugin." << std::endl;

    return foot_step_ptr;
  }

  Default_FootStepPlanner::Default_FootStepPlanner() {
    node_ptr_ = rclcpp::Node::make_shared("FootStepPlanner");
    client_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_, "RobotParameterServer");

    WALKING_CYCLE_ = client_param_->get_parameter<double>("control_times.walking_cycle");
    WAIST_HEIGHT_ = client_param_->get_parameter<double>("control_constant.waist_pos_z");
    motion_mode_ = client_param_->get_parameter<std::string>("mode_switch.motion_mode");

    RCLCPP_INFO(node_ptr_->get_logger(), "Start Up FootStepPlanner (motion_mode=%s).", motion_mode_.c_str());
  }
}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(foot_step_planner::Default_FootStepPlanner, control_plugin_base::FootStepPlanner)
