#ifndef RDC_JACOBIAN_HPP
#define RDC_JACOBIAN_HPP


#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_Jacobian.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ForwardKinematics.hpp"

#include "Eigen/Dense"


namespace rdc_kinematics
{
  pluginlib::ClassLoader<control_plugin_base::ForwardKinematics> fk_loader("robot_manager", "control_plugin_base::ForwardKinematics");

  class RDC_Jacobian : public control_plugin_base::Jacobian
  {
    public:
      RDC_Jacobian();
      ~RDC_Jacobian(){}

      void jacobian(
        const std::shared_ptr<control_plugin_base::LegStates_ToJac> leg_states_jac_ptr,
        Eigen::Matrix<double, 3, 3>& leg_jacobian
      ) override;

    private:
      std::shared_ptr<control_plugin_base::ForwardKinematics> fk_;
  };
}


#endif