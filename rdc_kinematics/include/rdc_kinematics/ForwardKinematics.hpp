#ifndef RDC_FORWARD_KINEMATICS_HPP
#define RDC_FORWARD_KINEMATICS_HPP

#include "rclcpp/rclcpp.hpp"
#include "robot_manager/control_plugin_bases/PluginBase_ForwardKinematics.hpp"

#include "Eigen/Dense"

namespace rdc_kinematics
{
  class RDC_ForwardKinematics : public control_plugin_base::ForwardKinematics {
    public:
      RDC_ForwardKinematics(){}
      ~RDC_ForwardKinematics(){}
      void forward_kinematics(
        std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
        Eigen::Vector3d& end_eff_pos_ptr
      ) override;
      void forward_kinematics(
        std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
        const int joint_point,
        Eigen::Vector3d& end_eff_pos_ptr
      ) override;

      std::array<Eigen::Matrix3d, 3> getR_leg(
        std::array<double, 3> Q_leg
      );

    private:
      Eigen::Matrix3d Rx(double rad = 0);
      Eigen::Matrix3d Ry(double rad = 0);
      Eigen::Matrix4d Tx(double rad = 0, const Eigen::Vector3d& pos);  // 同次変換行列
      Eigen::Matrix4d Ty(double rad = 0, const Eigen::Vector3d& pos);
  };
}

#endif // RDC_FORWARD_KINEMATICS_HPP