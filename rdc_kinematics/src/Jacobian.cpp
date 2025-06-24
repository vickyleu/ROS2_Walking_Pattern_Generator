#include "rdc_kinematics/Jacobian.hpp"

using namespace Eigen;

namespace rdc_kinematics
{
  void RDC_Jacobian::jacobian(
    const std::shared_ptr<control_plugin_base::LegStates_ToJac> leg_states_jac_ptr,
    Eigen::Matrix<double, 6, 3>& leg_jacobian_ptr
  ) {
    leg_jacobian_ptr = MatrixXd::Zero(6, 3);

    std::array<Eigen::Vector3d, 3> P_FK_leg;
    Vector3d end_eff_pos = {0, 0, 0};
    std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_fk_ptr = std::make_shared<control_plugin_base::LegStates_ToFK>();
    leg_states_fk_ptr->joint_ang = leg_states_jac_ptr->joint_ang;
    leg_states_fk_ptr->link_len = leg_states_jac_ptr->link_len;
    for(int joint_point = 0; joint_point < 3; joint_point++) {
      fk_->forward_kinematics(leg_states_fk_ptr, joint_point, end_eff_pos);
      P_FK_leg[joint_point] = end_eff_pos;
    }

    Vector3d mat_leg = Vector3d::Zero(3);
    Vector3d pt_P_leg = Vector3d::Zero(3);
    for(int tag = 0; tag < 3; tag++) {
      if(tag == 2) {
        mat_leg = Vector3d::Zero(3);
      }
      else { 
        pt_P_leg = P_FK_leg[2] - P_FK_leg[tag];
        mat_leg = leg_states_jac_ptr->unit_vec[tag].cross(pt_P_leg);
      }

      for(int i = 0; i < 3; i++) {
        leg_jacobian_ptr(i, tag) = mat_leg[i];
        leg_jacobian_ptr(i+3, tag) = leg_states_jac_ptr->unit_vec[tag][i];
      }
    }
  }

  RDC_Jacobian::RDC_Jacobian() {
    fk_ = fk_loader.createSharedInstance("rdc_kinematics::RDC_ForwardKinematics");
  }
}
   

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rdc_kinematics::RDC_Jacobian, control_plugin_base::Jacobian)