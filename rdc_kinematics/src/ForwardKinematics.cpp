#include "rdc_kinematics/ForwardKinematics.hpp"

using namespace Eigen;

namespace rdc_kinematics
{
  Matrix3d RDC_ForwardKinematics::Rx(double rad = 0) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d RDC_ForwardKinematics::Ry(double rad = 0) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }
  Matrix4d RDC_ForwardKinematics::Tx(double rad = 0, const Eigen::Vector3d& pos) {
    Matrix4d T_x;
    T_x << 1,       0,          0, pos[0],
           0, cos(rad), -sin(rad), pos[1],
           0, sin(rad),  cos(rad), pos[2],
           0,       0,          0, 1;
    return(T_x);
  }
  Matrix4d RDC_ForwardKinematics::Ty(double rad = 0, const Eigen::Vector3d& pos) {
    Matrix4d T_y;
    T_y <<  cos(rad), 0, sin(rad), pos[0],
                   0, 1,        0, pos[1],
           -sin(rad), 0, cos(rad), pos[2],
                   0, 0,        0, 1;
    return(T_y);
  }

  std::array<Matrix3d, 3> RDC_ForwardKinematics::getR_leg(std::array<double, 3> Q_leg) {
    return {Rx(Q_leg[0]), Ry(Q_leg[1]), Ry(Q_leg[2])};
  }

  void RDC_ForwardKinematics::forward_kinematics(
    std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
    Eigen::Vector3d& end_eff_pos_ptr
  ) {
    forward_kinematics(
      leg_states_ptr,
      3,
      end_eff_pos_ptr
    );
  }

  void RDC_ForwardKinematics::forward_kinematics(
    std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
    int joint_point,
    Eigen::Vector3d& end_eff_pos_ptr
  ) {
    Matrix4d T_wl = RDC_ForwardKinematics::Tx(
      leg_states_ptr->joint_ang[0],
      leg_states_ptr->link_len[0]
    );
    Matrix4d T_wp = RDC_ForwardKinematics::Ty(
      leg_states_ptr->joint_ang[1],
      leg_states_ptr->link_len[1]
    );
    Matrix4d T_kp = RDC_ForwardKinematics::Ty(
      leg_states_ptr->joint_ang[2],
      leg_states_ptr->link_len[2]
    );

    switch(joint_point) {
      case 0:
        end_eff_pos_ptr = leg_states_ptr->link_len[0];
        break;
      case 1:
        end_eff_pos_ptr = Vector4d(T_wl.dot(
                            Vector4d(
                              leg_states_ptr->link_len[1][0],
                              leg_states_ptr->link_len[1][1],
                              leg_states_ptr->link_len[1][2], 
                              1.0
                            ))).segment(0, 2);
        break;
      case 2:
        end_eff_pos_ptr = Vector4d(T_wl.dot(
                            Vector4d(T_wp.dot(
                              Vector4d(
                                leg_states_ptr->link_len[2][0],
                                leg_states_ptr->link_len[2][1],
                                leg_states_ptr->link_len[2][2], 
                                1.0
                              ))))).segment(0, 2);
        break;
      case 3:
        end_eff_pos_ptr = Vector4d(T_wl.dot(
                            Vector4d(T_wp.dot(
                              Vector4d(T_kp.dot(
                                Vector4d(
                                  leg_states_ptr->link_len[3][0],
                                  leg_states_ptr->link_len[3][1],
                                  leg_states_ptr->link_len[3][2], 
                                  1.0
                                ))))))).segment(0, 2);
        break;
    }

  }

}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rdc_kinematics::RDC_ForwardKinematics, control_plugin_base::ForwardKinematics)