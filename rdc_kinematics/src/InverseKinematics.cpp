#include "rdc_kinematics/InverseKinematics.hpp"

using namespace Eigen;

namespace rdc_kinematics
{
  Vector3d RDC_InverseKinematics::Array2Vector(std::array<double, 3> array) {
    return {array[0], array[1], array[2]};
  }
  Matrix3d RDC_InverseKinematics::Array2Matrix(std::array<double, 9> array) {
    Matrix3d R;
    R << array[0], array[1], array[2],
         array[3], array[4], array[5],
         array[6], array[7], array[8];
    return R;
  }

  Matrix3d RDC_InverseKinematics::Rx(double rad) {
    Matrix3d R_x;
    R_x << 1,        0,         0,
           0, cos(rad), -sin(rad),
           0, sin(rad),  cos(rad);
    return(R_x);
  }
  Matrix3d RDC_InverseKinematics::Ry(double rad) {
    Matrix3d R_y;
    R_y <<  cos(rad), 0, sin(rad),
                   0, 1,        0,
           -sin(rad), 0, cos(rad);
    return(R_y);
  }

  void RDC_InverseKinematics::inverse_kinematics(
    const std::shared_ptr<control_plugin_base::LegStates_ToIK> leg_states_ptr,
    std::array<double, 3>& joint_ang_ptr
  ) {
    // IK (RDC's Leg only)
    (void)leg_states_ptr;
    (void)joint_ang_ptr;

  }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rdc_kinematics::RDC_InverseKinematics, control_plugin_base::InverseKinematics)