#ifndef ROBOT_MANAGER_PLUGIN_BASE_FORWARD_KINEMATICS_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_FORWARD_KINEMTAICS_HPP

#include "Eigen/Dense"

// TODO: control_plugin_baseではなく、kinematics_plugin_baseとして、別pkgとして管理するべきかも
namespace control_plugin_base 
{
  // TODO: 型は全て一箇所にまとめたい。ライブラリとか。
  struct LegStates_ToFK {
    std::array<Eigen::Vector3d, 7> link_len;
    std::array<double, 6> joint_ang;
  };

  class ForwardKinematics {
    public:
      virtual void forward_kinematics(
        std::shared_ptr<LegStates_ToFK> leg_states_ptr,
        Eigen::Vector3d& end_eff_pos_ptr
      ) = 0;
      virtual void forward_kinematics(  // overload. jacobian計算時に用いる、始端の関節から特定の関節（joint_point）までの順運動学計算
        std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
        const int joint_point,
        Eigen::Vector3d& end_eff_pos_ptr
      ) = 0;
      virtual void forward_kinematics_3dof(
        std::shared_ptr<LegStates_ToFK> leg_states_ptr,
        Eigen::Vector3d& end_eff_pos_ptr
      ) = 0;
      virtual void forward_kinematics_3dof(  // 仮想関数は、全継承先クラスで実装する必要があるらしい 間違いー＞（2か所（default, rdc）からoverrideするとエラーを吐くっぽい？多分、どちらの定義を使えばよいか分からなくなるから？
        std::shared_ptr<control_plugin_base::LegStates_ToFK> leg_states_ptr,
        const int joint_point,
        Eigen::Vector3d& end_eff_pos_ptr
      ) = 0;
      virtual ~ForwardKinematics(){}
    
    protected:
      ForwardKinematics(){}
  };
}

#endif