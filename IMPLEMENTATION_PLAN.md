# ROS2_Walking_Pattern_Generator 实施方案（ROS 节点）

## 目标
在 Webots 中生成稳定的双足行走关节轨迹，并导出为 ESP32 可用的动作表。

## 当前状态
- Webots 仿真链路已搭建（`webots_robot_handler`）。
- 步态管线存在：足步规划 → 步态生成 → 稳定化 → 关节状态。
- `/joint_states` 可用。

## 实施计划
### 阶段 0：仿真基线（1-2 天）
- 确认 Webots world 与模型加载正常。
- 验证 `/joint_states` 更新与 TF 正常。
- 达成稳定站立姿态。

### 阶段 1：动作表生成（3-5 天）
- 生成离散动作：
  - `stand`、`step_in_place`、`walk_forward_slow`、`turn_left_slow`、`turn_right_slow`、`stop`。
- 录制 `/joint_states` 轨迹。
- 导出为 CSV/JSON，供固件导入。

### 阶段 2：ROS → 固件接口（2-3 天）
- 新增桥接节点：
  - 输入：`/wukong/action`（String）
  - 输出：`/wukong/trajectory`（自定义或复用消息）
- 若暂不引入自定义消息，则固件仅根据 action 名称选择动作表。

### 阶段 3：可选 `cmd_vel`（后续）
- 将 `/wukong/cmd_vel` 映射为步频、步幅、转向比例。
- 轨迹仍在 ROS 侧生成，固件只做插值执行。

## ROS 接口（阶段 1）
- 现有：
  - `/joint_states`（sensor_msgs/JointState）
- 计划：
  - `/wukong/action`（std_msgs/String）
  - `/wukong/trajectory`（自定义或 action-only）

## 里程碑
- M0：单关节控制正常，`/joint_states` 正确。
- M1：稳定站立。
- M2：原地踏步不摔倒。
- M3：慢速前进（开环步态）。
- M4：动作表导出并能在固件回放。

## 验收标准
- 动作表输出可重复、确定。
- 固件至少可执行 3 个动作。

## 风险与对策
- 仿真/硬件模型不一致 → 关节限位 + 时间标定修正。
- 轨迹过大 → 采样下采样或压缩。

## 下一步决策点
- 是否定义轨迹消息格式。
- 是否引入 IMU 闭环稳定。
