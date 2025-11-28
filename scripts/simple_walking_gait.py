#!/usr/bin/env python3
"""
simple_walking_robot 步态生成与仿真
- 3DOF 腿逆运动学
- 简化步态生成
- PyBullet 仿真
"""
import os
import sys
import math
import time
import tempfile
import numpy as np

# ============== 机器人参数 ==============
class RobotParams:
    """机器人物理参数（来自 URDF）"""
    MASS = 3.0  # kg
    THIGH_LENGTH = 0.093  # m (大腿)
    SHIN_LENGTH = 0.093   # m (小腿)
    HIP_OFFSET_Y = 0.037  # m (髋关节横向偏移)
    HIP_OFFSET_Z = 0.0907 # m (髋关节垂直偏移)
    
    # 步态参数
    STANDING_HEIGHT = 0.16  # m (站立时髋关节到脚底的距离)
    STEP_LENGTH = 0.02      # m (步长)
    STEP_HEIGHT = 0.015     # m (抬脚高度)
    STEP_PERIOD = 1.0       # s (单步周期)

# ============== 逆运动学 ==============
class InverseKinematics3DOF:
    """
    3自由度腿逆运动学
    计算 hip_pitch, knee, ankle_pitch 使脚达到目标位置
    """
    def __init__(self, L1=RobotParams.THIGH_LENGTH, L2=RobotParams.SHIN_LENGTH):
        self.L1 = L1
        self.L2 = L2
    
    def solve(self, x, z):
        """
        输入: 脚相对于髋关节的位置 (x=前向, z=向下为负)
        输出: [hip_pitch, knee, ankle_pitch]
        """
        # 目标距离
        r = math.sqrt(x*x + z*z)
        
        # 限制在可达范围内
        max_reach = self.L1 + self.L2 - 0.005
        min_reach = abs(self.L1 - self.L2) + 0.005
        r = np.clip(r, min_reach, max_reach)
        
        # 余弦定理求膝关节角度
        # knee_angle = pi - (L1和L2之间的夹角)
        cos_knee_inner = (self.L1**2 + self.L2**2 - r**2) / (2 * self.L1 * self.L2)
        cos_knee_inner = np.clip(cos_knee_inner, -1, 1)
        knee_inner = math.acos(cos_knee_inner)  # L1-L2夹角
        knee = knee_inner - math.pi  # 膝关节弯曲角度（负值）
        
        # 求髋关节角度
        # alpha: 目标点相对于髋关节的方向角
        alpha = math.atan2(x, -z)  # x前向，-z向下
        
        # beta: 大腿相对于髋-脚连线的偏角
        sin_beta = self.L2 * math.sin(knee_inner) / r
        sin_beta = np.clip(sin_beta, -1, 1)
        beta = math.asin(sin_beta)
        
        hip_pitch = alpha - beta
        
        # 踝关节保持脚底水平
        ankle_pitch = -(hip_pitch + knee)
        
        return np.array([hip_pitch, knee, ankle_pitch])

# ============== 步态生成器 ==============
class SimpleWalkingGait:
    """简化步态生成器"""
    
    def __init__(self):
        self.params = RobotParams()
        self.ik = InverseKinematics3DOF()
        
    def compute_joint_angles(self, t):
        """
        计算给定时刻的所有关节角度
        返回: [l_hip, l_knee, l_ankle, r_hip, r_knee, r_ankle]
        """
        T = self.params.STEP_PERIOD
        H = self.params.STANDING_HEIGHT
        step_len = self.params.STEP_LENGTH
        step_h = self.params.STEP_HEIGHT
        
        # 归一化时间
        phase = (t % T) / T  # 0 ~ 1
        
        # 判断哪条腿在摆动
        cycle = int(t / T)
        left_swing = (cycle % 2 == 0)
        
        # 重心前后摆动（简化：小幅度正弦）
        com_x_offset = 0.005 * math.sin(2 * math.pi * phase)
        
        # 计算两腿的脚位置
        if left_swing:
            # 左腿摆动，右腿支撑
            # 右脚（支撑）
            r_foot_x = -com_x_offset
            r_foot_z = -H
            
            # 左脚（摆动）- 正弦轨迹
            swing_phase = phase
            l_foot_x = step_len * (swing_phase - 0.5) - com_x_offset
            l_foot_z = -H + step_h * math.sin(math.pi * swing_phase)
        else:
            # 右腿摆动，左腿支撑
            l_foot_x = -com_x_offset
            l_foot_z = -H
            
            swing_phase = phase
            r_foot_x = step_len * (swing_phase - 0.5) - com_x_offset
            r_foot_z = -H + step_h * math.sin(math.pi * swing_phase)
        
        # 逆运动学
        l_angles = self.ik.solve(l_foot_x, l_foot_z)
        r_angles = self.ik.solve(r_foot_x, r_foot_z)
        
        return np.concatenate([l_angles, r_angles])
    
    def get_standing_angles(self):
        """获取站立姿态的关节角度"""
        H = self.params.STANDING_HEIGHT
        angles = self.ik.solve(0, -H)
        return np.concatenate([angles, angles])

# ============== PyBullet 仿真 ==============
def run_simulation():
    """运行 PyBullet 仿真"""
    import pybullet as p
    import pybullet_data
    
    PROJECT_ROOT = "/home/vickyleu/ros2_ws/src/ROS2_Walking_Pattern_Generator"
    URDF_FILE = os.path.join(PROJECT_ROOT, "robot_description/models/robotis_op2/urdf/simple_working_robot.urdf")
    OUTPUT_DIR = os.path.join(PROJECT_ROOT, "Records/screenshots")
    
    # 读取并修改 URDF 路径
    with open(URDF_FILE, 'r') as f:
        urdf_content = f.read()
    urdf_content = urdf_content.replace(
        'package://robot_description/',
        os.path.join(PROJECT_ROOT, 'robot_description/') + '/'
    )
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
        tmp.write(urdf_content)
        tmp_urdf = tmp.name
    
    # 初始化 PyBullet
    try:
        physicsClient = p.connect(p.GUI)
        gui_mode = True
        print("PyBullet GUI 模式启动")
    except:
        physicsClient = p.connect(p.DIRECT)
        gui_mode = False
        print("PyBullet DIRECT 模式")
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # 加载地面
    planeId = p.loadURDF("plane.urdf")
    p.changeDynamics(planeId, -1, lateralFriction=1.0)
    
    # 加载机器人
    startPos = [0, 0, 0.27]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF(tmp_urdf, startPos, startOrientation, useFixedBase=True)
    
    print(f"机器人加载成功，ID: {robotId}")
    
    # 获取关节信息
    joint_map = {}  # name -> index
    for i in range(p.getNumJoints(robotId)):
        info = p.getJointInfo(robotId, i)
        name = info[1].decode('utf-8')
        if info[2] != p.JOINT_FIXED:
            joint_map[name] = i
            print(f"  关节 {i}: {name}")
    
    # 初始化步态生成器
    gait = SimpleWalkingGait()
    
    # 关节名称和对应的符号（适配 URDF 中的轴定义）
    # URDF 中：l_hip_pitch axis=(0,-1,0), l_knee axis=(0,-1,0), l_ankle_pitch axis=(0,1,0)
    #          r_hip_pitch axis=(0,-1,0), r_knee axis=(0,-1,0), r_ankle_pitch axis=(0,1,0)
    joint_config = [
        ('l_hip_pitch', -1),   # 需要取反
        ('l_knee', -1),        # 需要取反
        ('l_ankle_pitch', 1),  # 不取反
        ('r_hip_pitch', -1),   # 需要取反
        ('r_knee', -1),        # 需要取反
        ('r_ankle_pitch', 1),  # 不取反
    ]
    
    # 设置初始站立姿态
    print("\n设置站立姿态...")
    standing_angles = gait.get_standing_angles()
    print(f"站立角度: {np.degrees(standing_angles)}")
    
    for i, (name, sign) in enumerate(joint_config):
        if name in joint_map:
            idx = joint_map[name]
            angle = standing_angles[i] * sign
            p.resetJointState(robotId, idx, angle)
            p.setJointMotorControl2(robotId, idx, p.POSITION_CONTROL,
                                    targetPosition=angle, force=10.0, maxVelocity=2.0)
    
    # 稳定
    for _ in range(200):
        p.stepSimulation()
    
    # 设置相机
    if gui_mode:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=60,
            cameraPitch=-20,
            cameraTargetPosition=[0, 0, 0.15]
        )
    
    # 仿真循环
    print("\n开始步态仿真...")
    sim_time = 0.0
    dt = 1/240
    total_time = 4.0
    
    frames = []
    
    while sim_time < total_time:
        # 计算关节角度
        joint_angles = gait.compute_joint_angles(sim_time)
        
        # 应用到仿真
        for i, (name, sign) in enumerate(joint_config):
            if name in joint_map:
                idx = joint_map[name]
                angle = joint_angles[i] * sign
                p.setJointMotorControl2(robotId, idx, p.POSITION_CONTROL,
                                        targetPosition=angle, force=10.0, maxVelocity=3.0)
        
        p.stepSimulation()
        sim_time += dt
        
        if gui_mode:
            time.sleep(dt * 0.5)  # 稍微加速播放
    
    print("仿真完成!")
    
    # 保存截图
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[0.5, 0.4, 0.3],
        cameraTargetPosition=[0, 0, 0.15],
        cameraUpVector=[0, 0, 1]
    )
    proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.01, farVal=10)
    
    width, height = 800, 800
    _, _, rgb, _, _ = p.getCameraImage(width, height, viewMatrix=view_matrix,
                                        projectionMatrix=proj_matrix,
                                        renderer=p.ER_TINY_RENDERER)
    
    from PIL import Image
    img = Image.fromarray(np.array(rgb, dtype=np.uint8).reshape(height, width, 4)[:, :, :3])
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output_path = os.path.join(OUTPUT_DIR, "simple_walking_gait.png")
    img.save(output_path)
    print(f"截图保存: {output_path}")
    
    p.disconnect()
    os.unlink(tmp_urdf)
    
    return output_path

if __name__ == "__main__":
    print("=" * 60)
    print("Simple Walking Robot 步态仿真")
    print("=" * 60)
    output = run_simulation()
    print(f"\n完成！截图: {output}")
