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
    STANDING_HEIGHT = 0.19   # m (站立时髋关节到脚底的距离)
    STEP_LENGTH = 0.08       # m (步长)
    STEP_HEIGHT = 0.04       # m (抬脚高度)
    STEP_PERIOD = 0.9        # s (单步周期)
    DUTY_FACTOR = 0.6        # 支撑相占比 (>0.5)

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
        
        坐标系: x向前, z向下为负
        关节正方向: 绕Y轴，向前踢腿为正
        """
        # z应该是负值（脚在髋关节下方）
        # 转换为正值计算
        leg_length = -z  # 腿的垂直长度（正值）
        forward = x       # 前向偏移
        
        # 计算髋关节到脚的距离
        r = math.sqrt(forward**2 + leg_length**2)
        
        # 限制在可达范围内
        max_reach = self.L1 + self.L2 - 0.002
        min_reach = abs(self.L1 - self.L2) + 0.002
        if r > max_reach:
            r = max_reach
        if r < min_reach:
            r = min_reach
        
        # 余弦定理求膝关节角度
        cos_knee = (self.L1**2 + self.L2**2 - r**2) / (2 * self.L1 * self.L2)
        cos_knee = np.clip(cos_knee, -1, 1)
        knee = -(math.pi - math.acos(cos_knee))  # 膝关节弯曲（负值表示向后弯）
        
        # 髋关节角度
        # 使用几何关系
        cos_alpha = (self.L1**2 + r**2 - self.L2**2) / (2 * self.L1 * r)
        cos_alpha = np.clip(cos_alpha, -1, 1)
        alpha = math.acos(cos_alpha)  # 大腿与髋-脚连线的夹角
        
        # 髋-脚连线与垂直方向的夹角
        gamma = math.atan2(forward, leg_length)
        
        # 髋关节角度 = gamma - alpha（向前为正）
        hip_pitch = gamma - alpha
        
        # 踝关节保持脚底水平
        ankle_pitch = -(hip_pitch + knee)
        
        return np.array([hip_pitch, knee, ankle_pitch])

# ============== 步态生成器 ==============
class SimpleWalkingGait:
    """简化步态生成器"""
    
    def __init__(self):
        self.params = RobotParams()
        self.ik = InverseKinematics3DOF()
    
    def _foot_pose(self, leg_phase: float):
        """
        根据腿相位（0~1）计算脚底位置 (x, z)
        - [0, swing_portion) : 摆动期（脚抬起并从后向前摆）
        - [swing_portion, 1) : 支撑期（脚贴地向后滑动）
        """
        H = self.params.STANDING_HEIGHT
        step_len = self.params.STEP_LENGTH
        step_h = self.params.STEP_HEIGHT
        duty = self.params.DUTY_FACTOR
        swing_portion = max(0.05, 1.0 - duty)  # 防止除 0
        
        if leg_phase < swing_portion:
            # 摆动
            s = leg_phase / swing_portion  # 0~1
            x = -step_len / 2 + step_len * s
            z = -H + step_h * math.sin(math.pi * s)
        else:
            # 支撑
            s = (leg_phase - swing_portion) / duty  # 0~1
            x = step_len / 2 - step_len * s
            z = -H
        
        return x, z
        
    def compute_joint_angles(self, t):
        """
        计算给定时刻的所有关节角度
        返回: [l_hip, l_knee, l_ankle, r_hip, r_knee, r_ankle]
        """
        T = self.params.STEP_PERIOD
        phase = (t % T) / T  # 0 ~ 1
        
        # 左右腿相位错开 180°
        left_phase = phase
        right_phase = (phase + 0.5) % 1.0
        
        l_x, l_z = self._foot_pose(left_phase)
        r_x, r_z = self._foot_pose(right_phase)
        
        l_angles = self.ik.solve(l_x, l_z)
        r_angles = self.ik.solve(r_x, r_z)
        
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
    # URDF: l_hip_pitch axis=(0,-1,0) -> 正=后摆
    #       l_knee axis=(0,-1,0)      -> 正=后摆(弯曲)
    #       l_ankle_pitch axis=(0,1,0)-> 正=上勾
    #
    # IK 输出定义:
    #   hip: 正=前摆
    #   knee: 负=弯曲(后摆)
    #   ankle: 正=上勾(抵消hip+knee)
    #
    # 映射关系:
    #   hip: IK(正) -> URDF(负) => -1
    #   knee: IK(负) -> URDF(正) => -1
    #   ankle: IK(正) -> URDF(正) => 1
    joint_config = [
        ('l_hip_pitch', -1),
        ('l_knee', -1),
        ('l_ankle_pitch', 1),
        ('r_hip_pitch', -1),
        ('r_knee', -1),
        ('r_ankle_pitch', 1),
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
    
    # 用于生成多帧截图
    frames = []
    frame_times = []
    capture_interval = 0.1  # 每0.1秒截一帧
    last_capture = -capture_interval
    
    # 相机设置
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[0.4, 0.3, 0.25],
        cameraTargetPosition=[0, 0, 0.12],
        cameraUpVector=[0, 0, 1]
    )
    proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.01, farVal=10)
    
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
        
        # 每隔一段时间截一帧
        if sim_time - last_capture >= capture_interval:
            _, _, rgb, _, _ = p.getCameraImage(400, 400, viewMatrix=view_matrix,
                                                projectionMatrix=proj_matrix,
                                                renderer=p.ER_TINY_RENDERER)
            frames.append(rgb)
            frame_times.append(sim_time)
            last_capture = sim_time
        
        sim_time += dt
        
        if gui_mode:
            time.sleep(dt * 0.3)
    
    print(f"仿真完成! 共捕获 {len(frames)} 帧")
    
    # 保存多帧截图
    from PIL import Image
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # 转换所有帧为 PIL Image
    images = []
    for rgb in frames:
        img = Image.fromarray(np.array(rgb, dtype=np.uint8).reshape(400, 400, 4)[:, :, :3])
        images.append(img)
    
    # 生成 GIF 动画
    gif_path = os.path.join(OUTPUT_DIR, "simple_walking_gait.gif")
    if len(images) > 1:
        images[0].save(gif_path, save_all=True, append_images=images[1:], 
                       duration=100, loop=0)  # 100ms per frame
        print(f"GIF 动画保存: {gif_path}")
    
    # 生成帧拼接图 (选取关键帧)
    key_frame_indices = [0, len(images)//4, len(images)//2, 3*len(images)//4, len(images)-1]
    key_frames = [images[i] for i in key_frame_indices if i < len(images)]
    
    # 横向拼接
    total_width = 400 * len(key_frames)
    combined = Image.new('RGB', (total_width, 400))
    for i, img in enumerate(key_frames):
        combined.paste(img, (i * 400, 0))
    
    output_path = os.path.join(OUTPUT_DIR, "simple_walking_gait.png")
    combined.save(output_path)
    print(f"帧拼接图保存: {output_path} ({len(key_frames)} 帧)")
    
    p.disconnect()
    os.unlink(tmp_urdf)
    
    return output_path

if __name__ == "__main__":
    print("=" * 60)
    print("Simple Walking Robot 步态仿真")
    print("=" * 60)
    output = run_simulation()
    print(f"\n完成！截图: {output}")
