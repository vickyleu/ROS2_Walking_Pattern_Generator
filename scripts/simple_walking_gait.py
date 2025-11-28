#!/usr/bin/env python3
"""
simple_walking_robot 步态仿真 - 简化版
直接用关节角度控制，不用复杂IK
"""
import os
import math
import time
import tempfile
import numpy as np
from PIL import Image

PROJECT_ROOT = "/home/vickyleu/ros2_ws/src/ROS2_Walking_Pattern_Generator"
# 使用干净的几何体版本URDF
URDF_FILE = os.path.join(PROJECT_ROOT, "robot_description/models/robotis_op2/urdf/simple_walking_robot_clean.urdf")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "Records/screenshots")

def run():
    import pybullet as p
    import pybullet_data
    
    # 初始化
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    
    robot = p.loadURDF(URDF_FILE, [0, 0, 0.25], useFixedBase=True)
    
    # 关节索引
    joints = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        name = info[1].decode('utf-8')
        if info[2] != p.JOINT_FIXED:
            joints[name] = i
            print(f"关节 {i}: {name}")
    
    # 简单的行走关节角度序列（手工调试）
    # 每行: [l_hip, l_knee, l_ankle, r_hip, r_knee, r_ankle]
    # 角度单位：弧度
    # URDF定义: hip axis=(0,-1,0), knee axis=(0,-1,0), ankle axis=(0,1,0)
    # 正值hip=后摆, 正值knee=弯曲, 正值ankle=上勾
    
    walk_sequence = []
    n_frames = 40
    
    for i in range(n_frames):
        phase = i / n_frames * 2 * math.pi  # 0 到 2π
        
        # 站立姿态的基础角度（正常站立）
        # hip正=后摆, knee正=弯曲, ankle正=上勾
        base_hip = 0.0
        base_knee = 0.25   # 微微弯膝
        base_ankle = 0.12  # 补偿让脚掌水平
        
        # 行走幅度
        hip_amp = 0.25
        knee_amp = 0.4
        lift_phase_offset = 0.3  # 抬腿时膝盖多弯
        
        # 左腿
        l_hip = base_hip + hip_amp * math.sin(phase)
        # 抬腿时膝盖弯更多
        l_knee = base_knee + knee_amp * max(0, math.sin(phase))
        l_ankle = base_ankle + (l_hip + l_knee) * 0.3  # 补偿让脚掌水平
        
        # 右腿（相位差180度）
        r_hip = base_hip + hip_amp * math.sin(phase + math.pi)
        r_knee = base_knee + knee_amp * max(0, math.sin(phase + math.pi))
        r_ankle = base_ankle + (r_hip + r_knee) * 0.3
        
        walk_sequence.append([l_hip, l_knee, l_ankle, r_hip, r_knee, r_ankle])
    
    # 设置相机
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=45,
        cameraPitch=-15,
        cameraTargetPosition=[0, 0, 0.15]
    )
    
    # 录制帧 - 相机从-Y方向看（机器人面朝-Y）
    frames = []
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[0.3, -0.5, 0.3],  # 从-Y方向观察
        cameraTargetPosition=[0, 0, 0.15],
        cameraUpVector=[0, 0, 1]
    )
    proj_matrix = p.computeProjectionMatrixFOV(fov=50, aspect=1.0, nearVal=0.01, farVal=10)
    
    joint_names = ['l_hip_pitch', 'l_knee', 'l_ankle_pitch', 
                   'r_hip_pitch', 'r_knee', 'r_ankle_pitch']
    
    print("\n开始录制...")
    for frame_idx, angles in enumerate(walk_sequence):
        # 设置关节角度
        for j, name in enumerate(joint_names):
            if name in joints:
                p.resetJointState(robot, joints[name], angles[j])
        
        p.stepSimulation()
        
        # 截图
        _, _, rgb, _, _ = p.getCameraImage(400, 400, view_matrix, proj_matrix, 
                                           renderer=p.ER_TINY_RENDERER)
        img = Image.fromarray(np.array(rgb, dtype=np.uint8).reshape(400, 400, 4)[:, :, :3])
        frames.append(img)
        
        time.sleep(0.02)
    
    print(f"录制完成，共 {len(frames)} 帧")
    
    # 保存 GIF
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    gif_path = os.path.join(OUTPUT_DIR, "simple_walking_gait.gif")
    frames[0].save(gif_path, save_all=True, append_images=frames[1:], 
                   duration=50, loop=0)
    print(f"GIF 保存: {gif_path}")
    
    # 保存关键帧拼接图
    key_indices = [0, 10, 20, 30, 39]
    key_frames = [frames[i] for i in key_indices]
    combined = Image.new('RGB', (400 * len(key_frames), 400))
    for i, img in enumerate(key_frames):
        combined.paste(img, (i * 400, 0))
    png_path = os.path.join(OUTPUT_DIR, "simple_walking_gait.png")
    combined.save(png_path)
    print(f"PNG 保存: {png_path}")
    
    p.disconnect()

if __name__ == "__main__":
    print("=" * 50)
    print("Simple Walking Robot 步态仿真")
    print("=" * 50)
    run()
