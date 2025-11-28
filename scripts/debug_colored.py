#!/usr/bin/env python3
"""
给机器人每个部件加不同颜色，诊断mesh位置问题
"""
import pybullet as p
import pybullet_data
import os
import tempfile
import numpy as np
from PIL import Image

PROJECT_ROOT = "/home/vickyleu/ros2_ws/src/ROS2_Walking_Pattern_Generator"
URDF_FILE = os.path.join(PROJECT_ROOT, "robot_description/models/robotis_op2/urdf/simple_working_robot.urdf")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "Records/screenshots")

# 每个link的颜色 (RGBA)
LINK_COLORS = {
    'body_link': [1, 0, 0, 1],       # 红色 - 躯干
    'l_thigh_link': [0, 1, 0, 1],    # 绿色 - 左大腿
    'l_shin_link': [0, 0, 1, 1],     # 蓝色 - 左小腿
    'l_foot_link': [1, 1, 0, 1],     # 黄色 - 左脚
    'r_thigh_link': [1, 0, 1, 1],    # 紫色 - 右大腿
    'r_shin_link': [0, 1, 1, 1],     # 青色 - 右小腿
    'r_foot_link': [1, 0.5, 0, 1],   # 橙色 - 右脚
}

def main():
    # 修正 URDF 路径
    with open(URDF_FILE, 'r') as f:
        content = f.read()
    content = content.replace('package://robot_description/', 
                              os.path.join(PROJECT_ROOT, 'robot_description/'))
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
        tmp.write(content)
        tmp_urdf = tmp.name
    
    # 初始化 PyBullet
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    
    robot = p.loadURDF(tmp_urdf, [0, 0, 0.35], useFixedBase=True)
    
    # 获取link名称映射
    link_name_to_idx = {-1: 'base_link'}  # base link
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        link_name = info[12].decode('utf-8')  # child link name
        link_name_to_idx[i] = link_name
        print(f"Link {i}: {link_name}")
    
    # 设置颜色
    for link_idx, link_name in link_name_to_idx.items():
        if link_name in LINK_COLORS:
            color = LINK_COLORS[link_name]
            p.changeVisualShape(robot, link_idx, -1, rgbaColor=color)
            print(f"  设置 {link_name} 颜色: {color[:3]}")
    
    # 设置关节角度（站立姿态）
    joint_angles = {
        'l_hip_pitch': 0.0,
        'l_knee': 0.3,
        'l_ankle_pitch': 0.15,
        'r_hip_pitch': 0.0,
        'r_knee': 0.3,
        'r_ankle_pitch': 0.15,
    }
    
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        name = info[1].decode('utf-8')
        if name in joint_angles:
            p.resetJointState(robot, i, joint_angles[name])
    
    p.stepSimulation()
    
    # 多角度截图
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    views = [
        ("front", [0.5, 0, 0.25], [0, 0, 0.15]),      # 正面
        ("side", [0, 0.5, 0.25], [0, 0, 0.15]),       # 侧面
        ("back_side", [0.3, -0.4, 0.3], [0, 0, 0.15]), # 斜后方（机器人面朝-Y）
        ("top", [0, 0, 0.8], [0, 0, 0.15]),           # 俯视
    ]
    
    images = []
    for name, eye, target in views:
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=eye,
            cameraTargetPosition=target,
            cameraUpVector=[0, 0, 1]
        )
        proj_matrix = p.computeProjectionMatrixFOV(fov=50, aspect=1.0, nearVal=0.01, farVal=10)
        
        _, _, rgb, _, _ = p.getCameraImage(400, 400, view_matrix, proj_matrix,
                                           renderer=p.ER_TINY_RENDERER)
        img = Image.fromarray(np.array(rgb, dtype=np.uint8).reshape(400, 400, 4)[:, :, :3])
        images.append(img)
        print(f"截图: {name}")
    
    # 拼接4个视角
    combined = Image.new('RGB', (800, 800))
    combined.paste(images[0], (0, 0))      # 左上：正面
    combined.paste(images[1], (400, 0))    # 右上：侧面
    combined.paste(images[2], (0, 400))    # 左下：斜后方
    combined.paste(images[3], (400, 400))  # 右下：俯视
    
    output_path = os.path.join(OUTPUT_DIR, "debug_colored.png")
    combined.save(output_path)
    print(f"\n彩色诊断图保存: {output_path}")
    
    # 颜色说明
    print("\n颜色说明:")
    print("  红色 = 躯干 (body)")
    print("  绿色 = 左大腿 (l_thigh)")
    print("  蓝色 = 左小腿 (l_shin)")
    print("  黄色 = 左脚 (l_foot)")
    print("  紫色 = 右大腿 (r_thigh)")
    print("  青色 = 右小腿 (r_shin)")
    print("  橙色 = 右脚 (r_foot)")
    
    p.disconnect()
    os.unlink(tmp_urdf)

if __name__ == "__main__":
    main()

