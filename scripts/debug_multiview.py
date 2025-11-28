#!/usr/bin/env python3
"""
多角度彩色诊断 - 强化地面背景 + 每个模块不同颜色
"""
import pybullet as p
import pybullet_data
import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = "/home/vickyleu/ros2_ws/src/ROS2_Walking_Pattern_Generator"
URDF_FILE = os.path.join(PROJECT_ROOT, "robot_description/models/robotis_op2/urdf/simple_walking_robot_clean.urdf")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "Records/screenshots")

# 每个部件的颜色
COLORS = {
    'body_link':    [0.9, 0.2, 0.2, 1],   # 红色 - 躯干
    'l_thigh_link': [0.2, 0.8, 0.2, 1],   # 绿色 - 左大腿
    'l_shin_link':  [0.2, 0.4, 0.9, 1],   # 蓝色 - 左小腿
    'l_foot_link':  [0.9, 0.9, 0.2, 1],   # 黄色 - 左脚
    'r_thigh_link': [0.9, 0.2, 0.9, 1],   # 紫色 - 右大腿
    'r_shin_link':  [0.2, 0.9, 0.9, 1],   # 青色 - 右小腿
    'r_foot_link':  [0.9, 0.6, 0.2, 1],   # 橙色 - 右脚
}

def create_colored_ground():
    """创建彩色格子地面"""
    # 使用默认地面，但改变颜色
    plane = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane, -1, rgbaColor=[0.3, 0.3, 0.35, 1])
    return plane

def main():
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    
    # 创建地面
    create_colored_ground()
    
    # 加载机器人
    robot = p.loadURDF(URDF_FILE, [0, 0, 0.22], useFixedBase=True)
    
    # 获取link映射并设置颜色
    link_map = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        link_name = info[12].decode('utf-8')
        link_map[i] = link_name
        if link_name in COLORS:
            p.changeVisualShape(robot, i, -1, rgbaColor=COLORS[link_name])
            print(f"Link {i}: {link_name} -> 颜色设置")
    
    # 设置站立姿态
    joints = {
        'l_hip_pitch': 0.0, 'l_knee': 0.3, 'l_ankle_pitch': 0.15,
        'r_hip_pitch': 0.0, 'r_knee': 0.3, 'r_ankle_pitch': 0.15,
    }
    for i in range(p.getNumJoints(robot)):
        name = p.getJointInfo(robot, i)[1].decode('utf-8')
        if name in joints:
            p.resetJointState(robot, i, joints[name])
    
    p.stepSimulation()
    
    # 多角度拍摄
    views = [
        ("正面", [0.4, 0, 0.2]),
        ("左侧", [0, 0.4, 0.2]),
        ("右侧", [0, -0.4, 0.2]),
        ("背面", [-0.4, 0, 0.2]),
        ("左前45°", [0.3, 0.3, 0.2]),
        ("右前45°", [0.3, -0.3, 0.2]),
        ("俯视", [0, 0, 0.6]),
        ("低角度", [0.35, 0.2, 0.08]),
    ]
    
    images = []
    target = [0, 0, 0.12]
    proj = p.computeProjectionMatrixFOV(fov=50, aspect=1, nearVal=0.01, farVal=10)
    
    for name, eye in views:
        view = p.computeViewMatrix(eye, target, [0, 0, 1])
        _, _, rgb, _, _ = p.getCameraImage(400, 400, view, proj, renderer=p.ER_TINY_RENDERER)
        img = Image.fromarray(np.array(rgb, dtype=np.uint8).reshape(400, 400, 4)[:, :, :3])
        
        # 添加视角标签
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 100, 25], fill=(0, 0, 0))
        draw.text((5, 5), name, fill=(255, 255, 255))
        
        images.append(img)
        print(f"拍摄: {name}")
    
    # 拼接 4x2 网格
    combined = Image.new('RGB', (800, 800), (40, 40, 45))
    for i, img in enumerate(images):
        x = (i % 4) * 200
        y = (i // 4) * 400
        # 缩放到200x200
        img_small = img.resize((200, 200), Image.Resampling.LANCZOS)
        combined.paste(img_small, (x, y))
    
    # 添加颜色图例
    draw = ImageDraw.Draw(combined)
    legend_y = 420
    legend_items = [
        ("红=躯干", (230, 50, 50)),
        ("绿=左大腿", (50, 200, 50)),
        ("蓝=左小腿", (50, 100, 230)),
        ("黄=左脚", (230, 230, 50)),
        ("紫=右大腿", (230, 50, 230)),
        ("青=右小腿", (50, 230, 230)),
        ("橙=右脚", (230, 150, 50)),
    ]
    for i, (text, color) in enumerate(legend_items):
        x = 20 + (i % 4) * 200
        y = legend_y + (i // 4) * 30
        draw.rectangle([x, y, x+20, y+20], fill=color)
        draw.text((x+25, y+2), text, fill=(255, 255, 255))
    
    # 保存
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output = os.path.join(OUTPUT_DIR, "debug_multiview.png")
    combined.save(output)
    print(f"\n多角度诊断图保存: {output}")
    
    # 单独保存大图（正面+侧面）
    big_combined = Image.new('RGB', (800, 400), (40, 40, 45))
    big_combined.paste(images[0], (0, 0))      # 正面
    big_combined.paste(images[4], (400, 0))    # 左前45°
    big_output = os.path.join(OUTPUT_DIR, "debug_bigview.png")
    big_combined.save(big_output)
    print(f"大图保存: {big_output}")
    
    p.disconnect()

if __name__ == "__main__":
    print("=" * 50)
    print("多角度彩色诊断")
    print("=" * 50)
    main()

