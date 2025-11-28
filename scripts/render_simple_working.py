#!/usr/bin/env python3
"""渲染 simple_working_robot.urdf 效果"""
import os
import re
import tempfile

# 项目路径
PROJECT_ROOT = "/home/vickyleu/ros2_ws/src/ROS2_Walking_Pattern_Generator"
URDF_FILE = os.path.join(PROJECT_ROOT, "robot_description/models/robotis_op2/urdf/simple_working_robot.urdf")
OUTPUT_IMG = os.path.join(PROJECT_ROOT, "Records/screenshots/simple_working_robot.png")

# 读取 URDF 并替换 package:// 路径为绝对路径
with open(URDF_FILE, 'r') as f:
    urdf_content = f.read()

# 替换 package://robot_description/ 为实际路径
urdf_content = urdf_content.replace(
    'package://robot_description/',
    os.path.join(PROJECT_ROOT, 'robot_description/') + '/'
)

# 写入临时文件
with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
    tmp.write(urdf_content)
    tmp_urdf = tmp.name

print(f"临时 URDF: {tmp_urdf}")

# PyBullet 渲染
import pybullet as p
import pybullet_data

# 使用 DIRECT 模式（无窗口）
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载地面和机器人
p.loadURDF("plane.urdf")
robot_id = p.loadURDF(tmp_urdf, [0, 0, 0.3], useFixedBase=True)

print(f"机器人加载成功，ID: {robot_id}")
print(f"关节数量: {p.getNumJoints(robot_id)}")

# 列出所有关节
for i in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"  关节 {i}: {joint_info[1].decode('utf-8')} ({joint_info[12].decode('utf-8')})")

# 设置相机视角（正面稍偏）
view_matrix = p.computeViewMatrix(
    cameraEyePosition=[0.5, 0.5, 0.4],
    cameraTargetPosition=[0, 0, 0.15],
    cameraUpVector=[0, 0, 1]
)
proj_matrix = p.computeProjectionMatrixFOV(
    fov=60,
    aspect=1.0,
    nearVal=0.01,
    farVal=10
)

# 渲染图像
width, height = 800, 800
_, _, rgb, _, _ = p.getCameraImage(
    width, height,
    viewMatrix=view_matrix,
    projectionMatrix=proj_matrix,
    renderer=p.ER_TINY_RENDERER
)

# 保存图像
from PIL import Image
import numpy as np

img = Image.fromarray(np.array(rgb, dtype=np.uint8).reshape(height, width, 4)[:, :, :3])
os.makedirs(os.path.dirname(OUTPUT_IMG), exist_ok=True)
img.save(OUTPUT_IMG)
print(f"\n截图已保存: {OUTPUT_IMG}")

# 清理
p.disconnect()
os.unlink(tmp_urdf)

