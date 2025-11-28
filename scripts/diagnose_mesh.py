import pybullet as p
import pybullet_data
import os
import math
from PIL import Image
import numpy as np

# 路径
PROJECT_ROOT = "/home/vickyleu/ros2_ws/src/ROS2_Walking_Pattern_Generator"
URDF_FILE = os.path.join(PROJECT_ROOT, "robot_description/models/robotis_op2/urdf/simple_working_robot.urdf")
OUTPUT_IMG = os.path.join(PROJECT_ROOT, "Records/screenshots/debug_mesh.png")

# 修正 URDF 路径
with open(URDF_FILE, 'r') as f:
    content = f.read()
content = content.replace('package://robot_description/', os.path.join(PROJECT_ROOT, 'robot_description/'))
import tempfile
with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
    tmp.write(content)
    tmp_urdf = tmp.name

# 启动 PyBullet
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot = p.loadURDF(tmp_urdf, [0, 0, 0.5], useFixedBase=True)

# 摆一个特定的 Pose 来暴露问题
# 左腿弯曲，右腿直立
joint_vals = {
    'l_hip_pitch': 0.0,
    'l_knee': -0.5,  # 弯曲左膝，看小腿 mesh 是否跟随且位置正确
    'l_ankle_pitch': 0.5,
    'r_hip_pitch': 0.0,
    'r_knee': 0.0,
    'r_ankle_pitch': 0.0
}

for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    name = info[1].decode('utf-8')
    if name in joint_vals:
        p.resetJointState(robot, i, joint_vals[name])

p.stepSimulation()

# 截图
view_matrix = p.computeViewMatrix(
    cameraEyePosition=[0.6, 0.6, 0.5],
    cameraTargetPosition=[0, 0, 0.2],
    cameraUpVector=[0, 0, 1]
)
proj_matrix = p.computeProjectionMatrixFOV(fov=50, aspect=1.0, nearVal=0.01, farVal=10)
w, h, rgb, _, _ = p.getCameraImage(600, 600, view_matrix, proj_matrix, renderer=p.ER_TINY_RENDERER)

# 保存
img = Image.fromarray(np.array(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3])
os.makedirs(os.path.dirname(OUTPUT_IMG), exist_ok=True)
img.save(OUTPUT_IMG)
print(f"Debug 截图已保存: {OUTPUT_IMG}")

p.disconnect()
os.unlink(tmp_urdf)

