import pybullet as p
import pybullet_data
import time
import math

# 连接 PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# 加载机器人
urdf_path = "robot_description/models/robotis_op2/urdf/simple_working_robot.urdf"
robot = p.loadURDF(urdf_path, [0, 0, 0.5], useFixedBase=True)

# 添加调试滑块
sliders = []
joint_map = {}
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    name = info[1].decode('utf-8')
    if info[2] != p.JOINT_FIXED:
        sid = p.addUserDebugParameter(name, -3.14, 3.14, 0)
        sliders.append((sid, i))
        joint_map[name] = i

# 主循环
while True:
    for sid, jid in sliders:
        val = p.readUserDebugParameter(sid)
        p.resetJointState(robot, jid, val)
    time.sleep(0.01)

