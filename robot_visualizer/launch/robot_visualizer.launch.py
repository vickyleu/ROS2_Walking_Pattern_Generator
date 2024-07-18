# Visualizer launch file
import os
from time import sleep
import yaml

import launch
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  launch_description = launch.LaunchDescription()

  debug_mode_yaml_path = os.path.join(get_package_share_directory("robot_bringup"), "config", "param_debug_mode.yaml")
  with open(debug_mode_yaml_path, "r") as f:
    debug_mode_yaml = yaml.safe_load(f)["/**"]["ros__parameters"]["setting_debug_mode"]

# Rviz2のlaunch
  if debug_mode_yaml["using_rviz"] == True:
    rviz_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("robot_visualizer"), "launch"),
        "/rviz.launch.py"
      ])
    )
    launch_description.add_action(rviz_launch)

# Loggerとの違いは、リアルタイムに表示をするだけ　｜　記録を取る

# TODO: リアルタイムのモニタリングNodeとかの実行（作れれば。rqt、imgui,etc?）
# ros関連を詳しく出してくれるtopを実行するのも良さげ？
  # これ：https://github.com/iwatake2222/rotop
    # TODO: 動作確認と、pythonソースから実行できるか確認（コマンドラインでも、pythonでも）

  # RQT Plotのlaunch
  if debug_mode_yaml["using_rqt"] == True:
    rqt_plot_launch = Node(
      package = "rqt_plot",
      executable = "rqt_plot",
      output = "screen",
      arguments = [
        "/joint_states/position[9]",  # DEBUG: ここをDEBUG用のPublish Topicを指定すれば良い．んで，これをparameterから指定するようにすれば良い．
      ]
    )
    rqt_plot_launch_delay = TimerAction(
      period = 1.0, 
      actions = [rqt_plot_launch]
    )
    launch_description.add_action(rqt_plot_launch_delay)

  # sleep(4)
  return launch_description