> This README was created by translating the Japanese text using GitHub Copilot.
---
# robot_tools

(Writing in progress)

## model_parameter_generator.py

A Python script to extract parameters for using a new robot with this software.

### Usage

#### 1. Preparation

- Place the directory and model files in the following structure under `robot_description/models/`:

```
models/
  |-- <Model's Dirctory>/
        |-- meshes/
              |-- <Model's Mesh Files>
        |-- urdf/
              |-- <Model's Urdf File(no xacro)>
```

- Modify the code around lines 27 and 28 in `robot_tools/robot_tools/model_parameter_generator.py` as follows:

```diff
- robot_name = "rdc_robot_v1"
- urdf_name = "simple_model.urdf"
+ robot_name = "<Model's Dirctory>"
+ urdf_name = "<Model's Urdf File(no xacro)>.urdf"
```

#### 2. Execution

Run the Python script by entering the following commands:

```bash
cd robot_tools/robot_tools/
python3 model_parameter_generator.py
```

During execution, you will be prompted to name each link. Follow the displayed instructions and provide input as needed. 

Below is an example of the output (the parts marked with ">" are where you input values):

```
[INFO] List of tree in URDF file. (with fixed joints)
   tree 0 : ['CPG']
   tree 1 : ['luggage']
   tree 2 : ['r_waist_roll', 'r_waist_pitch', 'r_knee_pitch', 'r_ankle']
   tree 3 : ['l_waist_roll', 'l_waist_pitch', 'l_knee_pitch', 'l_ankle']

[INFO] Please input free joint_tree_name. 
[INFO] If nothing is inputted, joint_tree_name is 'names_group_<number>'.
[INFO] tree 0 name? > 
[INFO] tree 1 name? > 
[INFO] tree 2 name? > 
[INFO] tree 3 name? > 

name's number | joint_group_name | joint_group_value
--------------|------------------|------------------
      0       |  names_group_0   | ['CPG']
      1       |  names_group_1   | ['luggage']
      2       |  names_group_2   | ['r_waist_roll', 'r_waist_pitch', 'r_knee_pitch', 'r_ankle']
      3       |  names_group_3   | ['l_waist_roll', 'l_waist_pitch', 'l_knee_pitch', 'l_ankle']

[INFO] What's left_leg's name? (please input name's number) > 3
[INFO] What's right_leg's name? (please input name's number) > 2
[WARNING][CPG] Axis_tag is not defined.
[WARNING][luggage] Axis_tag is not defined.
[WARNING][r_ankle] Axis_tag is not defined.
[WARNING][l_ankle] Axis_tag is not defined.
```

Once the script is complete, a directory with the robot model's name will be generated under robot_description/config/. Inside, three YAML files will be created, each containing parameters.

(Writing in progress)

