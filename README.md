# ROS2 and Moveit for manipulators

This repository provides the Docker container to run ROS 2 with Moveit. It provides simple programs to move the robot in joint space and working space.

![image](https://github.com/user-attachments/assets/2f5a92f5-3bbf-4154-aa1f-27a02604aeca)


## Build

```bash

cd docker
sudo ./build.sh

```

## Run

 ```bash
 
sudo ./run.sh

colcon build

source ./install/setup.bash

```
## Start Moveit 

```bash
# workingspace: ws_moveit2
ros2 launch moveit2_tutorials demo.launch.py

```
and following the [link](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)


## Start motion application

```bash

# start your second terminal and connect to the running Docker container
cd ../colcon_ws

colcon build

source ./install/setup.bash

# move robot in joint space
ros2 run alpha_moveit joint_moveit


# move robot in working space
ros2 run alpha_moveit ik_moveit

```


## Launch Fanuc arm

![image](https://github.com/user-attachments/assets/dc2a3afb-a89b-45f0-9e1d-6ce4d26c9bb2)


```bash

cd ../colcon_ws

ros2 launch fanuc_moveit_config demo.launch.py

```

## Launch Yaskawa GP-8

![image](https://github.com/user-attachments/assets/63add41d-ed58-44b4-8246-7ed3afeb93b9)

```bash

cd ../colcon_ws

ros2 launch moveit_resources_moto_moveit_config xy_start.launch.py

```

## Launch Kuka

![image](https://github.com/user-attachments/assets/fb348660-1ba1-4d10-bac4-bfe73821b946)

```bash

ros2 launch kuka_arm_pkg p1_c_moveit_kuka_arm.launch.py

# run the launch file in order to send custom goals using Move group interface

ros2 launch kuka_arm_pkg p1_d_moveit_node_controlling.launch.py

```

Run IK using Moveit,


```bash
ros2 launch kuka_arm_pkg launch_kuka_arm.launch.py
```
One XYZ goal (working space)

```bash
ros2 run kuka_arm_pkg ik_kuka
```
Multiple XYZ goals (working space)

```bash
ros2 run kuka_arm_pkg ik_x_kuka
```

Move robot (joint space), check final position and call IK

```bash
ros2 run kuka_arm_pkg move_kuka
```

## Lanuch Reach Alpha 5


Note: run in ```colcon_ws```

```bash

ros2 launch alpha_bringup_simulation planning_alpha5.launch.py

```

---

### Run motion program (working space - IK Solver)


1. Run in terminal 1,

```bash
ros2 launch alpha_bringup_simulation planning_alpha5.launch.py

```

2. Run in terminal 2 (IK Solver),

```bash
cd /root/colcon_ws/src/py_alpha_move/py_alpha_move

python3 alpha_ik_controller_sim.py

```

3. Run in terminal 3,

```bash
ros2 topic pub /target_position std_msgs/msg/Float64MultiArray "data: [0.19, -0.07, 0.03]" -1

```

4. Move the robot in working space using keyboard,

```bash
cd /root/colcon_ws/src/py_alpha_move/py_alpha_move

python3 xyz_mapper_sim.py

```

There is possible to check performance of [QuIK](https://github.com/steffanlloyd/quik.git) for Alpha 5 but it does not produce correct joint values (due to wrong DH we passed in yaml file ?).

```bash
ros2 run quik alpha5_ros_cpp_node --ros-args --params-file /root/colcon_ws/src/quik/config/alpha5.yaml
```

publish to this node,


```bash
ros2 topic pub /target_position std_msgs/msg/Float64MultiArray "data: [0.19, -0.07, 0.03]" -1

```


---

### Run motion program (joint space)

```bash

ros2 run alpha_moveit joint_moveit

```

### Run motion (pos control)

```bash

ros2 launch alpha_bringup_simulation planning_alpha5.launch.py

```

Verify Controller is Active:

```bash
ros2 control list_controllers
```

Expected Output:
```bash
xsubsea                 joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active

```

Move robot:

```bash
ros2 topic pub /xsubsea/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names:
  - 'axis_a'
  - 'axis_b'
  - 'axis_c'
  - 'axis_d'
  - 'axis_e'
points:
  - positions: [1.0, 0.5, -0.5, 0.0, 1.5]
    time_from_start:
      sec: 2
      nanosec: 0"

```



## Recommended reads

- [How Do Robot Manipulators Move?](https://roboticseabass.com/2024/06/30/how-do-robot-manipulators-move/)
- [pyroboplan](https://github.com/sea-bass/pyroboplan)
- [panda_ign_moveit2](https://github.com/AndrejOrsula/panda_ign_moveit2/tree/humble_devel)
- [How to start with ROS 2 and Interface to Yaskawa](https://github.com/YaskawaEurope/ros2-starter-for-yaskawa-robots)
- [Reach Robotics SDK](https://reach-robotics.github.io/reach_robotics_sdk/index.html#)
- [Reach Robotics GitHub](https://github.com/Reach-Robotics)
- [ref](https://github.com/Robotisim/robotic_arms_ROS2/wiki/Project-%231:-Kuka-6DOF-Moveit2)
- [QuIK](https://github.com/steffanlloyd/quik.git)
