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


## Recommended reads

- [How Do Robot Manipulators Move?](https://roboticseabass.com/2024/06/30/how-do-robot-manipulators-move/)
- [pyroboplan](https://github.com/sea-bass/pyroboplan)
- [panda_ign_moveit2](https://github.com/AndrejOrsula/panda_ign_moveit2/tree/humble_devel)
- [How to start with ROS 2 and Interface to Yaskawa](https://github.com/YaskawaEurope/ros2-starter-for-yaskawa-robots)
