# COMP0244 CW1

---

## 1. License
This project is released under the [MIT License](LICENSE). 

## 2. Authors
- **Zhengtong Zhang**
- **Kai Gan**
- **RenKai Liu**

## 3. Build and Run Instructions

### 3.1 Dependencies
- Python 3.3
- ROS2 (Ubuntu 22.04)
- Any additional Python libraries

### 3.2 Build and run

```bash
xhost +
sudo docker container start comp0244_unitree
sudo docker exec -it comp0244_unitree /bin/bash
```

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-rviz2 ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools
cd /usr/app/comp0244_ws
cd comp0244-go2/src/livox_ros_driver2 && ./build.sh humble
cd /usr/app/comp0244_ws/comp0244-go2
colcon build
source install/setup.bash
```
#### Task 1:
##### Terminal 1: Launch Gazebo, SLAM, Waypoint Follower
```bash
xhost +
sudo docker container start comp0244_unitree
sudo docker exec -it comp0244_unitree /bin/bash
source /usr/app/comp0244_ws/comp0244-go2/install/setup.bash
cd /usr/app/comp0244_ws/comp0244-go2/scripts
ros2 launch robot_launch.launch.py
```

##### Terminal 2: Run the task
```bash
ros2 launch cw1_team_3 run_solution_task_<task number>.launch.py
```

### Terminal 3: 
```bash
ros2 topic pub /goal geometry_msgs/Pose2D "{x: <x>, y: <y>, theta: <theta>}" -r 1
```
this is goal point, if you want to run bug0


## 4. Working time 
- Task 1: 20hs
- Task 2: 20hs
- Task 3: 0h

## 5. Percentage
- Zhengtong Zhang 100%
- Kai Gan 100%
- Renkai Liu 100%

## One page report





