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

### 3.2 Build
Open the first terminal to pull recursively all repos, re-compile, and load the gazebo environment:
```bash






cd /home/$USER/comp0244_ws/comp0244-go2
git pull --recurse-submodules
cd /home/$USER/comp0244_ws/comp0244-go2/src/FAST_LIO
git checkout -f && git checkout comp0244 && git pull
cd /home/$USER/comp0244_ws/comp0244-go2/src/waypoint_follower
git checkout -f && git checkout master && git pull
cd /home/$USER/comp0244_ws/comp0244-go2/src/local_map_creator
git checkout -f && git checkout master && git pull
cd /home/$USER/comp0244_ws/comp0244-go2/src/edge_follower
git checkout -f && git checkout master && git pull
cd /home/$USER/comp0244_ws/comp0244-go2
```

Move this folder under `/home/$USER/comp0244_ws/comp0244-go2/src/`

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

### Terminal 1: Launch Gazebo, SLAM, Waypoint Follower
```bash
xhost +
sudo docker container start comp0244_unitree
sudo docker exec -it comp0244_unitree /bin/bash
source /usr/app/comp0244_ws/comp0244-go2/install/setup.bash
cd /usr/app/comp0244_ws/comp0244-go2/scripts
ros2 launch robot_launch.launch.py
```

### Terminal 2: Publish a waypoint {x, y, theta} (w.r.t the odom frame)
```bash
sudo docker exec -it comp0244_unitree /bin/bash
source /usr/app/comp0244_ws/comp0244-go2/install/setup.bash
ros2 topic pub /waypoint geometry_msgs/Pose2D "{x: <x>, y: <y>, theta: <theta>}" -r 1
Ctrl+C
```

### Terminal 2: Run the task
```bash
ros2 launch cw1_team_3 run_solution_task_<number of task>.launch.py
```

