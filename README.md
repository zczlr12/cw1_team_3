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


# MIT License

Copyright (c) 2025 Renkai Liu, Kai Gan, Zhengtong Zhang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
