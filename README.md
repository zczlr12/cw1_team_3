# COMP0244 CW1

---

## 1. License
This project is released under the [MIT License](LICENSE). 
DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2025 Zhengtong Zhang, Kai Gan, Renkai Liu except where specified

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


## 4. Team Contribution Summary

| Task  | Renkai Liu (hrs)    | Kai Gan (hrs)   | Zhengtong Zhang (hrs)  | Total (hrs) |
|-------|------------------|------------------|------------------|------------|
| Task 1 | 35       | 15      | 25          | 95         |
| Task 2 | 30     | 20       | 20     |      70    |
| Task 3 | 5       | 30           | 30      | 65        |
| **Total** | **70**  | **80**  | **75**  | **230**     |


## One page report





