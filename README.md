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
Open the docker
```bash
xhost +
sudo docker container start comp0244_unitree
sudo docker exec -it comp0244_unitree /bin/bash
```
Build the package
```bash
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-rviz2 ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools
cd /usr/app/comp0244_ws
cd comp0244-go2/src/livox_ros_driver2 && ./build.sh humble
cd /usr/app/comp0244_ws/comp0244-go2
colcon build
source install/setup.bash
```
#### Task 1: Obstacle Follower
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
cd /usr/app/comp0244_ws/comp0244-go2
source install/setup.bash
ros2 launch cw1_team_3 run_solution_task_1.launch.py
```

#### Task 2: Bug0

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
cd /usr/app/comp0244_ws/comp0244-go2
source install/setup.bash
ros2 launch cw1_team_3 run_solution_task_2.launch.py
```

### Terminal 3: Set the goal point
```bash
ros2 topic pub /goal geometry_msgs/Pose2D "{x: <x>, y: <y>, theta: <theta>}" -r 1
```
This is goal point you set.

#### Task 3: Bug1
##### Terminal 1: Launch Gazebo, SLAM, Waypoint Follower
```bash
xhost +
sudo docker container start comp0244_unitree
sudo docker exec -it comp0244_unitree /bin/bash
source /usr/app/comp0244_ws/comp0244-go2/install/setup.bash
cd /usr/app/comp0244_ws/comp0244-go2/scripts
ros2 launch robot_launch.launch.py
```

#### Terminal 2: Run the task
```bash
cd /usr/app/comp0244_ws/comp0244-go2
source install/setup.bash
ros2 launch cw1_team_3 run_solution_task_3.launch.py
```
### Terminal 3: Set the goal point
```bash
ros2 topic pub /goal geometry_msgs/Pose2D "{x: <x>, y: <y>, theta: <theta>}" -r 1
```
This is goal point you set.


## 4. Working time Summary

| Task  | Renkai Liu (hrs)    | Kai Gan (hrs)   | Zhengtong Zhang (hrs)  | Total (hrs) |
|-------|------------------|------------------|------------------|------------|
| Task 1 | 35       | 15      | 25          | 95         |
| Task 2 | 30     | 20       | 20     |      70    |
| Task 3 | 5       | 30           | 30      | 65        |
| **Total** | **70**  | **80**  | **75**  | **230**     |

## 5. Percentage
-- Renkai Liu -- 35%
-- Kai Gan -- 32.5%
-- Zhengtong Zhang -- 32.5% 

## 6. One page report
### Task 1: Obstacle Follower
In this task, we need to consider many aspects. We have made modifications to the edge follower in the source code. The original code finds the nearest point on the edge to the robot through projection, then moves forward along the edge direction. It switches to the next edge if it goes beyond the current edge. The direction is clockwise. And it is only suitable for circles because of the dot product of the edge direction and the robot direction.
Therefore, based on this step, we modified the direction of cross_z to allow counterclockwise rotation. Additionally, we introduced Exponential Smoothing to make the robot dog's movement more stable. To enable the robot to adapt to more obstacles, we added rectangular and concave obstacles. When the robot reaches a corner, it adds an overshoot to the next waypoint, allowing it to detect the edge on the other side. If the robot gets too close to the edge, it returns to the previous waypoint. This algorithm is applicable to both concave and rectangular obstacles.

### Task 2: Bug0

### Task 3: Bug1










