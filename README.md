# ROS 2 Programming Assignment 4 - Working with Gazebo Classic

## Build instructions

### Clone the repo
```bash
git clone git@github.com:Sounderya22/my_gazebo_tutorials.git
```

### Create a ROS workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
mv walker .
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run the launch file

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch walker turtlebot_world.launch.py
```

To disable ros bag recording

```bash
ros2 launch walker turtlebot_world.launch.py enable_recording:=false
```
