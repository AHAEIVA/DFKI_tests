# DFKI Tests

## Process

### 1. Open Terminator  
Run the ROS core:

```bash
roscore
```

---

### 2. Run `rosbridge`

```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

---

### 3. Run Motion Capture Node

Navigate to your `mocap_ws`:

```bash
cd ~/mocap_ws
source devel/setup.bash
roslaunch mocap_qualisys qualisys.launch server_address:="192.168.0.154" udp_port:=-1
```

**Note**: Make sure you're connected via Ethernet cable to the Qualisys computer.

---


### Run Simulation vs Real Robot 
real robot
```bash
roslaunch bluerov2_nmpc bluerov2_nmpc.launch
```
simulation
```bash
roslaunch bluerov2_nmpc bluerov2_nmpc.launch odom_source:=mobula
```




### 4. Run Controller and Trajectory Generation

Navigate to your `catkin_ws`:

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch bluerov2_nmpc bluerov2_nmpc.launch
roslaunch bluerov2_trajectory bluerov2_trajectory.launch
```

### Recording Data

```bash
rosrun topic_tools throttle messages /my_topic /my_topic_throttled 5.0
rosbag record /my_topic_throttled
```


## TODO
-get mocap brdige node and controller inside tether_ws
- get bluerov2 MPC to work with tether_planner 
- test planner (no prop)

### Pipe dimensions
2m length X 0.7 m diameter
