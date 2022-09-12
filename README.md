# bayu_offboard_mavros

## Description
This is a ROS package written in C++ which implements a position controller and sends normalized thrust setpoint and attitude setpoint to PX4 through MAVROS.

It has a node called `bayu_offboard_node` which will perform a flight example by going through three predefined setpoints in `OFFBOARD` mode. In this mode, the node will perform its own position control and send throttle and attitude setpoints to PX4 through MAVROS. For convenience, it has a launch file that runs MAVROS node and `bayu_offboard_node` complete with MAVROS configuration `px4_config.yaml` and `px4_pluginlists.yaml`. However, this package does not contain MAVROS so it is mandatory to install MAVROS before using the launch file.

## Requirements

1. ROS (the package has been tested on ROS Noetic)
2. MAVROS package

## Usage

1. Open a new terminal, create a catkin workspace, and navigate to folder `src`
```bash
mkdir -p ~/bayu_offboard_ws/src
cd ~/bayu_offboard_ws/src
```
2. Clone this package to the workspace
```bash
git clone https://github.com/bayukael/bayu_offboard_mavros.git
```
3. Build and source the workspace
```bash
cd ~/bayu_offboard_ws && catkin_make
source ~/bayu_offboard_ws/devel/setup.bash
```
4. Open a new terminal and run PX4 simulator. Please change `path/to/dir` to where the PX4 simulator is located.
```bash
cd path/to/dir/PX4-Autopilot
make px4_sitl gazebo
```
5. Open the first terminal and launch `bayu_offboard.launch`
```bash
roslaunch bayu_offboard_mavros bayu_offboard.launch
```
6. Observe the drone. It should move to these setpoints in a local ENU frame:
    1. `[x,y,z,yaw] = [0.0, 0.0, 10.0, 0]`
    2. `[x,y,z,yaw] = [10.0, -20.0, 15.0, PI/2]`
    3. `[x,y,z,yaw] = [-20.0, -25.0, 10.0, -PI/2]`
7. After the drone reached the last setpoint, it will switch to `HOLD` flight mode and node `bayu_offboard_node` will be terminated. MAVROS will still be running.
## License
MIT License. Please look at `LICENSE.txt` for the text.
