# REMARO_Scenarios

This repository belongs to different scenarios extracted from [REMARO worlds](https://github.com/remaro-network/remaro_worlds)

## ScreenShots
General and typical path to inspect different subsea infrustructures

[![Watch REMARO_AUV get out of docking station](assets/imgs/remaro_scenario.jpg)](assets/GIFs/docking_station.mp4)

The safer path (alternative solution) exists to inspect the last vertical tank
![Safe Path](assets/imgs/small_vertical_tank_population.jpg)

We also use population of small vertical tank to make scenario more realistic; position of this object is based on random distribution. We populated 10 times for each run.
![Safe Path](assets/imgs/safe_path_remaro.jpg)

## Installation
 - Install [Ubuntu 20 LTS](https://releases.ubuntu.com/20.04/ubuntu-20.04.3-desktop-amd64.iso).

- Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).  Choose the "Desktop-Full Install" option so simulators are installed.

- Once you have installed ros packages, the <b>uuv_simulator</b> can be installed by cloning the modified version of `Field-Robotics-Lab_uuv_simulator` repository [here](https://github.com/mahyamkashani/uuv_simulator) in your ros workspace.


```bash
# Make directory for your workspace
mkdir -p ~/yourworkspacename/src

# Change directory to your workspace
cd ~/yourworkspacename/src

# Clone this package 
git clone https://github.com/remaro-network/remaro_scenarios.git

Clone BlueRov2 package 

git clone https://github.com/mahyamkashani/bluerov2

# Clone REMARO_AUV package
git clone https://github.com/jpcoffelt/remaro_auv

# Move to root of your workspace directory
cd ~/yourworkspacename

# Make the workspace 
catkin_make

# Source your workspace in each terminal using this package.
source ~/yourworkspacename/devel/setup.bash

# If this is your only active ROS workspace,
# modify ~./bashrc to automatically source this workspace.
echo "source ~/yourworkspacename/devel/setup.bash" >> ~/.bashrc
```

## planSimulation
You can simulate different plans, e.g. $P_1, P_2, ..., P_5$ by typing below commands:

#### Dangerous Scenario ($P_1$)

```bash
# To launch the simulated world
chmod +x run_dangerous_scenario.sh
./run_dangerous_scenario.sh
```
or 
```bash
# To launch the simulated world
roslaunch remaro_scenarios launch_remaro_scenario.launch
roslaunch remaro_scenarios send_dangerous_waypoints.launch
roslaunch remaro_scenarios launch_small_vertical_tank_inspection.launch
```
#### Run out of battery Scenario ($P_2$)

```bash
# To launch the simulated world
roslaunch remaro_scenarios launch_remaro_scenario.launch
roslaunch remaro_scenarios send_battery_waypoints.launch
roslaunch remaro_scenarios send_platform_waypoints.launch
roslaunch remaro_scenarios send_dangerous_waypoints.launch
roslaunch remaro_scenarios launch_small_vertical_tank_inspection.launch

```

#### Other1 Scenario ($P_3$): long path but dangerous
```bash
# To launch the simulated world
roslaunch remaro_scenarios launch_remaro_scenario.launch
roslaunch remaro_scenarios send_battery_waypoints.launch
roslaunch remaro_scenarios send_back_init_waypoints.launch
roslaunch remaro_scenarios send_platform_waypoints.launch
roslaunch remaro_scenarios send_dangerous_waypoints.launch
roslaunch remaro_scenarios launch_small_vertical_tank_inspection.launch
```

#### Safe Scenario ($P_4$)
```bash
# To run the simulated world
chmod +x run_safe_scenario.sh
./run_safe_scenario.sh
```
or 
```bash
# To launch the simulated world
roslaunch remaro_scenarios launch_remaro_scenario.launch
roslaunch remaro_scenarios send_safe_waypoints.launch
roslaunch remaro_scenarios launch_small_vertical_tank_inspection.launch

```
#### Other2 Scenario($P_5$): other long path but dangerous
```bash
# To launch the simulated world
roslaunch remaro_scenarios launch_remaro_scenario.launch
roslaunch remaro_scenarios send_battery_waypoints.launch
roslaunch remaro_scenarios send_back_init_waypoints.launch
roslaunch remaro_scenarios send_dangerous_waypoints.launch
roslaunch remaro_scenarios launch_small_vertical_tank_inspection.launch

```



#### Record Scenario
In case you couldn't install requirements, there exist a few rosbags and csv files for positions of robot and objects' location.

If you would like to record specific rostopics, see follows instruction
```
rosbag record bagfile.bag /rostopic-you-would-like
```

If you would like generate new waypoints, you can run below python program:
```
cd src
python3 gen_waypoints.py
```
#### Single/ Multiple Object Collision Detector


If you would like to know whether there exist any collisions, open 3 TABs and follow below commands.

- TAB1
```
roscore
```
- TAB2 (play rosbag)
```
rosbag play danger_2024-03-09-11-40-23.bag
```
- TAB3
```
roscd remaro_scenarios
cd src
python3 collision_detector.py
```
- or TAB3
```
roscd remaro_scenarios
cd src
python3 multiple_objects_collision_detector.py
```

## Acknowledgements
This project has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.

Pleave visit [our website](https://remaro.eu/) for more info on our project.

![REMARO Logo](https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png)
