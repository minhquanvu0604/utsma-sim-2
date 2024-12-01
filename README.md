# Simulation Package for UTS Motorsports Autonomous

The simulation is a extracted, streamlined version of the Edinburgh University Formula Student (EUFS) Driverless Team's complex simulator, tailored for focused autonomous motorsport development. Refer to [their wiki](#reference) for more indepth documentation.

The simulator, functioning as an integration hub for various robotic components such as sensors and path planning algorithms, currently serves primarily as a **visualizer**. It interprets and responds to desired velocity commands, effectively simulating the movement of autonomous vehicles in a virtual environment. 

The simulator enables section-specific testing in controlled environments, such as directly obtaining cone positions, bypassing the need for computer vision. This feature enhances targeted development and fine-tuning of autonomous motorsport systems.

Looking ahead, there are plans to enhance the simulator with advanced physics and high-fidelity features, aiming to further bridge the gap between virtual testing and real-world performance in autonomous motorsports.                   

<p align="center">
  <img height="500" alt="Gazebo Environment Image" src="wiki/image/gazebo_img.png">
</p>


## Launch

The digram illustrates the hierarchy of launch files and xacro for robot description. To launch the Gazebo Race Track environment: `roslaunch utsma small_track.launch`

<p align="center">
  <img height="300" alt="Gazebo Environment Image" src="wiki/image/launch_hierarchy.png">
</p>

## Custom Plugin (eufs_plugins)
- `gazebo_cone_ground_truth`
- `gazebo_race_car_model`

- Install docker and nvidia container toolkit
- make sure docker daemon is running
- run `sudo docker build -t ros-humble-pfms:latest -f 'docker/Dockerfile.ros-humble-pfms.amd64' .`
- run `sudo docker compose up -d`

### MacOS Installation Steps (NOT WORKING)

- make sure docker daemon is running
- install XQuartz
- defaults write org.xquartz.X11 enable_iglx -bool true
- restart computer
- start XQuartz
- After installing XQuartz, start it and open the preferences (XQuartz > Preferences). Go to the "Security" tab and check "Allow connections from network clients".
- Open a terminal and run the following commands (These commands need to be run every time XQuartz is started)
  - export DISPLAY=:0
  - xhost +local:root
  - xhost si:localuser:root
  - xhost + 127.0.0.1
- run `sudo -E docker build -t ros-humble-pfms:latest -f 'docker/Dockerfile.ros-humble-pfms.amd64' .`
- run `sudo -E docker compose up -d`

### commands for when container is running

- if you quit the application and dont want to restart the container use `ros2 launch eufs_launcher eufs_launcher.launch.py` to relaunch the application.

## Packages

This project contains a number of packages. The package READMEs supply information about the package API (launch parameters, ROS 2 publishers, subscribers and services).
For information on the package design and usage guides see the [eufs_sim wiki](https://gitlab.com/eufs/eufs_sim/-/wikis/home).

- [eufs_racecar](./eufs_racecar/README.md) : launch, resource and urdf files for the simulated vehicle.
- [eufs_launcher](./eufs_launcher/README.md) : configures and launches the simulation.
- [eufs_models](./eufs_models/README.md) : vehicle physics library.
- [eufs_plugins](./eufs_plugins/README.md) : Gazebo plugins.
- [eufs_tracks](./eufs_tracks/README.md) : track generator and resource files for the track.
- [eufs_sensors](./eufs_sensors/README.md) : sensor mesh and urdf files.
- [eufs_rqt](./eufs_rqt/README.md) : rqt GUI's for eufs_sim (currently mission control and robot steering).

## Dependencies
- `Ubuntu 20.04 & ROS Noetic`
- [`ackermann_msgs`](https://github.com/ros-drivers/ackermann_msgs) - branch master
- [`eufs_msgs`](https://gitlab.com/eufs/eufs_msgs/-/tree/ros1?ref_type=heads) - branch ros1
- cgal (no longer depends on this as Path Planning is now self-contained in a repo)
```bash
sudo apt install libcgal-dev
```
- can_msgs (temp: for d_triang and simulation to communicate with can system)


## External Links
UTSMA:
- [System Diagram](wiki/system.md)

## Reference 
- Edinburgh University Formula Student (EUFS) Driverless Team: 
    - https://gitlab.com/eufs/eufs_sim
    - Archived [README](wiki\EUFS_MAINPAGE.md) 
- AMZ Driverless:  
    - https://github.com/AMZ-Driverless/fssim/tree/master
    - https://www.amzracing.ch/en

