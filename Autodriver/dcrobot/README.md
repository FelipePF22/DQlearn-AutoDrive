# DC ROBOT - UFSCar

This package contains a toolkit designed for the DC Autonomous Mobile Robot. Developed as part of the Department of Computer - Autonomous Mobile Robot (DCAMR) Project, this toolkit provides essential functionalities for the DC robot's autonomous capabilities.

 
| Operational System          	|  Ubuntu 20.04        	|
| ---------------------------- | ------------------------ |
| ROS                        	| ![image](https://user-images.githubusercontent.com/74054598/149457205-fd48db89-0658-4511-af36-bcd8662562da.png)|
| Gazebo   	              	| Gazebo multi-robot simulator - version 11.13.0 	|

**Features**
   - Host robot DC
   - RPLidar
   - Camera


## Required packages in apt-get:

```bash
sudo apt-get install ros-noetic-amcl ros-noetic-costmap-converter ros-noetic-depthimage-to-laserscan ros-noetic-dynamic-reconfigure ros-noetic-ddynamic-reconfigure ros-noetic-ddynamic-reconfigure-dbgsym ros-noetic-ddynamic-reconfigure-python ros-noetic-geometry2 ros-noetic-hector-slam ros-noetic-hector-gazebo-plugins ros-noetic-move-base ros-noetic-move-base-flex ros-noetic-navigation ros-noetic-openslam-gmapping ros-noetic-rplidar-ros ros-noetic-slam-gmapping ros-noetic-spatio-temporal-voxel-layer ros-noetic-teb-local-planner ros-noetic-teleop-twist-keyboard ros-noetic-teleop-twist-joy ros-noetic-urg-node ros-noetic-rtabmap ros-noetic-rtabmap-ros ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server ros-noetic-octovis ros-noetic-imu-filter-madgwick ros-noetic-robot-localization ros-noetic-robot-pose-ekf ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server ros-noetic-map-server ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins ros-noetic-ompl ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-global-planner ros-noetic-costmap-2d ros-noetic-robot-self-filter ros-noetic-ros-numpy ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-grid-map-costmap-2d ros-noetic-grid-map-ros ros-noetic-grid-map-filters ros-noetic-grid-map-visualization ros-noetic-tf2-tools pcl-tools
```

### Create the directories

```bash
mkdir -p /home/$USER/dcrobot_ws/src
cd /home/$USER/dcrobot_ws/
```


### Initialize the Catkin workspace
```bash
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release
```

### Navigate to the directory of `src` to clone the `DC Robot project`

```bash
cd /home/$USER/dcrobot_ws/src
git clone https://github.com/vivaldini/dcrobot.git
```

### Build the project
```bash
cd /home/$USER/dcrobot_ws/
catkin build
```

### Source your catkin workspace
```bash
source /home/$USER/dcrobot_ws/devel/setup.bash
```

### (optional) may you find some errors, so you can use the "Magic" of rosdep
```bash
cd /home/$USER/dcrobot_ws/src
rosdep install --from-paths . --ignore-src --os=ubuntu:focal -r -y

cd /home/$USER/dcrobot_ws/src/dcrobot
catkin build
source /home/$USER/dcrobot_ws/devel/setup.bash

```
### Environments Test

Configure the environment in the file `gazebo.launch` located in the directory `/home/$USER/dcrobot_ws/src/dcrobot/mobile_rob_dev_sim/launch`. Adjust the following options:

1. DC Robot starts in MakerSpace room - 1st floor
   ```xml
   <arg name="world_name" value="$(find envrobotz)/worlds/EnvDC_MakerSpace.world"/>
   ``` 

2. DC Robot starts in front of LARIS room - 2nd floor
   ```xml
   <arg name="world_name" value="$(find envrobotz)/worlds/EnvDC_2ndfloor.world"/>
   ```

## Support

For support, send an email to vivaldini@ufscar.br.


