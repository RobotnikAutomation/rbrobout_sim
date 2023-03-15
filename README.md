# rbrobout_sim

Packages for the simulation of the RB-Robout

This packages contains:

## rbrobout_gazebo

Launch files and world files to start the models in gazebo

## rbrobout_sim_bringup

Launch files that launch the complete simulation of the robot/s

## Simulating RB-Robout

This simulation has been tested using Gazebo 9 version.

## Installation and run instruccions

### 1. Install the following dependencies:

 To facilitate the installation you can use the vcstool:

```bash
sudo apt-get install -y python3-vcstool
```

### 2. Create a workspace and clone the repository:

```bash
mkdir catkin_ws
cd catkin_ws
```

For the latest version:

```bash
vcs import --input \
  https://raw.githubusercontent.com/RobotnikAutomation/rbrobout_sim/melodic-devel/repos/rbrobout_sim_devel.repos
rosdep install --from-paths src --ignore-src --skip-keys="rbrobout_robot_control marker_mapping robotnik_locator robotnik_pose_filter" -y
```
<!--
For the stable version (some latest features may be not available):

```bash
vcs import --input \
  https://raw.githubusercontent.com/RobotnikAutomation/rbrobout_sim/melodic-master/doc/rbrobout_sim.repos
rosdep install --from-paths src --ignore-src --skip-keys="rbrobout_robot_control" -y
``` -->

### 3. Compile:

```bash
catkin build
source devel/setup.bash
```

**ONLY: if catkin build doesn't work:** The package catkin-tools is need to compile with catkin build:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

### 4. Launch RB-Robout simulation (1 robot by default, up to 3 robots):

#### RB-Robout:

```bash
roslaunch rbrobout_sim_bringup rbrobout_complete.launch
```

#### RB-Robout with Trossen Arm

```bash
roslaunch rbrobout_sim_bringup rbrobout_complete.launch default_xacro:=rbrobout_tix_std.urdf.xacro launch_arm_a:=true arm_manufacturer_a:=trossen arm_model_a:=vx300s
```

Launch moveit to plan trajectories:

```bash
ROS_NAMESPACE=robot roslaunch rbrobout_vx300s_moveit_config demo.launch
```


#### RB-Robout with Kinova Arm

```bash
roslaunch rbrobout_sim_bringup rbrobout_complete.launch default_xacro:=rbrobout_gen_std.urdf.xacro launch_arm_a:=true arm_manufacturer_a:=kinova arm_model_a:=j2s7s300 amcl_and_mapserver_a:=false move_base_robot_a:=false
```

**Note:** in this configuration the robot has not laser, therefore the amcl is turned off. When Rviz is opened, change robot_map to robot_odom in ```fixed_frame```  in order to visualize the robot.

#### or RB-Robout Steel:

```bash
roslaunch rbrobout_sim_bringup rbrobouts_complete.launch
```

#### Optional general arguments:

```xml
<arg name="launch_rviz" default="true"/>
<arg name="gazebo_world" default="$(find rbrobout_gazebo)/worlds/rbrobout_office.world"/>
<arg name="omni_drive" default="false"/> (only for RB-Robout)
<arg name="use_gpu_for_simulation" default="false"/>
```

By default the Gazebo plugin [Planar Move](http://gazebosim.org/tutorials?tut=ros_gzplugins) to ignore the physics of the wheels + the skid steering kinematics. In case you want to disable this plugin, set the following arguments:

```bash
roslaunch rbrobout_sim_bringup rbrobout_complete.launch \
  ros_planar_move_plugin:=false \
  omni_drive:=false
```

#### Optional robot arguments:

```xml
<!--arguments for each robot (example for robot A)-->
<arg name="id_robot_a" default="robot"/>
<arg name="launch_robot_a" default="true"/>
<arg name="map_file_a" default="willow_garage/willow_garage.yaml"/>
<arg name="localization_robot_a" default="false"/>
<arg name="gmapping_robot_a" default="false"/>
<arg name="amcl_and_mapserver_a" default="true"/>
<arg name="x_init_pose_robot_a" default="0" />
<arg name="y_init_pose_robot_a" default="0" />
<arg name="z_init_pose_robot_a" default="0" />
<arg name="xacro_robot_a" default="rbrobout_std.urdf.xacro"/>
```

- Example to launch simulation with 3 RB-Robout robots:

```bash
roslaunch rbrobout_sim_bringup rbrobout_complete.launch \
  launch_robot_b:=true \
  launch_robot_c:=true
```

- Example to launch simulation with 1 RB-Robout robot with navigation:

```bash
roslaunch rbrobout_sim_bringup rbrobout_complete.launch \
  move_base_robot_a:=true \
  amcl_and_mapserver_a:=true
```

Enjoy! You can use the topic `${id_robot}/robotnik_base_control/cmd_vel` to control the RB-Robout robot or send simple goals using `/${id_robot}/move_base_simple/goal`

## Docker usage

In order to run this simulation you will need nvidia graphical accelation

### Installation of required files
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- nvidia-drivers

### Usage

```bash
git clone https://github.com/RobotnikAutomation/rbrobout_sim.git
cd rbrobout_sim
git checkout melodic-devel
docker/simulation-in-container-run.sh

```

#### Selecting the robot model

You can select the robot, the launch file of package using the optional arguments on launch
By default the selected robot is `rbrobout`

```bash
docker/simulation-in-container-run.sh --help
```

```
ROBOTNIK AUTOMATION S.L.L. 2021

Simulation of SUMMIT XL using docker

Usage:
docker/simulation-in-container-run.sh [OPTIONS]

Optional arguments:
 --robot -r ROBOT       Select robot to simulate
                        Valid robots:
                            rbrobout rbrobout_gen rbrobouts
                        default: rbrobout

 --launch -l            Select launch file
                        default: rbrobout_complete.launch

 --package -p           Select ros package
                        default: rbrobout_sim_bringup

 --ros-port -u PORT     Host ros port
                        default: 11345

 --gazebo-port -g PORT  Host ros port
                        default: 11345

 -h, --help             Shows this help

```

**RB-Robout GEN**
```bash
docker/simulation-in-container-run.sh --robot rbrobout_gen
```

**RB-RoboutS**
```bash
docker/simulation-in-container-run.sh --robot rbrobouts
```

#### Manual Build

If you wish to build manually the image without the use of the script use one the following commands:

**Optiona A**
```bash
cd docker
docker build -f Dockerfile ..
```
**Option B**
```bash
docker build -f docker/Dockerfile .
```

#### Notes

- This is docker requires a graphical interface
- The ros master uri is accesible outside the container, so in the host any ros command should work
- You could also run a roscore previous to launch the simulation in order to have some processes on the host running
- if you want to enter on the container use the following command in another terminal
```bash
docker container exec -it rbrobout_sim_instance bash
```
- In order to exit you have to 2 options
1. Close `gazebo` and `rviz` and wait a bit
2. execute in another terminal:
```bash
docker container rm --force rbrobout_sim_instance
```
