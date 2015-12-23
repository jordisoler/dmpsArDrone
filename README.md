# dmpsArDrone

This repository corresponds to the Robot Learning  project. It contains a ROS package to be developed to implement DMPs in an AR Drone 2.0 quadrotor in order to learn generalized trajectories. It also includes some metafiles.

The ROS package is being developped in Ubuntu 14.04/indigo and assumes that the user has the following packages:

- ardrone_autonomy: 
  Can be easily installed as an Ubuntu package with 
```shell
sudo apt-get install ros-ROSDISTRO-ardrone-autonomy
```
- ar_tools ROS package:
  The cource code can be found in https://github.com/ar-tools/ar_tools.git
- dmp ROS package:
  The source code can be found in https://github.com/sniekum/dmp.git

This repository is supposed to be cloned within a the `src/` folder of a catkin workspace.

In order to properly load the Gazebo models one may include some paths to the Gazebo environment variables:
```shell
<path_to_the_repo>/intro_ros/ardrone_sim/models/:$GAZEBO_MODEL_PATH
<path_to_the_repo>/intro_ros/ardrone_sim/models/:$GAZEBO_RESOURCE_PATH
```
One is likely to be interested in putting these lines in the ```~/.bashrc``` file.

After compilation, to play with a simple scenario run
```shell
roslaunch prj_drone prj_drone_tirar_full.launch
```
