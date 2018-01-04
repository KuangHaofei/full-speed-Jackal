# full-speed-Jackal
ShanghaiTech University CS283 Robotics Course Fall 2017 Project

# How to use leg_ package
+ build a catkin workspace
```
$ cd ~
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/KuangHaofei/full-speed-Jackal.git
$ git clone https://github.com/KuangHaofei/jackal_simulation.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```
+ Running the package
```
roslaunch pointcloud_to_laserscan sample_node.launch
```
Open a new window
```
roslaunch leg_detector jackal_leg_detector.launch
```
+ Running rviz
```
rosrun rviz rviz
```
