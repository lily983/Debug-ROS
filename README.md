# Debug-ROS
## Build a catkin workspace
```
mkdir  -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```
## In the terminal
```
source ~/catkin_ws/devel/setup.bash
```
## Build a new package
```
cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
catkin build
```
## Common used commands
```
rm -rf "folder"
rosclean check 
rosclean purge

```


## Common Mistakes in roslaunch 
1. Invalid roslaunch XML syntax: mismatched tag
--> Check if every tag has been self-closing. e.g. <arg hhhhh > is wrong, <arg hhhh /> is self-closing
