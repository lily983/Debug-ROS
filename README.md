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
check missing dependences
```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro melodic
```
Install missing dependences
```
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
```
Download pkg from github
```
git clone http://xxxx -b melodic-devel
```

## Common Mistakes in roslaunch 
Invalid roslaunch XML syntax: mismatched tag
--> Check if every tag has been self-closing. e.g. <arg hhhhh > is wrong, <arg hhhh /> is self-closing

# ROS Program
## Start a ros node
  NodeHandle is the way to communicate with ros master. You advertise/subscribe on topic/service/action through nodehandle, so that ros master knows what this node wants.
```
/**
**  Simple ROS Node
**/
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "vision_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  ROS_INFO("Hello, World!");

  // Don't exit the program.
  ros::spin();
}
```
## Publish topic through node handle.
  advertise<>() creats a publisher object that can publish topic. The topic's message type is defined in <> and its name is defined in (). 
  
  msg is message object that can be published latter.
  
  The publisher object chatter_pub has function to publish the message object.
```
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);
  std_msgs::String msg;
  ...
  chatter_pub.publish(msg);
 ```
  
  
  
# ROS-Gazebo
## Spawn model into the world 
  First we need to create two pkg in the catkin_ws: robot_description, robot_gazebo
  Then we have two method to spawn model:
  method 1: first roslaunch world and then roslaunch model
  method 2: export the model into gazebo model database, and include model in the world file
  
  
  
  
  
  
