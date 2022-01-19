# Debug-ROS
## Gazebo-ROS
The important thing is to export the model path, otherwise we cannot find model in the gazebo database.
```
cd catkin_ws
source devel/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/gazebo_model_population/models/
```

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
rosdep update
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
  http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node
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
  ## Publish service
  advertiseService() creats a ros service server that provides the service named "add_two_ints" to ros master and the service is accomplished by callback function add.
  ```
  ros::ServiceServer service = n.advertiseService("add_two_ints",add);
  bool add(beginner::AddTwoInts::Request &req,
           beginner::AddTwoInts::Response &res){
  ...
  return true};
  ```
  The second method
  ```
  ros::AdvertiseServiceOptions GetPose_aso = ros::AdvertiseServiceOptions::create<custom_msgs::GetPose>(
                                                                                 "name",
                                                                                  boost::bind(&class::callbackfunction, this, _1, _2),
                                                                                  ros::VoidPtr(),
                                                                                  &this->rosQueue);
  ros::ServiceServer GetPoseSrv = n.advertiseService(GetPose_aso);
  ```
  
  ## callback for ros service
  callback takes in the request and calculate the response. callback can be a function/class/structure.
  
  1. when the callback is a function
  
  ## Subscribe service
  The ServiceClient object is able to call the service using call(srv). To call the service, the client needs to create srv and assign request value to it. Once the service call finish, the call() return bool success and srv.response is valid. 
  ```
  ros::ServiceClient client = n.serviceClient<beginner::AddTwoInts>("add_two_ints");
  beginner::AddTwoInts srv;
  srv.request.a = ...
  client.call(srv)
  ```
  
  ## How to check the correct ros service name
  This problem is because unclear understanding towards node/nodehandle namespace. Please check ros tutorial.
  
  I spent a lot of time to solve the problem: why my ros node in the gazebo plugin has been initialized while the nodehandle cannot advertise service. It turns out that my service name is in this format "node_name/service_name", but what I check is this "service_name". So, I cannot call the service because it is a wrong name.
  
  To avoid the same problem, please check the service/topic/action name first beforing using them.
  ```
  rosservice list
  rosservice call args
  e.g. rosservice call laddle
  ```
  ## callback and spinning
  ros::spin() ros::spinOnce() are both to invoke callback function when the service/action are been called. 
  
  Write callbackqueue and thread works similar as use spin function directly. Check ros tutorial for more detail.
  
  ## Debug unable to call ros service
  Just check the spelling of service name!! I spelled 'test' to 'text' and spent a hour to debug it.... f..k
  
  ## Call ros service from command line
  ```
  rosservice call /name 
  Press 'Tab'
  ```
  This command+Tab will generate template for you to fill in the service request parameters.
  
# ROS-Gazebo
## Spawn model into the world 
  First we need to create two pkg in the catkin_ws: robot_description, robot_gazebo
  Then we have two method to spawn model:
  method 1: first roslaunch world and then roslaunch model
  method 2: export the model into gazebo model database, and include model in the world file
  
  
  
  
  
  
