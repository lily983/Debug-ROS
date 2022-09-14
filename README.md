# Debug-ROS

##ROS dep error
```
ERROR: the following packages/stacks could not have their rosdep keys resolved to system dependencies: CGAL
```
Check pkg package.xml file! In pkg obj_reconstruction, we set find_pkg(CGAL) im CMake.list, and I write <build_dependence>CGAL<> in package.xml. This is incorrect! CGAL is not a ROS pkg.

## Debug mode
To see the error msg when running a node, use debug mode when build the pkg
```
catkin build -DCMAKE_BUILD_TYPE=Debug pkg_name
```

## Open Azure kinect camera
```
k4aviewer
```
https://vinesmsuic.github.io/2020/10/31/azure-kinect-setup/#adding-path-in-vscode

## Realsense SDK2.0 IDE
To open IDE: e.g. for on-chip calibration https://www.intelrealsense.com/self-calibration-for-depth-cameras/
```
realsense-viewer
```
## Surface reconstruction
### Reference web
How TSDF integration works: https://gist.github.com/savuor/407fdc1807f9d5836d68aebfee726ef7

TSDF params: https://docs.opencv.org/4.x/d9/d94/structcv_1_1kinfu_1_1VolumeParams.html#ad377fbc71190ba8c715f69dc3f64288f

Mesh manipulation: 
	1. http://www.huyaoyu.com/technical/2020/10/07/cgal-point-cloud-and-mesh.html
	2. https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#Chapter_PolygonMeshProcessing
```
./HoleFilling /home/xiaoli/Desktop/results_mesh.ply /home/xiaoli/Desktop
```

1. Error: raycasted preview surface eroded as the arm scanning. It looks like the error caused by drift of pose.

Possible reason: the timestamp between the depth image and arm are inconsistent. We may align the current depth image with camera pose publihed n seconds age. https://github.com/ros-industrial/yak_ros/issues/41
```
Your mesh is being eroded as you scan because the calculated position of the camera at a given time does not match the real-world position of the camera at that time. This can happen if one of the sources of TF information within your system has a clock that is out of sync relative to the rest of your system. The last time I encountered this, it was because the system clock in my robot's controller was 5 seconds behind my PC, so when Yak was trying to match up the depth images with camera positions it would use positions that were 5 seconds in the past, and it produced an issue similar to the one in your video.
```
Possible solution:
```
To solve that particular problem I made a little ROS node that subscribed to /joint_states, copied the contents of the incoming messages to a new message while setting the timestamp in the header to the node's current time, and publishing them to a different topic (/joint_states_restamped or similar). In that particular case the root cause was that the clock of the robot's control computer (a Kuka iiwa7) was out of sync with the computer running the rest of the ROS nodes, which isn't directly applicable to the way you have things set up (in other words: I'm not confident that the "fix" I described is actually a solution for your problem).
```
2. min_weight, tsdf_max_weight meaning:

min_weight: the minimum number of observations a voxel has.  marching cubes function not meshing voxels with low weights. https://github.com/ros-industrial/yak_ros/issues/30

tsdf_max_weight: the number of new observations at new distances a voxel required before the isosurface change appreciably.

e: To actually answer the question: tsdf_max_weight caps the weight value of each voxel. If the max weight is 100, voxels that have been updated 100 times and voxels that have been updated 10000 times will require the same number of new observations at a new distance before the shape of the isosurface begins to change appreciably.https://github.com/ros-industrial/yak/issues/24


## Moveit constraint approximation
The correct pkg is moveit_planners_ompl, not ompl_interface in the tutorial
```
roslaunch moveit_planners_ompl generate_state_database.launch constraints_file:=$(rospack find panda_moveit_config)/config/constraints.yaml planning_group:=panda_arm
```
## Gazebo-ROS
The important thing is to export the model path, otherwise we cannot find model in the gazebo database.
```
cd catkin_ws
source devel/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/gazebo_model_population/models/
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_model_population/build/
```

## Failed to launch 
```
xacro: in-order processing became default in ROS Melodic. You can drop the option.
RLException: Unable to contact my own server at [http://lily:44227/].
This usually means that the network is not configured properly.

A common cause is that the machine cannot connect to itself.  Please check
for errors by running:

	ping lily

For more tips, please see

	http://wiki.ros.org/ROS/NetworkSetup

The traceback for the exception was written to the log file
```
Solution:
```
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```
```
sudo nano /etc/hosts
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
 rosrun image_view image_view image:=/aruco_single/result
```
Get the aruco result
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
  
  
  
  
  
  
