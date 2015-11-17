<a id="top"/> 
# Motion Planning with ROS
## Tutorial

### Contents

1. <a href="#1--introduction">Introduction</a>   
2. <a href="#2-moveit---setup-assistant">MoveIt! - Setup Assistant</a>  
3. <a href="#3-moveit---rviz-plugin">MoveIt! - RVIZ-Plugin</a>  
4. <a href="#4-moveit---enhanced-configuration">MoveIt! - Enhanced Configuration</a>  
5. <a href="#5-moveit---scripting-api">MoveIt! - Scripting API</a>  
6. <a href="#6-help">Help</a>  




### 1.  Introduction

This section quickly introduces basic tools used within this tutorial.  
It also lists several helpful (terminal) commands that are used frequently throughout this tutorial.  

#### 1.1. Tools  

It is recommended to use __terminator__ as a command prompt. A shortcut can be found on the left bar of your screen.  
Commands given in the following can be copied into the terminator window. 

We wil use __gedit__ as a text edior. A shortcut can be found on the left bar of your screen as well.  

#### 1.2. Commands

Use the command `roscd <package_name>` to browse to the folder of a specific ROS package, where `<package_name>` is the name of the ROS package, e.g. `lbr_bringup`.  `roscd` can be used from any previous location.    

For editing a file simply use `roscd` to navigate to the folder of the file and then type `gedit <file_name>`, where `<file_name>` is the name of the file you want to edit.  

For more helpful ROS commands have a look at the [ROS Cheat Sheet](http://download.ros.org/downloads/ROScheatsheet.pdf "ROS Cheat Sheet").  

All required ROS packages are already installed and the environment (i.e. environment variables) is set up correctly.  
An example solution for this tutorial is available. In order to use the solution rather than your own version, simply type `solution` in the terminal before opening the respective file or running the command.  

<a href="#top">top</a> 





### 2. MoveIt! - Setup Assistant  

MoveIt! requires a little configuration before offering its capabilities to your robot. The configuration can easily be done with the __MoveIt! Setup Assistant__ - a graphical tool that comes shipped with MoveIt! automatically.

The following steps will lead you to a valid MoveIt!-configuration for the __Kuka LBR__.  

#### 2.1. Start  

To start the MoveIt-SetupAssistant GUI, run: 
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
You should see the screen below.

![SetupAssistant1](./doc/SetupAssistant1.png "SetupAssistant1")

In order to create a new configuration, select __Create New MoveIt Configuration Package__.  
Load the robot model (URDF) by browsing to `~/seminar/src/ipa_seminar/ipa_seminar_manipulation/lbr_bringup/urdf` and selecting the `lbr_solo.urdf.xacro` file.  
You should now see our robot within the SetupAssistant.  

![SetupAssistant2](./doc/SetupAssistant2.png "SetupAssistant2")


#### 2.2. Self-Collision  

In order to speed up later collision checking, a SelfCollision-Matrix can be computed in the __Self Collision__ tab.  
Select the highest Sampling Density and click __Regenerate Default Collision Matrix__.    

![SetupAssistant3](./doc/SetupAssistant3.png "SetupAssistant3")

You will see a list of pairs of links which never or always collide within the robot model. This knowledge is used to speed up SelfCollision-Checking as these checks need not to be done during a planning step again.

![SetupAssistant4](./doc/SetupAssistant4.png "SetupAssistant4")


#### 2.3. Virtual Joints  

The next tab __Virtual Joints__ is less important for our scenario.  
This tab gets important in case you want to use MoveIt! with a mobile robot. A virtual joint connects the robot to the world. While mobile robots would have __planar__ (2d) or __floating__ (6d) virtual joints, we simply define a __fixed__ virtual joint stating that our robot does not move.  

![SetupAssistant5](./doc/SetupAssistant5.png "SetupAssistant5")

#### 2.4. Planning Groups  

In step four __Planning Groups__, we define collections of links and joints of the robot and declare them as a Planning Group respectively. For each Planning Group defined here, MoveIt! will generate a configuration in order to perform motion planning later.  

For this tutorial we add two Planning Groups:  

1. Group "arm"  
  * Select `kdl_kinematics_plugin/KDLKinematicsPlugin` as __Kinematic Solver__. This is a numerical Solver for Inverse Kinematics.  
  * Use the __Add Kin. Chain__ button to assign the kinematic chain starting with `arm_0_link` (Base Link) and ending with `arm_7_link` (Tip Link).  

2. Group "gripper"  
  * The gripper does not need a kinematic solver. Thus, we leave it `None`.  
  * As the gripper group only consists of just one link (i.e. `gripper_link`), we use the __Add Links__ button here.  

![SetupAssistant6](./doc/SetupAssistant6.png "SetupAssistant6")  

After the configuration of the two Planning Groups, the tab should look like the following.  

![SetupAssistant7](./doc/SetupAssistant7.png "SetupAssistant7")

#### 2.5. Robot Poses  

The tab __Robot Poses__ allows us to pre-define default robot poses. These robot poses can later be used as goals for a motion command.  
Define poses for the group "arm" by using the sliders to set the joint values of each joint within the group.  
You can see what the current configuration looks like in the visualization. In case the current configuration is in collision, this is indicated by a notification.  

![SetupAssistant8](./doc/SetupAssistant8.png "SetupAssistant8")

Create at least a robot pose `left` and a robot pose `right` as those are going to be used lateron.  


![SetupAssistant9](./doc/SetupAssistant9.png "SetupAssistant9")

#### 2.6. End Effectors  

Next, we define our gripper to be the __End Effector__ for our arm.  
This will give us an __Interactive Marker__ for moving the arm in the RVIZ Plugin. This is also important for High(er)-Level Capabilities of MoveIt! - such as __Pick-and-Place__.  

![SetupAssistant10](./doc/SetupAssistant10.png "SetupAssistant10")

#### 2.7. Passive Joints  

The tab __Passive Joints__ is not relevant for our scenario. Just skip it.

#### 2.8. Configuration Files  

In the final step, the MoveIt! SetupAssistant generates all files required for motion planning automatically.  
You can see a list of files to be generated below. It comprises configuration files (.yaml) as well as startup files (.launch). By clicking on a file you can get more information about it in the text box beside it.  

To generate the files, specify a location where the files should be stored by browsing to the folder `~/seminar/src/ipa_seminar/ipa_seminar_manipulation`. Create a new folder `lbr_moveit_config` and select it as destination.  
Then press __Generate Package__.  

Close the Setup Assistant by clicking __Exit Setup Assistant__.

![SetupAssistant12](./doc/SetupAssistant12.png "SetupAssistant12")


#### 2.9 Summary  

We just created a new ROS package that provides us with all configuration files and basic startup files.  Navigate to the package with `roscd lbr_moveit_config` and have a look at the generated files.  

Configuration files:  
* __config/joint_limits.yaml__: This file specifies velocity and acceleration limits for all joints of the robot. Position limits are defined in the URDF already.
* __config/kinematics.yaml__: This file specifies the kinematic solvers to be used with each group.
* __config/ompl_planning.yaml__: This file specifies a set of several (sampling-based) motion planner from the OMPL motion library with according default settings.
* __config/lbr_solo.srdf__: This SRDF (Semantic Robot Description Format) file holds the main configuration to be used with MoveIt!

Startup files:  
* __launch/demo.launch__: This is the most basic startup file. It opens RVIZ where the planning capability can be tested without needing to run either a simulation or a robot hardware.
* __launch/move_group.launch__: This file will be used when MoveIt! is to be used in connection with a simulation or a robot hardware

Both of launch files call other launch files contained in the launch folder which start up specific MoveIt! modules respectively.  

<a href="#top">top</a> 





### 3. MoveIt! - RVIZ-Plugin  

The following will introduce the __MoveIt! RVIZ-Plugin__ and explain how to use it.  

#### 3.1. Plugin Environment Basics  

In order to get familar with MoveIt! step-by-step, we will first use MoveIt! in its most basic form - the __demo mode__.  
For starting the demo mode, run: 
```
roslaunch lbr_moveit_config demo.launch config:=true
```

This will start up the main MoveIt! node called __move_group__ and a RVIZ window with the MoveIt!-Plugin loaded.  

The RVIZ window consists of three main parts:  
* the visualization on the right
* the displays section on the top left
* the MoveIt! control panel on the lower left

In the visualization, you will see several models of the robot:
* the __Scene Robot__ which displays the current state of the robot
* the __Start State__ which virtually displays the start state for a planning request (depicted in green with an interactive marker attached)
* the __Goal State__ which displays the virtual goal state for a planning request (depicted in orange with an interactive marker attached)
* the __Planned Path__ which virtually displays the result trajectory of a planning request

You can modify what you see in the visualization by adjusting the settings in the display section. You can e.g. toggle some of the displays or change colors. Save the changes into the RVIZ configuration file through the menu bar at the top.  

![RVIZ-Plugin-Trail](./doc/rviz_plugin_trail_annotated.png "RVIZ-Plugin-Trail")


#### 3.2. Planning Request  

In order to send planning requests to MoveIt!, we will use the interactive markers and the MoveIt! control panel on the lower left.  

In the __Context__ tab of the MoveIt! control panel, make sure it says __OMPL__ underneath the Planning Library. This means that MoveIt! has successfully loaded all required components. Then switch to the __Planning__ tab.  

In this tab, we can modify the Start State and the Goal State of the robot by dragging the interactive markers in the visualization. You can also set either the Start State or the Goal State to one of the previously defined robot poses using the drag down menu in the MoveIt! control panel.  

Once you configured the Start State and the Goal State of the robot, click __Plan__ in the control panel for planning the trajectory. If successful, the result is shown in the visualization.  

The buttons __Execute__ and __Plan and Execute__ are without function in the demo mode. We will use them lateron when working with the real robot hardware.  


<a href="#top">top</a> 





### 4. MoveIt! - Enhanced Configuration  

In this step we will enhance the basic MoveIt! configuration generated in step 3.  
We will add support for visual sensors to be able to consider the dynamic environment during collision checking.  
Also we will add controllers that allow us to use MoveIt! with the real robot hardware. 

#### 4.1 Perception  

MoveIt! uses a concept called __Planning Scene__ to provide information about an environment situation. The Planning Scene combines the following information:  
* __Robot State:__ The current state of the robot, i.e. its current joint configuration
* __Collision Objects:__ A set of (known) collision objects. A collision object is defined by its geometry and its pose
* __Visual Information:__ Information retrieved from a visual sensor

Now, in order to consider visual information within the Planning Scene, we need to specify from which ROS topics such sensor information can be received. Therefore we will add a new configuration file within the lbr_moveit_config that specifies all required information.

Create a new file `sensor_kinect.yaml` with the following content and save it within `lbr_moveit_config/config`.  
```
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /cam3d/depth/points
    max_range: 2.0
    point_subsample: 1
    shape_padding: 0.05
    shape_scale: 1.0
```

In order to use this new configuration file, paste the following lines to `lbr_solo_moveit_sensor_manager.launch` in `lbr_moveit_config/launch.xml`:  
```
<launch>
  <param name="octomap_frame" type="string" value="base_link" />
  <param name="octomap_resolution" type="double" value="0.05" />
  <param name="max_range" type="double" value="2.0" />
  <rosparam file="$(find lbr_moveit_config)/config/sensor_kinect.yaml"/>
</launch>
```


#### 4.2 Control  

In order to be able to use MoveIt! with a real robot hardware we need to tell MoveIt!:  
1. from which topic it can receive information about the robot's current configuration (i.e. the joint states)  
2. to which topic the planned trajectory is to be send for execution  

Both of this can be achieved by configuring an according controller for our KUKA LBR.

Create a new file `controllers.yaml` with the following content and save it within `lbr_moveit_config/config`.  
```
controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - arm_1_joint
      - arm_2_joint
      - arm_3_joint
      - arm_4_joint
      - arm_5_joint
      - arm_6_joint
      - arm_7_joint
```

In order to use this new configuration file, paste the following lines to `lbr_solo_moveit_controller_manager.launch` in `lbr_moveit_config/launch.xml`:  
```
<launch>
  <rosparam file="$(find lbr_moveit_config)/config/controllers.yaml"/>
  <param name="use_controller_manager" value="false"/>
  <param name="trajectory_execution/execution_duration_monitoring" value="false"/>
  <param name="moveit_controller_manager" value="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
</launch>
```

#### 4.3 Enhanced Usage  

Now, we configured everything to be able to use MoveIt! on our KUKA LBR!  
In order to work with the real robot hardware, the robot needs to be initialized first. Ask your tutor to do so.  

Please note:
* Only use the robot under supervision of your tutor
* Make sure you have a clear view to the robot and the emergency button is in your hands before executing a command 
* In case of unexpected behavior or in case the robot is about to collide hit the emergency button immediately
* After the emergency button has been pressed, ask your tutor to recover the robot

We will use the MoveIt! RVIZ-Plugin again to experience the new capabilities we just configured. However, this time we will not use the demo mode.  

To start MoveIt! with the robot hardware run:  
```
export ROS_MASTER_URI=http://pyramid-2:11311
roslaunch lbr_moveit_config move_group.launch
```

Also we need to start RVIZ seperately using (in a new terminal):
```
export ROS_MASTER_URI=http://pyramid-2:11311
roslaunch lbr_moveit_config moveit_rviz.launch config:=true
```

We will see our robot in its current state and its current environment. 

![RealPlanningScene](./doc/real_planning_scene.png "RealPlanningScene")

As in the previous step we can compose Planning Requests in RVIZ using the InteractiveMarkers or move to pre-defined robot poses. Make sure to select only `current` for the _Start State_ and click __update__ before planning to avoid unexpected jumps of the robot.  

Once you composed a new Planning Request, click __Plan__ in the control panel of the plugin. MoveIt! starts to solve your request and - if successfull - you should see the resulting trajectory. The resulting trajectory can then be executed on the robot by clicking __Execute__ in the control panel.  

By clicking __Plan and Execute__, MoveIt! will directly execute your Planning Request - if planned successfully.  
When using this mode - __and only in this mode!__ - also reactive planning is activated. This means that MoveIt! monitors the execution of the trajectory, updating the Planning Scene continuously during execution. As soon as changes in the environment, e.g. a new obstacle, intersects with the trajectory thus leading to a collision, MoveIt! stops the execution and tries to replan, i.e. find a new trajectory to the specified goal considering the new environment situation.  

<a href="#top">top</a> 




### 5. MoveIt! - Scripting API  

Beside the MoveIt!-RVIZ-Plugin, MoveIt! offers powerful and easy-to-use scripting APIs that can be used to implement complex manipulation applications. APIs are provided both for C++ and Python. The full C++ API can be found [here](http://docs.ros.org/hydro/api/moveit_core/html/).  

For this tutorial we will use the Python API to implement an example script in which we add virtual objects to the Planning Scene and perform various movements with our robot. As we will see, the same script can be used with our simulated robot as well as with the real robot hardware without any changes (see also IPA-Seminar-Application).

For your script(s) you can use the template file `scripting_template.py` in `lbr_bringup/scripts`. Create a copy of this script in the same folder before editing. 

#### 5.0. Helpfull commands  

The following commands are useful for finding joint and Cartesian goals during e.g. a teach in process. 
The current joint configuration can be querried by using:  
```
rostopic echo /joint_states -n 1
```

The current endeffector pose (in Cartesian coordinates) can be seen by using:  
```
rosrun tf tf_echo <from> <to>
```
where `<from>` is the frame in which the endeffector pose is received (e.g. `/base_link`) and `<to>` is the endeffector frame (e.g. `/arm_wrist_3_link`).


#### 5.1. PlanningSceneInterface  

This part of the API allows you to add and remove (virtual) static obstacles (geometric primitives or meshes) to the Planning Scene. Also objects can be attached and detached to the robot. This is particularly interesting when grasping objects as attached objects become part of the robot itself and thus are considered during motion planning. The full API can be found [here](https://github.com/ros-planning/moveit_commander/blob/groovy-devel/src/moveit_commander/planning_scene_interface.py "PlanningSceneInterface").  
In this tutorial we will only use the following functions in our script:  
```python
def add_box(name, pose, size = (1, 1, 1))   ### add a box  
def remove_world_object(name)               ### remove an object from scene
```

In order to use one of these functions in your script, add the following lines of code to your script once:  
```python
psi = PlanningSceneInterface()
rospy.sleep(1.0)
```
This brings in a handle `psi` for the PlanningSceneInterface. `psi.add_box()` adds a box the the Planning Scene. Give the function calls appropriate parameters:  
* __name__ a unique name for the box objects
* __pose__ the pose of the object in the world. Use the helper function to generate the according type
* __size__ the size of the box, i.e. the extension in x-, y- and z-direction

#### 5.2. MoveGroupCommander  

This part of the API provides a huge set of functions to interact with your robot. It consists of functions for retrieving information about your robot as well as various commands for moving the robot. The full API can be found [here](https://github.com/ros-planning/moveit_commander/blob/groovy-devel/src/moveit_commander/move_group.py "MoveGroupCommander").  
An excerpt that can be used within our script can be seen below:  
```python
def set_named_target(name)          ### sets the goal configuration to the pre-defined robot pose name
def plan(joints = None)             ### plan to the given goal (JointState or Pose)
def execute(plan_msg)               ### execute a previously planned trajectory
def go(joints = None, wait = True)  ### plan to the given goal (JointState or Pose) and then execute the trajectory.
def compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions = True)   ### plan a linear trajectory via the given waypoints
```
In order to use one of these functions in your script, add the following lines of code to your script once:  
```python
mgc = MoveGroupCommander()
rospy.sleep(1.0)
```
This brings in a handle `mgc` for the MoveGroupCommander.  
The functions mentioned above can now be used as `mgc.<function_name>()` with the according parameters given.  

#### 5.3. Script-Execution

In order to run your script, the robot hardware needs to be running.  Also MoveIt! needs to be started (see <a href="#43-enhanced-usage">Enhanced Usage</a>):  
```
export ROS_MASTER_URI=http://pyramid-2:11311
roslaunch lbr_moveit_config move_group.launch
```
and in a new terminal:
```
export ROS_MASTER_URI=http://pyramid-2:11311
roslaunch lbr_moveit_config moveit_rviz.launch config:=true
```

Run your script (in a new terminal):
```
export ROS_MASTER_URI=http://pyramid-2:11311
roscd <your_file_location>
python <your_file_name>
```



The following example shows a script that combines everything we learned in this section. It can be found [here](https://github.com/ipa320/ipa_seminar/blob/master/ipa_seminar_manipulation/lbr_bringup/scripts/scripting_example.py "Script Example").  
```python
#!/usr/bin/env python
import roslib; roslib.load_manifest('lbr_bringup')
import rospy

from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
 
### Helper function 
def gen_pose(frame_id="/base_link", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

if __name__ == '__main__':
	rospy.init_node('scripting_example')
	while rospy.get_time() == 0.0: pass
	
	### Create a handle for the Planning Scene Interface
	psi = PlanningSceneInterface()
	rospy.sleep(1.0)
	
	### Create a handle for the Move Group Commander
	mgc = MoveGroupCommander("arm")
	rospy.sleep(1.0)
	
	
	### Add virtual obstacle
	pose = gen_pose(pos=[-0.2, -0.1, 1.2])
	psi.add_box("box", pose, size=(0.15, 0.15, 0.6))
	rospy.sleep(1.0)
	
	### Move to stored joint position
	mgc.set_named_target("left")
	mgc.go()
	
	### Move to Cartesian position
	goal_pose = gen_pose(pos=[0.123, -0.417, 1.361], euler=[3.1415, 0.0, 1.5707])
	mgc.go(goal_pose.pose)
	
	### Move Cartesian linear
	goal_pose.pose.position.z -= 0.1
	(traj,frac) = mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, True)
	mgc.execute(traj)
	
	print "Done"
```
It first adds an additional (virtual) obstacle to the Planning Scene. Then it performs three different kinds of __planned__ motion:  
* move to a pre-defined robot configuration
* move to a given Cartesian goal pose
* move to a given Cartedsian goal pose using linear motion


### This concludes this tutorial on Motion Planning with ROS. 

<a href="#top">top</a> 


### 6. Help  

* Official Website: [http://moveit.ros.org/wiki/MoveIt!](http://moveit.ros.org/wiki/MoveIt!)
* MoveIt!-ROS-Wiki: [http://wiki.ros.org/moveit](http://wiki.ros.org/moveit)
* MoveIt!-Tutorials: [http://moveit.ros.org/wiki/Tutorials](http://moveit.ros.org/wiki/Tutorials)
* API-Documentation: [http://docs.ros.org/hydro/api/moveit_core/html/](http://docs.ros.org/hydro/api/moveit_core/html/)  


===

In case of questions - now or later - do not hestate to contact your manipulation expert at Fraunhofer IPA:  

```
Dipl.-Inform. Felix Meßmer  
e-mail: [felix.messmer@ipa.fraunhofer.de](mailto: felix.messmer@ipa.fraunhofer.de)  
phone: +49 711 970-1452  
```


