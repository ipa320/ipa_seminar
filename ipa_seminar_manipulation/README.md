# Motion Planning with ROS
## Tutorial

### Contents

1. Prerequesites  
  1.1. Simulation  
  1.2. Robot Hardware 
2. MoveIt! - Setup Assistant  
  2.1. Start  
  2.2. Self-Collision  
  2.3. Virtual Joints  
  2.4. Planning Groups  
  2.5. Robot Poses  
  2.6. End Effectors  
  2.7. Passive Joints  
  2.8. Configuration Files  
  2.9. Summary  
3. MoveIt! - RVIZ-Plugin  
  3.1. Plugin Environment Basics  
  3.2. Planning Request  
4. MoveIt! - CommandLine Tool  
5. MoveIt! - Scripting API  
  5.1. PlanningSceneInterface  
  5.2. MoveGroupCommander  
6. Help  


### 1.  Prerequesites

For convenience, all required ROS packages are already installed and the environment (i.e. environment variables) is set up correctly.
Changes can be made within the [setup file](https://github.com/ipa-fxm/ipa_seminar/blob/master/ipa_seminar_manipulation/setup_env_manipulation.bash "Setup Shell").

#### 1.1. Simulation

# call sim alias  

In order to use MoveIt! with a simulated robot, simply run:
```
roslaunch lbr_bringup sim.launch
```
This will start up the [Gazebo](http://gazebosim.org "Gazebo") simulation with the [KUKA LBR](http://www.kuka-labs.com/en/medical_robotics/lightweight_robotics/start.htm "KUKA LBR") model.  
Use the mouse to rotate/translate the view or zoom.

![Gazebo Simulation](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/gazebo_simulation.png?login=ipa-fxm&token=36e39b9f21a46033bd0331aeb8be8c5f "Gazebo Simulation")

# make sure configs are safe -- check arm_1_joint!

The simplest way of moving the robot is by using the cob_commmand_gui. Therefore run in a new terminal:
```
roslaunch lbr_bringup dashboard.launch
```
Clicking one of the buttons will move the robot to the according (pre-defined) configuration.  
__NOTE:__ This is __unplanned__ motion!

![Command GUI](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/command_gui_lbr.png?login=ipa-fxm&token=85d23ec80df4c8404c58809cc869f31c "Command GUI")

#### 1.2 Robot Hardware

![CAUTION](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/yellow-warning.gif?login=ipa-fxm&token=de7de97c787a393b6b8acf19dd87890e "CAUTIOIN")![CAUTION](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/yellow-warning.gif?login=ipa-fxm&token=de7de97c787a393b6b8acf19dd87890e "CAUTIOIN")![CAUTION](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/yellow-warning.gif?login=ipa-fxm&token=de7de97c787a393b6b8acf19dd87890e "CAUTIOIN")  
__CAUTION:__
* Only use the robot under supervision of your tutor
* Make sure you have a clear view to the robot and the emergency button is in your hands before executing a command 
* In case of unexpected behavior or in case the robot is about to collide hit the emergency button immediately
* After the emergency button has been pressed, ask your tutor to recover the robot

# call robot alias  


In order to use MoveIt! with the real robot hardware, the robot needs to be initialized first. Ask your tutor to do so.  
Then, simply run:
```
roslaunch lbr_bringup robot.launch
```
This will start up all necessary drivers.  
Similar to simulation, the cob_command_gui can be used to perform __unplanned__ motion - be careful!


### 2. MoveIt! - Setup Assistant  

MoveIt! requires a little  configuration before offering its capabilities to your robot. The configuration can easily be done with the MoveIt! Setup Assistant - a graphical tool that comes shipped with MoveIt! automatically.

The following steps will lead you to a valid MoveIt!-configuration for the KUKA LBR.  
A PR2-specific tutorial can be found [here](http://moveit.ros.org/wiki/PR2/Setup_Assistant/Quick_Start "PR2-SetupAssistant")  

#### 2.1. Start  

To start the MoveIt-SetupAssistant GUI, first run: 
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
You should see the screen below.

![SetupAssistant1](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant1.png?login=ipa-fxm&token=88811838d3283279087c8437b58604e8 "SetupAssistant1")

As we are creating the configuration for the first time, select "Create New MoveIt Configuratin Package".  
Now we need to load the robot model (URDF) on which basis the configuration is generated.  
Browse to _URDF-LOCATION_ and load the lbr_solo.urdf.xacro file.  
You should now see our robot within the SetupAssistant.  

![SetupAssistant2](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant2.png?login=ipa-fxm&token=4badbe3dec86cf96f070bb4638e0255b "SetupAssistant2")


#### 2.2. Self-Collision  

In order to speed up later collision checking, a _SelfCollisionMatrix_ can be computed in the next tap ("Self Collision").  
Select the highest _Sampling Density_ and click "Regenerate Default Collision Matrix".  

![SetupAssistant3](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant3.png?login=ipa-fxm&token=81361c40216514f1463a42152d612d9f "SetupAssistant3")

After about 10 seconds you will see a list of pairs of links which never or always collide within the robot model. This knowledge is used to speed up Self-CollisionChecking as these checks need not to be done during a planning step again.

![SetupAssistant4](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant4.png?login=ipa-fxm&token=545970dafcf3398c9571fe160220335a "SetupAssistant4")


#### 2.3. Virtual Joints  

The next tap "Virutal Joints" is less important for our scenario.  
This tap gets important in case you want to use MoveIt! with a mobile robot. A _virtual joint_ connects the robot to the world. While mobile robots would have _planar_ (2d) or _floating_ (6d) virtual joints, we simply define a _fixed_ virtual joint stating that our robot does not move.  

![SetupAssistant5](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant5.png?login=ipa-fxm&token=232f90b281c043158520e71cf8d46934 "SetupAssistant5")

#### 2.4. Planning Groups  

In step four ("Planning Groups") we define collections of links and joints of the robot and declare them as _Planning Groups_. Each Planning Group defines semantically related parts of the robot. For each Planning Group defined here MoveIt! will generate a configuration in order to perform motion planning later.  

For this tutorial we add two Planning Groups:  

1. Group "arm"  
Select `kdl_kinematics_plugin/KDLKinematicsPlugin` as _Kinematic Solver_. This is a numerical Solver for Inverse Kinematics.  
Use the _Add Kin. Chain_ option to assign the kinematic chain starting with _arm_0_link_ (Base Link) and ending with _arm_7_link_ (Tip Link).  

2. Group "gripper"  
For the gripper does not need a kinematic solver. Thus we leave it _None_.  
As the gripper group consists of just one link (_gripper_link_), we use the _Add Links_ option for assigning.  

![SetupAssistant6](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant6.png?login=ipa-fxm&token=8e7c227e3436b613fbfae736f409f2e6 "SetupAssistant6")  

![SetupAssistant7](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant7.png?login=ipa-fxm&token=7b8c8aebe83686659b6743c843923ba7 "SetupAssistant7")

#### 2.5. Robot Poses  

The tap "Robot Poses" allows us to define some _robot poses_. These robot poses can later be used as goals for a motion plan within the RVIZ plugin or the CommandLineTool.  
Now define some poses for the group "arm". Use the sliders to set the joint values of each joint within the group.  
In the view you can see what the currently set configuration would look like. In case the current configuration is in collision a notification is displayed.  

![SetupAssistant8](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant8.png?login=ipa-fxm&token=6056b6fa7b6f593a86dcc178c2fcb185 "SetupAssistant8")

![SetupAssistant9](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant9.png?login=ipa-fxm&token=190bf7b0c962fee77f22f29368b0314a "SetupAssistant9")
#### 2.6. End Effectors  

Next, we define our gripper to be the _End Effector_ for our arm.  
This will give us an _Interactive Marker_ for moving the arm in the RVIZ Plugin. This is also important for High(er)-Level Capabilities of MoveIt! - such as _Pick-and-Place_.  

![SetupAssistant10](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant10.png?login=ipa-fxm&token=ba81dec7bb9bb0a460540a901e99a080 "SetupAssistant10")

#### 2.7. Passive Joints  

This tap ("Passive Joints") is not relevant for our scenario. So we can just skip it.

#### 2.8. Configuration Files  

In the final step, the MoveIt! SetupAssistant generates all files required for MotionPlanning for us automatically.  
You can see a list of the files to be generated below. It comprises configuration files as well as startup files.
The only thing we need to do is to specify a location where the files should be stored. Then press _Generate Package__.  
After the files have been generated, we can click _Exit Setup Assistant_.

![SetupAssistant12](https://raw.github.com/ipa-fxm/ipa_seminar/master/ipa_seminar_manipulation/doc/SetupAssistant12.png?login=ipa-fxm&token=484c5aed30d0d5cb393fe1bbf8c6be84 "SetupAssistant12")


#### 2.9 Summary  

We now created a MoveIt! configuration package that provides us with all configuration files and basic startup files.  
The ROS package can be found at _MOVEIT_CONFIG_LOCATION_.  
The package includes the following files (amongst others):  
* __config/joint_limits.yaml__: This file specifies velocity and acceleration limits for all joints of the robot. Position limits are defined in the URDF already.
* __config/kinematics.yaml__: This file specifies the kinematic solvers to be used with each group.
* __config/ompl_planning.yaml__: This file specifies a set of several (sampling-based) motion planner from the OMPL motion library with according default settings.
* __config/lbr_solo.srdf__: This SRDF (Semantic Robot Description Format) file holds the main configuration to be used with MoveIt!
* __launch/demo.launch__: This is the most basic startup file. It opens RVIZ where the planning capability can be tested without needing to run either a simulation nor a robot hardware.
* __launch/move_group.launch__: This file will be used when MoveIt! is to be used in connection with a simulation or a robot hardware

Both of these launch files combine other launch files contained in the launch folder. Those other files start up a specific module for MoveIt! respectively.

Whenever something needs to be changed within the MoveIt! configuration package, the Setup Assistant can simply be run again with the already existing package being loaded instead of creating a new package. Running the Setup Assistant again will only update files where changes apply.  

The configuration files can also be modified manually. In fact, we will do so during the remainder of this tutorial.  
In such case the Setup Assistant will notify you that configuratin files have been edited outside the Setup Assistant.  


### 3. MoveIt! - RVIZ-Plugin  

We will now use MoveIt! in its 

#### 3.1. Plugin Environment Basics  

#### 3.2. Planning Request  

### 4. MoveIt! - CommandLine Tool  

### 5. MoveIt! - Scripting API  

#### 5.1. PlanningSceneInterface  

#### 5.2. MoveGroupCommander  

### 6. Help  

