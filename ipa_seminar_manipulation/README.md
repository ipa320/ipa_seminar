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

First run 
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
to start the MoveIt-SetupAssistant GUI. You should see the screen below.



In the first tap, we need to load the robot model (URDF) on which basis the configuration is generated.  
Browse to _URDF-LOCATION_ and load the lbr_solo.urdf.xacro file.  
You should now see our robot within the SetupAssistant.  




#### 2.2. Self-Collision  

#### 2.3. Virtual Joints  

#### 2.4. Planning Groups  

#### 2.5. Robot Poses  

#### 2.6. End Effectors  

#### 2.7. Passive Joints  

#### 2.8. Configuration Files  

### 3. MoveIt! - RVIZ-Plugin  

#### 3.1. Plugin Environment Basics  

#### 3.2. Planning Request  

### 4. MoveIt! - CommandLine Tool  

### 5. MoveIt! - Scripting API  

#### 5.1. PlanningSceneInterface  

#### 5.2. MoveGroupCommander  

### 6. Help  

