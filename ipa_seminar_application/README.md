<a id="top"/> 
# Application development with ROS
## Tutorial

### Contents

1. <a href="#1--introduction">Introduction</a>
2. <a href="#2-Running-a-pick-and-place-application-with-SMACH">Running a pick and place application</a>  


### 1.  Introduction

This section quickly introduces basic tools used within this tutorial.  
It also lists several helpful (terminal) commands that are used frequently throughout this tutorial.  

#### 1.1. Tools  

We use _Terminator_ as a command prompt. You can find a shortcut on the left bar of your screen. Commands given in the following can be copied into the _Terminator_ window. During the tutorial sometimes several terminal windows are needed. The terminal can be split into several sub-windows by using the _Split Horizontically/Vertically_ feature after a right-click on the _Terminator_ window.  

We wil use _gedit_ as a text edior. You can find a shortcut on the left bar of your screen as well. The editor can also be opened by running `gedit` in a terminal window.  

#### 1.2. Helpful Commands

For navigating to specific ROS packages or files, the easiest way to do so is to use the command `roscd <package_name>` where `<package_name>` is the name of the ROS package you want to navigate to, e.g. `lbr_bringup`.  `roscd` brings you to the desired package from any previous location. After the `<package_name>`, pressing `TAB` can be used for auto-completion in order to navigate further within the ROS package.  

For editing a file simply use `roscd` to navigate to the folder of the file and then type `gedit <file_name>` where `<file_name>` is the name of the file you want to edit.  

For more helpful ROS commands have a look at the [ROS Cheat Sheet](http://download.ros.org/downloads/ROScheatsheet.pdf "ROS Cheat Sheet").  

For convenience, all required ROS packages are already installed and the environment (i.e. environment variables) is set up correctly. Whenever an environment variable (e.g. ROS_MASTER_URI) needs to be changed this is stated below.  

Also, an example solution for this tutorial is already available. Whenever you want to have a look at the solution files or you want to run the solution, simply type `solution` in the terminal before opening the respective file or running the command.  

<a href="#top">top</a> 

#### SMACH
SMACH is a finite state machine programming approach in ROS. It is based on python and allows to define states as basic building blocks as well as run them in various containers and compose them to state machines. State machines can on the other hand consist of sub-state machines again. You can find more information about SMACH at the [SMACH wiki page](http://wiki.ros.org/smach).

#### Exporting ROS_MASTER_URI
The `ROS_MASTER_URI` can be used to connect to different ROS cores. In this tutorial we'll use several robots (partly real, partly in simulation). To connect to the correct robot to run your applications you will have to export an environment variable called `ROS_MASTER_URI`. Here's a list of the robots we're using and their corresponding `HOSTNAME`.
* Universal Robot UR10 (real):  `robot-ur-real`
* Universal Robot UR10 (sim):   `robot-ur-sim`
* Kuka LBR (sim):               `robot-lbr`
* Schunk LWA4d (sim):           `robot-lwa4d`
You can export the `ROS_MASTER_URI` with
```
export ROS_MASTER_URI=http://<<HOSTNAME>>:11311
```
So for the simulated Kuka LBR this would be
```
export ROS_MASTER_URI=http://robot-lbr:11311
```


### 2. Running a pick and place application with SMACH  
**Task**: Run a pick and place application for the universal robot arm.
**Goal**: Lern how to start and monitor a SMACH application.

#### Start the application
To start the application:
```
roslaunch ipa_seminar_application_pick_and_place pick_and_place.launch
```

#### Monitor execution
There's a graphical tool which visualizes the states and transitions. The current state or sub-state machine is highlighted. You can start the tool with
```
rosrun smach_viewer smach_viewer.py
```

#### Coding details
There are mainly four files involved here:
`pick_and_place_states.py`:   Defines basic states (=building blocks for our application)
`pick_and_place.py`:          Defines the pick and place application (=coordination for our application)
`application_config.yaml`:    Defines the target areas (configuration for our application)
`pick_and_place.launch`:      Defines which components need to be started (=deployment of our application)

IMAGE BUILDING BLOCKS

The image shows the atomic building blocks (basic states and sub-state machines) which can be used for our application.
Basic states:
* prepare_robot()             Brings the robot into a defined starting position
* move_planned(pose)          Moves to a given pose avoiding collisions
* move_lin(pose)              Moves to a given pose on a straight line avoiding collisions
* open_gripper()              Opens the gripper
* close_gripper()             Closes the gripper
Sub-state machines:
* pick_object(area)           Picks up an object from a given target area (uses move_planned, move_lin and close_gripper)
* place_object(area)          Places an object on a given target area (uses move_planned, move_lin and open_gripper)


### 3. Run the pick an place application continuously
**Task**: Modify the application in a way that it runs continuously

#### Modify the application code
We will have to modify the state machine which coordinates the pick and place application. At the moment the state machine looks like this:

SCREENSHOT PICK_AND_PLACE SINGLE

Now we want to have a continuously running pick and place application which should look like this:

SCREENSHOT PICK_AND_PLACE CONTINUOUS

You can open the pick and place application file with
```
roscd ipa_seminar_application_pick_and_place
gedit src/pick_and_place.py
```
In the file you'll find the `pick_and_place_object` class which defines a SMACH state machine out of two sub-state machines `pick_object` and `place_object`. 
```
class pick_and_place_object(smach.StateMachine):
	def __init__(self, source_area, target_area):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

		with self:
			smach.StateMachine.add('PICK_OBJECT', pick_object(source_area),
				transitions={'object_picked':'PLACE_OBJECT', 
							'object_not_picked':'failed',	
							'error':'error'})

			smach.StateMachine.add('PLACE_OBJECT', place_object(target_area),
				transitions={'object_placed':'succeeded',
							'object_not_placed':'failed',
							'error':'error'})

```
Each state has a name, a type and a list of transitions which connect the outcome of a state to the next state. Our state machine itself has a list of three outcomes ```outcomes=['succeeded', 'failed', 'error']```. All transitions inside the state machine should either point to another state inside the same state machine or point to an outcome of the state machine. If we connect the `object_placed` outcome of the `PLACE_OBJECT` state to `PICK_OBJECT` we will continuously pick and place an object from the `source_area` to the `target_area`. Please modify the `PLACE_OBJECT` state like this:
```
smach.StateMachine.add('PLACE_OBJECT', place_object(target_area),
	transitions={'object_placed':'PICK_OBJECT',
				'object_not_placed':'failed',
				'error':'error'})
```

To start the application again and check your modifications:
```
roslaunch ipa_seminar_application_pick_and_place pick_and_place.launch
```




Thinks to keep in mind while developing hardware independent applications:
* do not use hardcoded joint space positions or trajectories. Use cartesian poses or semantic names to describe your application.
* separation of concerns
* separation of roles

Based on the standardizes ROS API to the driver layer and the higher level capabilities (e.g. motion planning) it is possible to define an application which is hardware independent.
