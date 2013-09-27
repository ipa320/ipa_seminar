<a id="top"/> 
# Application development with ROS
## Tutorial

### Contents

1. <a href="#1--introduction">Introduction</a>   
  1.1. <a href="#11-tools">Tools</a>  
  1.2. <a href="#12-helpful-commands">Helpful Commands</a>  
2. <a href="#2-moveit---setup-assistant">MoveIt! - Setup Assistant</a>  
  2.1. <a href="#21-start">Start</a>  
  2.2. <a href="#22-self-collision">Self-Collision</a>  
  2.3. <a href="#23-virtual-joints">Virtual Joints</a>  
  2.4. <a href="#24-planning-groups">Planning Groups</a>  




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

A pick and place application for the universal robot arm is already prepared.

#### Start the application

To start the application:
```
roslaunch ipa_seminar_application_pick_and_place pick_and_place.py
```

#### Coding details
There are mainly two files involved here:
`pick_and_place_states.py`: asd
`pick_and_place.py`: asd






Based on the standardizes ROS API to the driver layer and the higher level capabilities (e.g. motion planning) it is possible to define an application which is hardware independent.
