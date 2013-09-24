Mobile robot navigation with ROS
===========
<a id="top"/>

### Contents

1. <a href="#1-prerequisites">Prerequisites</a>
2. <a href="#2-using-the-mobile-robot">Using the mobile robot</a>
3. <a href="#3-mapping-the-environment">Mapping the environment</a>
4. <a href="#4-localizing-in-the-environment">Localizing in the environment</a>
5. <a href="#5-path-planning-using-elastic-band">Path planning using Elastic Band</a>
6. <a href="#6-writing-a-small-application">Writing a small application</a>
7. <a href="#7-optional-using-3d-sensors-in-navigation">Optional: Using 3D sensors in navigation</a>
8. <a href="#8-help">Help</a>


### 1. Prerequisites
The navigation tasks in this tutorial can be run both on the real robot and in simulation. Depending on where you want to run the task you have to start different prepared bringup scripts. The ROS Master will always be the local PC.

To run the robot in simulation please use:

	roslaunch seminar_navigation simulation_bringup.launch

This launchfile will start a gazebo simulation of the robot and the environment with the necessary hardware and sensor interfaces.

To run the real mobile robot make sure you have the "robot flag", that means you tell the others that you take control of the robot. Afterwards you can start the drivers of the mobile platform running:

	roslaunch seminar_navigation robot_bringup.launch

<a href="#top">top</a> 
### 2. Using the mobile robot

Start the robot on a terminal:

	roslaunch seminar_navigation robot_bringup.launch
	
Now start the prepared rviz configuration to show the current state of the robot.

	roslaunch seminar_navigation rviz_joystick.launch

The rviz window should look like this:

![RVIZ during free moving of robot](/doc/rviz_joystick.png)

Note following rviz settings:

* base_link origin
* Activated Laserscanner plugins for front and rear
* Activated robot_model plugin

To initialize the mobile base press the deadman button on the joystick (see picture below) and the start button at the same time. (not required in simulation)

You can now move around the robot using the joystick. Use the deadman button and the two analog joysticks. You will notice how the laser scans show the environment and that you can move tranlational and rotational at the same time. Try to get a feeling how the platform move, as you need to move the robot lateron during mapping.


<a href="#top">top</a> 
### 3. Mapping the environment

Start the robot on a terminal:

	roslaunch seminar_navigation robot_bringup.launch
	
In a second terminal you can start the gmapping localization using:

	roslaunch cob_mapping_slam 2dslam.launch
	
Start rviz config

	roslaunch seminar_navigation rviz_gmapping.launch

The rviz window should look like this:

![RVIZ during map creation](/doc/rviz_gmapping.png)


Note following rviz settings:

* map origin
* Activated map plugin with topic /map
* Activated Laserscanner plugins for front and rear
* Activated robot_model plugin

After initialization of the base (deadman + start) you can move around using the joystick. Notice the map getting created step by step.

When finished go into the scenario folder and save the map by running the following commands

	roscd seminar_navi_scenario/map
	rosrun map_server map_saver

Analyse created files map.yaml and map.pgm.


<a href="#top">top</a> 
### 4. Localizing in the environment
Start the robot on a terminal:

	roslaunch seminar_navigation robot_bringup.launch
	
In a second terminal you can start the amcl with the just created map:

	roslaunch seminar_navigation amcl.launch
	
Start rviz config

	roslaunch seminar_navigation rviz_amcl.launch

Note following rviz settings:

* map origin
* Activated map plugin with topic /map
* Activated Laserscanner plugins for front and rear
* Activated pose estimate plugin
* Activated robot_model plugin

Use set_position estimate tool in rviz to set initial localization. See the picture below:

![RVIZ during localization with amcl](/doc/rviz_amcl.png)

Move around using joystick and see the localization converge.

Analyze tf frames by activating tf plugin. Find and watch frame /map, /odom\_combined and /base\_link.

<a href="#top">top</a> 
### 5. Path planning using Elastic Band

Start bringup
Start amcl and move_base with just created map
Start rviz config
Note following rviz settings:

* map origin
* Activated map plugin with topic /map
* Activated Laserscanner plugins for front and rear
* Activated pose estimate plugin
* Activated costmap plugin
* Activated marker plugin
* Activated robot_model plugin

Use set_position estimate tool in rviz to set initial localization

<a href="#top">top</a> 
### 6. Writing a small application
<a href="#top">top</a> 
### 7. Optional: Using 3D sensors in navigation



<a href="#top">top</a> 
### 8. Help  



===

In case of questions - now or later - do not hestate to contact your navigation expert at Fraunhofer IPA:

Dipl.-Ing. Alexander Bubeck
e-mail: [alexander.bubeck@ipa.fraunhofer.de](mailto: alexander.bubeck@ipa.fraunhofer.de)
phone: +49 711 970-1314

<a href="#top">top</a> 
