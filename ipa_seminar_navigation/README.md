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
7. <a href="#7-optional-online-configuration-of-navigation">Optional: Online configuration of navigation</a>
8. <a href="#8-help">Help</a>


### 1. Prerequisites
The navigation tasks in this tutorial can be run both on the real robot and in simulation. Depending on where you want to run the task you have to start different prepared bringup scripts. The ROS Master will always be the local PC.

To run the robot in simulation please use:

	roslaunch seminar_navigation simu_bringup.launch

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

* Fixed frame: base_link
* Activated Laserscanner plugins for front and rear
* Activated robot_model plugin

To initialize the mobile base press the "deadman" button on the joystick (see picture below) and the "Init" button at the same time. (not required in simulation)

![Position of buttons on the joystick](/doc/Joystick.png)

You can now move around the robot using the joystick. Use the "deadman" button and the two analog joysticks for base movement. You will notice how the laser scans show the environment and that you can move tranlational and rotational at the same time. Try to get a feeling how the platform move, as you need to move the robot lateron during mapping.


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

* Fixed frame: map 
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

* Fixed frame: map
* Activated map plugin with topic /map
* Activated Laserscanner plugins for front and rear
* Activated pose estimate plugin
* Activated robot_model plugin

Use "2D pose estimate" tool in rviz to set initial localization. See the picture below:

![RVIZ during localization with amcl](/doc/rviz_amcl.png)

Move around using joystick and see the localization converge.

Analyze tf frames by activating tf plugin. Find and watch frame /map, /odom\_combined and /base\_link.

<a href="#top">top</a> 
### 5. Path planning using Elastic Band
Start the robot on a terminal:

	roslaunch seminar_navigation robot_bringup.launch
	
Now start the prepared launch file to start the overall navigation with move_base and amcl:

	roslaunch seminar_navigation navigation_eband.launch

To control the robot we will use RVIZ again:

	roslaunch seminar_navigation rviz_eband.launch

Note following rviz settings:

* Fixed frame: map
* Activated map plugin with topic /map
* Activated Laserscanner plugins for front and rear
* Activated pose estimate plugin
* Activated costmap plugin
* Activated marker plugin
* Activated robot_model plugin

Once everything is started you should localize the platform again using the "2D pose estimate" tool (see previous section). Now you can use the "2D nav goal" tool right of the other tool from rviz to command goals to the mobile platform. See the picture below:

![RVIZ during localization with amcl](/doc/rviz_amcl.png)

While commanding the goals to the platform you can see the global path that was planned by move_base as well as the green bubles that are representing the reactive path. 

Keep the navigation running for the next part of the seminar.

<a href="#top">top</a> 
### 6. Writing a small application

To write an application with the navigation system you will have to write a small script that triggers different movements.
An example of an application is the following. Please create a new file with gedit in the seminar\_navi\_scenario package and copy the following in the file:

	#!/usr/bin/python
	import roslib
	roslib.load_manifest('cob_script_server')
	import rospy

	from simple_script_server import script

	class MyScript(script):
    		def Initialize(self):
        		if(self.sss.parse == False):
            			rospy.loginfo("Please set initial pose in RVIZ")
            			raw_input("Press Enter when done")

    		def Run(self):
        		rospy.loginfo("Running script...")
        		self.sss.move("base",[-0.831, 0.082, -0.376]) # Moving base to position x, y, yaw with unit [m, m, rad]
        		self.sss.move("base",[-0.286, -4.197, 0.754])

	if __name__ == "__main__":
    		SCRIPT = MyScript()
    		SCRIPT.Start()

Note that this script has already filled out positions to move to. This positions will not fit to your map and have to be tought again.
Therefore move the platform to the desired position using the joystick and find out the position with TF. Besides looking the TF position up in RVIZ you can also use the following terminal call:

	rosrun tf tf_echo /map /base_link

This will command the current position of the /base_link in the /map coordinate system. Take the x and y value and the last elemement of the rpy rotation (the rotation around the z-axis) and fill it in the application.
Do the same process by moving with the joystick and finding out the position with the second target position.

Now we finished creating the script. To actually run it we have to move to the seminar\_navi_scenario package and run the python script:

	roscd seminar_navi_scenario
	python ./script_name.py
	
Watch the base moving on the area and in RVIZ.
Now extend the script with more positions to let the platform move in each part of the area.


<a href="#top">top</a> 

### 7. Optional: Online configuration of navigation  

When running the created script you can adapt the planning algorithm during runtime by using the dynamic\_reconfigure tool. To do so run the following command in a terminal:

	rosrun rqt_reconfigure rqt_reconfigure
	
You should now see a window like in the picture below:

![Online configuring the navigation](/doc/configure.png)

You can now change the parameters during execution and see what happens.


<a href="#top">top</a> 

### 8. Help  

You will can find online help on the documentation pages of the cob_navigation and ROS navigation stack at:

	http://wiki.ros.org/cob_navigation
	http://wiki.ros.org/navigation


===

In case of questions - now or later - do not hestate to contact your navigation expert at Fraunhofer IPA:

	Dipl.-Ing. Alexander Bubeck
	e-mail: alexander.bubeck@ipa.fraunhofer.de
 	phone: +49 711 970-1314

<a href="#top">top</a> 
