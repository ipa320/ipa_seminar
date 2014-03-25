# ROS driver programming using BRIDE
## Tutorial

The goal of this tutorial is to create a ROS driver for the KUKA KR16 robot using the BRIDE tool-chain for model-driven engineering. Basic information and details about Bride are available [here](http://wiki.ros.org/bride).

### 1.  Startup BRIDE and create a new project

#### 1.1.  Installing BRIDE

For installing BRIDE please follow the [BRIDE installation instructions](https://github.com/ipa320/bride/wiki/Installation).

#### 1.2.  Running the BRIDE IDE
You can startup bride from your terminal by calling
```
rosrun bride eclipse
```
After the start of BRIDE you will be asked for a workspace directory. Choose the /src directory that lies in your [catkin](http://wiki.ros.org/catkin) workspace e.g. /home/ros/catkin_ws/src.

When you close the welcome page, you should now see the eclipse IDE on your desktop.

![eclipse_startup](./doc/eclipse_startup.png "eclipse IDE after startup")

#### 1.2.  Creating a new BRIDE project

You can now create a new project by choosing the wizard in File -> New -> Project and then choose C/C++ -> C++ Project. Hit the "Next>" button to continue.

![eclipse_create_project1](./doc/eclipse_create_project1.png "Creating a BRIDE project with the eclipse IDE: Step 1")

In the wizard give the new project the name of the package you want to create and choose as project type "Makefile project" -> "Empty Project". In the case of the tutorial we want to create the package "kr16_driver". 

![eclipse_create_project2](./doc/eclipse_create_project2.png "Creating a BRIDE project with the eclipse IDE: Step 2")

Finish the wizard by hitting the "Finish" button and accept the opening of the C/C++ perspective when being asked
 
![eclipse_create_project3](./doc/eclipse_create_project3.png "Creating a BRIDE project with the eclipse IDE: Step 3")

### 2.  Creating the driver model

Create a new folder in the kr16_driver project called "model" by right clicking on the project in the project explorer on the left and choose New -> Folder.

![eclipse_new_folder](./doc/eclipse_new_folder.png "Creating a new folder: Step 1")
![eclipse_new_folder2](./doc/eclipse_new_folder2.png "Creating a new folder: Step 2")

Now create a new capability model by right clicking on the new folder and choosing New -> Other... and then "ROS Package Diagram" in the BRIDE section. Follow the wizard by giving the RosPackage Diagram the name of the node. In the case of this tutorial "kr16_driver.ros_package_diagram". 

![bride_model_creation1](./doc/bride_model_creation1.png "Creating a BRIDE model: Step 1")

Then on the next page make sure the name of the model is also the name of the node. In this tutorial it should be kr16_driver.ros_package. 

![bride_model_creation2](./doc/bride_model_creation2.png "Creating a BRIDE model: Step 2")

Now you can finish the wizard.

You now should have 2 new files inside the model directory and the graphical editor for your new model should be opened. Choose the "Capability Developer" perspective by hitting the "Open Perspective" button in the upper right corner of the eclipse IDE.

![capability_perspective](./doc/capability_perspective.png "Opening the capability persprctive")

With the palette on the right you can now add a "Node" to the graphical plane by selecting "Node" and drawing a rectangle. Name the node "kr16_driver".

![create_a_node](./doc/create_a_node.png "Creating a node")

Following the [ROS-I driver specs](http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec), add a Publisher called "joint_states" and "states" and an ActionServer called "folllow_joint_trajectory_action" by selecting the corresponding type on the right pane and clicking on the corner of the graphical node representation. You should then see the following screen: 

![create_action_pub](./doc/create_action_pub.png "Creating a publisher and action")

Now we have to do a few setting for the package and the nodes. Select the properties pane in the lower area of the screen. When you click in the white area of the screen you should see the package properties below. Fill them out as in the screenshot below: 

![package_properties](./doc/package_properties.png "Setting package properties")

Now select the other components you created in order to show their settings in the property pane. Fill out the settings as in the following screens: 

![publisher_properties](./doc/publisher_properties.png "Setting properties for the joint_states publisher")
![publisher_properties2](./doc/publisher_properties2.png "Setting properties for the state publisher")
![action_server_properties](./doc/action_server_properties.png "Setting properties for the action server")

Follwoing the [ROS-I driver specs](http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec), we now add the following parameters to the node model: robot_ip_address (ip address of the robot), robot_description (urdf description parameter) and robot_port (the port the robot is listening to. Not defined in the spec.). Your model should then look as follows

![kr16_driver_model](./doc/kr16_driver_model.png "The complete driver model")

Again, we have to set the properties for the node paramteres, by selecting each of them and entering the following information:

![ip_properties](./doc/ip_properties.png "Setting properties for the robot_ip_address parameters")
![description_properties](./doc/description_properties.png "Setting properties for the robot_ip_description parameters")
![port_properties](./doc/port_properties.png "Setting properties for the robot_port parameters")

Finally, our driver model is now complete. Congratulations! (but don't forget to save your model)


### 3.  Generating source code from the model

You can now generate the source code of the node automatically based on the model you created. This is done by selecting BRIDE -> ROS -> "Generate C++ Code" from the menu. Be sure you saved your diagram before generating. When you refresh your project in the project explorer by right-clicking on the project and selecting "Refresh". You should now see the complete ROS package structure in the project explorer as in the screenshot below: 

![generating_code_from_model](./doc/generating_code_from_model.png "Generating source code form the model")

### 4.  Integrating the driver capabilities

To implement the actual capability code for your ROS driver you have to only focus on the kr16_node_common.cpp file in the common/src/ directory. You can open the file by double clicking it in the project explorer. The file contains protected regions you can use for your code that will not be overwritten once you regenerate the node by the tool chain.

Once the file is open you can now go to the kr16_node_impl class and edit the code as follows: 

```
class kr16_node_impl
{
	/* protected region user member variables on begin */
	/* protected region user member variables end */

public:
    kr16_node_impl() 
    {
        /* protected region user constructor on begin */
		/* protected region user constructor end */
    }
    void configure(kr16_node_config config) 
    {
        /* protected region user configure on begin */
		/* protected region user configure end */
    }
    void update(kr16_node_data &data, kr16_node_config config)
    {
        /* protected region user update on begin */
		/* protected region user update end */
    }

    
    void callback_follow_joint_trajectory_action_(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal, actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *as_)
    {
    	/* protected region user implementation of action callback for follow_joint_trajectory_action on begin */
		/* protected region user implementation of action callback for follow_joint_trajectory_action end */
    
    }
    


    
    /* protected region user additional functions on begin */
	/* protected region user additional functions end */
    
};
```
You can now build the node by executing catkin_make in your catkin workspace on the console
```
cd /home/ros/catkin_ws
catkin_make
```
Congratulation you just created your first model-based ROS node.

### 5.  Create a corresponding node to execute motion commands

To create another node that can listen to the talker node you should create a new C++ Project as above and call it "kr16_test_client". Now again create a new folder "model" and a "New -> Other..." and then "ROS Package Diagram" from the BRIDE section. Name the new diagram kr16_test_client.ros_package_diagram" and the new model "kr16_test_client.ros_package". 

![filestructure_test_client](./doc/filestructure_test_client.png "File structure for the kr16_test_client")

Now click on the white are of the daring panel and fill out the package property as follows: 

![tc_package_properties](./doc/tc_package_properties.png "Package parameters for the kr16_test_client")

You can now add a action client named "follow_joint_trajectory_action", a subscriber named "state" and two parameters called "joint_poisition_1" and "joint_position_2" to the new node. 

![tc_model](./doc/tc_model.png "The kr16_test_client model")

Edit their properties as follows:

![tc_node_properties](./doc/tc_node_properties.png "Node properties for the kr16_test_client")
![tc_sub_properties](./doc/tc_sub_properties.png "Subscriber properties for the kr16_test_client")
![tc_action_properties](./doc/tc_action_properties.png "Action client properties for the kr16_test_client")
![tc_param1_properties](./doc/tc_param1_properties.png "Parameter properties for the kr16_test_client")
![tc_param2_properties](./doc/tc_param2_properties.png "Parameter properties for the kr16_test_client")

After generating the code out of the model you can alter the update function of the kr16_test_client_impl class of the common/src/kr16_test_client_common.cpp file as follows: 
```
class kr16_test_client_impl
{
	/* protected region user member variables on begin */
	/* protected region user member variables end */

public:
    kr16_test_client_impl() 
    {
        /* protected region user constructor on begin */
		/* protected region user constructor end */
    }
    void configure(kr16_test_client_config config) 
    {
        /* protected region user configure on begin */
		/* protected region user configure end */
    }
    void update(kr16_test_client_data &data, kr16_test_client_config config)
    {
        /* protected region user update on begin */
		/* protected region user update end */
    }

    


    
    /* protected region user additional functions on begin */
	/* protected region user additional functions end */
    
};
```
### 6.  Using the BRIDE system model for deployment

Create a new Eclipse project by "File -> New -> Project" and then "General -> Project". Give the project a name, e.g. "kr16_deployment" and finish the creation wizard. Afterwards you can create a new folder named "model" as when creating capability models.

Now you can create a new deployment model by right clicking on the model folder and selecting "New -> Other..." and then "ROS System diagram" (note the difference to "ROS Package Diagram") below the BRIDE folder. Now name the new model diagramm "kr16_deployment.ros_system_diagram" and then after clicking next name the model "kr16_deployment.ros_system". You should now see an empty ros_diagram editor. 

![dp_filestructure](./doc/dp_filestructure.png "File structure of the kr16_deployment package")

#### 6.1  Adding packages to the system model

To add capability models to your system you select the Package tool on the right and click on the white area of the ros system. You can now select the Capability and Coordinator models from the dialog (one only). Repeat the step until you have added the  kr16_driver and the kr16_test_client. The screenshot below shows BRIDE while adding the kr16_driver and the kr16_test_client. 

![dp_select_packages](./doc/dp_select_packages.png "Adding nodes to the system model")

#### 6.2  Connecting actions, services and topics

In the system diagram you can now use the topic tool in the pallet to connect the Publishers with the corresponding Subscribers and name the topic. Similar you can connect the services and actions in the system. In this example we connect the "state" Publisher port of the kr16_driver with the "state" subscriber port of the kr16_test_client node. We name the topic "/arm_controller/state". From now on the topic will be show up in ROS under this name (e.g. when using "rostopic list" command).

Afterwards connect the action server and the action client and name the connection /arm_controller/follow_joint_trajectory_action. Your model should then look similar to this picture:

![system_model](./doc/system_model.png "System model")

Finally, edit the package settings by clicking on the white area of the drawing pane and give the system model the name kr16_deployment

#### 6.2  Generating launch files and the manifest

After finishing the system configuration you can generate the manifest and launch file for the system package. You can do that in the BRIDE menu in ROS->"Generate Launch file for ros system". After refreshing your project explorer by right clicking the kr16_deployment package and pressing "Refresh" you should now see a "manifest.xml" that includes all dependencies and an "system.launch" file that configures and connects all nodes in the system. 

#### 6.3  Testing the system

When you open a new terminal you should be able to run
```
roslaunch kr16_deployment system.launch 
```
With the standard ros tools (e.g. rostopic, rxgraph, etc.) you should now be able to investigate your system. If so, you are ready to move on to the real robot!


