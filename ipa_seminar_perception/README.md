# 3D Perception with ROS and PCL
## Tutorial

### 1.  3D camera driver

In this tutorial an Asus XTion RGBD camera is used. The ROS driver is located in the package [openni_camera](http://www.ros.org/wiki/openni_camera) and the launch files in [openni_launch](http://www.ros.org/wiki/openni_launch).

#### 1.1.  Run the driver

To start the driver, run in a new terminal
```
roslaunch openni_launch openni.launch
```
To see the output topics, type
```
rostopic list
```
You should see a lot of topics starting with /camera/.

#### 1.2.  Visualize the camera data in RVIZ

To open rviz, run
```
rosrun rviz rviz
```
![rviz1](./doc/rviz1.png "Plain RVIZ")

Click on "Add" and select "PointCloud2":

![rviz2](./doc/rviz2.png "Add PointCloud")

You will see the new display but no point cloud and an error message:

![rviz3](./doc/rviz3.png "Error PointCloud")

In order to visualize correctly, you have to choose the reference coordinate frame. In "Global Options", set "Fixed Frame" to "/camera_link". Now you can see the sensor data:

![rviz3](./doc/rviz4.png "Set frame")

Now you can change the visualization options, such as "Style", "Size", "Alpha" and "Color Transform" in order to alter the visualizaiton appearance.

### 2.  Passthrough filter

### 3.  Plane segmentation

### 4.  Voxel filter (optional)
