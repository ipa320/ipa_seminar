/*
 * voxel_filter.cpp
 *
 *  Created on: 06.09.2013
 *      Author: goa
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_tutorial/voxel_filter_nodeConfig.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>


class VoxelFilterNode
{
public:
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  VoxelFilterNode()
  {
    pub_ = nh_.advertise<PointCloud>("point_cloud_out",1);
    sub_ = nh_.subscribe ("point_cloud_in", 1,  &VoxelFilterNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&VoxelFilterNode::dynReconfCallback, this, _1, _2));
  }

  ~VoxelFilterNode() {}

  void
  dynReconfCallback(pcl_tutorial::voxel_filter_nodeConfig &config, uint32_t level)
  {

  }

  void
  cloudCallback(const PointCloud::ConstPtr& cloud)
  {

  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pcl_tutorial::voxel_filter_nodeConfig> config_server_;



};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "voxel_filter_node");

  VoxelFilterNode vf;

  ros::spin();
}

