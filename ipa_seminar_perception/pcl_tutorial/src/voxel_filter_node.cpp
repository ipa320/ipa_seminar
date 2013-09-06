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
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>


class PassthroughFilterNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  PassthroughFilterNode()
  {
    pub_ = nh_.advertise<PointCloud>("point_cloud_out",1);
    sub_ = nh_.subscribe ("point_cloud_in", 1,  &PassthroughFilterNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&PassthroughFilterNode::dynReconfCallback, this, _1, _2));

    vg_.setLeafSize (0.01f, 0.01f, 0.01f);
  }

  ~PassthroughFilterNode() {}

  void
  dynReconfCallback(pcl_tutorial::voxel_filter_nodeConfig &config, uint32_t level)
  {
    vg_.setLeafSize(config.leafsize, config.leafsize, config.leafsize);
  }

  void
  cloudCallback(const PointCloud::ConstPtr& cloud_in)
  {
    vg_.setInputCloud(cloud_in);
    PointCloud cloud_out;
    vg_.filter(cloud_out);
    pub_.publish(cloud_out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pcl_tutorial::voxel_filter_nodeConfig> config_server_;

  pcl::VoxelGrid<Point> vg_;

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "voxel_filter_node");

  PassthroughFilterNode vf;

  ros::spin();
}

