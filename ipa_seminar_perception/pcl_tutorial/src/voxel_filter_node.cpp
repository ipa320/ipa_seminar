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


class PlaneSegmentationNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  PlaneSegmentationNode()
  {
    pub_ = nh_.advertise<PointCloud>("point_cloud_out",1);
    sub_ = nh_.subscribe ("point_cloud_in", 1,  &PlaneSegmentationNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&PlaneSegmentationNode::dynReconfCallback, this, _1, _2));

    double leafsize;

    // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
    ros::NodeHandle private_nh("~");

    //read parameters with default value
    private_nh.param("leafsize", leafsize, 0.01);

    vg_.setLeafSize (leafsize,leafsize,leafsize);
  }

  ~PlaneSegmentationNode() {}

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

  PlaneSegmentationNode vf;

  ros::spin();
}

