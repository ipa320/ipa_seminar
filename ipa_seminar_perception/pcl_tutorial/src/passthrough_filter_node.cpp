/*
 * passthrough_filter.cpp
 *
 *  Created on: 06.09.2013
 *      Author: goa
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_tutorial/passthrough_filter_nodeConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>


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

    pt_.setFilterFieldName ("z");
    pt_.setFilterLimits (0.0, 2.0);
  }

  ~PassthroughFilterNode() {}

  void
  dynReconfCallback(pcl_tutorial::passthrough_filter_nodeConfig &config, uint32_t level)
  {
    pt_.setFilterLimits(config.lower_limit, config.upper_limit);
  }

  void
  cloudCallback(const PointCloud::ConstPtr& cloud_in)
  {
    pt_.setInputCloud(cloud_in);
    PointCloud cloud_out;
    pt_.filter(cloud_out);
    pub_.publish(cloud_out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pcl_tutorial::passthrough_filter_nodeConfig> config_server_;

  pcl::PassThrough<Point> pt_;

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "voxel_filter_node");

  PassthroughFilterNode vf;

  ros::spin();
}

