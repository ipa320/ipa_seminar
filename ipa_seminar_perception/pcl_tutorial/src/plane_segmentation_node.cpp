/*
 * voxel_filter.cpp
 *
 *  Created on: 06.09.2013
 *      Author: goa
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>

#include <pcl_tutorial/plane_segmentation_nodeConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <stop_watch.h>


class PlaneSegmentationNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  /*
   * constructor
   * setup which topics are passed out (advertise) and to which topics are listend (subscribe)
   */
  PlaneSegmentationNode()
  {
    pub_ = nh_.advertise<PointCloud>("point_cloud_out",1);
    pub_2_ = nh_.advertise<PointCloud>("point_cloud_out_2",1);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1);
    pub_marker_2_ = nh_.advertise<visualization_msgs::Marker>("marker_2",1);
    sub_ = nh_.subscribe ("point_cloud_in", 1,  &PlaneSegmentationNode::cloudCallback, this);
    config_server_.setCallback(boost::bind(&PlaneSegmentationNode::dynReconfCallback, this, _1, _2));

    double dist_thr;
    int max_its;

    // "~" means, that the node hand is opened within the private namespace (to get the "own" paraemters)
    ros::NodeHandle private_nh("~");

    //read parameters with default value
    private_nh.param("dist_thresh", dist_thr, 0.01);
    private_nh.param("max_iterations", max_its, 50);

    seg_.setModelType (pcl::SACMODEL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setOptimizeCoefficients (true);
    seg_.setDistanceThreshold (dist_thr);
    seg_.setMaxIterations (max_its);
  }

  ~PlaneSegmentationNode() {}

  void
  dynReconfCallback(pcl_tutorial::plane_segmentation_nodeConfig &config, uint32_t level)
  {
    seg_.setDistanceThreshold (config.dist_thresh);
    seg_.setMaxIterations (config.max_iterations);
  }

  void
  publishMarker(pcl::ModelCoefficients::Ptr& coefficients, PointCloud inliers, const std_msgs::Header& header)
  {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for( unsigned int i = 0; i < inliers.points.size(); i++)
    {
      centroid += inliers.points[i].getVector3fMap();
    }
    centroid /= inliers.points.size();

    Eigen::Vector3f normal;
    normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
    if(coefficients->values[3] < 0)
    {
      normal *= -1;
    }
    //Eigen::Vector3f origin = -coefficients->values[3] * normal;
    visualization_msgs::Marker marker_normal;
    marker_normal.type = visualization_msgs::Marker::ARROW;
    marker_normal.header = header;
    marker_normal.id = 0;
    marker_normal.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point pt;
    pt.x = centroid(0);
    pt.y = centroid(1);
    pt.z = centroid(2);
    marker_normal.points.push_back(pt);
    pt.x = centroid(0) + normal(0);
    pt.y = centroid(1) + normal(1);
    pt.z = centroid(2) + normal(2);
    marker_normal.points.push_back(pt);
    marker_normal.color.r = 1.0;
    marker_normal.color.a = 1.0;
    marker_normal.scale.x = 0.02;
    marker_normal.scale.y = 0.05;
    marker_normal.scale.z = 0.1;
    pub_marker_.publish(marker_normal);

    visualization_msgs::Marker marker_centroid;
    marker_centroid.type = visualization_msgs::Marker::SPHERE;
    marker_centroid.header = header;
    marker_centroid.id = 1;
    marker_centroid.action = visualization_msgs::Marker::ADD;
    marker_centroid.pose.position.x = centroid(0);
    marker_centroid.pose.position.y = centroid(1);
    marker_centroid.pose.position.z = centroid(2);
    marker_centroid.color.b = 1.0;
    marker_centroid.color.a = 1.0;
    marker_centroid.scale.x = 0.1;
    marker_centroid.scale.y = 0.1;
    marker_centroid.scale.z = 0.1;
    pub_marker_.publish(marker_centroid);

    visualization_msgs::Marker marker_plane;
    marker_plane.type = visualization_msgs::Marker::CUBE;
    marker_plane.header = header;
    marker_plane.id = 2;
    marker_plane.action = visualization_msgs::Marker::ADD;
    marker_plane.pose.position.x = centroid(0);
    marker_plane.pose.position.y = centroid(1);
    marker_plane.pose.position.z = centroid(2);
    Eigen::Affine3f rotation;
    pcl::getTransformationFromTwoUnitVectors(normal.unitOrthogonal(), normal, rotation);
    Eigen::Quaternionf qu(rotation.inverse().rotation());
    marker_plane.pose.orientation.x = qu.x();
    marker_plane.pose.orientation.y = qu.y();
    marker_plane.pose.orientation.z = qu.z();
    marker_plane.pose.orientation.w = qu.w();
    marker_plane.color.g = 1.0;
    marker_plane.color.a = 1.0;
    marker_plane.scale.x = 1;
    marker_plane.scale.y = 1;
    marker_plane.scale.z = 0.02;
    pub_marker_2_.publish(marker_plane);
  }

  void
  cloudCallback(const PointCloud::ConstPtr& cloud_in)
  {
    if(!cloud_in || cloud_in->size()<=0) {
	ROS_WARN("got empty or invalid pointcloud --> ignoring");
	return;
    }

    PrecisionStopWatch sw;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    sw.precisionStart();
    seg_.setInputCloud (cloud_in);
    seg_.segment (*inliers, *coefficients);

    //extract indices
    PointCloud cloud_out, cloud_out_2;
    extract_.setInputCloud(cloud_in);
    extract_.setIndices(inliers);
    extract_.filter(cloud_out);

    extract_.setNegative(true);
    extract_.filter(cloud_out_2);
    extract_.setNegative(false);

    ROS_INFO("Segmentation took %f s.", sw.precisionStop());

    pub_.publish(cloud_out);
    pub_2_.publish(cloud_out_2);
    publishMarker(coefficients, cloud_out, cloud_in->header);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub_2_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_marker_2_;
  dynamic_reconfigure::Server<pcl_tutorial::plane_segmentation_nodeConfig> config_server_;

  pcl::SACSegmentation<Point> seg_;
  pcl::ExtractIndices<Point> extract_;

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "plane_segmentation_node");

  PlaneSegmentationNode ps;

  ros::spin();
}

