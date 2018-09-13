#ifndef FLOORDETECTION_H
#define FLOORDETECTION_H
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include<time.h>
namespace loam {
class FloorDetection
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FloorDetection(const float& sensor_height=1,
                          const float& height_clip_range=0.8,
                          const int& floor_pts_thresh=50,
                          const float& floor_normal_thresh=10.0,
                          const bool& use_normal_filtering=false,
                          const float& normal_filter_thresh=20);
 void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfMsg);
 boost::optional<Eigen::Vector4f> detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
 pcl::PointCloud<pcl::PointXYZI>::Ptr plane_clip(const pcl::PointCloud<pcl::PointXYZI>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative);
 pcl::PointCloud<pcl::PointXYZI>::Ptr normal_filtering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
 bool setup(ros::NodeHandle nh,ros::NodeHandle privateNode);
 void process();
 void spin();
 void reset();
 void publishResult(boost::optional<Eigen::Vector4f> floor);
private:
  // ROS topics
  ros::Subscriber _sub_SubmapFlatCloud;

  ros::Publisher _pub_floor;
  ros::Time _laser_cloud_stamp;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laser_surf_cloud;
  // floor detection parameters
  // see initialize_params() for the details
  double _tilt_deg;
  double _sensor_height;
  double _height_clip_range;

  int _floor_pts_thresh;
  double _floor_normal_thresh;

  bool _use_normal_filtering;
  double _normal_filter_thresh;
};
}
#endif // FLOORDETECTION_H
