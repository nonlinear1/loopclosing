#ifndef KEYFRAMEUPDATE_H
#define KEYFRAMEUPDATE_H
#include <ros/ros.h>
//#include <Eigen/Core>
//#include<Eigen/Geometry>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include"keyframe.hpp"
#include <nav_msgs/Odometry.h>
#include"Twist.h"
namespace loam {
class KeyframeUpdater {
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KeyframeUpdater(ros::NodeHandle& pnh)
    : is_first(true),
      _prev_keypose(Eigen::Isometry3d::Identity()),
      _keyframe_delta_trans(2.0),
      _keyframe_delta_angle(2.0),
      _keyframe_delta_time(2.0),
      _accum_distance(0),
      _submap_cloud( new pcl::PointCloud<pcl::PointXYZI>()),
      _submap_flat_cloud( new pcl::PointCloud<pcl::PointXYZI>())
  {
     float fParam;
     if(pnh.getParam("keyframe_delta_trans",fParam))
     {
         _keyframe_delta_trans=fParam;
     }
     if(pnh.getParam("keyframe_delta_trans",fParam))
     {
         _keyframe_delta_angle=fParam;
     }
     if(pnh.getParam("keyframe_delta_angle",fParam))
     {
         _keyframe_delta_time=fParam;
     }
    _kefFrame_ptr.reset(new KeyFrame());

  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose,ros::Time nowTime,
              const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
              const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& flat_cloud) {
    // first frame is always registered to the graph
    if(is_first) {
      is_first = false;
      _prev_keypose = pose;
      _prev_time=nowTime;
      *_submap_cloud+=*cloud;
      *_submap_flat_cloud+=*flat_cloud;
      return false;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = _prev_keypose.inverse() * pose;
    float delta_time=nowTime.toSec()-_prev_time.toSec();
    double dx = std::abs(delta.translation().norm());
    double da = std::abs(std::acos(Eigen::Quaterniond(delta.linear()).w()));

    // too close to the previous frame
    if(dx < _keyframe_delta_trans && da < _keyframe_delta_angle && delta_time<_keyframe_delta_time) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      transform_cloud->reserve(cloud->size());
      pcl::transformPointCloud(*(cloud),*(transform_cloud),delta.cast<float>());
      *_submap_cloud+=*transform_cloud;

      pcl::PointCloud<pcl::PointXYZI>::Ptr transform_flat_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      transform_flat_cloud->reserve(flat_cloud->size());
      pcl::transformPointCloud(*flat_cloud,*transform_flat_cloud,delta.cast<float>);
      *_submap_flat_cloud+=transform_flat_cloud;
      return false;
    }
    _accum_distance += dx;
    _prev_keypose = pose;
    _prev_time=nowTime;
    return true;
  }
double get_accum_distance()
{
  return _accum_distance;
}
public:
  std::shared_ptr<KeyFrame> _kefFrame_ptr;
private:
  // parameters
  double _keyframe_delta_trans;      //
  double _keyframe_delta_angle;      //
  double _keyframe_delta_time;
  bool is_first;
  double _accum_distance;
public:
  pcl::PointCloud<pcl::PointXYZI>::Ptr _submap_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _submap_flat_cloud;
  Eigen::Isometry3d _prev_keypose;
  ros::Time _prev_time;
};
}
#endif // KEYFRAMEUPDATE_H
