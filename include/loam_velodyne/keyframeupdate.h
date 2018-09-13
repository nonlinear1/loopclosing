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
      _keyframeDeltaTrans(2.0),
      _keyframeDeltaAngle(2.0),
      _keyframeDeltaTime(2.0)
  {
     float fParam;
     if(pnh.getParam("keyframe_delta_trans",fParam))
     {
         _keyframeDeltaTrans=fParam;
     }
     if(pnh.getParam("keyframe_delta_trans",fParam))
     {
         _keyframeDeltaAngle=fParam;
     }
     if(pnh.getParam("keyframe_delta_angle",fParam))
     {
         _keyframeDeltaTime=fParam;
     }
    _keyFramePtr.reset(new KeyFrame());

  }
  nav_msgs::Odometry poseToodometry(const Eigen::Isometry3d& pose,ros::Time nowTime)
  {
       Eigen::Quaterniond qRelativePose(pose.rotation());
       Eigen::Vector3d translate=pose.translation();
       nav_msgs::Odometry laserRelativeOdometry;
       laserRelativeOdometry.header.stamp = nowTime;
       laserRelativeOdometry.pose.pose.orientation.x = qRelativePose.x();
       laserRelativeOdometry.pose.pose.orientation.y = qRelativePose.y();
       laserRelativeOdometry.pose.pose.orientation.z = qRelativePose.z();
       laserRelativeOdometry.pose.pose.orientation.w = qRelativePose.w();
       laserRelativeOdometry.pose.pose.position.x = translate(0);
       laserRelativeOdometry.pose.pose.position.y = translate(1);
       laserRelativeOdometry.pose.pose.position.z = translate(2);

       return laserRelativeOdometry;
  }
  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose,ros::Time nowTime,const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& submapPointCloud,
              const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& submapFlatCloud) {
    // first frame is always registered to the graph
    if(is_first) {
      is_first = false;
      _prev_keypose = pose;
      nav_msgs::Odometry relativeOdometry=poseToodometry(Eigen::Isometry3d::Identity(),nowTime);
      _keyFramePtr->_odom.push_back(relativeOdometry);
      _prev_time=nowTime;
      return false;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = _prev_keypose.inverse() * pose;
    nav_msgs::Odometry relativeOdometry=poseToodometry(delta,nowTime);
    float delta_time=nowTime.toSec()-_prev_time.toSec();
    double dx = delta.translation().norm();
    double da = std::acos(Eigen::Quaterniond(delta.linear()).w());

    // too close to the previous frame
    if(dx > _keyframeDeltaTrans || da > _keyframeDeltaAngle || delta_time>_keyframeDeltaTime) {

      pcl::PointCloud<pcl::PointXYZI>::Ptr keyframe_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      keyframe_cloud->reserve(submapPointCloud->size());
      pcl::transformPointCloud(*(submapPointCloud),*(keyframe_cloud),_prev_keypose.inverse().cast<float>());
      //std::cout<<"_prev_keypose"<<_prev_keypose.matrix()<<std::endl;
      _keyFramePtr->_cloud=keyframe_cloud;
      pcl::PointCloud<pcl::PointXYZI>::Ptr keyframe_cloud_flat(new pcl::PointCloud<pcl::PointXYZI>());
      keyframe_cloud_flat->reserve(submapFlatCloud->size());
      pcl::transformPointCloud(*(submapFlatCloud),*(keyframe_cloud_flat),_prev_keypose.inverse().cast<float>());
      _keyFramePtr->_flat_cloud=keyframe_cloud_flat;
      _keyFramePtr->_stamp=_prev_time;
       nav_msgs::Odometry submapPose=poseToodometry(_prev_keypose,_prev_time);
      _keyFramePtr->pose=submapPose;
      _prev_keypose = pose;
      _prev_time=nowTime;

      return true;
    }
    _keyFramePtr->_odom.push_back(relativeOdometry);
    return false;
  }

public:
  std::shared_ptr<KeyFrame> _keyFramePtr;
private:
  // parameters
  double _keyframeDeltaTrans;      //
  double _keyframeDeltaAngle;      //
  double _keyframeDeltaTime;
  bool is_first;
  Eigen::Isometry3d _prev_keypose;
  ros::Time _prev_time;
};
}
#endif // KEYFRAMEUPDATE_H
