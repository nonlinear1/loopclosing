#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include<g2o/types/slam3d/vertex_se3.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
namespace loam {
class KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
using Ptr=std::shared_ptr<KeyFrame>;
  KeyFrame(const nav_msgs::Odometry& odom):
      _cloud(new pcl::PointCloud<pcl::PointXYZI>()),
      _node(new g2o::VertexSE3)
  {
  }
  KeyFrame():
    _cloud(new pcl::PointCloud<pcl::PointXYZI>()),
    _node(new g2o::VertexSE3)
  {
  }
  ~KeyFrame()
  {

  }

public:
  ros::Time _stamp;                                // timestamp
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr _cloud;        // point cloud

  Eigen::Isometry3d _pose;
  boost::optional<Eigen::Vector4f> _floor_coeffes;
  float _accumulate_distance;
  g2o::VertexSE3* _node;
};
class KeyFrameSnapshot
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr=std::shared_ptr<KeyFrameSnapshot>;
  KeyFrameSnapshot(const KeyFrame::Ptr& key_frame):
    _cloud(key_frame->_cloud),
  _pose(key_frame->_node->estimate()){}
    //_cloud(key_frame->_cloud){}
 // _pose(key_frame->_node->estimate()){}
  KeyFrameSnapshot(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,const Eigen::Isometry3d& pose):
    _cloud(pointcloud),
    _pose(pose)
  {}
 public:
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr _cloud;
  Eigen::Isometry3d _pose;
};
}

#endif // KEYFRAME_HPP
