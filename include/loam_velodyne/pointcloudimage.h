#ifndef POINTCLOUDIMAGE_H
#define POINTCLOUDIMAGE_H
#include<opencv2/opencv.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<ros/time.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include"loam_velodyne/CircularBuffer.h"
#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>
#include<ros/publisher.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/subscriber.h>
#include<nav_msgs/Odometry.h>
namespace loam {
struct Image
{
public:
  cv::Mat _image;
  Eigen::Quaterniond _r;
  Eigen::Vector3d _t;
  ros::Time _stamp;
};
template<class PointT>
struct PCloudPose
{
  pcl::PointCloud<PointT>::Ptr _cloud;
  Eigen::Quaterniond _r;
  Eigen::Vector3d _t;
  ros::Time _stamp;
  bool _is_rgb=false;
};
class PointCloudImage
{
public:
  PointCloudImage();
  void readConfig(const std::string& path);
  cv::Vec3b interpolate(double x,double y,const cv::Mat img);
  void projected(int index);
  bool setup(ros::NodeHandle& nh,ros::NodeHandle& private_nh);
  void imageCallback(const sensor_msgs::ImageConstPtr& img);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointscloud,const nav_msgs::OdometryConstPtr& odom);
  void publishResult();
  void calImagePose(int index);
public:
  Eigen::Matrix<double,3,4> _projected;
  Eigen::Isometry3d _relative_pose;
  int _height,_width;
  std::vector<std::unique_ptr<CircularBuffer<Image>>> _images_bufffer;
  std::vector<PCloudPose<pcl::PointXYZRGBA>> _vec_pointcloud;
  std::vector<std::vector<Image>> _vec_pose_image;
  int _camera_number;
  ros::Publisher _pub_pointscloud;
  ros::Publisher _pub_odom;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_pointscloud;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> _sub_odom;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>> _syn;
  ros::Subscriber _sub_image;
};
}
#endif // POINTCLOUDIMAGE_H
