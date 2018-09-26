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
struct PCloudPose
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
  Eigen::Quaterniond _r;
  Eigen::Vector3d _t;
  ros::Time _stamp;
};
class PointCloudImage
{
public:
  PointCloudImage();
  void readConfig(const std::string& path);
  void interpolate();
  void projected();
  bool setup(ros::NodeHandle& nh,ros::NodeHandle& private_nh);
  void imageCallback(const sensor_msgs::ImageConstPtr& img);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointscloud,const nav_msgs::OdometryConstPtr& odom);
  void publishResult();
  void calImagePose();
public:
  Eigen::Matrix<double,3,4> _projected;
  std::unique_ptr<CircularBuffer<Image>> _images_bufffer;
  std::vector<PCloudPose> _vec_pointcloud;
  std::vector<Image> _vec_pose_image;
  ros::Publisher _pub_pointscloud;
  ros::Publisher _pub_odom;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_pointscloud;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> _sub_odom;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>> _syn;
  ros::Subscriber _sub_image;
};
}
#endif // POINTCLOUDIMAGE_H
