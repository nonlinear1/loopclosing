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
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud;
  Eigen::Quaterniond _r;
  Eigen::Vector3d _t;
  ros::Time _stamp;
  bool _is_rgb=false;
};
class PointCloudImage
{
public:
  PointCloudImage();
  void readIntrinsic(const std::string& path);
  void readExtrinsic(const std::string& path,Eigen::Isometry3d& relativePose);
  cv::Vec3b interpolate(double x,double y,const cv::Mat img);
  void projected(int index);
  bool setup(ros::NodeHandle& nh,ros::NodeHandle& private_nh);
  void imageCallback(const sensor_msgs::ImageConstPtr& img,int index);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointscloud,const nav_msgs::OdometryConstPtr& odom);
  void publishResult();
  void calImagePose(int index);
public:
  std::vector<CircularBuffer<Image>> _images_bufffer;
  std::vector<PCloudPose> _vec_pointcloud;
  std::vector<std::vector<Image>> _vec_pose_image;
  //------------------node param
  int _camera_number;
  std::string _intrinsicPath;
  std::string _extrinsic1Path;
  std::string _extrinsic2Path;
  std::string _extrinsic3Path;
  std::string _extrinsic4Path;
  float camera1_pre_theta;
  float camera2_pre_theta;
  float camera3_pre_theta;
  float camera4_pre_theta;
  //------------------end node param
  //------------------lidar to camera relative pose
  std::vector<Eigen::Isometry3d> _relative_poses;
  //------------------end lidar to camera relative pose
  //------------------camera intrinsic
  Eigen::Matrix<double,3,4> _projected;
  int _height,_width;
  //------------------end camera intrinsic
  ros::Publisher _pub_pointscloud;
  ros::Publisher _pub_odom;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_pointscloud;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> _sub_odom;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>> _syn;
  ros::Subscriber _sub_image1;
  ros::Subscriber _sub_image2;
  ros::Subscriber _sub_image3;
  ros::Subscriber _sub_image4;
  int _resizeImageTimes;
  //debug param
  int view=0;
};
}
#endif // POINTCLOUDIMAGE_H
