#include"loam_velodyne/pointcloudimage.h"
#include<fstream>
#include<ros/subscriber.h>
#include<pcl_ros/point_cloud.h>
#include<cv_bridge/cv_bridge.h>
namespace loam {
bool PointCloudImage::setup(ros::NodeHandle& nh,ros::NodeHandle& private_nh)
{
  std::string path=private_nh.param<std::string>("config_file","");
  if(path.size()==0)
  {
    std::cout<<"camera intinsic is none"<<std::endl;
    return false;
  }
  readConfig(path);

  _images_bufffer.reset(new CircularBuffer<Image>(50));
  _sub_pointscloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_points",2));
  _sub_odom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/odom",2));
  _syn.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>(*_sub_pointscloud,*_sub_odom,2));
  _syn->registerCallback(boost::bind(&PointCloudImage::pointcloudCallback,this,_1,_2));
  _sub_image=nh.subscribe<sensor_msgs::Image>("/image",2,&PointCloudImage::imageCallback,this);

  _pub_pointscloud=nh.advertise<sensor_msgs::PointCloud2>("/points",2);
  _pub_odom=nh.advertise<nav_msgs::Odometry>("odom",2);
}
PointCloudImage::PointCloudImage()
{
   _vec_pointcloud.reserve(10);
   _vec_pose_image.reserve(50);
}
void PointCloudImage::readConfig(const std::string& path)
{
   std::ifstream in(path);
   float projected[12];
   for(int i=0;i<12;i++)
   {
     in>>projected[i];
   }
   _projected<<projected[0],projected[1],projected[2],projected[3],
             projected[4],projected[5],projected[6],projected[7],
             projected[8],projected[9],projected[10],projected[11];

}
void PointCloudImage::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointscloud,const nav_msgs::OdometryConstPtr& odom)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*pointscloud,*points);

  geometry_msgs::Quaternion geo_quat=odom->pose.pose.orientation;
  Eigen::Quaterniond eig_qua(geo_quat.w,geo_quat.x,geo_quat.y,geo_quat.z);

  PCloudPose points_pose;
  points_pose._cloud=points;
  points_pose._r=eig_qua;
  points_pose._t=Eigen::Vector3d(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z);
  points_pose._stamp=pointscloud->header.stamp;
  _vec_pointcloud.push_back(points_pose);
  calImagePose();

}
void PointCloudImage::imageCallback(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    std::cout<<"image callback is wrong"<<std::endl;
    ros::shutdown();
  }
  Image tem_image;
  tem_image._image=cv_ptr->image;
  tem_image._stamp=img->header.stamp;
  _images_bufffer->push(tem_image);
}
void PointCloudImage::calImagePose()
{
  if(_vec_pointcloud.empty()) return;
  for(int i=0;i<_images_bufffer->size();i++)
  {
    ros::Time image_time=(*_images_bufffer)[i]._stamp;
    if(image_time.toSec()<_vec_pointcloud.front()._stamp.toSec() ||
       image_time.toSec()>_vec_pointcloud.back()._stamp.toSec())
      continue;
    for(int j=0;j<_vec_pointcloud.size()-1;j++)
    {
      if(image_time.toSec()>=_vec_pointcloud[j]._stamp.toSec() &&
         image_time.toSec()<=_vec_pointcloud[j+1]._stamp.toSec())
      {
        float time_diff=(image_time.toSec()-_vec_pointcloud[j]._stamp.toSec())/
            (_vec_pointcloud[j+1]._stamp.toSec()-_vec_pointcloud[j]._stamp.toSec());
        _vec_pointcloud[j]._r.normalize();
        _vec_pointcloud[j+1]._r.normalize();
        Eigen::Quaterniond quati=_vec_pointcloud[j]._r.slerp(time_diff,_vec_pointcloud[j+1]._r);
        Eigen::Vector3d dis_diff=_vec_pointcloud[j+1]._t-_vec_pointcloud[j]._t;
        Eigen::Vector3d disi=_vec_pointcloud[j]._t+dis_diff*time_diff;
        Image image_pose;
        image_pose._image=(*_images_bufffer[i])._image;
        image_pose._t=disi;
        image_pose._r=quati;
        image_pose._stamp=(*_images_bufffer[i])._stamp;
        _vec_pose_image.push_back(image_pose);
        break;
      }
    }
  }

}
void PointCloudImage::interpolate()
{
   if(_vec_pose_image.empty() || _vec_pointcloud.empty()) return;
   for(int i=0;i<_vec_pointcloud.size();i++)
   {
     for(int j=0;j<_vec_pose_image.size()-1;j++)
     {
       if(_vec_pointcloud[i]._stamp.toSec()>=_vec_pose_image[j]._stamp.toSec() &&
          _vec_pointcloud[i]._stamp.toSec()<=_vec_pose_image[j+1]._stamp.toSec())
       {
         float time_to_start=_vec_pointcloud[i]._stamp.toSec()-_vec_pose_image[j]._stamp.toSec();
         float time_to_end=_vec_pose_image[j+1]._stamp.toSec()-_vec_pointcloud[i]._stamp.toSec();
         const Image& select_image=time_to_start>=time_to_end?_vec_pose_image[j+1]:_vec_pose_image[j];

       }
     }
   }
}
void PointCloudImage::projected();

void PointCloudImage::publishResult();
  /*Eigen::Isometry3d _projected;
  std::unique_ptr<CircularBuffer<Image>> _images_bufffer;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _vec_pointcloud;
  std::vector<Image> _vec_pose_image;
  ros::Publisher _pub_pointscloud;
  ros::Publisher _pub_odom;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_pointscloud;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> _sub_odom;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>> _syn;
  ros::Subscriber _sub_image;*/

}
