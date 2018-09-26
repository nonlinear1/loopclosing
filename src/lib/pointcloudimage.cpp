#include"loam_velodyne/pointcloudimage.h"
#include<fstream>
#include<ros/subscriber.h>
#include<pcl_ros/point_cloud.h>
#include<cv_bridge/cv_bridge.h>
#include<pcl/common/transforms.h>
#include<algorithm>
#include"loam_velodyne/common.h"
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
   in>>_width>>_height;
   float projected[12];
   for(int i=0;i<12;i++)
   {
     in>>projected[i];
   }
   _projected<<projected[0],projected[1],projected[2],projected[3],
             projected[4],projected[5],projected[6],projected[7],
             projected[8],projected[9],projected[10],projected[11];
   float rela_pose[16];
   for(int i=0;i<16;i++)
   {
     in>>rela_pose[i];
   }
   _relative_pose<<rela_pose[0],rela_pose[1],rela_pose[2],rela_pose[3],
                   rela_pose[4],rela_pose[5],rela_pose[6],rela_pose[7],
                   rela_pose[8],rela_pose[9],rela_pose[10],rela_pose[11],
                   rela_pose[12],rela_pose[13],rela_pose[14],rela_pose[15];

}
void PointCloudImage::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointscloud,const nav_msgs::OdometryConstPtr& odom)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*pointscloud,*points);

  geometry_msgs::Quaternion geo_quat=odom->pose.pose.orientation;
  Eigen::Quaterniond eig_qua(geo_quat.w,geo_quat.x,geo_quat.y,geo_quat.z);

  PCloudPose<pcl::PointXYZI> points_pose;
  points_pose._cloud=points;
  points_pose._r=eig_qua;
  points_pose._t=Eigen::Vector3d(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z);
  points_pose._stamp=pointscloud->header.stamp;
  _vec_pointcloud.push_back(points_pose);
  calImagePose();
  projected();
  publishResult();
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
      //image stamp is between pointscloud
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
void PointCloudImage::projected()
{
   if(_vec_pose_image.empty() || _vec_pointcloud.empty()) return;
   int erase_cloud_index=0;
   int erase_image_index=0;
   for(int i=0;i<_vec_pointcloud.size();i++)
   {
     for(int j=0;j<_vec_pose_image.size()-1;j++)
     {
       //if pointcloud is in between pose
       if(_vec_pointcloud[i]._stamp.toSec()>=_vec_pose_image[j]._stamp.toSec() &&
          _vec_pointcloud[i]._stamp.toSec()<=_vec_pose_image[j+1]._stamp.toSec())
       {
         float time_to_start=_vec_pointcloud[i]._stamp.toSec()-_vec_pose_image[j]._stamp.toSec();
         float time_to_end=_vec_pose_image[j+1]._stamp.toSec()-_vec_pointcloud[i]._stamp.toSec();
         const Image& select_image=time_to_start>=time_to_end?_vec_pose_image[j+1]:_vec_pose_image[j];
         const PCloudPose<pcl::PointXYZI>& select_points=_vec_pointcloud[i];

         Eigen::Isometry3d image_pose=Eigen::Isometry3d::Identity();
         image_pose.rotate(select_image._r.toRotationMatrix());
         image_pose.pretranslate(select_image._t);

         Eigen::Isometry3d pointscloud_pose=Eigen::Isometry3d::Identity();
         pointscloud_pose.rotate(select_points._r.toRotationMatrix());
         pointscloud_pose.pretranslate(select_points._t);
         // the relative of pointcloud and image
         Eigen::Isometry3d relative_pose=_relative_pose*image_pose.inverse()*pointscloud_pose;
         pcl::PointCloud<pcl::PointXYZI>::Ptr orignal_cloud=select_points._cloud;
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
         rgb_cloud->reserve(orignal_cloud->size());
         //set BGR
         for(int p=0;p<orignal_cloud->size();p++)
         {
           pcl::PointXYZRGB rgb_point;
           Eigen::Vector4d transform_point=relative_pose*orignal_cloud->points[p].getVector4fMap();
           //if the point is in the front of camera
           if(transform_point(3)>0)
           {
             Eigen::Vector3d project_point=_projected*transform_point;
             project_point(0)/=project_point(2);
             project_point(1)/=project_point(2);
             if(project_point(0)>=0 && project_point(0)<=_width-1 &&
                project_point(1)>=0 && project_point(1)<=_height-1)
             {
                cv::Vec3b BGR=interpolate(project_point(0),project_point(1),select_image._image);
                rgb_point.r=BGR(2);
                rgb_point.g=BGR(1);
                rgb_point.b=BGR(0);
                rgb_point.x=orignal_cloud->points[p].x;
                rgb_point.y=orignal_cloud->points[p].y;
                rgb_point.z=orignal_cloud->points[p].z;
                rgb_cloud->push_back(rgb_point);
             }
           }
         }
         PCloudPose<pcl::PointXYZRGB> rgb_cloudpose;
         rgb_cloudpose._cloud=rgb_cloud;
         rgb_cloudpose._r=select_points._r;
         rgb_cloudpose._t=select_points._t;
         rgb_cloudpose._stamp=select_points._stamp;
         _vecp_pub_pointcloud.push_back(rgb_cloudpose);
         erase_cloud_index=i+1;
         erase_image_index=std::max(erase_image_index,j);
         break;
       }
     }
   }
   _vec_pointcloud.erase(_vec_pointcloud.begin(),_vec_pointcloud.begin()+erase_cloud_index);
   _vec_pose_image.erase(_vec_pose_image.begin(),_vec_pose_image.begin()+erase_image_index);
}
cv::Vec3b PointCloudImage::interpolate(double x,double y,const cv::Mat img)
{
  int hlow=std::floor(y);
  int hheight=std::ceil(y);
  int wleft=std::floor(x);
  int wright=std::ceil(x);
  double alpha=wleft==wright?0.5:(x-wleft)/(wright-wleft);
  double beta=hlow==hheight?0.5:(y-hlow)/(hheight-hlow);
  cv::Vec3b BGR(0,0,0);
  BGR+=img.at<cv::Vec3b>(hlow,wleft)*(1-alpha)*(1-beta);
  BGR+=img.at<cv::Vec3b>(hlow,wright)*alpha*(1-beta);
  BGR+=img.at<cv::Vec3b>(hheight,wleft)*(1-alpha)*beta;
  BGR+=img.at<cv::Vec3b>(hheight,wright)*alpha*beta;
  return BGR;
}

void PointCloudImage::publishResult()
{
  for(int i=0;i<_vecp_pub_pointcloud.size();i++)
  {
    publishCloudMsg(_pub_pointscloud, *(_vecp_pub_pointcloud[i]._cloud), _vecp_pub_pointcloud[i]._stamp, "/camera_init");

    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.stamp = _vecp_pub_pointcloud[i]._stamp;
    laserOdometry.pose.pose.orientation.x =_vecp_pub_pointcloud[i]._r.x();
    laserOdometry.pose.pose.orientation.y = _vecp_pub_pointcloud[i]._r.y();
    laserOdometry.pose.pose.orientation.z = _vecp_pub_pointcloud[i]._r.z();
    laserOdometry.pose.pose.orientation.w = _vecp_pub_pointcloud[i]._r.w();
    laserOdometry.pose.pose.position.x = _vecp_pub_pointcloud[i]._t(0);
    laserOdometry.pose.pose.position.y = _vecp_pub_pointcloud[i]._t(1);
    laserOdometry.pose.pose.position.z = _vecp_pub_pointcloud[i]._t(2);
    _pub_odom.publish(laserOdometry);
  }
  _vecp_pub_pointcloud.clear();
}

}