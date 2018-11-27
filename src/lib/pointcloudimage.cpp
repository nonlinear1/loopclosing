#include"loam_velodyne/pointcloudimage.h"
#include<fstream>
#include<ros/subscriber.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<cv_bridge/cv_bridge.h>
#include<pcl/common/transforms.h>
#include<algorithm>
#include"loam_velodyne/common.h"
#include<ros/package.h>
#include<pcl/io/pcd_io.h>
namespace loam {
bool PointCloudImage::setup(ros::NodeHandle& nh,ros::NodeHandle& private_nh)
{
  _camera_number=private_nh.param<int>("camera_number",4);
  _intrinsicPath=private_nh.param<std::string>("intrinsic","");
  _extrinsic1Path=private_nh.param<std::string>("extrinsic1","");
  _extrinsic2Path=private_nh.param<std::string>("extrinsic2","");
  _extrinsic3Path=private_nh.param<std::string>("extrinsic3","");
  _extrinsic4Path=private_nh.param<std::string>("extrinsic4","");
  camera1_pre_theta=private_nh.param<float>("camera1_pre_theta",0);
  camera2_pre_theta=private_nh.param<float>("camera2_pre_theta",0);
  camera3_pre_theta=private_nh.param<float>("camera3_pre_theta",0);
  camera4_pre_theta=private_nh.param<float>("camera4_pre_theta",0);
  _resizeImageTimes=private_nh.param<int>("resizeImageTimes",4);
  if(_intrinsicPath.length() == 0)
  {
    std::cout<<"intrinsic path is empty"<<std::endl;
    exit(0);
  }
  if(_extrinsic1Path.length() != 0 )
  {
    Eigen::Isometry3d temrelaPose;
    Eigen::AngleAxisd rotateY(camera1_pre_theta*M_PI/180,Eigen::Vector3d(0,1,0));
    readExtrinsic(_extrinsic1Path,temrelaPose);
    _relative_poses.push_back(temrelaPose*rotateY);
  }
  if(_extrinsic2Path.length() != 0 )
  {
    Eigen::Isometry3d temrelaPose;
    Eigen::AngleAxisd rotateY(camera2_pre_theta*M_PI/180,Eigen::Vector3d(0,1,0));
    readExtrinsic(_extrinsic2Path,temrelaPose);
    _relative_poses.push_back(temrelaPose*rotateY);
  }
  if(_extrinsic3Path.length() != 0 )
  {
    Eigen::Isometry3d temrelaPose;
    Eigen::AngleAxisd rotateY(camera3_pre_theta*M_PI/180,Eigen::Vector3d(0,1,0));
    readExtrinsic(_extrinsic3Path,temrelaPose);
    _relative_poses.push_back(temrelaPose*rotateY);
  }
  if(_extrinsic4Path.length() != 0 )
  {
    Eigen::Isometry3d temrelaPose;
    Eigen::AngleAxisd rotateY(camera4_pre_theta*M_PI/180,Eigen::Vector3d(0,1,0));
    readExtrinsic(_extrinsic4Path,temrelaPose);
    _relative_poses.push_back(temrelaPose*rotateY);
  }
  if(!boost::filesystem::exists(_intrinsicPath))
    std::cout<<"path is not exists"<<std::endl;
  readIntrinsic(_intrinsicPath);

  _images_bufffer.resize(_camera_number);
  _vec_pose_image.resize(_camera_number);
  for(int i=0;i<_camera_number;i++)
  {
    _vec_pose_image[i].reserve(10);
    _images_bufffer[i].ensureCapacity(25);
  }
  //_sub_pointscloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_cloud_surround",2));
  _sub_pointscloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_cloud_surround",2));
  _sub_odom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/aft_mapped_to_init",2));
  _syn.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>(*_sub_pointscloud,*_sub_odom,2));
  _syn->registerCallback(boost::bind(&PointCloudImage::pointcloudCallback,this,_1,_2));
  _sub_image1=nh.subscribe<sensor_msgs::Image>("/pylon_cam_raw1",2,boost::bind(&PointCloudImage::imageCallback,this,_1,0));
  _sub_image2=nh.subscribe<sensor_msgs::Image>("/pylon_cam_raw2",2,boost::bind(&PointCloudImage::imageCallback,this,_1,1));
  _sub_image3=nh.subscribe<sensor_msgs::Image>("/pylon_cam_raw3",2,boost::bind(&PointCloudImage::imageCallback,this,_1,2));
  _sub_image4=nh.subscribe<sensor_msgs::Image>("/pylon_cam_raw4",2,boost::bind(&PointCloudImage::imageCallback,this,_1,3));

  _pub_pointscloud=nh.advertise<sensor_msgs::PointCloud2>("/rgb_points_cloud",2);
  _pub_odom=nh.advertise<nav_msgs::Odometry>("/rgb_points_cloud_odom",2);

  return true;
}
PointCloudImage::PointCloudImage()
{
   _vec_pointcloud.reserve(10);
}
void PointCloudImage::readIntrinsic(const std::string& path)
{
   std::ifstream in(path);
   in>>_width>>_height;
   std::cout<<_width<<" "<<_height<<std::endl;
   float projected[12];
   for(int i=0;i<12;i++)
   {
     in>>projected[i];
   }
   _projected<<projected[0],projected[1],projected[2],projected[3],
             projected[4],projected[5],projected[6],projected[7],
             projected[8],projected[9],projected[10],projected[11];

   std::cout<<"project"<<_projected<<std::endl;
}
void PointCloudImage::readExtrinsic(const std::string& path,Eigen::Isometry3d& relativePose)
{
  std::ifstream in(path);
   float rela_pose[16];
  for(int i=0;i<16;i++)
  {
    in>>rela_pose[i];
  }
  Eigen::Matrix4d relative_pose;
  relative_pose<<rela_pose[0],rela_pose[1],rela_pose[2],rela_pose[3],
                  rela_pose[4],rela_pose[5],rela_pose[6],rela_pose[7],
                  rela_pose[8],rela_pose[9],rela_pose[10],rela_pose[11],
                  rela_pose[12],rela_pose[13],rela_pose[14],rela_pose[15];
  Eigen::Isometry3d tem_pose(relative_pose);
  relativePose=tem_pose;
  std::cout<<"relative_pose"<<tem_pose.matrix()<<std::endl;
}
void PointCloudImage::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointscloud,const nav_msgs::OdometryConstPtr& odom)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*pointscloud,*points);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud_rgb(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pointcloud_rgb->resize(points->size());
  for(int i=0;i<points->size();i++)
  {
    pointcloud_rgb->points[i].x=points->points[i].x;
    pointcloud_rgb->points[i].y=points->points[i].y;
    pointcloud_rgb->points[i].z=points->points[i].z;
    pointcloud_rgb->points[i].a=0;
  }
  //std::cout<<"size"<<pointcloud_rgb->size()<<std::endl;
  geometry_msgs::Quaternion geo_quat=odom->pose.pose.orientation;
  Eigen::Quaterniond eig_qua(geo_quat.w,geo_quat.x,geo_quat.y,geo_quat.z);

  PCloudPose points_pose;
  points_pose._cloud=pointcloud_rgb;
  points_pose._r=eig_qua;
  points_pose._t=Eigen::Vector3d(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z);
  points_pose._stamp=pointscloud->header.stamp;
  _vec_pointcloud.push_back(points_pose);
  //ros::Time start=ros::Time::now();
  for(int i=0;i<_camera_number;i++)
  {
    calImagePose(i);
    projected(i);
  }
  publishResult();
  //std::cout<<"process time :"<<(ros::Time::now()-start).toSec()*1000<<std::endl;
}
void PointCloudImage::imageCallback(const sensor_msgs::ImageConstPtr& img,int index)
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
  tem_image._stamp=img->header.stamp;//-ros::Duration(0.04);
  _images_bufffer[index].push(tem_image);
}
void PointCloudImage::calImagePose(int index)
{
  if(_vec_pointcloud.empty()) return;
  for(int i=0;i<_images_bufffer[index].size();i++)
  {
    ros::Time image_time=_images_bufffer[index][i]._stamp;
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
        image_pose._image=_images_bufffer[index][i]._image;
        image_pose._t=disi;
        image_pose._r=quati;
        image_pose._stamp=_images_bufffer[index][i]._stamp;
        _vec_pose_image[index].push_back(image_pose);
        break;
      }
    }
  }
}
void PointCloudImage::projected(int index)
{
   if(_vec_pose_image[index].empty() || _vec_pointcloud.empty()) return;
   int erase_image_index=0;
   std::vector<Image>& vec_pose_image=_vec_pose_image[index];
   for(int i=0;i<_vec_pointcloud.size();i++)
   {
     PCloudPose& select_points=_vec_pointcloud[i];
     for(int j=0;j<vec_pose_image.size()-1;j++)
     {
       //if pointcloud is in between pose
       if(select_points._stamp.toSec()>=vec_pose_image[j]._stamp.toSec() &&
          select_points._stamp.toSec()<=vec_pose_image[j+1]._stamp.toSec())
       {
         float time_to_start=select_points._stamp.toSec()-vec_pose_image[j]._stamp.toSec();
         float time_to_end=vec_pose_image[j+1]._stamp.toSec()-select_points._stamp.toSec();
         const Image& select_image=time_to_start>=time_to_end?vec_pose_image[j+1]:vec_pose_image[j];

         //pcl::io::savePCDFileASCII("/home/mameng/PCDs/msg_point_cloud_debug.pcd",*(select_points._cloud));
         //std::ofstream in("/home/mameng/PCDs/pose.txt");
         //cv::imwrite("/home/mameng/PCDs/image_debug.png",select_image._image);
         Eigen::Isometry3d image_pose=Eigen::Isometry3d::Identity();
         image_pose.rotate(select_image._r.toRotationMatrix());
         image_pose.pretranslate(select_image._t);
         //in<<image_pose.matrix()<<std::endl;
         Eigen::Isometry3d pointscloud_pose=Eigen::Isometry3d::Identity();
         pointscloud_pose.rotate(select_points._r.toRotationMatrix());
         pointscloud_pose.pretranslate(select_points._t);
         //in<<pointscloud_pose.matrix()<<std::endl;
         // the relative of pointcloud and image
         Eigen::Isometry3d relative_pose=image_pose.inverse()*pointscloud_pose;
         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr orignal_cloud=select_points._cloud;
         //pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
         //rgb_cloud->reserve(orignal_cloud->size());
         //set BGR
         //pcl::io::savePCDFileASCII("/home/mameng/PCDs/msg_point_cloud3.pcd",*(select_points._cloud));
         //cv::imwrite("/home/mameng/PCDs/image3.png",select_image._image);
         cv::Mat resizeImage;
         cv::resize(select_image._image,resizeImage,cv::Size(select_image._image.cols*_resizeImageTimes,select_image._image.rows*_resizeImageTimes));

         for(int p=0;p<orignal_cloud->size();p++)
         {
           pcl::PointXYZRGBA& point=orignal_cloud->points[p];
           //pcl::PointXYZRGB rgb_point;
           Eigen::Vector4d transform_point=relative_pose*point.getVector4fMap().cast<double>();
           //if the point is in the front of camera
           transform_point(0)=-transform_point(0);
           transform_point(1)=-transform_point(1);
           Eigen::Vector4d camera_point=_relative_poses[index]*transform_point;
           if(camera_point(2)>0)
           {
             Eigen::Vector3d project_point=_projected*camera_point;
             project_point(0)/=project_point(2);
             project_point(1)/=project_point(2);
             if(project_point(0)>=0 && project_point(0)<=_width-1 &&
                project_point(1)>=0 && project_point(1)<=_height-1)
             {
                cv::Vec3b RGB=interpolate(project_point(0),project_point(1),resizeImage);
               // image_debug.at<cv::Vec3b>(int(project_point(1)),int(project_point(0)))[0]=255;
               // image_debug.at<cv::Vec3b>(int(project_point(1)),int(project_point(0)))[1]=255;
               // image_debug.at<cv::Vec3b>(int(project_point(1)),int(project_point(0)))[2]=255;
                //std::cout<<"RGB"<<" "<<int(RGB(0))<<" "<<int(RGB(1))<<" "<<int(RGB(2))<<std::endl;
                if(RGB(0)>240 && RGB(1)>240 && RGB(2)>240)
                  continue;
                point.r=(point.r*point.a+RGB(2))/(point.a+1);
                point.g=(point.g*point.a+RGB(1))/(point.a+1);
                point.b=(point.b*point.a+RGB(0))/(point.a+1);
                point.a++;
                //rgb_point.x=orignal_cloud->points[p].x;
               // rgb_point.y=orignal_cloud->points[p].y;
                //rgb_point.z=orignal_cloud->points[p].z;
                //rgb_cloud->push_back(rgb_point);
             }
           }
         }
        /* std::stringstream ss;
         ss<<view;
         std::string name;
         ss>>name;
         std::ofstream in("/home/mameng/calibration/pose/"+name+"pose_view.txt");
         in<<relative_pose.matrix()<<std::endl;
         in.close();
         cv::imwrite("/home/mameng/calibration/image/"+name+"image_view.png",image_debug);
         pcl::io::savePCDFileASCII("/home/mameng/calibration/cloud/"+name+"cloud_view.pcd",*orignal_cloud);
         view++;*/
         select_points._is_rgb=true;
         erase_image_index=std::max(erase_image_index,j);
         break;
       }
     }
   }
   //_vec_pointcloud.erase(_vec_pointcloud.begin(),_vec_pointcloud.begin()+erase_cloud_index);
   vec_pose_image.erase(vec_pose_image.begin(),vec_pose_image.begin()+erase_image_index);
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
  int erase_index=-1;
  //std::cout<<"_vec_pointcloud.size() before"<<_vec_pointcloud.size()<<std::endl;
  for(int i=0;i<_vec_pointcloud.size();i++)
  {
    if(_vec_pointcloud[i]._is_rgb)
    {
      //only publish the point which has rgb information
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rbg_points(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr deal_points=_vec_pointcloud[i]._cloud;
      int cloud_size=deal_points->size();
      rbg_points->reserve(cloud_size);
      for(int p=0;p<cloud_size;p++)
      {
        pcl::PointXYZRGB point;
        if(deal_points->points[p].a>0)
        {
          point.x=deal_points->points[p].x;
          point.y=deal_points->points[p].y;
          point.z=deal_points->points[p].z;
          point.r=deal_points->points[p].r;
          point.g=deal_points->points[p].g;
          point.b=deal_points->points[p].b;
          rbg_points->push_back(point);
        }
        else
        {
          point.x=deal_points->points[p].x;
          point.y=deal_points->points[p].y;
          point.z=deal_points->points[p].z;
          point.r=255;
          point.g=255;
          point.b=255;
          rbg_points->push_back(point);
        }
      }
      erase_index=i;
      //std::cout<<"size"<<rbg_points->size()<<std::endl;
      publishCloudMsg(_pub_pointscloud, *(rbg_points), _vec_pointcloud[i]._stamp, "/camera_init");
      //pcl::io::savePCDFileASCII("/home/mameng/PCDs/cloud/msg_point_cloud3.pcd",*rbg_points);
      nav_msgs::Odometry laserOdometry;
      laserOdometry.header.stamp = _vec_pointcloud[i]._stamp;
      laserOdometry.pose.pose.orientation.x =_vec_pointcloud[i]._r.x();
      laserOdometry.pose.pose.orientation.y = _vec_pointcloud[i]._r.y();
      laserOdometry.pose.pose.orientation.z = _vec_pointcloud[i]._r.z();
      laserOdometry.pose.pose.orientation.w = _vec_pointcloud[i]._r.w();
      laserOdometry.pose.pose.position.x = _vec_pointcloud[i]._t(0);
      laserOdometry.pose.pose.position.y = _vec_pointcloud[i]._t(1);
      laserOdometry.pose.pose.position.z = _vec_pointcloud[i]._t(2);
      _pub_odom.publish(laserOdometry);
    }
  }
  _vec_pointcloud.erase(_vec_pointcloud.begin(),_vec_pointcloud.begin()+erase_index+1);
  //std::cout<<"_vec_pointcloud.size() aft"<<_vec_pointcloud.size()<<std::endl;
}
}
