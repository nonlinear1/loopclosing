#include"loam_velodyne/floordetection.h"
#include"loam_velodyne/FloorCoeffs.h"
#include<time.h>
namespace loam {
FloorDetection::FloorDetection(const float& sensor_height,
                               const float& height_clip_range,
                               const int& floor_pts_thresh,
                               const float& floor_normal_thresh,
                               const bool& use_normal_filtering,
                               const float& normal_filter_thresh):
  _sensor_height(sensor_height),
  _height_clip_range(height_clip_range),
  _floor_pts_thresh(floor_pts_thresh),
  _floor_normal_thresh(floor_normal_thresh),
  _use_normal_filtering(use_normal_filtering),
  _normal_filter_thresh(normal_filter_thresh),
  _laser_surf_cloud(new pcl::PointCloud<pcl::PointXYZI>())

{
}
bool FloorDetection::setup(ros::NodeHandle nh,
                           ros::NodeHandle privateNode)
{
  bool bparam;
  int iparam;
  float fparam;
  if(privateNode.getParam("sensor_height",fparam))
  {
    _sensor_height=fparam;
  }
  if(privateNode.getParam("height_clip_range",fparam))
  {
    _height_clip_range=fparam;
  }
  if(privateNode.getParam("floor_pts_thresh",iparam))
  {
    _floor_pts_thresh=iparam;
  }
  if(privateNode.getParam("floor_normal_thresh",fparam))
  {
     _floor_normal_thresh=fparam;
  }
  if(privateNode.getParam("use_normal_filtering",bparam))
  {
    _use_normal_filtering=bparam;
  }
  if(privateNode.getParam("normal_filter_thresh",fparam))
  {
    _normal_filter_thresh=fparam;
  }
  _sub_SubmapFlatCloud=nh.subscribe<sensor_msgs::PointCloud2>("/laser_submap_flatcloud",2,&FloorDetection::cloud_callback,this);
  _pub_floor=nh.advertise<loam_velodyne::FloorCoeffs>("/laser_cloud_normal",2);
  return true;
}
void FloorDetection::spin()
{
  ros::spin();
}
void FloorDetection::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfMsg)
{
  _laser_cloud_stamp=laserCloudSurfMsg->header.stamp;
  _laser_surf_cloud->clear();
  pcl::fromROSMsg(*laserCloudSurfMsg,*_laser_surf_cloud);
  ros::Time start=ros::Time::now();
  boost::optional<Eigen::Vector4f> floor=detect(_laser_surf_cloud);
  publishResult(floor);
  std::cout<<"floor time:"<<(ros::Time::now()-start).toSec()*1000<<"ms"<<std::endl;
}
boost::optional<Eigen::Vector4f> FloorDetection::detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  // compensate the tilt rotation

  // filtering before RANSAC (height and normal filtering)
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

  filtered = plane_clip(cloud, Eigen::Vector4f(0.0f, 1.0f, 0.0f, _sensor_height + _height_clip_range), false);
  filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 1.0f, 0.0f, _sensor_height - _height_clip_range), true);
  if(_use_normal_filtering) {
    filtered = normal_filtering(filtered);
  }
 // std::cout<<"filter"<<filtered->size()<<std::endl;
  // too few points for RANSAC
  if(filtered->size() < _floor_pts_thresh) {
    return boost::none;
  }

  // RANSAC
  pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(filtered));
  pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
  ransac.setDistanceThreshold(0.1);
  ransac.computeModel();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  ransac.getInliers(inliers->indices);
  //std::cout<<"ransac"<<inliers->indices.size()<<std::endl;
  // too few inliers
  if(inliers->indices.size() < _floor_pts_thresh) {
    return boost::none;
  }

  // verticality check of the detected floor's normal
  Eigen::Vector4f reference =Eigen::Vector4f::UnitY();

  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);

  double dot = coeffs.head<3>().dot(reference.head<3>());
  if(std::abs(dot) < std::cos(_floor_normal_thresh * M_PI / 180.0)) {
    // the normal is not vertical
    return boost::none;
  }

  // make the normal upward
  if(coeffs.head<3>().dot(Eigen::Vector3f::UnitY()) < 0.0f) {
    coeffs *= -1.0f;
  }
  //std::cout<<"floor norm:"<<coeffs<<std::endl;
  return Eigen::Vector4f(coeffs);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr FloorDetection::plane_clip(const pcl::PointCloud<pcl::PointXYZI>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative)
{
  pcl::PlaneClipper3D<pcl::PointXYZI> clipper(plane);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  clipper.clipPointCloud3D(*src_cloud, indices->indices);

  pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*dst_cloud);

  return dst_cloud;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr FloorDetection::normal_filtering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(10);
  ne.setViewPoint(0.0f, 0.0f, _sensor_height);
  ne.compute(*normals);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  filtered->reserve(cloud->size());

  for (int i = 0; i < cloud->size(); i++) {
    float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitY());
    if (std::abs(dot) > std::cos(_normal_filter_thresh * M_PI / 180.0)) {
      filtered->push_back(cloud->at(i));
    }
  }

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}
void FloorDetection::publishResult(boost::optional<Eigen::Vector4f> floor)
{
    loam_velodyne::FloorCoeffs floorCoeff;
    floorCoeff.header.stamp = _laser_cloud_stamp;
    if(floor) {
      floorCoeff.coeffs.resize(4);
      for(int i=0; i<4; i++) {
        floorCoeff.coeffs[i] = (*floor)[i];
      }
    }
    _pub_floor.publish(floorCoeff);
}
}
