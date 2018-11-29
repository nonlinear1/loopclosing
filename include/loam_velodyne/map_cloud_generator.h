#ifndef MAP_CLOUD_GENERATOR_H
#define MAP_CLOUD_GENERATOR_H
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<loam_velodyne/keyframe.hpp>
#include<pcl/common/transforms.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
namespace loam {
class MapCloudGenerate
{
public:
  using Ptr=std::shared_ptr<MapCloudGenerate>;
  using PointT=pcl::PointXYZRGB;
  MapCloudGenerate()
  {
    _downSizeFilter.setLeafSize(0.4,0.4,0.4);
    _downSizeFilterXYZ.setLeafSize(0.4,0.4,0.4);
  }

  pcl::PointCloud<PointT>::Ptr generateRGBCloud(std::vector<KeyFrameSnapshot::Ptr> keyframes,double resolution,double distance,bool global)
  {
    _downSizeFilter.setLeafSize(resolution,resolution,resolution);
    if(keyframes.empty()) return nullptr;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    int cloud_size=0;
    for(int i=0;i<keyframes.size();i++)
    {
      cloud_size+=keyframes[i]->_cloud->size();
    }
    cloud->reserve(cloud_size);
    Eigen::Isometry3d last_keyframe_pose=keyframes.back()->_pose;
    std::cout<<"map cloud size:"<<keyframes.front()->_cloud->size()<<std::endl;
    for(KeyFrameSnapshot::Ptr keyframe:keyframes)
    {
      if(!global)
      {
        Eigen::Isometry3d relative_pose=keyframe->_pose.inverse()*last_keyframe_pose;
        double dx=relative_pose.translation().norm();
        if(dx>distance)
          continue;
      }
      pcl::PointCloud<PointT>::Ptr validCloud(new pcl::PointCloud<PointT>());
      validCloud->reserve(keyframe->_cloud->size());
      for(int i=0;i<keyframe->_cloud->size();i++)
      {
        PointT validPoint=keyframe->_cloud->points[i];
        if(validPoint.r !=255 || validPoint.g!=255 || validPoint.b !=255)
           validCloud->push_back(validPoint);
      }
      pcl::PointCloud<PointT>::Ptr keyframe_cloud(new pcl::PointCloud<PointT>());
      keyframe_cloud->reserve(keyframe->_cloud->size());
      pcl::transformPointCloud(*validCloud,*(keyframe_cloud),keyframe->_pose.cast<float>());
      *cloud+=*keyframe_cloud;
    }
    cloud->width=cloud->size();
    cloud->height=1;
    cloud->is_dense=false;
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());
    _downSizeFilter.setInputCloud(cloud);
    _downSizeFilter.filter(*filtered);

    return filtered;
  }
  pcl::PointCloud<PointT>::Ptr generateFlatRGBCloud(std::vector<KeyFrameSnapshot::Ptr> keyframes,double resolution,double distance,bool global)
  {
    _downSizeFilter.setLeafSize(resolution,resolution,resolution);
    if(keyframes.empty()) return nullptr;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    int cloud_size=0;
    for(int i=0;i<keyframes.size();i++)
    {
      cloud_size+=keyframes[i]->_cloud->size();
    }
    cloud->reserve(cloud_size);
    Eigen::Isometry3d last_keyframe_pose=keyframes.back()->_pose;
    std::cout<<"map cloud size:"<<keyframes.front()->_cloud->size()<<std::endl;
    for(KeyFrameSnapshot::Ptr keyframe:keyframes)
    {
      if(!global)
      {
        Eigen::Isometry3d relative_pose=keyframe->_pose.inverse()*last_keyframe_pose;
        double dx=relative_pose.translation().norm();
        if(dx>distance)
          continue;
      }
      pcl::PointCloud<PointT>::Ptr validCloud(new pcl::PointCloud<PointT>());
      validCloud->reserve(keyframe->_cloud->size());
      for(int i=0;i<keyframe->_cloud->size();i++)
      {
        PointT validPoint=keyframe->_cloud->points[i];
        validPoint.r=255;
        validPoint.g=255;
        validPoint.b=255;
        validCloud->push_back(validPoint);
      }
      pcl::PointCloud<PointT>::Ptr keyframe_cloud(new pcl::PointCloud<PointT>());
      keyframe_cloud->reserve(keyframe->_cloud->size());
      pcl::transformPointCloud(*validCloud,*(keyframe_cloud),keyframe->_pose.cast<float>());
      *cloud+=*keyframe_cloud;
    }
    cloud->width=cloud->size();
    cloud->height=1;
    cloud->is_dense=false;
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());
    _downSizeFilter.setInputCloud(cloud);
    _downSizeFilter.filter(*filtered);

    return filtered;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr generateFlatCloud(std::vector<KeyFrameSnapshot::Ptr> keyframes,double resolution,double distance,bool global)
  {
    _downSizeFilterXYZ.setLeafSize(resolution,resolution,resolution);
    if(keyframes.empty()) return nullptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    int cloud_size=0;
    for(int i=0;i<keyframes.size();i++)
    {
      cloud_size+=keyframes[i]->_cloud->size();
    }
    cloud->reserve(cloud_size);
    Eigen::Isometry3d last_keyframe_pose=keyframes.back()->_pose;
    for(KeyFrameSnapshot::Ptr keyframe:keyframes)
    {
      if(!global)
      {
        Eigen::Isometry3d relative_pose=keyframe->_pose.inverse()*last_keyframe_pose;
        double dx=relative_pose.translation().norm();
        if(dx>distance)
          continue;
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr validCloud(new pcl::PointCloud<pcl::PointXYZ>());
      validCloud->reserve(keyframe->_cloud->size());
      pcl::copyPointCloud(*(keyframe->_cloud),*validCloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      keyframe_cloud->reserve(keyframe->_cloud->size());
      pcl::transformPointCloud(*validCloud,*(keyframe_cloud),keyframe->_pose.cast<float>());
      *cloud+=*keyframe_cloud;
    }
    cloud->width=cloud->size();
    cloud->height=1;
    cloud->is_dense=false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    filteredCloud->reserve(cloud->size());
    _downSizeFilterXYZ.setInputCloud(cloud);
    _downSizeFilterXYZ.filter(*filteredCloud);

    return filteredCloud;
  }
private:
  pcl::VoxelGrid<PointT> _downSizeFilter;
  pcl::VoxelGrid<pcl::PointXYZ> _downSizeFilterXYZ;
};
}
#endif // MAP_CLOUD_GENERATOR_H
