#ifndef MAP_CLOUD_GENERATOR_H
#define MAP_CLOUD_GENERATOR_H
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<loam_velodyne/keyframe.hpp>
#include<pcl/common/transforms.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
namespace loam {
class MapCloudGenerate
{
public:
  using Ptr=std::shared_ptr<MapCloudGenerate>;
  MapCloudGenerate()
  {
    _downSizeFilter.setLeafSize(0.4,0.4,0.4);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr generate(std::vector<KeyFrameSnapshot::Ptr> keyframes,double resolution,double distance,bool global)
  {
    _downSizeFilter.setLeafSize(resolution,resolution,resolution);
    if(keyframes.empty()) return nullptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->reserve(keyframes.front()->_cloud->size()*keyframes.size());
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
     // std::cout<<"pose"<<keyframe->_pose.matrix()<<std::endl;
      pcl::PointCloud<pcl::PointXYZI>::Ptr keyframe_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      keyframe_cloud->reserve(keyframe->_cloud->size());
      pcl::transformPointCloud(*(keyframe->_cloud),*(keyframe_cloud),keyframe->_pose.cast<float>());
      //pcl::transformPointCloud(*(keyframe->_cloud),*(keyframe_cloud),relative_pose);
      *cloud+=*keyframe_cloud;
    }
    cloud->width=cloud->size();
    cloud->height=1;
    cloud->is_dense=false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filtered->reserve(cloud->size());
    _downSizeFilter.setInputCloud(cloud);
    _downSizeFilter.filter(*filtered);
   /* pcl::octree::OctreePointCloud<pcl::PointXYZI> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    octree.getOccupiedVoxelCenters(filtered->points);

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;*/

    return filtered;
  }
private:
  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilter;
};
}
#endif // MAP_CLOUD_GENERATOR_H
