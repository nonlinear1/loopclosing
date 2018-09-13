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
  MapCloudGenerate(){}

  pcl::PointCloud<pcl::PointXYZI>::Ptr generate(std::vector<KeyFrameSnapshot::Ptr> keyframes,double resolution)
  {
    if(keyframes.empty()) return nullptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->reserve(keyframes.front()->_cloud->size()*keyframes.size());
    std::cout<<"map cloud size:"<<keyframes.front()->_cloud->size()<<std::endl;
    for(KeyFrameSnapshot::Ptr keyframe:keyframes)
    {
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
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    downSizeFilter.setLeafSize(0.2,0.2,0.2);
    downSizeFilter.setInputCloud(cloud);
    downSizeFilter.filter(*filtered);
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
};
}
#endif // MAP_CLOUD_GENERATOR_H
