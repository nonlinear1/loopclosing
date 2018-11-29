#include<pclomp/ndt_omp.h>
#include<ros/ros.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/approximate_voxel_grid.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/visualization/pcl_visualizer.h>
int main(int argc, char **argv)
{
  ros::init(argc,argv,"register_test");
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if(pcl::io::loadPCDFile("/home/mameng/projects/rgbpointcloud/target.pcd",*target_cloud)==-1)
  {
    ROS_ERROR("can't load source.pcd");
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if(pcl::io::loadPCDFile("/home/mameng/projects/rgbpointcloud/source.pcd",*source_cloud)==-1)
  {
    ROS_ERROR("can't load target.pcd");
    return -1;
  }
  std::cout<<"target_cloud size"<<target_cloud->size()<<std::endl;
  std::cout<<"source_cloud size"<<source_cloud->size()<<std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_filter_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_filter_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_source;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_target;
  voxel_grid_target.setLeafSize(0.4,0.4,0.4);
  voxel_grid_source.setLeafSize(0.4,0.4,0.4);
  voxel_grid_target.setInputCloud(target_cloud);
  voxel_grid_target.filter(*target_filter_cloud);
  voxel_grid_source.setInputCloud(source_cloud);
  voxel_grid_source.filter(*source_filter_cloud);
  boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  ndt_omp->setInputSource(source_filter_cloud);
  ndt_omp->setInputTarget(target_filter_cloud);
  ndt_omp->setMaximumIterations(100);
  ndt_omp->setTransformationEpsilon(1e-6);
  ndt_omp->setResolution(6);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->align(*output_cloud);

  std::cout<<"output_cloud"<<output_cloud->size()<<std::endl;
  std::cout<<"target_filter_cloud"<<target_filter_cloud->size()<<std::endl;
  std::cout<<"source_filter_cloud"<<source_filter_cloud->size()<<std::endl;
  std::cout<<"score"<<ndt_omp->getFitnessScore()<<std::endl;
  std::cout<<"transform"<<ndt_omp->getFinalTransformation()<<std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> vis(new pcl::visualization::PCLVisualizer("viewer"));
  vis->setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> green(target_filter_cloud,0,255,0);
  vis->addPointCloud<pcl::PointXYZI>(target_filter_cloud,green,"target");
  vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"target");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red(output_cloud,255,0,0);
  vis->addPointCloud<pcl::PointXYZI>(output_cloud,red,"output");
  vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"output");


  vis->addCoordinateSystem(1.0);
  vis->initCameraParameters();
  while(!vis->wasStopped())
  {
    vis->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
}
