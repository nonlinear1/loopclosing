#include<ros/ros.h>
#include"loam_velodyne/multi_thread_backend_optimization.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "backendOptimization");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::BackendOptimization optimization;

  if (optimization.setup(node, privateNode)) {
    // initialization successful
    std::cout<<"ffffffff"<<std::endl;
    optimization.spin();
  }

  return 0;
}
