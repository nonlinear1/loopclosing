#include <ros/ros.h>
#include "loam_velodyne/pointcloudimage.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserImage");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::PointCloudImage laserImage;

  if (laserImage.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
