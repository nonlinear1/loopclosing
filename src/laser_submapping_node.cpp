#include <ros/ros.h>
#include "loam_velodyne/LaserSubMapping.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserSubMapping laserSubMapping(0.1);

  if (laserSubMapping.setup(node, privateNode)) {
    // initialization successful
    laserSubMapping.spin();
  }

  return 0;
}
