#include <ros/ros.h>
#include "loam_velodyne/windowoptim.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "windowoptim");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::WindowOptim windowoptim(0.1);

  if (windowoptim.setup(node, privateNode)) {
    // initialization successful
    windowoptim.spin();
  }

  return 0;
}
