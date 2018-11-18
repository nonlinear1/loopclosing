#include <ros/ros.h>
#include "loam_velodyne/floordetection.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "floorDetection");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::FloorDetection floor_detection;

  if (floor_detection.setup(node, privateNode)) {
    // initialization successful
    floor_detection.spin();
  }

  return 0;
}
