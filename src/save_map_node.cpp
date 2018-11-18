#include<ros/ros.h>
#include "ros/ros.h"
#include"loam_velodyne/SaveMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "save_map");
   if (argc != 3)
     {
      ROS_INFO("need the savemap destination and resolution");
      return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<loam_velodyne::SaveMap>("/save_map");
    loam_velodyne::SaveMap srv;
    srv.request.destination = argv[1];
    srv.request.resolution = atoll(argv[2]);
    if (client.call(srv))
    {
      if(srv.response.success)
      {
          ROS_INFO("success to save pointcloud");
      }
      else
      {
          ROS_INFO("fail to save pointcloud");
      }
    }
    else
    {
      ROS_INFO("fail to save pointcloud");
      return 1;
    }

    return 0;
}
