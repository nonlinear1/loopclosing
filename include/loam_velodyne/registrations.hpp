#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <ros/ros.h>

#include <pcl/registration/registration.h>

namespace loam {

/**
 * @brief select a scan matching algorithm according to rosparams
 * @param pnh
 * @return selected scan matching
 */
using PointT=pcl::PointXYZI;
boost::shared_ptr<pcl::Registration<PointT, PointT>> select_registration_method(ros::NodeHandle& pnh);
boost::shared_ptr<pcl::Registration<PointT, PointT>> select_init_registration_method(ros::NodeHandle& pnh);

}

#endif //
