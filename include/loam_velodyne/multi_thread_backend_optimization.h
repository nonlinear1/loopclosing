#ifndef MULTI_THREAD_BACKEND_OPTIMIZATION_H
#define MULTI_THREAD_BACKEND_OPTIMIZATION_H
#include<ros/ros.h>
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<sensor_msgs/PointCloud2.h>
#include<nav_msgs/Odometry.h>
#include<loam_velodyne/FloorCoeffs.h>
#include<loam_velodyne/VectorOdometry.h>
#include"loam_velodyne/keyframe.hpp"
#include"graph_slam.h"
#include<deque>
#include<mutex>
#include"loam_velodyne/information_matrix_calculator.hpp"
#include"loam_velodyne/ros_time_hash.hpp"
#include"loop_detector.hpp"
#include"loam_velodyne/map_cloud_generator.h"
#include"loam_velodyne/SaveMap.h"
#include"loam_velodyne/GlobalMap.h"
#include<visualization_msgs/MarkerArray.h>
namespace loam {
class BackendOptimization
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BackendOptimization();
  bool setup(ros::NodeHandle& nh,ros::NodeHandle&private_nh );
  void process();
  void spin();
  void laser_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void laser_odom_callback(const nav_msgs::OdometryConstPtr& odom );
  void laser_floor_coeffs_callback(const loam_velodyne::FloorCoeffsConstPtr& floor);
  void laser_submap_vecodom_callback(const loam_velodyne::VectorOdometryConstPtr &vec_odom);

  void laser_cloud_odom_vecodom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                         const nav_msgs::OdometryConstPtr& odom ,
                                         const loam_velodyne::VectorOdometryConstPtr &vec_odom);
  bool flush_floor_queue();
  bool flush_keyFrame_queue();
  void graph_optimization_timer_callback(const ros::TimerEvent& event);
  void pub_map_pointcloud_timer_callback(const ros::TimerEvent& event);
  bool save_map_cllback(loam_velodyne::SaveMapRequest& req,loam_velodyne::SaveMapResponse& res);
  bool mapvis_info_cllback(loam_velodyne::GlobalMapRequest& req,loam_velodyne::GlobalMapResponse& res);
  visualization_msgs::MarkerArray creat_trajectory(const ros::Time& stamp);
  //keyframe variable
  std::vector<std::shared_ptr<KeyFrame>> _keyFrames;
  std::vector<std::shared_ptr<KeyFrame>> _new_keyFrames;
  std::deque<std::shared_ptr<KeyFrame>> _deque_KeyFrames;
  std::mutex _keyFrames_queue_mutex;
  float _accumulate_distance;
  Eigen::Isometry3d _pre_pose;
  Eigen::Isometry3d _trans_odom2map;//update the pose of new keyframe with respect to updated frame

  //floor coeffs variable
  std::deque<loam_velodyne::FloorCoeffsConstPtr> _deque_floor;
  std::mutex _deque_floor_mutex;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> _keyframe_hash;

  //subrible pointcloud,odometry and submap vector odometry
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_laser_submap_pointcloud;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> _sub_submap_odom;
  std::unique_ptr<message_filters::Subscriber<loam_velodyne::VectorOdometry>> _sub_submap_vecodom;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry,
  loam_velodyne::VectorOdometry>> _time_syn;

  ros::Subscriber _sub_floor_coeffs;

  int _max_keyframes_per_update;
  float _floor_edge_stddev;
  //loop closure
  std::unique_ptr<LoopDetector> _loop_detector;

  //define timer to execute graph optimization and publish map pointcloud
  ros::Timer _optimization_timer;
  ros::Timer _pub_map_pointcloud;
  //information matrix
  std::unique_ptr<InformationMatrixCalculator> _info_calculator;
  //update interval
  float _graph_optimization_time_duration;
  float _pub_map_pointcloud_time_duration;
  //pose graph
  std::unique_ptr<GraphSLAM> graph_slam;
  //pointcloud publish and save
  std::mutex _snapshot_cloud_mutex;
  std::vector<KeyFrameSnapshot::Ptr> _snapshot_cloud;
  std::unique_ptr<MapCloudGenerate> _map_generate;
  ros::Publisher _pub_map_point;
  ros::ServiceServer _map_save_server;
  ros::ServiceServer _map_info_server;
  double _display_distance_threash;
  bool _is_global_map;
  double _display_resolution;
  //visual maker
  ros::Publisher _vis_pub;
  ros::Publisher _vis_odom_pub;
};
}
#endif // MULTI_THREAD_BACKEND_OPTIMIZATION_H
