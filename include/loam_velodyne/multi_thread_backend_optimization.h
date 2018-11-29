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
#include"loam_velodyne/keyframeupdate.h"
#include"floordetection.h"
#include<pcl/compression/octree_pointcloud_compression.h>
#include<std_msgs/String.h>
#include<std_msgs/ByteMultiArray.h>
#include<std_msgs/Byte.h>
#include "draco/compression/encode.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/point_cloud_io.h"
#include "draco/core/macros.h"
#include<draco/attributes/geometry_attribute.h>
#include<loam_velodyne/loop_detector_pcl.hpp>
namespace loam {
struct Options {
  Options(int posQBits);

  bool is_point_cloud;
  int pos_quantization_bits;
  int tex_coords_quantization_bits;
  bool tex_coords_deleted;
  int normals_quantization_bits;
  bool normals_deleted;
  int generic_quantization_bits;
  bool generic_deleted;
  int compression_level;
  bool use_metadata;
  std::string input;
  std::string output;
};
class BackendOptimization
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef pcl::PointXYZRGB PointT;
  BackendOptimization();
  bool setup(ros::NodeHandle& nh,ros::NodeHandle&private_nh );
  void process();
  void spin();
  void laser_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void laser_odom_callback(const nav_msgs::OdometryConstPtr& odom );
  void laser_floor_coeffs_callback(const loam_velodyne::FloorCoeffsConstPtr& floor);
  void laser_submap_vecodom_callback(const loam_velodyne::VectorOdometryConstPtr &vec_odom);

  void laser_cloud_odom_vecodom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                         const nav_msgs::OdometryConstPtr& odom,
                                         const sensor_msgs::PointCloud2ConstPtr& flat,
                                         const sensor_msgs::PointCloud2ConstPtr& flatcorner,
                                         const sensor_msgs::PointCloud2ConstPtr& outlier);
  void received_cloud_callback(const std_msgs::ByteMultiArrayConstPtr& cloudAndPosePtr);
  std::vector<std::string> split(const std::string& str, const std::string& delim);
  bool flush_floor_queue();
  bool flush_keyFrame_queue();
  bool flushReceivedKeyframeQueue();
  void graph_optimization_timer_callback(const ros::TimerEvent& event);
  void pub_map_pointcloud_timer_callback(const ros::TimerEvent& event);
  bool save_map_cllback(loam_velodyne::SaveMapRequest& req,loam_velodyne::SaveMapResponse& res);
  bool mapvis_info_cllback(loam_velodyne::GlobalMapRequest& req,loam_velodyne::GlobalMapResponse& res);
  visualization_msgs::MarkerArray creat_trajectory(const ros::Time& stamp);
  g2o::VertexPlane* select_global_plane_node(const Eigen::Isometry3d& pose_now,const Eigen::Vector4d& plane_now);
  void transportCloudToAnathorMachine(const Eigen::Isometry3d& pose,pcl::PointCloud<PointT>::ConstPtr cloud,uint64_t keyframeId,pcl::PointCloud<PointT>::Ptr& test_cloud);
  void decoder_image(std_msgs::ByteMultiArray received_image,pcl::PointCloud<PointT>::Ptr& testCloud);
  bool calChildToMasterPose();
  bool childNodeInit();
  void pubPoseGraphTochild();
  void receivedPoseGraph(const std_msgs::ByteMultiArrayConstPtr& poseGraph);
  void decodereceivedPosegraph(const std_msgs::ByteMultiArray poseGraph);
  void float2Pose(const std_msgs::ByteMultiArray& posebegin,std::vector<double>& posegraph);
  std::unique_ptr<draco::PointCloud> pclCloudToDracoCloud(pcl::PointCloud<PointT>::ConstPtr cloud);
  void dracoCloudToPCLCloud(const draco::PointCloud& cloud,pcl::PointCloud<PointT>::Ptr convert_cloud);
  void pubedlishChilsTomasterPose();
  void pubAnathorMachineVisOdom(const Eigen::Isometry3d& pose);
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
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_flat_cloud;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_flatcorner_cloud;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> _sub_outlier_cloud;
  std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry,
  sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2>> _time_syn;

  ros::Subscriber _sub_floor_coeffs;

  int _max_keyframes_per_update;
  float _floor_edge_stddev;
  //loop closure
  std::unique_ptr<LoopDetectorICP> _loop_detector;
  std::unique_ptr<LoopDetectorICP> _child_loop_detector;
  std::unique_ptr<LoopDetectorICP> _childmaster_loop_detector;
  std::unique_ptr<LoopDetectorICP> _masterchild_loop_detector;


  //define timer to execute graph optimization and publish map pointcloud
  ros::Timer _optimization_timer;
  ros::Timer _pub_map_pointcloud;
  //information matrix
  std::unique_ptr<InformationMatrixCalculator> _info_calculator;
  //update interval
  float _map_cloud_update_interval;
  float _graph_update_interval;
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
 // ros::Publisher _vis_odom_pub;
  ros::Publisher optim_trans2map_pub;
  ros::Time laser_cloud_time;

  std::unique_ptr<KeyframeUpdater> _keyframe_update;
  std::unique_ptr<FloorDetection> _floor_detecter;

  //whether floor optim
  bool _is_floor_optim;
  std::vector<g2o::VertexPlane*> _global_floor_plane_nodes;
  std::vector<Eigen::Isometry3d> _global_floor_plane_pose;
  double _floor_normal_thresh;
  double _floor_distance_thresh;

  //pointcloud compression
  std::shared_ptr<pcl::io::OctreePointCloudCompression<PointT>> pointCloudEncoder;
  std::shared_ptr<pcl::io::OctreePointCloudCompression<PointT>> pointCloudDecoder;
  ros::Publisher _pubCompressCloud;
  ros::Subscriber _subCompressCloud;
  std::vector<std::shared_ptr<KeyFrame>> _newReceivedKeyFrames;
  std::deque<std::shared_ptr<KeyFrame>> _dequeReceivedKeyFrames;
  std::vector<std::shared_ptr<KeyFrame>> _receivedKeyFrames;
  //
  std::string _carId="child";
  std::string _calMethod="distribution";
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
  Eigen::Isometry3d _childToMaster;
  Eigen::Isometry3d _preReceivedPose;
  Eigen::Isometry3d _receivedTransodom2map;
  double _receivedAccumDistance;
  bool initial=false;
  std::mutex _receivedKeyframesQueueMutex;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> _receivedKeyframeHash;
  //publish and subscribe pose graph
  ros::Publisher _pubPoseGraph;
  ros::Subscriber _subPoseGraph;
  std::mutex _poseGraphMutex;
  std::unordered_map<uint64_t, KeyFrame::Ptr> _masterPoseGraphHash;
  std::unordered_map<uint64_t, KeyFrame::Ptr> _childPoseGraphHash;
  //frame id
  uint64_t _frameId;
  pcl::VoxelGrid<PointT> _downSizeFilterSendCloud; 
  int send_cloud_num=0;
  int receied_cloud_num=0;
  int _pos_quantization_bits;
  std::unique_ptr<Options> _options;
  draco::Encoder _encoder;
  std::vector<KeyFrame::Ptr> _initLocalkeyframe;

  int init_received_dequeue_lengh;
  int init_local_dequeue_lengh;
  int init_search_radius_num;
  double init_fitness_score_thresh;
  pcl::VoxelGrid<pcl::PointXYZI> _downloadInitSourceCloud;
  pcl::VoxelGrid<pcl::PointXYZI> _downloadInitTargetCloud;
  ros::ServiceClient childToMasterClient;
  bool pubedlishChilsTomasterPoseFlag;
  ros::Publisher pubAnathorVisOdom;

  ros::Publisher pubVisOdom_debug;
};
typedef union
{
  float dataf;
  char datac[4];
}FloatChar;
typedef union
{
  float datad;
  char datac[4];
}DoubleChar;
}
#endif // MULTI_THREAD_BACKEND_OPTIMIZATION_H
