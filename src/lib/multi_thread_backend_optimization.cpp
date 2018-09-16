#include"loam_velodyne/multi_thread_backend_optimization.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/filters/filter.h>
#include<geometry_msgs/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include"loam_velodyne/SaveMap.h"
#include <std_srvs/Empty.h>
#include<g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include<boost/filesystem.hpp>
namespace loam {
BackendOptimization::BackendOptimization():
  _max_keyframes_per_update(10),
  _floor_edge_stddev(10.0),
  _graph_optimization_time_duration(3.0),
  _pub_map_pointcloud_time_duration(10.0),
  _accumulate_distance(0)
{
   _trans_odom2map=Eigen::Isometry3d::Identity();
    _display_distance_threash=100;
    _display_resolution=0.4;
   _is_global_map=false;
}
bool BackendOptimization::setup(ros::NodeHandle& nh,ros::NodeHandle&private_nh )
{
  _max_keyframes_per_update=private_nh.param<int>("max_keyframes_per_update",10);
 _floor_edge_stddev=private_nh.param<float>("floor_edge_stddev",10.0);
 _graph_optimization_time_duration=private_nh.param<float>("graph_optimization_time_duration",3.0);
 _pub_map_pointcloud_time_duration=private_nh.param<float>("pub_map_pointcloud_time_duration",10.0);
 _floor_edge_stddev=private_nh.param<float>("floor_edge_stddev",10.0);

  _info_calculator.reset(new InformationMatrixCalculator(nh));
  _loop_detector.reset(new LoopDetector(nh));
  graph_slam.reset(new GraphSLAM());
   _map_generate.reset(new MapCloudGenerate());
  _sub_laser_submap_pointcloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_submap_cloud",2));
  _sub_submap_odom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/submap_odometry",2));
  _sub_submap_vecodom.reset(new message_filters::Subscriber<loam_velodyne::VectorOdometry>(nh,"/submap_vecOdom",2));
  _time_syn.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry,
                  loam_velodyne::VectorOdometry>(*_sub_laser_submap_pointcloud,*_sub_submap_odom,
                                                 *_sub_submap_vecodom,2));
  _time_syn->registerCallback(boost::bind(&BackendOptimization::laser_cloud_odom_vecodom_callback, this, _1, _2,_3));

  _sub_floor_coeffs=nh.subscribe<loam_velodyne::FloorCoeffs>("/laser_cloud_normal",32,&BackendOptimization::laser_floor_coeffs_callback,this);

  _optimization_timer=nh.createTimer(ros::Duration(_graph_optimization_time_duration),&BackendOptimization::graph_optimization_timer_callback,this);
  _pub_map_pointcloud=nh.createTimer(ros::Duration(_pub_map_pointcloud_time_duration),&BackendOptimization::pub_map_pointcloud_timer_callback,this);
  _pub_map_point=nh.advertise<sensor_msgs::PointCloud2>("/map_points",1);

  _map_save_server=nh.advertiseService("/save_map",&BackendOptimization::save_map_cllback,this);
  _map_info_server=nh.advertiseService("/setup_map",&BackendOptimization::mapvis_info_cllback,this);
  _vis_pub=nh.advertise<visualization_msgs::MarkerArray>("/markers",2);
  _vis_odom_pub=nh.advertise<visualization_msgs::Marker>("/odom_marker",2);
}
void BackendOptimization::BackendOptimization::process(){}
void BackendOptimization::spin()
{
   ros::AsyncSpinner spinner(3); // Use 3 threads
   spinner.start();
   ros::waitForShutdown();
}
void BackendOptimization::laser_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud){}
void BackendOptimization::laser_odom_callback(const nav_msgs::OdometryConstPtr& odom ){}
void BackendOptimization::laser_submap_vecodom_callback(const loam_velodyne::VectorOdometryConstPtr &vec_odom){}

void BackendOptimization::laser_cloud_odom_vecodom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                       const nav_msgs::OdometryConstPtr& odom ,
                                       const loam_velodyne::VectorOdometryConstPtr &vec_odom)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud, *laser_pointcloud);
  std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
  key_frame->_cloud=laser_pointcloud;
  geometry_msgs::Quaternion geo_quat=odom->pose.pose.orientation;
  Eigen::Quaterniond eig_qua(geo_quat.w,geo_quat.x,geo_quat.y,geo_quat.z);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  pose.rotate(eig_qua.toRotationMatrix());
  pose.pretranslate(Eigen::Vector3d(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z));
  key_frame->iso_pose=pose;

  //publish now odometry
  visualization_msgs::Marker sphere_marker;
  sphere_marker.header.frame_id = "/camera_init";
  sphere_marker.header.stamp = ros::Time::now();
  sphere_marker.ns = "odometry";
  sphere_marker.id = 0;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  Eigen::Vector3d pos = pose.translation();
  sphere_marker.pose.position.x = pos(0);
  sphere_marker.pose.position.y = pos(1);
  sphere_marker.pose.position.z = pos(2);
  sphere_marker.pose.orientation.w = 1.0;
  sphere_marker.scale.x=_loop_detector->get_distance_thresh();
  sphere_marker.scale.y =_loop_detector->get_distance_thresh();
  sphere_marker.scale.z =_loop_detector->get_distance_thresh();
  sphere_marker.color.r = 1.0;
  sphere_marker.color.a = 0.3;
  _vis_odom_pub.publish(sphere_marker);

  key_frame->vec_pose.reserve(vec_odom->odomes.size());
  for(std::vector<nav_msgs::Odometry>::const_iterator it=vec_odom->odomes.begin();it !=vec_odom->odomes.end();it++)
  {
    Eigen::Quaterniond vec_quat(it->pose.pose.orientation.w,it->pose.pose.orientation.x,it->pose.pose.orientation.y,it->pose.pose.orientation.z);
    Eigen::Isometry3d vec_pose;
    vec_pose.rotate(vec_quat.toRotationMatrix());
    vec_pose.pretranslate(Eigen::Vector3d(it->pose.pose.position.x,it->pose.pose.position.y,it->pose.pose.position.z));
    key_frame->vec_pose.push_back(vec_pose);
  }
  //calculate the accumulate distance
  Eigen::Isometry3d relative_pose=_pre_pose.inverse()*pose;
  _pre_pose=pose;
  float dx=relative_pose.translation().norm();
  _accumulate_distance+=dx;
  key_frame->_accumulate_distance=_accumulate_distance;

  key_frame->_stamp=cloud->header.stamp;
  std::lock_guard<std::mutex> lock(_keyFrames_queue_mutex);
  _deque_KeyFrames.push_back(key_frame);
}
bool BackendOptimization::flush_keyFrame_queue()
{
  std::lock_guard<std::mutex> lock(_keyFrames_queue_mutex);
  if(_deque_KeyFrames.empty()) return false;
  //get minimize update frames
  int add_frame_num=std::min<int>(_deque_KeyFrames.size(),_max_keyframes_per_update);
  for(int i=0;i<add_frame_num;i++)
  {
    std::shared_ptr<KeyFrame> keyFrame_ptr=_deque_KeyFrames[i];
    //update pose
    Eigen::Isometry3d odom2map= _trans_odom2map*keyFrame_ptr->iso_pose;
    //add se3 vertex to map
    //g2o::VertexSE3* pose_node= graph_slam->addVertexSE3(odom2map);
    g2o::VertexSE3* pose_node= graph_slam->add_se3_node(odom2map);
    keyFrame_ptr->_node=pose_node;
    _keyframe_hash[keyFrame_ptr->_stamp]=keyFrame_ptr;
    _new_keyFrames.push_back(keyFrame_ptr);
    if(i==0 && _keyFrames.empty())
    {
      continue;
    }
    //calculate information matrix and add se3 edge
    std::shared_ptr<KeyFrame> pre_keyFrame=(i==0?_keyFrames.back():_deque_KeyFrames[i-1]);
    Eigen::Isometry3d relative_pose=keyFrame_ptr->iso_pose.inverse()*pre_keyFrame->iso_pose;
    Eigen::MatrixXd information= _info_calculator->calc_information_matrix(keyFrame_ptr->_cloud,pre_keyFrame->_cloud,relative_pose);
    //graph_slam->addEdgeSE3(keyFrame_ptr->_node,pre_keyFrame->_node,relative_pose,information);
    graph_slam->add_se3_edge(keyFrame_ptr->_node,pre_keyFrame->_node,relative_pose,information);
  }
  //erase somes frame
  _deque_KeyFrames.erase(_deque_KeyFrames.begin(),_deque_KeyFrames.begin()+add_frame_num);
  return true;
}
void BackendOptimization::laser_floor_coeffs_callback(const loam_velodyne::FloorCoeffsConstPtr& floor)
{
  if(floor->coeffs.empty()) return;
  std::lock_guard<std::mutex> lock(_deque_floor_mutex);
  _deque_floor.push_back(floor);
}
bool BackendOptimization::flush_floor_queue()
{
  std::lock_guard<std::mutex> lock(_deque_floor_mutex);
  if((_keyFrames.size()+_new_keyFrames.size())==0 || _deque_floor.empty()) return false;
  bool updated=false;
  ros::Time last_keyframe_stamp=_new_keyFrames.back()->_stamp;
  for(loam_velodyne::FloorCoeffsConstPtr floor_coeffs:_deque_floor)
  {
    if(floor_coeffs->header.stamp>last_keyframe_stamp)
    {
      break;
    }
    auto found = _keyframe_hash.find(floor_coeffs->header.stamp);
    if(found == _keyframe_hash.end()) {
      continue;
    }
    KeyFrame::Ptr keyframe = found->second;
    Eigen::Vector4d floor_coeff(floor_coeffs->coeffs[0],floor_coeffs->coeffs[1],floor_coeffs->coeffs[2],floor_coeffs->coeffs[3]);
    Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / _floor_edge_stddev);
//    graph_slam->addEdgeSE3Plane(keyframe->_node,floor_coeff,information);
    graph_slam->add_se3_plane_edge(keyframe->_node,graph_slam->floor_plane_node,floor_coeff,information);
    updated=true;
  }
  auto remove_loc=std::upper_bound(_deque_floor.begin(),_deque_floor.end(),last_keyframe_stamp,[](ros::Time value,loam_velodyne::FloorCoeffsConstPtr floor)
  {
    return value<floor->header.stamp;
  });
  _deque_floor.erase(_deque_floor.begin(),remove_loc);
  return updated;
}

void BackendOptimization::graph_optimization_timer_callback(const ros::TimerEvent& event)
{
  if(!flush_keyFrame_queue() & !flush_floor_queue())
  //if(!flush_keyFrame_queue())
  {
    return;
  }
  ros::Time start_loop=ros::Time::now();
  std::vector<Loop::Ptr> detect_loops=_loop_detector->detect(_keyFrames,_new_keyFrames);
  for(int i=0;i<detect_loops.size();i++)
  {
    Loop::Ptr loop=detect_loops[i];
    Eigen::Isometry3d relative_pose(loop->relative_pose);
    Eigen::MatrixXd infomation= _info_calculator->calc_information_matrix(loop->key1->_cloud,loop->key2->_cloud,relative_pose);
    //graph_slam->addEdgeSE3(loop->key1->_node,loop->key2->_node,relative_pose,infomation);
    graph_slam->add_se3_edge(loop->key1->_node,loop->key2->_node,relative_pose,infomation);
  }
  std::cout<<"loop closure using time is:"<<(ros::Time::now()-start_loop).toSec()*1000<<"ms"<<std::endl;
  //graph_slam->optimization();
  graph_slam->optimize();
  std::copy(_new_keyFrames.begin(),_new_keyFrames.end(),std::back_inserter(_keyFrames));
  _new_keyFrames.clear();
  KeyFrame::Ptr last_key_frame= _keyFrames.back();
  _trans_odom2map=last_key_frame->_node->estimate()*last_key_frame->iso_pose.inverse();
  std::vector<KeyFrameSnapshot::Ptr> snapshot(_keyFrames.size());
  std::transform(_keyFrames.begin(),_keyFrames.end(),snapshot.begin(),[](KeyFrame::Ptr key_frame)
  {
    KeyFrameSnapshot::Ptr tem_snapshot(new KeyFrameSnapshot(key_frame));
    return tem_snapshot;
  });
  _snapshot_cloud_mutex.lock();
  _snapshot_cloud.swap(snapshot);
  _snapshot_cloud_mutex.unlock();
  if(_vis_pub.getNumSubscribers())
  {
    visualization_msgs::MarkerArray makers=creat_trajectory(ros::Time::now());
    _vis_pub.publish(makers);
  }
}
void BackendOptimization::pub_map_pointcloud_timer_callback(const ros::TimerEvent& event)
{
  std::vector<KeyFrameSnapshot::Ptr> snapshot;
  _snapshot_cloud_mutex.lock();
  snapshot=_snapshot_cloud;
  _snapshot_cloud_mutex.unlock();

  ros::Time start=ros::Time::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud=_map_generate->generate(snapshot,_display_resolution,_display_distance_threash,_is_global_map);
  if(!cloud)
  {
    std::cout<<"cloud is empty"<<std::endl;
    return ;
  }
  std::cout<<"publish map time:"<<(ros::Time::now()-start).toSec()*1000<<"ms"<<std::endl;
  cloud->header.stamp=snapshot.back()->_cloud->header.stamp;
  cloud->header=snapshot.back()->_cloud->header;
  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud,*cloud_msg);
  _pub_map_point.publish(cloud_msg);
}
bool BackendOptimization::save_map_cllback(loam_velodyne::SaveMapRequest& req,loam_velodyne::SaveMapResponse& res)
{
 // std::cout<<"fffffffff"<<std::endl;
  std::vector<KeyFrameSnapshot::Ptr> snapshot;
  _snapshot_cloud_mutex.lock();
  snapshot=_snapshot_cloud;
  _snapshot_cloud_mutex.unlock();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud=_map_generate->generate(snapshot,req.resolution,_display_distance_threash,true);
  if(!cloud)
  {
    res.success=false;
    return true;
  }
  size_t position=req.destination.find_last_of("/");
  std::string path_dir(req.destination.substr(0,position));
  //std::cout<<"save path:"<<path_dir<<std::endl;
  boost::filesystem::path save_path(path_dir.c_str());
  if(!boost::filesystem::exists(save_path))
  {
    boost::filesystem::create_directory(save_path);
  }
  //std::cout<<"destion"<<req.destination<<std::endl;

  bool ret=pcl::io::savePCDFileBinary(req.destination,*cloud);
  res.success=(ret==0);
  return true;
}
bool BackendOptimization::mapvis_info_cllback(loam_velodyne::GlobalMapRequest& req,loam_velodyne::GlobalMapResponse& res)
{
  _display_distance_threash=req.distance;
  _is_global_map=req.isDisGlobalMap;
  _display_resolution=req.resolution;
  //std::cout<<"_display_distance_threash:"<<_display_distance_threash<<"global"<<_is_global_map
  //        <<"_display_resolution"<<_display_resolution<<std::endl;
  res.success=true;
  return true;
}

visualization_msgs::MarkerArray BackendOptimization::creat_trajectory(const ros::Time& stamp)
{
  visualization_msgs::MarkerArray maker_array;
  maker_array.markers.resize(2);
  visualization_msgs::Marker& nodes_maker=maker_array.markers[0];
  nodes_maker.header.frame_id="/camera_init";
  nodes_maker.header.stamp=stamp;
  nodes_maker.ns="nodes";
  nodes_maker.type=visualization_msgs::Marker::POINTS;
  nodes_maker.action=visualization_msgs::Marker::ADD;
  nodes_maker.pose.orientation.w=1;
  nodes_maker.id=0;
  nodes_maker.scale.x=0.2;
  nodes_maker.scale.y=0.2;
  nodes_maker.scale.z=0.2;
  nodes_maker.color.g=1.0f;
  nodes_maker.color.a=1.0f;
  nodes_maker.points.resize(_keyFrames.size());
  for(int i=0;i<_keyFrames.size();i++)
  {
    KeyFrame::Ptr keyframe=_keyFrames[i];
    Eigen::Vector3d translate= keyframe->_node->estimate().translation();
    nodes_maker.points[i].x=translate(0);
    nodes_maker.points[i].y=translate(1);
    nodes_maker.points[i].z=translate(2);
  }
  visualization_msgs::Marker& edges_maker=maker_array.markers[1];
  edges_maker.header.frame_id="/camera_init";
  edges_maker.header.stamp=stamp;
  edges_maker.ns="edges";
  edges_maker.type=visualization_msgs::Marker::LINE_LIST;
  edges_maker.action=visualization_msgs::Marker::ADD;
  edges_maker.pose.orientation.w=1;
  edges_maker.id=0;
  edges_maker.scale.x=0.1;
  edges_maker.scale.y=0.1;
  edges_maker.scale.z=0.1;
  edges_maker.color.r=1.0f;
  edges_maker.color.a=1.0f;
  edges_maker.points.resize(graph_slam->graph->edges().size()*2);
  auto iter=graph_slam->graph->edges().begin();
  for(int i=0;iter!=graph_slam->graph->edges().end();i+=2,iter++)
  {
    g2o::HyperGraph::Edge* edge=(*iter);
    g2o::EdgeSE3* se3_vertex=dynamic_cast<g2o::EdgeSE3*>(edge);
    if(se3_vertex)
    {
      g2o::VertexSE3* node1=static_cast<g2o::VertexSE3*>(se3_vertex->vertices()[0]);
      g2o::VertexSE3* node2=static_cast<g2o::VertexSE3*>(se3_vertex->vertices()[1]);
      Eigen::Vector3d translate1=node1->estimate().translation();
      Eigen::Vector3d translate2=node2->estimate().translation();
      edges_maker.points[i].x=translate1(0);
      edges_maker.points[i].y=translate1(1);
      edges_maker.points[i].z=translate1(2);

      edges_maker.points[i+1].x=translate2(0);
      edges_maker.points[i+1].y=translate2(1);
      edges_maker.points[i+1].z=translate2(2);
      continue;
    }
    g2o::EdgeSE3Plane* se3plane=dynamic_cast<g2o::EdgeSE3Plane*>(edge);
    if(se3plane)
    {
      g2o::VertexSE3* node1=static_cast<g2o::VertexSE3*>(se3plane->vertices()[0]);
      Eigen::Vector3d translate1=node1->estimate().translation();
      edges_maker.points[i].x=translate1(0);
      edges_maker.points[i].y=translate1(1);
      edges_maker.points[i].z=translate1(2);

      edges_maker.points[i+1].x=translate1(0);
      edges_maker.points[i+1].y=0;
      edges_maker.points[i+1].z=translate1(2);
    }
  }

  return maker_array;
}

}
