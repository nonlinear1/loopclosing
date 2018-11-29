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
#include<std_msgs/String.h>
#include<boost/lexical_cast.hpp>
#include<sensor_msgs/CompressedImage.h>
#include<std_msgs/ByteMultiArray.h>
#include<std_msgs/Byte.h>
#include<loam_velodyne/loop_detector_pcl.hpp>
#include<loam_velodyne/ChildToMaster.h>
namespace loam {
Options::Options(int posQbits)
    : is_point_cloud(true),
      pos_quantization_bits(14),
      tex_coords_quantization_bits(12),
      tex_coords_deleted(false),
      normals_quantization_bits(10),
      normals_deleted(false),
      generic_quantization_bits(8),
      generic_deleted(false),
      compression_level(7),
      use_metadata(false)
{
  pos_quantization_bits=posQbits;
}
BackendOptimization::BackendOptimization():
  _max_keyframes_per_update(10),
  _floor_edge_stddev(10.0),
  _graph_update_interval(3.0),
  _map_cloud_update_interval(10.0),
  _accumulate_distance(0),
  _floor_normal_thresh(8),
  _floor_distance_thresh(1),
  _is_floor_optim(true)
{
   _trans_odom2map=Eigen::Isometry3d::Identity();
    _display_distance_threash=100;
    _display_resolution=0.4;
   _is_global_map=false;
   _preReceivedPose=Eigen::Isometry3d::Identity();
   _receivedAccumDistance=0;
   _receivedTransodom2map=Eigen::Isometry3d::Identity();
   _childToMaster=Eigen::Isometry3d::Identity();
   _frameId=0;
   pubedlishChilsTomasterPoseFlag=false;
}
bool BackendOptimization::setup(ros::NodeHandle& nh,ros::NodeHandle&private_nh )
{
  _max_keyframes_per_update=private_nh.param<int>("max_keyframes_per_update",10);
 _floor_edge_stddev=private_nh.param<float>("floor_edge_stddev",20.0);
 _map_cloud_update_interval=private_nh.param<float>("map_cloud_update_interval",10.0);
 _graph_update_interval=private_nh.param<float>("graph_update_interval",3.0);
 _is_floor_optim=private_nh.param<bool>("floor_optim",true);
 _floor_normal_thresh=private_nh.param<float>("floor_normal_thresh",4);
 _floor_distance_thresh=private_nh.param<float>("floor_distance_thresh",0.002);
 _carId=private_nh.param<string>("carId","master");
 _calMethod=private_nh.param<string>("calMethod","distribution");
 _pos_quantization_bits=private_nh.param<int>("pos_quantization_bits",14);
 init_received_dequeue_lengh=private_nh.param<int>("init_received_dequeue_lengh",30);
 init_local_dequeue_lengh=private_nh.param<int>("init_local_dequeue_lengh",30);
 init_search_radius_num=private_nh.param<int>("init_search_radius_num",30);
 init_fitness_score_thresh=private_nh.param<float>("init_fitness_score_thresh",1);

 registration = select_init_registration_method(private_nh);
 _downloadInitSourceCloud.setLeafSize(0.4,0.4,0.4);
 _downloadInitTargetCloud.setLeafSize(0.4,0.4,0.4);

 _options.reset(new Options(_pos_quantization_bits));
 if (_options->pos_quantization_bits < 0) {
   printf("Error: Position attribute cannot be skipped.\n");
   exit(0);
 }
 const int speed = 10 - _options->compression_level;
 // Setup encoder options.
 if (_options->pos_quantization_bits > 0) {
   _encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,
                                    _options->pos_quantization_bits);
 }
 if (_options->tex_coords_quantization_bits > 0) {
   _encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,
                                    _options->tex_coords_quantization_bits);
 }
 if (_options->generic_quantization_bits > 0) {
   _encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,
                                    _options->generic_quantization_bits);
 }
 _encoder.SetSpeedOptions(speed, speed);

  _downSizeFilterSendCloud.setLeafSize(0.4, 0.4, 0.4);

 _keyframe_update.reset(new KeyframeUpdater(private_nh));
 _floor_detecter.reset(new FloorDetection());
 _floor_detecter->setup(nh,private_nh);

  _info_calculator.reset(new InformationMatrixCalculator(private_nh));
  _loop_detector.reset(new LoopDetectorICP(private_nh,false));
  _child_loop_detector.reset(new LoopDetectorICP(private_nh,false));
  _childmaster_loop_detector.reset(new LoopDetectorICP(private_nh,true));
  _masterchild_loop_detector.reset(new LoopDetectorICP(private_nh,true));

  graph_slam.reset(new GraphSLAM());
  _map_generate.reset(new MapCloudGenerate());
 // _sub_laser_submap_pointcloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/rgb_points_cloud",2));
 // _sub_submap_odom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/rgb_points_cloud_odom",2));
  _sub_laser_submap_pointcloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_cloud_surround",2));
  _sub_submap_odom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/aft_mapped_to_init",2));
  _sub_flat_cloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_flat_cloud",2));
  _sub_flatcorner_cloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_flatcorner_cloud",2));
  _sub_outlier_cloud.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/laser_outlier_cloud",2));


  _time_syn.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry,
                  sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2>
                  (*_sub_laser_submap_pointcloud,*_sub_submap_odom,*_sub_flat_cloud,
                   *_sub_flatcorner_cloud,*_sub_outlier_cloud,2));
  _time_syn->registerCallback(boost::bind(&BackendOptimization::laser_cloud_odom_vecodom_callback, this, _1, _2,_3,_4,_5));

  //_sub_floor_coeffs=nh.subscribe<loam_velodyne::FloorCoeffs>("/laser_cloud_normal",32,&BackendOptimization::laser_floor_coeffs_callback,this);

  _optimization_timer=nh.createTimer(ros::Duration(_graph_update_interval),&BackendOptimization::graph_optimization_timer_callback,this);
  _pub_map_pointcloud=nh.createTimer(ros::Duration(_map_cloud_update_interval),&BackendOptimization::pub_map_pointcloud_timer_callback,this);
  _pub_map_point=nh.advertise<sensor_msgs::PointCloud2>("/map_points",1);
  if(_carId=="child")
    _optimization_timer.stop();

  pubAnathorVisOdom=nh.advertise<visualization_msgs::Marker>("/anathor_odom_marker",2);
  _pubPoseGraph=nh.advertise<std_msgs::ByteMultiArray>("/poseGraphToChild",3);
  _subPoseGraph=nh.subscribe<std_msgs::ByteMultiArray>("/poseGraphFromMaster",3,&BackendOptimization::receivedPoseGraph,this);

  _pubCompressCloud=nh.advertise<std_msgs::ByteMultiArray>("/compressCloudAndDataPose",3);
  _subCompressCloud=nh.subscribe<std_msgs::ByteMultiArray>("/recievedCloudAndDataPose",3,&BackendOptimization::received_cloud_callback,this);
  _map_save_server=nh.advertiseService("/save_map",&BackendOptimization::save_map_cllback,this);
  _map_info_server=nh.advertiseService("/setup_map",&BackendOptimization::mapvis_info_cllback,this);
  childToMasterClient=nh.serviceClient<loam_velodyne::ChildToMaster>("/getchildToMasterTransform");
  _vis_pub=nh.advertise<visualization_msgs::MarkerArray>("/markers",2);
  //_vis_odom_pub=nh.advertise<visualization_msgs::Marker>("/odom_marker",2);
  optim_trans2map_pub=nh.advertise<nav_msgs::Odometry>("/trans2map",2);

  pubVisOdom_debug=nh.advertise<visualization_msgs::Marker>("/debug_odom_marker",2);
  return true;
}
void BackendOptimization::BackendOptimization::process(){}
void BackendOptimization::spin()
{
   ros::AsyncSpinner spinner(3); // Use 3 threads
   spinner.start();
   ros::waitForShutdown();
}
void BackendOptimization::pubedlishChilsTomasterPose()
{
  if(_carId=="child" && initial && !pubedlishChilsTomasterPoseFlag)
  {
    Eigen::Quaterniond quadR(_childToMaster.rotation());
    Eigen::Vector3d translation=_childToMaster.translation();
    loam_velodyne::ChildToMaster srv;
    srv.request.qx=quadR.x();
    srv.request.qy=quadR.y();
    srv.request.qz=quadR.z();
    srv.request.qw=quadR.w();
    srv.request.x=translation(0);
    srv.request.y=translation(1);
    srv.request.z=translation(2);
    if(childToMasterClient.call(srv))
    {
      pubedlishChilsTomasterPoseFlag=true;
    }
  }
}
void BackendOptimization::pubAnathorMachineVisOdom(const Eigen::Isometry3d& pose)
{
  Eigen::Isometry3d updatePose=_receivedTransodom2map*pose;
  Eigen::Vector3d translation=updatePose.translation();
  visualization_msgs::Marker sphere_marker;
  sphere_marker.header.frame_id = "/camera_init";
  sphere_marker.header.stamp = ros::Time::now();
  sphere_marker.ns = "Anathorodometry";
  sphere_marker.id = 0;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.pose.position.x = translation(0);
  sphere_marker.pose.position.y = translation(1);
  sphere_marker.pose.position.z = translation(2);
  sphere_marker.pose.orientation.w = 1.0;
  sphere_marker.scale.x=5;
  sphere_marker.scale.y =5;
  sphere_marker.scale.z =5;
  sphere_marker.color.b = 1.0;
  sphere_marker.color.a = 0.3;
  pubAnathorVisOdom.publish(sphere_marker);
}
//decode received cloud
void BackendOptimization::received_cloud_callback(const std_msgs::ByteMultiArrayConstPtr& cloudAndPosePtr)
{
  receied_cloud_num++;
  std::cout<<"receied_cloud_num"<<receied_cloud_num<<std::endl;
  std::vector<float> decode_float;
  for(int i=0;i<8;i++)
  {
    FloatChar numeric_convert;
    numeric_convert.datac[0]=cloudAndPosePtr->data[4*i+0];
    numeric_convert.datac[1]=cloudAndPosePtr->data[4*i+1];
    numeric_convert.datac[2]=cloudAndPosePtr->data[4*i+2];
    numeric_convert.datac[3]=cloudAndPosePtr->data[4*i+3];
    decode_float.push_back(numeric_convert.dataf);
  }
  Eigen::Quaterniond eig_qua(decode_float[1],decode_float[2],decode_float[3],decode_float[4]);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

  pose.rotate(eig_qua.toRotationMatrix());
  pose.pretranslate(Eigen::Vector3d(decode_float[5],decode_float[6],decode_float[7]));
  pubAnathorMachineVisOdom(pose);
  std::vector<char> tem_convert(cloudAndPosePtr->data.size()-32);
  std::copy(cloudAndPosePtr->data.begin()+32,cloudAndPosePtr->data.end(),tem_convert.begin());
  draco::DecoderBuffer decoder_buffer;
  decoder_buffer.Init(tem_convert.data(), tem_convert.size());

  draco::CycleTimer decoder_timer;
  // Decode the input data into a geometry.
  std::unique_ptr<draco::PointCloud> decoder_pc;
  auto type_statusor = draco::Decoder::GetEncodedGeometryType(&decoder_buffer);
  if (!type_statusor.ok()) {
      printf("Failed to decode the input file %s\n", type_statusor.status().error_msg());
      return;
  }
  const draco::EncodedGeometryType geom_type = type_statusor.value();
  if (geom_type == draco::POINT_CLOUD) {
    // Failed to decode it as mesh, so let's try to decode it as a point cloud.
    decoder_timer.Start();
    draco::Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&decoder_buffer);
    if (!statusor.ok()) {
        printf("Failed to decode the input file %s\n", statusor.status().error_msg());
        return;
    }
    decoder_pc = std::move(statusor).value();
    decoder_timer.Stop();
    std::cout<<decoder_timer.GetInMs()<<"ms to decode"<<std::endl;
  }

  if (decoder_pc == nullptr) {
    printf("Failed to decode the input file.\n");
    return;
  }
   pcl::PointCloud<PointT>::Ptr receivedCloud(new pcl::PointCloud<PointT>());
   dracoCloudToPCLCloud(*decoder_pc.get(),receivedCloud);
   Eigen::Isometry3d delta = _preReceivedPose.inverse() * pose;
   double dx = std::abs(delta.translation().norm());
   _receivedAccumDistance+=dx;
   std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
   key_frame->_pose=pose;
   key_frame->id=(uint64_t)decode_float[0];
   key_frame->_cloud=receivedCloud;
   key_frame->_accumulate_distance=_receivedAccumDistance;
   _preReceivedPose=pose;
   std::lock_guard<std::mutex> lock(_receivedKeyframesQueueMutex);
   _dequeReceivedKeyFrames.push_back(key_frame);

/*  std::vector<float> decode_float;
  for(int i=0;i<8;i++)
  {
    FloatChar numeric_convert;
    numeric_convert.datac[0]=cloudAndPosePtr->data[4*i+0];
    numeric_convert.datac[1]=cloudAndPosePtr->data[4*i+1];
    numeric_convert.datac[2]=cloudAndPosePtr->data[4*i+2];
    numeric_convert.datac[3]=cloudAndPosePtr->data[4*i+3];
    decode_float.push_back(numeric_convert.dataf);
  }
  Eigen::Quaterniond eig_qua(decode_float[1],decode_float[2],decode_float[3],decode_float[4]);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

  pose.rotate(eig_qua.toRotationMatrix());
  pose.pretranslate(Eigen::Vector3d(decode_float[5],decode_float[6],decode_float[7]));
  if((cloudAndPosePtr->data.size()-32)%15 !=0)
  {
    std::cout<<"cloud decode error"<<std::endl;
    return;
  }
  std::cout<<"debug 2"<<std::endl;
  size_t pointNum=(cloudAndPosePtr->data.size()-32)/15;
  pcl::PointCloud<PointT>::Ptr laser_pointcloud_decode(new pcl::PointCloud<PointT>());
  laser_pointcloud_decode->reserve(pointNum);

  for(int i=0;i<pointNum;i++)
  {
    PointT point;
    FloatChar numeric_convert;
    numeric_convert.datac[0]=cloudAndPosePtr->data[i*15+32+0];
    numeric_convert.datac[1]=cloudAndPosePtr->data[i*15+32+1];
    numeric_convert.datac[2]=cloudAndPosePtr->data[i*15+32+2];
    numeric_convert.datac[3]=cloudAndPosePtr->data[i*15+32+3];
    point.x=numeric_convert.dataf;

    numeric_convert.datac[0]=cloudAndPosePtr->data[i*15+32+4];
    numeric_convert.datac[1]=cloudAndPosePtr->data[i*15+32+5];
    numeric_convert.datac[2]=cloudAndPosePtr->data[i*15+32+6];
    numeric_convert.datac[3]=cloudAndPosePtr->data[i*15+32+7];
    point.y=numeric_convert.dataf;

    numeric_convert.datac[0]=cloudAndPosePtr->data[i*15+32+8];
    numeric_convert.datac[1]=cloudAndPosePtr->data[i*15+32+9];
    numeric_convert.datac[2]=cloudAndPosePtr->data[i*15+32+10];
    numeric_convert.datac[3]=cloudAndPosePtr->data[i*15+32+11];
    point.z=numeric_convert.dataf;

    point.r=cloudAndPosePtr->data[i*15+32+12];
    point.g=cloudAndPosePtr->data[i*15+32+13];
    point.b=cloudAndPosePtr->data[i*15+32+14];
    laser_pointcloud_decode->push_back(point);
  }
  Eigen::Isometry3d delta = _preReceivedPose.inverse() * pose;
  double dx = std::abs(delta.translation().norm());
  _receivedAccumDistance+=dx;
  std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
  key_frame->_pose=pose;
  key_frame->id=(uint64_t)decode_float[0];
  key_frame->_cloud=laser_pointcloud_decode;
  key_frame->_accumulate_distance=_receivedAccumDistance;
  _preReceivedPose=pose;
  std::lock_guard<std::mutex> lock(_receivedKeyframesQueueMutex);
  _dequeReceivedKeyFrames.push_back(key_frame);*/
}
void BackendOptimization::receivedPoseGraph(const std_msgs::ByteMultiArrayConstPtr& poseGraph)
{
  FloatChar numeric_convert;
  numeric_convert.datac[0]=poseGraph->data[0];
  numeric_convert.datac[1]=poseGraph->data[1];
  numeric_convert.datac[2]=poseGraph->data[2];
  numeric_convert.datac[3]=poseGraph->data[3];
  int allPoseNum=numeric_convert.dataf;
  if(allPoseNum*32 !=(poseGraph->data.size()-8))
  {
    std::cout<<"decode error"<<std::endl;
    return;
  }
  numeric_convert.datac[0]=poseGraph->data[4];
  numeric_convert.datac[1]=poseGraph->data[5];
  numeric_convert.datac[2]=poseGraph->data[6];
  numeric_convert.datac[3]=poseGraph->data[7];
  int masterPoseNum=numeric_convert.dataf;
  std::vector<float> poseGraphFloat;
  int data_size=poseGraph->data.size()/4;
  for(int i=2;i<data_size;i++)
  {
    FloatChar numeric_convert;
    numeric_convert.datac[0]=poseGraph->data[4*i+0];
    numeric_convert.datac[1]=poseGraph->data[4*i+1];
    numeric_convert.datac[2]=poseGraph->data[4*i+2];
    numeric_convert.datac[3]=poseGraph->data[4*i+3];
    poseGraphFloat.push_back(numeric_convert.dataf);
  }
  std::lock_guard<std::mutex> lock(_poseGraphMutex);
  for(int i=0;i<masterPoseNum;i++)
  {
     double frameId=poseGraphFloat[i*8+0];
     double qw=poseGraphFloat[i*8+1];
     double qx=poseGraphFloat[i*8+2];
     double qy=poseGraphFloat[i*8+3];
     double qz=poseGraphFloat[i*8+4];
     double x=poseGraphFloat[i*8+5];
     double y=poseGraphFloat[i*8+6];
     double z=poseGraphFloat[i*8+7];
     Eigen::Quaterniond eig_qua(qw,qx,qy,qz);
     Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

     pose.rotate(eig_qua.toRotationMatrix());
     pose.pretranslate(Eigen::Vector3d(x,y,z));

     std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
     key_frame->_pose=pose;
     key_frame->id=frameId;
     _childPoseGraphHash[key_frame->id]=key_frame;
  }
  for(int i=masterPoseNum;i<allPoseNum;i++)
  {
    double frameId=poseGraphFloat[i*8+0];
    double qw=poseGraphFloat[i*8+1];
    double qx=poseGraphFloat[i*8+2];
    double qy=poseGraphFloat[i*8+3];
    double qz=poseGraphFloat[i*8+4];
    double x=poseGraphFloat[i*8+5];
    double y=poseGraphFloat[i*8+6];
    double z=poseGraphFloat[i*8+7];
    Eigen::Quaterniond eig_qua(qw,qx,qy,qz);
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

    pose.rotate(eig_qua.toRotationMatrix());
    pose.pretranslate(Eigen::Vector3d(x,y,z));

    std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
    key_frame->_pose=pose;
    key_frame->id=frameId;
    _masterPoseGraphHash[key_frame->id]=key_frame;
  }
    std::cout<<"received pose end"<<std::endl;
}
void BackendOptimization::decodereceivedPosegraph(const std_msgs::ByteMultiArray poseGraph)
{
 // std::cout<<"poseGraphSize"<<poseGraph.data.size()<<std::endl;
    FloatChar numeric_convert;
    numeric_convert.datac[0]=poseGraph.data[0];
    numeric_convert.datac[1]=poseGraph.data[1];
    numeric_convert.datac[2]=poseGraph.data[2];
    numeric_convert.datac[3]=poseGraph.data[3];
    int allPoseNum=numeric_convert.dataf;
    if(allPoseNum*32 !=(poseGraph.data.size()-8))
    {
      std::cout<<"decode error"<<std::endl;
      return;
    }
    numeric_convert.datac[0]=poseGraph.data[4];
    numeric_convert.datac[1]=poseGraph.data[5];
    numeric_convert.datac[2]=poseGraph.data[6];
    numeric_convert.datac[3]=poseGraph.data[7];
    int masterPoseNum=numeric_convert.dataf;
    std::vector<float> poseGraphFloat;
    int data_size=poseGraph.data.size()/4;
    for(int i=2;i<data_size;i++)
    {
      FloatChar numeric_convert;
      numeric_convert.datac[0]=poseGraph.data[4*i+0];
      numeric_convert.datac[1]=poseGraph.data[4*i+1];
      numeric_convert.datac[2]=poseGraph.data[4*i+2];
      numeric_convert.datac[3]=poseGraph.data[4*i+3];
      poseGraphFloat.push_back(numeric_convert.dataf);
    }
    std::lock_guard<std::mutex> lock(_poseGraphMutex);
    std::cout<<"masterPoseSize"<<masterPoseNum<<std::endl;
  //  std::cout<<"poseGraphFloat.size()"<<poseGraphFloat.size()<<std::endl;
    //int poseGraphSize=(poseGraphFloat.size()-1)/8;
    for(int i=0;i<masterPoseNum;i++)
    {
       double frameId=poseGraphFloat[i*8+0];
       double qw=poseGraphFloat[i*8+1];
       double qx=poseGraphFloat[i*8+2];
       double qy=poseGraphFloat[i*8+3];
       double qz=poseGraphFloat[i*8+4];
       double x=poseGraphFloat[i*8+5];
       double y=poseGraphFloat[i*8+6];
       double z=poseGraphFloat[i*8+7];
       Eigen::Quaterniond eig_qua(qw,qx,qy,qz);
       Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

       pose.rotate(eig_qua.toRotationMatrix());
       pose.pretranslate(Eigen::Vector3d(x,y,z));

        std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
        key_frame->_pose=pose;
        key_frame->id=frameId;
        _masterPoseGraphHash[key_frame->id]=key_frame;
        std::cout <<setiosflags(ios::fixed);
        std::cout <<"timestamp:"<< setprecision(10) << frameId<< endl;
       //_masterPosegraph.push_back(key_frame);
    }
    for(int i=masterPoseNum;i<allPoseNum;i++)
    {
      double frameId=poseGraphFloat[i*8+0];
      double qw=poseGraphFloat[i*8+1];
      double qx=poseGraphFloat[i*8+2];
      double qy=poseGraphFloat[i*8+3];
      double qz=poseGraphFloat[i*8+4];
      double x=poseGraphFloat[i*8+5];
      double y=poseGraphFloat[i*8+6];
      double z=poseGraphFloat[i*8+7];
      Eigen::Quaterniond eig_qua(qw,qx,qy,qz);
      Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

      pose.rotate(eig_qua.toRotationMatrix());
      pose.pretranslate(Eigen::Vector3d(x,y,z));

       std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
       key_frame->_pose=pose;
       key_frame->id=frameId;
       _childPoseGraphHash[key_frame->id]=key_frame;
      //_childPosegraph.push_back(key_frame);
    }
    std::cout<<"_masterPoseGraphHash"<<_masterPoseGraphHash.size()<<std::endl;
    int _keyFramesSize=0;
    for(int i=0;i<_keyFrames.size();i++)
    {
      auto found = _masterPoseGraphHash.find(_keyFrames[i]->id);
      if(found == _masterPoseGraphHash.end()) {
        continue;
      }
      _keyFramesSize+=1;
    }
    std::cout<<"received pose end"<<std::endl;
}
vector<string> BackendOptimization::split(const string& str, const string& delim) {
  vector<string> res;
  if("" == str) return res;

  char * strs = new char[str.length() + 1] ;
  strcpy(strs, str.c_str());

  char * d = new char[delim.length() + 1];
  strcpy(d, delim.c_str());

  char *p = strtok(strs, d);
  while(p) {
    string s = p;
    res.push_back(s);
    p = strtok(NULL, d);
  }

  return res;
}

void BackendOptimization::laser_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud){}
void BackendOptimization::laser_odom_callback(const nav_msgs::OdometryConstPtr& odom ){}
void BackendOptimization::laser_submap_vecodom_callback(const loam_velodyne::VectorOdometryConstPtr &vec_odom){}

void BackendOptimization::laser_cloud_odom_vecodom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                       const nav_msgs::OdometryConstPtr& odom,
                                       const sensor_msgs::PointCloud2ConstPtr& flat,
                                       const sensor_msgs::PointCloud2ConstPtr& flatcorner,
                                       const sensor_msgs::PointCloud2ConstPtr& outlier)

{
  laser_cloud_time=odom->header.stamp;
  pcl::PointCloud<PointT>::Ptr laser_pointcloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*cloud, *laser_pointcloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_flat_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*flat,*laser_flat_cloud);

  pcl::PointCloud<PointT>::Ptr laser_flatcorner_cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*flatcorner,*laser_flatcorner_cloud);

  pcl::PointCloud<PointT>::Ptr laser_outlier_cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*outlier,*laser_outlier_cloud);

  geometry_msgs::Quaternion geo_quat=odom->pose.pose.orientation;
  Eigen::Quaterniond eig_qua(geo_quat.w,geo_quat.x,geo_quat.y,geo_quat.z);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

  pose.rotate(eig_qua.toRotationMatrix());
  pose.pretranslate(Eigen::Vector3d(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z));

  if(!_keyframe_update->update(pose,cloud->header.stamp,laser_pointcloud,laser_flat_cloud,laser_flatcorner_cloud,laser_outlier_cloud))
  {
    return;
  }
  if(_carId=="child" && !initial)
  {
    std::shared_ptr<KeyFrame> init_key_frame(new KeyFrame());
    init_key_frame->_pose=_keyframe_update->get_prev_pose();
    init_key_frame->_accumulate_distance=_keyframe_update->get_accum_distance();
    init_key_frame->_stamp=_keyframe_update->get_prev_time();
    init_key_frame->_cloud=_keyframe_update->get_submap_cloud();
    init_key_frame->_flatcorner_cloud=_keyframe_update->get_submap_flatcorner_cloud();
    init_key_frame->_outlier_cloud=_keyframe_update->get_submap_outlier_cloud();
    _keyframe_update ->param_update();
    _initLocalkeyframe.push_back(init_key_frame);
    childNodeInit();
    return;
  }
  pubedlishChilsTomasterPose();
  std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
  key_frame->_pose=_childToMaster*_keyframe_update->get_prev_pose();
  key_frame->_accumulate_distance=_keyframe_update->get_accum_distance();
  key_frame->_stamp=_keyframe_update->get_prev_time();
  key_frame->_cloud=_keyframe_update->get_submap_cloud();
  key_frame->_flatcorner_cloud=_keyframe_update->get_submap_flatcorner_cloud();
  key_frame->_outlier_cloud=_keyframe_update->get_submap_outlier_cloud();
  key_frame->id=_frameId;
  _frameId++;
  pcl::PointCloud<PointT>::Ptr send_pointcloud(new pcl::PointCloud<PointT>());
  _downSizeFilterSendCloud.setInputCloud(key_frame->_cloud);
   _downSizeFilterSendCloud.filter(*send_pointcloud);
   //std::cout<<"send_pointcloud"<<send_pointcloud->size()<<std::endl;
  //key_frame->_cloud=key_frame->_flatcorner_cloud;
  // if(_frameId%1==0)
  // {
     pcl::PointCloud<PointT>::Ptr test_cloud(new pcl::PointCloud<PointT>());
       transportCloudToAnathorMachine( key_frame->_pose,send_pointcloud,key_frame->id,test_cloud);
       //key_frame->_cloud=test_cloud;

       send_cloud_num++;
       std::cout<<"send_cloud_num"<<send_cloud_num<<std::endl;
  // }

  if(_is_floor_optim)
  {
    boost::optional<Eigen::Vector4f> floor_coeffes=_floor_detecter->detect(_keyframe_update->get_submap_flat_cloud());
    key_frame->_floor_coeffes=floor_coeffes;
  }
 _keyframe_update ->param_update();
  std::lock_guard<std::mutex> lock(_keyFrames_queue_mutex);
  _deque_KeyFrames.push_back(key_frame);
}

bool BackendOptimization::childNodeInit()
{
  std::lock_guard<std::mutex> lock(_receivedKeyframesQueueMutex);
  if(_carId=="child" && _dequeReceivedKeyFrames.size()>init_received_dequeue_lengh && !initial && _initLocalkeyframe.size()>init_local_dequeue_lengh)
  {
    if(calChildToMasterPose())
    {
      initial=true;
      _initLocalkeyframe.clear();
      _optimization_timer.start();
      return true;
    }
    return false;
  }
  else if(_carId=="master" || initial)
  {
    return true;
  }
  return false;
}
void BackendOptimization::transportCloudToAnathorMachine(const Eigen::Isometry3d& pose,pcl::PointCloud<PointT>::ConstPtr cloud,uint64_t keyframeId,pcl::PointCloud<PointT>::Ptr& test_cloud)
{
 /*  Eigen::Quaterniond rotationQ(pose.rotation());
   Eigen::Vector3d translation(pose.translation());
   //transport cloud to anathor machine
   std::stringstream compressCloud;
   std::string dataAndPoseStr,dataAndposeLengthStr;
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(keyframeId))+" ";
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(rotationQ.w()))+" ";
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(rotationQ.x()))+" ";
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(rotationQ.y()))+" ";
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(rotationQ.z()))+" ";
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(translation(0)))+" ";
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(translation(1)))+" ";
   dataAndPoseStr+=boost::lexical_cast<std::string>(float(translation(2)));
   dataAndposeLengthStr+=boost::lexical_cast<std::string>(dataAndPoseStr.length());
   dataAndPoseStr=dataAndposeLengthStr+" "+dataAndPoseStr;

 //  ros::Time start_encode=ros::Time::now();
   pointCloudEncoder->encodePointCloud(cloud,compressCloud);
  // std::cout<<"encode using time:"<<(ros::Time::now()-start_encode).toSec()*1000<<"ms"<<std::endl;
   std::string compressString=compressCloud.str();
   dataAndPoseStr=dataAndPoseStr+compressString;

   std_msgs::ByteMultiArray byteMultiArray;
   int dataAndposeLength=dataAndPoseStr.length();
   byteMultiArray.data.resize(dataAndposeLength);
   std::copy(dataAndPoseStr.begin(),dataAndPoseStr.end(),byteMultiArray.data.begin());
    _pubCompressCloud.publish(byteMultiArray);
  // decoder_image(byteMultiArray);*/
/*  std_msgs::ByteMultiArray byteMultiArray;
  byteMultiArray.data.reserve(32+1+15*cloud->points.size());
  Eigen::Quaterniond rotationQ(pose.rotation());
  Eigen::Vector3d translation(pose.translation());
  std::vector<float> poseVec(8);
  poseVec[0]=keyframeId;poseVec[1]=rotationQ.w();
  poseVec[2]=rotationQ.x();poseVec[3]=rotationQ.y();
  poseVec[4]=rotationQ.z();poseVec[5]=translation(0);
  poseVec[6]=translation(1);poseVec[7]=translation(2);
  for(int i=0;i<8;i++)
  {
    FloatChar numeric_convert;
    numeric_convert.dataf=poseVec[i];
    byteMultiArray.data.push_back(numeric_convert.datac[0]);
    byteMultiArray.data.push_back(numeric_convert.datac[1]);
    byteMultiArray.data.push_back(numeric_convert.datac[2]);
    byteMultiArray.data.push_back(numeric_convert.datac[3]);
  }
  for(int i=0;i<cloud->points.size();i++)
  {
    const PointT& convert_point=cloud->points[i];
    FloatChar numeric_convert;
    numeric_convert.dataf=convert_point.x;
    byteMultiArray.data.push_back(numeric_convert.datac[0]);
    byteMultiArray.data.push_back(numeric_convert.datac[1]);
    byteMultiArray.data.push_back(numeric_convert.datac[2]);
    byteMultiArray.data.push_back(numeric_convert.datac[3]);

    numeric_convert.dataf=convert_point.y;
    byteMultiArray.data.push_back(numeric_convert.datac[0]);
    byteMultiArray.data.push_back(numeric_convert.datac[1]);
    byteMultiArray.data.push_back(numeric_convert.datac[2]);
    byteMultiArray.data.push_back(numeric_convert.datac[3]);

    numeric_convert.dataf=convert_point.z;
    byteMultiArray.data.push_back(numeric_convert.datac[0]);
    byteMultiArray.data.push_back(numeric_convert.datac[1]);
    byteMultiArray.data.push_back(numeric_convert.datac[2]);
    byteMultiArray.data.push_back(numeric_convert.datac[3]);

    byteMultiArray.data.push_back(convert_point.r);
    byteMultiArray.data.push_back(convert_point.g);
    byteMultiArray.data.push_back(convert_point.b);
  }
   _pubCompressCloud.publish(byteMultiArray);*/
    std_msgs::ByteMultiArray poseByteMultiArray;
    poseByteMultiArray.data.reserve(33);
    Eigen::Quaterniond rotationQ(pose.rotation());
    Eigen::Vector3d translation(pose.translation());
    std::vector<float> poseVec(8);
    poseVec[0]=keyframeId;poseVec[1]=rotationQ.w();
    poseVec[2]=rotationQ.x();poseVec[3]=rotationQ.y();
    poseVec[4]=rotationQ.z();poseVec[5]=translation(0);
    poseVec[6]=translation(1);poseVec[7]=translation(2);
    for(int i=0;i<8;i++)
    {
      FloatChar numeric_convert;
      numeric_convert.dataf=poseVec[i];
      poseByteMultiArray.data.push_back(numeric_convert.datac[0]);
      poseByteMultiArray.data.push_back(numeric_convert.datac[1]);
      poseByteMultiArray.data.push_back(numeric_convert.datac[2]);
      poseByteMultiArray.data.push_back(numeric_convert.datac[3]);
    }
    std::unique_ptr<draco::PointCloud> pc;
    auto maybe_pc=pclCloudToDracoCloud(cloud);
    pc=std::move(maybe_pc);
    draco::CycleTimer timer;
    // Encode the geometry.
    draco::EncoderBuffer buffer;
    timer.Start();
    const draco::Status status = _encoder.EncodePointCloudToBuffer(*pc.get(), &buffer);
    if (!status.ok()) {
      printf("Failed to encode the point cloud.\n");
      printf("%s\n", status.error_msg());
      return;
    }
    timer.Stop();
    printf("%" PRId64 " ms to encode\n",timer.GetInMs());
    int buffer_size=buffer.size();
    std::cout<<"buffer_size"<<buffer_size<<std::endl;

    std_msgs::ByteMultiArray sendByteMultiArray;
    sendByteMultiArray.data.resize(buffer_size+poseByteMultiArray.data.size());
    std::copy(poseByteMultiArray.data.begin(),poseByteMultiArray.data.end(),sendByteMultiArray.data.begin());
    std::copy(buffer.data(),buffer.data()+buffer.size(),sendByteMultiArray.data.begin()+poseByteMultiArray.data.size());
    _pubCompressCloud.publish(sendByteMultiArray);
    //  std::cout<<"debug 1"<<std::endl;
    decoder_image(sendByteMultiArray,test_cloud);
}
void BackendOptimization::decoder_image(std_msgs::ByteMultiArray received_image,pcl::PointCloud<PointT>::Ptr & testCloud)
{
 /* std::stringstream convert_ss;
//  convert_ss
  //char convert_data[received_image.size()];
  for(int i=0;i<received_image.data.size();i++)
  {
    convert_ss<<received_image.data[i];
     //convert_data[i]=(char)received_image[i];
  }
  std::cout<<"convert_data"<<received_image.data.size()<<std::endl;
  std::string cloudTimeAndPose=convert_ss.str();
  size_t start=cloudTimeAndPose.find(" ");
  std::cout<<"cloudTimeAndPose"<<cloudTimeAndPose.length()<<std::endl;
  std::string timeAndPoseLengthStr=cloudTimeAndPose.substr(0,start);
  std::cout<<"timeAndPoseLengthStr"<<timeAndPoseLengthStr<<std::endl;
  int timeAndposeLength=boost::lexical_cast<int>(timeAndPoseLengthStr);;
  std::string timeAndPoseStr=cloudTimeAndPose.substr(start+1,timeAndposeLength);
  std::string pointCLoudStr=cloudTimeAndPose.substr(start+timeAndposeLength+1);
  std::cout<<"timeAndPoseStr"<<timeAndPoseStr<<std::endl;
  //std::cout<<"pointCLoudStr"<<pointCLoudStr<<std::endl;
  std::cout<<"pointCLoudStrLength"<<pointCLoudStr.length()<<std::endl;
  std::vector<std::string> timeAndPoseSubstr= split(timeAndPoseStr," ");
  if(timeAndPoseSubstr.size() !=8) return;
  std::vector<float> timeAndPose;
  for(int i=0;i<timeAndPoseSubstr.size();i++)
  {
     float temDate=boost::lexical_cast<float>(timeAndPoseSubstr[i]);
     timeAndPose.push_back(temDate);
  }
  Eigen::Quaterniond eig_qua(timeAndPose[1],timeAndPose[2],timeAndPose[3],timeAndPose[4]);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

  pose.rotate(eig_qua.toRotationMatrix());
  pose.pretranslate(Eigen::Vector3d(timeAndPose[5],timeAndPose[6],timeAndPose[7]));

  std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
  key_frame->_pose=pose;
 // std::cout<<"recovery pose"<<pose.matrix()<<std::endl;
  //key_frame->_accumulate_distance=//_keyframe_update->get_accum_distance();
  //std::cout<<"accumulate distance:"<<key_frame->_accumulate_distance<<std::endl;
  std::stringstream decodeStream;
  decodeStream<<pointCLoudStr;
  pcl::PointCloud<PointT>::Ptr laser_pointcloud_decode(new pcl::PointCloud<PointT>());
  pointCloudDecoder->decodePointCloud(decodeStream,laser_pointcloud_decode);
  key_frame->id=(uint64_t)timeAndPose[0];
  key_frame->_cloud=laser_pointcloud_decode;
  _dequeReceivedKeyFrames.push_back(key_frame);
  std::cout<<"received done"<<std::endl;*/
 /* std::vector<float> decode_float;
  for(int i=0;i<8;i++)
  {
    FloatChar numeric_convert;
    numeric_convert.datac[0]=received_image.data[4*i+0];
    numeric_convert.datac[1]=received_image.data[4*i+1];
    numeric_convert.datac[2]=received_image.data[4*i+2];
    numeric_convert.datac[3]=received_image.data[4*i+3];
    decode_float.push_back(numeric_convert.dataf);
  }
  Eigen::Quaterniond eig_qua(decode_float[1],decode_float[2],decode_float[3],decode_float[4]);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

  pose.rotate(eig_qua.toRotationMatrix());
  pose.pretranslate(Eigen::Vector3d(decode_float[5],decode_float[6],decode_float[7]));

  if((received_image.data.size()-32)%15 !=0)
  {
    std::cout<<"cloud decode error"<<std::endl;
    return;
  }
  size_t pointNum=(received_image.data.size()-32)/15;
  pcl::PointCloud<PointT>::Ptr laser_pointcloud_decode(new pcl::PointCloud<PointT>());
  laser_pointcloud_decode->reserve(pointNum);

  for(int i=0;i<pointNum;i++)
  {
    PointT point;
    FloatChar numeric_convert;
    numeric_convert.datac[0]=received_image.data[i*15+32+0];
    numeric_convert.datac[1]=received_image.data[i*15+32+1];
    numeric_convert.datac[2]=received_image.data[i*15+32+2];
    numeric_convert.datac[3]=received_image.data[i*15+32+3];
    point.x=numeric_convert.dataf;

    numeric_convert.datac[0]=received_image.data[i*15+32+4];
    numeric_convert.datac[1]=received_image.data[i*15+32+5];
    numeric_convert.datac[2]=received_image.data[i*15+32+6];
    numeric_convert.datac[3]=received_image.data[i*15+32+7];
    point.y=numeric_convert.dataf;

    numeric_convert.datac[0]=received_image.data[i*15+32+8];
    numeric_convert.datac[1]=received_image.data[i*15+32+9];
    numeric_convert.datac[2]=received_image.data[i*15+32+10];
    numeric_convert.datac[3]=received_image.data[i*15+32+11];
    point.z=numeric_convert.dataf;

    point.r=received_image.data[i*15+32+12];
    point.g=received_image.data[i*15+32+13];
    point.b=received_image.data[i*15+32+14];
    laser_pointcloud_decode->push_back(point);
  }
  testCloud=laser_pointcloud_decode;
  std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
  key_frame->_pose=pose;
  key_frame->id=(uint64_t)decode_float[0];
  key_frame->_cloud=laser_pointcloud_decode;
  _dequeReceivedKeyFrames.push_back(key_frame);*/
  std::vector<float> decode_float;
  for(int i=0;i<8;i++)
  {
    FloatChar numeric_convert;
    numeric_convert.datac[0]=received_image.data[4*i+0];
    numeric_convert.datac[1]=received_image.data[4*i+1];
    numeric_convert.datac[2]=received_image.data[4*i+2];
    numeric_convert.datac[3]=received_image.data[4*i+3];
    decode_float.push_back(numeric_convert.dataf);
  }
  Eigen::Quaterniond eig_qua(decode_float[1],decode_float[2],decode_float[3],decode_float[4]);
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

  pose.rotate(eig_qua.toRotationMatrix());
  pose.pretranslate(Eigen::Vector3d(decode_float[5],decode_float[6],decode_float[7]));

  std::vector<char> tem_convert(received_image.data.size()-32);
  std::copy(received_image.data.begin()+32,received_image.data.end(),tem_convert.begin());
  draco::DecoderBuffer decoder_buffer;
  decoder_buffer.Init(tem_convert.data(), tem_convert.size());

  draco::CycleTimer decoder_timer;
  // Decode the input data into a geometry.
  std::unique_ptr<draco::PointCloud> decoder_pc;
  auto type_statusor = draco::Decoder::GetEncodedGeometryType(&decoder_buffer);
  if (!type_statusor.ok()) {
      printf("Failed to decode the input file %s\n", type_statusor.status().error_msg());
      return;
  }
  const draco::EncodedGeometryType geom_type = type_statusor.value();
  if (geom_type == draco::POINT_CLOUD) {
    // Failed to decode it as mesh, so let's try to decode it as a point cloud.
    decoder_timer.Start();
    draco::Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&decoder_buffer);
    if (!statusor.ok()) {
        printf("Failed to decode the input file %s\n", statusor.status().error_msg());
        return;
    }
    decoder_pc = std::move(statusor).value();
    decoder_timer.Stop();
    std::cout<<decoder_timer.GetInMs()<<"ms to decode"<<std::endl;
  }

  if (decoder_pc == nullptr) {
    printf("Failed to decode the input file.\n");
    return;
  }
  pcl::PointCloud<PointT>::Ptr receivedCloud(new pcl::PointCloud<PointT>());
 dracoCloudToPCLCloud(*decoder_pc.get(),receivedCloud);
 std::cout<<"receivedCloud"<<receivedCloud->size()<<std::endl;
 testCloud=receivedCloud;
/* std::shared_ptr<KeyFrame> key_frame(new KeyFrame());
 key_frame->_pose=pose;
 key_frame->id=(uint64_t)decode_float[0];
 key_frame->_cloud=receivedCloud;
 _dequeReceivedKeyFrames.push_back(key_frame);*/
}
std::unique_ptr<draco::PointCloud> BackendOptimization::pclCloudToDracoCloud(pcl::PointCloud<PointT>::ConstPtr cloud)
{
  std::unique_ptr<draco::PointCloud> pc(new draco::PointCloud());
  const draco::PointIndex::ValueType num_vertices = cloud->size();
  pc->set_num_points(num_vertices);
  {
      const draco::DataType dt =draco::DT_FLOAT32;
      draco::GeometryAttribute va;
      va.Init(draco::GeometryAttribute::POSITION, nullptr, 3, dt, false,
              draco::DataTypeLength(dt) * 3, 0);
      const int att_id = pc->AddAttribute(va, true, num_vertices);
      std::vector<float> position(3);
      for (draco::PointIndex::ValueType i = 0; i < static_cast<uint32_t>(num_vertices);++i) {
        position[0]=cloud->points[i].x;
        position[1]=cloud->points[i].y;
        position[2]=cloud->points[i].z;
        pc->attribute(att_id)->SetAttributeValue(draco::AttributeValueIndex(i), position.data());
      }
  }
 {
      draco::GeometryAttribute va;
      va.Init(draco::GeometryAttribute::COLOR, nullptr, 3, draco::DT_UINT8, true,
              sizeof(uint8_t) * 3, 0);
      const int32_t att_id =pc->AddAttribute(va, true, num_vertices);
      for (draco::PointIndex::ValueType i = 0; i < num_vertices; ++i) {
        std::array<uint8_t, 3> val;
        val[0]=cloud->points[i].r;
        val[1]=cloud->points[i].g;
        val[2]=cloud->points[i].b;
        pc->attribute(att_id)->SetAttributeValue(draco::AttributeValueIndex(i), &val[0]);
      }
  }
  return pc;
}
void BackendOptimization::dracoCloudToPCLCloud(const draco::PointCloud& cloud,pcl::PointCloud<PointT>::Ptr convert_cloud)
{
  pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>());
  const int pos_att_id =cloud.GetNamedAttributeId(draco::GeometryAttribute::POSITION);
  const int color_att_id =cloud.GetNamedAttributeId(draco::GeometryAttribute::COLOR);
  for (draco::PointIndex v(0); v < cloud.num_points(); ++v)
  {
    const auto *const pos_att = cloud.attribute(pos_att_id);
    const float *src_point_data = reinterpret_cast<const float *>(pos_att->GetAddress(pos_att->mapped_index(v)));
    PointT rgbPoint;
    rgbPoint.x=src_point_data[0];
    rgbPoint.y=src_point_data[1];
    rgbPoint.z=src_point_data[2];

    if (color_att_id >= 0)
    {
      const auto *const color_att = cloud.attribute(color_att_id);
      const uint8_t *src_rgbpoint_data = reinterpret_cast<const uint8_t *>(color_att->GetAddress(color_att->mapped_index(v)));
      rgbPoint.r=src_rgbpoint_data[0];
      rgbPoint.g=src_rgbpoint_data[1];
      rgbPoint.b=src_rgbpoint_data[2];
    }
    convert_cloud->push_back(rgbPoint);
  }
 // convert_cloud=pclCloud;
}
bool BackendOptimization::calChildToMasterPose()
{
  bool isValid=false;
  int medium_index=std::floor(_initLocalkeyframe.size()/(double)2);
  int min_index=std::max(medium_index-init_search_radius_num,0);
  int max_index=std::min(medium_index+init_search_radius_num,(int)_initLocalkeyframe.size());

  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for(int i=min_index;i<max_index;i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_source_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*(_initLocalkeyframe[i]->_cloud),*trans_source_cloud);
    pcl::transformPointCloud(*trans_source_cloud,*transform_cloud,_initLocalkeyframe[i]->_pose.cast<float>());
    *source_cloud+=*transform_cloud;
  }
  _downloadInitSourceCloud.setInputCloud(source_cloud);
  _downloadInitSourceCloud.filter(*filter_source_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for(const auto& candidate : _dequeReceivedKeyFrames)//master cloud
  {
     pcl::PointCloud<pcl::PointXYZI>::Ptr trans_src_cloud(new pcl::PointCloud<pcl::PointXYZI>);
     pcl::PointCloud<pcl::PointXYZI>::Ptr transform_src_cloud(new pcl::PointCloud<pcl::PointXYZI>);
     pcl::copyPointCloud(*(candidate->_cloud),*trans_src_cloud);
     pcl::transformPointCloud(*trans_src_cloud,*transform_src_cloud,candidate->_pose.cast<float>());
     *target_cloud+=*transform_src_cloud;
  }
 // pcl::io::savePCDFileASCII("/home/mameng/catkin_ws_loam/target.pcd",*target_cloud);
 // pcl::io::savePCDFileASCII("/home/mameng/catkin_ws_loam/source.pcd",*source_cloud);
  _downloadInitTargetCloud.setInputCloud(target_cloud);
  _downloadInitTargetCloud.filter(*filter_target_cloud);

  registration->setInputSource(filter_source_cloud);
  registration->setInputTarget(filter_target_cloud);
  registration->setMaximumIterations(100);
  registration->setTransformationEpsilon(1e-6);

  std::cout << std::endl;
  std::cout << "init matching" << std::flush;
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
  registration->align(*aligned);
  double score = registration->getFitnessScore();
  std::cout<<"child to master score:"<<score<<std::endl;
  if(!registration->hasConverged() || score > init_fitness_score_thresh)
    isValid=false;
  else
    isValid=true;
  _childToMaster =Eigen::Isometry3d(registration->getFinalTransformation().cast<double>());
  return isValid;
}
bool BackendOptimization::flushReceivedKeyframeQueue()
{
  std::lock_guard<std::mutex> lock(_receivedKeyframesQueueMutex);
  if(_dequeReceivedKeyFrames.empty()) return false;
  //get minimize update frames
  int addFrameNum=std::min<int>(_dequeReceivedKeyFrames.size(),_max_keyframes_per_update);
  for(int i=0;i<addFrameNum;i++)
  {
    std::shared_ptr<KeyFrame> keyframePtr=_dequeReceivedKeyFrames[i];
    //update pose
    Eigen::Isometry3d odom2map= _receivedTransodom2map*keyframePtr->_pose;
    //add se3 vertex to map
    g2o::VertexSE3* poseNode= graph_slam->add_se3_node(odom2map);
    keyframePtr->_node=poseNode;
    if(keyframePtr->_floor_coeffes)
    {
      Eigen::Vector4d floor_coeff((*(keyframePtr->_floor_coeffes))[0],(*(keyframePtr->_floor_coeffes))[1],
                                  (*(keyframePtr->_floor_coeffes))[2],(*(keyframePtr->_floor_coeffes))[3]);

      g2o::VertexPlane* keyframe_plane_nod=select_global_plane_node(odom2map,floor_coeff);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / _floor_edge_stddev);
//      graph_slam->add_se3_plane_edge(keyFrame_ptr->_node,keyframe_plane_nod,floor_coeff,information);
    }
    //add floor detection
    _receivedKeyframeHash[keyframePtr->_stamp]=keyframePtr;
    _newReceivedKeyFrames.push_back(keyframePtr);
    if(i==0 && _receivedKeyFrames.empty())
    {
      continue;
    }
    //calculate information matrix and add se3 edge
    std::shared_ptr<KeyFrame> pre_keyFrame=(i==0?_receivedKeyFrames.back():_dequeReceivedKeyFrames[i-1]);
    Eigen::Isometry3d relative_pose=keyframePtr->_pose.inverse()*pre_keyFrame->_pose;
    Eigen::MatrixXd information= _info_calculator->calc_information_matrix(keyframePtr->_cloud,pre_keyFrame->_cloud,relative_pose);
    graph_slam->add_se3_edge(keyframePtr->_node,pre_keyFrame->_node,relative_pose,information);
  }
  //erase somes frame
  _dequeReceivedKeyFrames.erase(_dequeReceivedKeyFrames.begin(),_dequeReceivedKeyFrames.begin()+addFrameNum);
 return true;
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
    Eigen::Isometry3d odom2map= _trans_odom2map*keyFrame_ptr->_pose;
    //add se3 vertex to map
    g2o::VertexSE3* pose_node= graph_slam->add_se3_node(odom2map);
    keyFrame_ptr->_node=pose_node;
    if(keyFrame_ptr->_floor_coeffes)
    {
      Eigen::Vector4d floor_coeff((*(keyFrame_ptr->_floor_coeffes))[0],(*(keyFrame_ptr->_floor_coeffes))[1],
                                  (*(keyFrame_ptr->_floor_coeffes))[2],(*(keyFrame_ptr->_floor_coeffes))[3]);

      g2o::VertexPlane* keyframe_plane_nod=select_global_plane_node(odom2map,floor_coeff);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / _floor_edge_stddev);
    }
    //add floor detection
    _new_keyFrames.push_back(keyFrame_ptr);
    if(i==0 && _keyFrames.empty())
    {
      continue;
    }
    //calculate information matrix and add se3 edge
    std::shared_ptr<KeyFrame> pre_keyFrame=(i==0?_keyFrames.back():_deque_KeyFrames[i-1]);
    Eigen::Isometry3d relative_pose=keyFrame_ptr->_pose.inverse()*pre_keyFrame->_pose;
    Eigen::MatrixXd information= _info_calculator->calc_information_matrix(keyFrame_ptr->_cloud,pre_keyFrame->_cloud,relative_pose);
    graph_slam->add_se3_edge(keyFrame_ptr->_node,pre_keyFrame->_node,relative_pose,information);
  }
  //erase somes frame
  _deque_KeyFrames.erase(_deque_KeyFrames.begin(),_deque_KeyFrames.begin()+add_frame_num);
 return true;
}
g2o::VertexPlane* BackendOptimization::select_global_plane_node(const Eigen::Isometry3d& pose_now, const Eigen::Vector4d& plane_now)
{
  Eigen::Vector4d floor_word;
  Eigen::Matrix3d R=pose_now.rotation();
  floor_word.head<3>() = R*plane_now.head<3>();
  floor_word(3)=plane_now(3) - pose_now.translation().dot(floor_word.head<3>());
  g2o::VertexPlane* floor_plane_node = graph_slam->add_plane_node(floor_word);
  floor_plane_node->setFixed(true);
  //std::cout<<"floor before:"<<plane_now(0)<<plane_now(1)<<plane_now(2)<<plane_now(3)<<std::endl;
  //std::cout<<"floor aft:"<<floor_word(0)<<floor_word(1)<<floor_word(2)<<floor_word(3)<<std::endl;
  /*if(_global_floor_plane_nodes.size()==0)
  {
    g2o::VertexPlane* floor_plane_node = graph_slam->add_plane_node(floor_word);
    floor_plane_node->setFixed(true);
    _global_floor_plane_nodes.push_back(floor_plane_node);
    _global_floor_plane_pose.push_back(pose_now);
    return floor_plane_node;
  }
  else
  {
    g2o::VertexPlane* floor_plane_node;
    double min_score=std::numeric_limits<double>::max();
    bool is_add_plane=false;
    for(int g_id=0;g_id<_global_floor_plane_nodes.size();g_id++)
    {
      Eigen::Vector4d global_floor_coeffes=_global_floor_plane_nodes[g_id]->estimate()._coeffs;
      Eigen::Isometry3d pose_prev=_global_floor_plane_pose[g_id];
      Eigen::Isometry3d delta = pose_prev.inverse() * pose_now;
      double dx = std::abs(delta.translation().norm());
      double dot = global_floor_coeffes.head<3>().dot(floor_word.head<3>());
      double distance_diff=std::abs(global_floor_coeffes(3)-floor_word(3));
     // std::cout<<"dot"<<dot<<"distence_diff"<<distance_diff<<std::endl;
      if(std::abs(dot) > std::cos(_floor_normal_thresh * M_PI / 180.0) && distance_diff<_floor_distance_thresh &&dx<1)
      {
         is_add_plane=true;
         double all_error=std::abs(dot)+distance_diff;
         if(all_error<min_score)
         {
            all_error=min_score;
            floor_plane_node=_global_floor_plane_nodes[g_id];
         }
      }
    }
    if(is_add_plane==false)
    {
       floor_plane_node = graph_slam->add_plane_node(floor_word);
       floor_plane_node->setFixed(true);
       _global_floor_plane_nodes.push_back(floor_plane_node);
       _global_floor_plane_pose.push_back(pose_now);
    }
    return floor_plane_node;
  }*/
}

/*void BackendOptimization::laser_floor_coeffs_callback(const loam_velodyne::FloorCoeffsConstPtr& floor)
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
*/
void BackendOptimization::graph_optimization_timer_callback(const ros::TimerEvent& event)
{
  if(!flush_keyFrame_queue() & !flushReceivedKeyframeQueue())
  {
    std::cout<<"graph empty"<<std::endl;
    return;
  }
  if(_carId=="master" || _calMethod=="distribution")
  {
    std::transform(_keyFrames.begin(),_keyFrames.end(),_keyFrames.begin(),[](KeyFrame::Ptr key_frame)
    {
      key_frame->_befOptimPose=key_frame->_node->estimate();
      return key_frame;
    });
    ros::Time start_loop=ros::Time::now();
    std::vector<Loop::Ptr> detect_loops=_loop_detector->detect(_keyFrames,_new_keyFrames);
    std::copy(_new_keyFrames.begin(),_new_keyFrames.end(),std::back_inserter(_keyFrames));
    for(int i=0;i<detect_loops.size();i++)
    {
      Loop::Ptr loop=detect_loops[i];
      Eigen::Isometry3d relative_pose(loop->relative_pose);
      Eigen::MatrixXd infomation= _info_calculator->calc_information_matrix(loop->key1->_cloud,loop->key2->_cloud,relative_pose);
      graph_slam->add_se3_edge(loop->key1->_node,loop->key2->_node,relative_pose,infomation);
    }
    std::cout<<"loop closure using time is:"<<(ros::Time::now()-start_loop).toSec()*1000<<"ms"<<std::endl;
    start_loop=ros::Time::now();
    if(_calMethod=="distribution")
    {
      std::cout<<"distribution"<<std::endl;
      std::copy(_newReceivedKeyFrames.begin(),_newReceivedKeyFrames.end(),std::back_inserter(_receivedKeyFrames));
      graph_slam->optimize();
    }
    else
    {
      std::transform(_receivedKeyFrames.begin(),_receivedKeyFrames.end(),_receivedKeyFrames.begin(),[](KeyFrame::Ptr key_frame)
      {
        key_frame->_befOptimPose=key_frame->_node->estimate();
        return key_frame;
      });
      std::vector<Loop::Ptr> receivedDetectLoops=_child_loop_detector->detect(_receivedKeyFrames,_newReceivedKeyFrames);
       for(int i=0;i<receivedDetectLoops.size();i++)
       {
         Loop::Ptr loop=receivedDetectLoops[i];
         Eigen::Isometry3d relative_pose(loop->relative_pose);
         Eigen::MatrixXd infomation= _info_calculator->calc_information_matrix(loop->key1->_cloud,loop->key2->_cloud,relative_pose);
         graph_slam->add_se3_edge(loop->key1->_node,loop->key2->_node,relative_pose,infomation);
       }
       std::cout<<"loop closure using time is:"<<(ros::Time::now()-start_loop).toSec()*1000<<"ms"<<std::endl;
       std::cout<<"_receivedKeyFrames"<<_receivedKeyFrames.size()<<std::endl;
       std::cout<<"_keyFrames:"<<_keyFrames.size()<<" " <<"_newReceivedKeyFrames"<<_newReceivedKeyFrames.size()<<std::endl;
       std::vector<Loop::Ptr> childMaster=_childmaster_loop_detector->detect(_keyFrames,_newReceivedKeyFrames);
       for(int i=0;i<childMaster.size();i++)
       {
         Loop::Ptr loop=childMaster[i];
         Eigen::Isometry3d relative_pose(loop->relative_pose);
         Eigen::MatrixXd infomation= _info_calculator->calc_information_matrix(loop->key1->_cloud,loop->key2->_cloud,relative_pose);
         graph_slam->add_se3_edge(loop->key1->_node,loop->key2->_node,relative_pose,infomation);
       }

       std::vector<Loop::Ptr> masterChild=_masterchild_loop_detector->detect(_receivedKeyFrames,_new_keyFrames);
       for(int i=0;i<masterChild.size();i++)
       {
         Loop::Ptr loop=masterChild[i];
         Eigen::Isometry3d relative_pose(loop->relative_pose);
         Eigen::MatrixXd infomation= _info_calculator->calc_information_matrix(loop->key1->_cloud,loop->key2->_cloud,relative_pose);
         graph_slam->add_se3_edge(loop->key1->_node,loop->key2->_node,relative_pose,infomation);
       }
       std::copy(_newReceivedKeyFrames.begin(),_newReceivedKeyFrames.end(),std::back_inserter(_receivedKeyFrames));
       graph_slam->optimize();
       pubPoseGraphTochild();
    }
  }
  else if(_carId=="child")
  {
      std::copy(_new_keyFrames.begin(),_new_keyFrames.end(),std::back_inserter(_keyFrames));
      std::copy(_newReceivedKeyFrames.begin(),_newReceivedKeyFrames.end(),std::back_inserter(_receivedKeyFrames));

      std::lock_guard<std::mutex> lock(_poseGraphMutex);
      Eigen::Isometry3d odomUpdate=Eigen::Isometry3d::Identity();
      for(int i=0;i<_keyFrames.size();i++)
      {
        auto found = _masterPoseGraphHash.find(_keyFrames[i]->id);
        if(found == _masterPoseGraphHash.end()) {
          _keyFrames[i]->_node->setEstimate(odomUpdate*_keyFrames[i]->_pose);
          continue;
        }
        KeyFrame::Ptr foundKeyframe=found->second;
        odomUpdate=foundKeyframe->_pose*_keyFrames[i]->_pose.inverse();
        _keyFrames[i]->_node->setEstimate(foundKeyframe->_pose);
      }
      Eigen::Isometry3d receivedOdomUpdate=Eigen::Isometry3d::Identity();
      for(int i=0;i<_receivedKeyFrames.size();i++)
      {
        auto found = _childPoseGraphHash.find(_receivedKeyFrames[i]->id);
        if(found == _childPoseGraphHash.end()) {
          _receivedKeyFrames[i]->_node->setEstimate(receivedOdomUpdate*_receivedKeyFrames[i]->_pose);
          continue;
        }
        KeyFrame::Ptr foundKeyframe=found->second;
        receivedOdomUpdate=foundKeyframe->_pose*_receivedKeyFrames[i]->_pose.inverse();
        _receivedKeyFrames[i]->_node->setEstimate(foundKeyframe->_pose);
      }
  }
  _new_keyFrames.clear();
  _newReceivedKeyFrames.clear();
  std::vector<KeyFrameSnapshot::Ptr> snapshot(_keyFrames.size()+_receivedKeyFrames.size());//+_receivedKeyFrames.size()
  std::transform(_keyFrames.begin(),_keyFrames.end(),snapshot.begin(),[](KeyFrame::Ptr key_frame)
  {
    KeyFrameSnapshot::Ptr tem_snapshot(new KeyFrameSnapshot(key_frame));
    return tem_snapshot;
  });
  std::transform(_receivedKeyFrames.begin(),_receivedKeyFrames.end(),snapshot.begin()+_keyFrames.size(),[](KeyFrame::Ptr key_frame)
  {
    KeyFrameSnapshot::Ptr tem_snapshot(new KeyFrameSnapshot(key_frame));
    return tem_snapshot;
  });
  //publish update
  if(!_keyFrames.empty())
  {
    KeyFrame::Ptr last_key_frame= _keyFrames.back();
    _trans_odom2map=last_key_frame->_node->estimate()*last_key_frame->_pose.inverse();

    Eigen::Quaterniond trans2map_quad(_trans_odom2map.rotation());
    nav_msgs::Odometry trans2map;
    trans2map.header.frame_id="/camera_init";
    trans2map.child_frame_id="/camera";
    trans2map.header.stamp=laser_cloud_time;
    trans2map.pose.pose.orientation.x=trans2map_quad.x();
    trans2map.pose.pose.orientation.y=trans2map_quad.y();
    trans2map.pose.pose.orientation.z=trans2map_quad.z();
    trans2map.pose.pose.orientation.w=trans2map_quad.w();
    trans2map.pose.pose.position.x=_trans_odom2map.translation()(0);
    trans2map.pose.pose.position.y=_trans_odom2map.translation()(1);
    trans2map.pose.pose.position.z=_trans_odom2map.translation()(2);
    optim_trans2map_pub.publish(trans2map);
  }
  if(!_receivedKeyFrames.empty())
  {
    KeyFrame::Ptr receivedLastKeyframe= _receivedKeyFrames.back();
    _receivedTransodom2map=receivedLastKeyframe->_node->estimate()*receivedLastKeyframe->_pose.inverse();
  }

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
  pcl::PointCloud<PointT>::Ptr cloud=_map_generate->generate(snapshot,_display_resolution,_display_distance_threash,_is_global_map);
  if(!cloud)
  {
    std::cout<<"cloud is empty"<<std::endl;
    return ;
  }
  std::cout<<"map size"<<cloud->size()<<std::endl;
  std::cout<<"publish map time:"<<(ros::Time::now()-start).toSec()*1000<<"ms"<<std::endl;
  cloud->header.stamp=snapshot.back()->_cloud->header.stamp;
  cloud->header=snapshot.back()->_cloud->header;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud,cloud_msg);
  cloud_msg.header.frame_id="/camera_init";
  cloud_msg.header.stamp=ros::Time::now();
  _pub_map_point.publish(cloud_msg);
}
void BackendOptimization::pubPoseGraphTochild()
{
  //transport pose graph to anathor machine
  std::vector<float> compressPose;
  compressPose.reserve(8*(_keyFrames.size()+_receivedKeyFrames.size())+2);
  compressPose.push_back(-1);
  compressPose.push_back(-1);
  for(int i=0;i<_keyFrames.size();i++)
  {
    Eigen::Isometry3d delta=_keyFrames[i]->_befOptimPose.inverse()*_keyFrames[i]->_node->estimate();
    double dx = std::abs(delta.translation().norm());
    double da = std::abs(std::acos(Eigen::Quaterniond(delta.linear()).w()));
    if(dx >0.001 || da>0.001)
    {
       Eigen::Quaterniond quadR(_keyFrames[i]->_node->estimate().rotation());
       compressPose.push_back(_keyFrames[i]->id);
       compressPose.push_back(quadR.w());
       compressPose.push_back(quadR.x());
       compressPose.push_back(quadR.y());
       compressPose.push_back(quadR.z());
       Eigen::Vector3d translation=_keyFrames[i]->_node->estimate().translation();
       compressPose.push_back(translation(0));
       compressPose.push_back(translation(1));
       compressPose.push_back(translation(2));
    }
  }
  compressPose[1]=(compressPose.size()-2)/8;
  for(int i=0;i<_receivedKeyFrames.size();i++)
  {
    Eigen::Isometry3d delta=_receivedKeyFrames[i]->_befOptimPose.inverse()*_receivedKeyFrames[i]->_node->estimate();
    double dx = std::abs(delta.translation().norm());
    double da = std::abs(std::acos(Eigen::Quaterniond(delta.rotation()).w()));
    if(dx >0.001 || da>0.001)
    {
      Eigen::Quaterniond quadR(_receivedKeyFrames[i]->_node->estimate().rotation());
      compressPose.push_back(_receivedKeyFrames[i]->id);
      compressPose.push_back(quadR.w());
      compressPose.push_back(quadR.x());
      compressPose.push_back(quadR.y());
      compressPose.push_back(quadR.z());
      Eigen::Vector3d translation=_receivedKeyFrames[i]->_node->estimate().translation();
      compressPose.push_back(translation(0));
      compressPose.push_back(translation(1));
      compressPose.push_back(translation(2));
    }
  }
  compressPose[0]=(compressPose.size()-2)/8;
  std_msgs::ByteMultiArray poseGraph;
  poseGraph.data.reserve(compressPose.size()*4);
  for(int i=0;i<compressPose.size();i++)
  {
    FloatChar numeric_convert;
    numeric_convert.dataf=compressPose[i];
    poseGraph.data.push_back(numeric_convert.datac[0]);
    poseGraph.data.push_back(numeric_convert.datac[1]);
    poseGraph.data.push_back(numeric_convert.datac[2]);
    poseGraph.data.push_back(numeric_convert.datac[3]);
  }
  _pubPoseGraph.publish(poseGraph);
 // decodereceivedPosegraph(poseGraph);
}
bool BackendOptimization::save_map_cllback(loam_velodyne::SaveMapRequest& req,loam_velodyne::SaveMapResponse& res)
{
  std::vector<KeyFrameSnapshot::Ptr> snapshot;
  _snapshot_cloud_mutex.lock();
  snapshot=_snapshot_cloud;
  _snapshot_cloud_mutex.unlock();
  pcl::PointCloud<PointT>::Ptr cloud=_map_generate->generate(snapshot,req.resolution,_display_distance_threash,true);
  if(!cloud)
  {
    res.success=false;
    return true;
  }
  size_t position=req.destination.find_last_of("/");
  std::string path_dir(req.destination.substr(0,position));
  boost::filesystem::path save_path(path_dir.c_str());
  if(!boost::filesystem::exists(save_path))
  {
    boost::filesystem::create_directory(save_path);
  }

  bool ret=pcl::io::savePCDFileBinary(req.destination,*cloud);
  res.success=(ret==0);
  return true;
}
bool BackendOptimization::mapvis_info_cllback(loam_velodyne::GlobalMapRequest& req,loam_velodyne::GlobalMapResponse& res)
{
  _display_distance_threash=req.distance;
  _is_global_map=req.isDisGlobalMap;
  _display_resolution=req.resolution;
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
