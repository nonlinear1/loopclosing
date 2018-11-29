#ifndef LOOP_DETECTORPCL_HPP
#define LOOP_DETECTORPCL_HPP

#include <loam_velodyne/keyframe.hpp>
#include <loam_velodyne/registrations.hpp>
#include <pcl/filters/voxel_grid.h>

namespace loam {

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetectorICP {
public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief constructor
   * @param pnh
   */
  LoopDetectorICP(ros::NodeHandle& pnh,bool child_to_master) {
    distance_thresh = pnh.param<double>("distance_thresh", 5.0);
    accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 8.0);
    distance_from_last_edge_thresh = pnh.param<double>("min_edge_interval", 5.0);
    fitness_score_thresh = pnh.param<double>("fitness_score_thresh", 0.5);
    child_master_distance_thresh=pnh.param<double>("child_master_distance_thresh", 1);
    search_radius_num=pnh.param<int>("search_radius_num", 5);
    childmaster_fitness_score_thresh=pnh.param<double>("childmaster_fitness_score_thresh",8);
    std::cout<<distance_thresh<<" "<<accum_distance_thresh<<" "<<distance_from_last_edge_thresh<<" "<<fitness_score_thresh<<" "<<child_master_distance_thresh<<std::endl;
    registration = select_registration_method(pnh);
    last_edge_accum_distance = 0.0;
    child_master_flag=child_to_master;
    _downSizeFilterCloud.setLeafSize(0.4, 0.4, 0.4);
  }

  /**
   * @brief detect loops and add them to the pose graph
   * @param keyframes       keyframes
   * @param new_keyframes   newly registered keyframes
   * @param graph_slam      pose graph
   */
  std::vector<Loop::Ptr> detect(const std::vector<KeyFrame::Ptr>& keyframes, const std::vector<KeyFrame::Ptr>& new_keyframes) {
    std::vector<Loop::Ptr> detected_loops;
    for(const auto& new_keyframe : new_keyframes) {
      int64_t candidate_index = find_candidates(keyframes, new_keyframe);
      //if(child_master_flag)
       //  std::cout<<"candidates size:"<<candidates.size()<<std::endl;
      auto loop = matching(candidate_index,keyframes, new_keyframe);
      if(loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }

  double get_distance_thresh() const {
    return distance_thresh;
  }

private:
  /**
   * @brief find loop candidates. A detected loop begins at one of #keyframes and ends at #new_keyframe
   * @param keyframes      candidate keyframes of loop start
   * @param new_keyframe   loop end keyframe
   * @return loop candidates
   */
  int64_t find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
  /* if(child_master_flag)
   {
       std::cout<<"new_keyframe->_accumulate_distance"<<new_keyframe->_accumulate_distance<<std::endl;
       std::cout<<"last_edge_accum_distance"<<last_edge_accum_distance<<" "<<"distance_from_last_edge_thresh"<<distance_from_last_edge_thresh<<std::endl;
   }*/
   if(new_keyframe->_accumulate_distance - last_edge_accum_distance < distance_from_last_edge_thresh) {
      return -1;
    }

    std::vector<KeyFrame::Ptr> candidates;
    candidates.reserve(32);
    double min_distance=std::numeric_limits<double>::max();
    int64_t condidate_index=-1;
    //for(const auto& k : keyframes) {
    for(int64_t i=0;i<keyframes.size();i++)
    {
      auto& k=keyframes[i];
      // traveled distance between keyframes is too small
      if(!child_master_flag)
      {
        if(new_keyframe->_accumulate_distance - k->_accumulate_distance < accum_distance_thresh) {
          continue;
        }
      }

      const auto& pos1 = k->_node->estimate().translation();
      const auto& pos2 = new_keyframe->_node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist=std::sqrt(std::pow(pos1(0)-pos2(0),2)+std::pow(pos1(2)-pos2(2),2));
      if(child_master_flag)
      {
         // std::cout<<"distance diff:"<<dist<<std::endl;
      }
      if(child_master_flag)
      {
         if(dist > child_master_distance_thresh) {
            continue;
         }
      }
      else
      {
         if(dist > distance_thresh) {
            continue;
         }
      }
      if(min_distance>dist)
      {
        min_distance=dist;
        condidate_index=i;
      }
      //double dist = (pos1.head<2>() - pos2.head<2>()).norm();
     // if(dist > distance_thresh) {
      //  continue;
     // }
      //std::cout<<"accum distance diff:"<<new_keyframe->_accumulate_distance - k->_accumulate_distance<<std::endl;
      //std::cout<<"distance diff:"<<dist<<std::endl;
      //std::cout<<"accum dis1:"<<new_keyframe->_accumulate_distance<<std::endl;
      //std::cout<<"accum dis2:"<<k->_accumulate_distance<<std::endl;
     // candidates.push_back(k);
    }

    return condidate_index;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched well, the loop is added to the pose graph
   * @param candidate_keyframes  candidate keyframes of loop start
   * @param new_keyframe         loop end keyframe
   * @param graph_slam           graph slam
   */
  Loop::Ptr matching(int64_t condidate_index,const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) {
    if(condidate_index==-1) {
         return nullptr;
       }
    ros::Time begin=ros::Time::now();
    pcl::PointCloud<PointT>::Ptr source_cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*(new_keyframe->_cloud),*source_cloud);
    registration->setInputSource(source_cloud);

    int64_t keySize=keyframes.size();
    int64_t min_index=std::max(condidate_index-search_radius_num,(int64_t)0);
    int64_t max_index=std::min(condidate_index+search_radius_num,keySize);
    pcl::PointCloud<PointT>::Ptr target_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for(int64_t i=min_index;i<max_index;i++)
    {
      pcl::PointCloud<PointT>::Ptr trans_target_cloud(new pcl::PointCloud<PointT>);
      pcl::PointCloud<PointT>::Ptr condidate_target_cloud(new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*(keyframes[i]->_cloud),*trans_target_cloud);
      Eigen::Matrix4f guess= (keyframes[condidate_index]->_node->estimate().inverse() * keyframes[i]->_node->estimate()).matrix().cast<float>();
      pcl::transformPointCloud(*trans_target_cloud,*condidate_target_cloud,guess);
      *target_cloud+=*condidate_target_cloud;
    }

    std::cout << std::endl;
    std::cout << "--- loop detection ---" << std::endl;
    std::cout << "matching" << std::flush;
    registration->setMaximumIterations(35);
    registration->setTransformationEpsilon(1e-6);
   /* registration->setMaxCorrespondenceDistance(100);
    registration->setMaximumIterations(100);
    registration->setTransformationEpsilon(1e-6);
    registration->setEuclideanFitnessEpsilon(1e-6);
    registration->setRANSACIterations(0);*/

    pcl::PointCloud<PointT>::Ptr filter_target_cloud(new pcl::PointCloud<PointT>);
    _downSizeFilterCloud.setInputCloud(target_cloud);
    _downSizeFilterCloud.filter(*filter_target_cloud);
    registration->setInputTarget(filter_target_cloud);
    Eigen::Matrix4f guess= (keyframes[condidate_index]->_node->estimate().inverse()*new_keyframe->_node->estimate()).matrix().cast<float>();
    guess(1, 3) = 0.0;
   registration->align(*aligned,guess);
   std::cout<<"loop using time is:"<<(ros::Time::now()-begin).toSec()<<std::endl;
   double score = registration->getFitnessScore();
   if(child_master_flag)
      std::cout<<"child master loop score:"<<score<<std::endl;
   else
     std::cout<<"loop score:"<<score<<std::endl;
   if(child_master_flag)
      fitness_score_thresh=childmaster_fitness_score_thresh;
   if(!registration->hasConverged() || score > fitness_score_thresh) {
     std::cout << "loop not found..." << std::endl;
     return nullptr;
   }

   Eigen::Matrix4d relative_pose = registration->getFinalTransformation().cast<double>();

   std::cout << "loop found!!" << std::endl;

   last_edge_accum_distance = new_keyframe->_accumulate_distance;
   return std::make_shared<Loop>(keyframes[condidate_index], new_keyframe,relative_pose);

    /*   if(candidate_keyframes.empty()) {
      return nullptr;
    }
    pcl::PointCloud<PointT>::Ptr trans_cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*(new_keyframe->_cloud),*trans_cloud);
    registration->setInputTarget(trans_cloud);
    double best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4d relative_pose;

    std::cout << std::endl;
    std::cout << "--- loop detection ---" << std::endl;
    std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    std::cout << "matching" << std::flush;
    auto t1 = ros::Time::now();
    registration->setMaximumIterations(20);
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for(const auto& candidate : candidate_keyframes) {
      ros::Time clo_start=ros::Time::now();
      pcl::PointCloud<PointT>::Ptr trans_src_cloud(new pcl::PointCloud<PointT>);
      //pcl::PointCloud<PointT>::Ptr filter_src_cloud(new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*(candidate->_cloud),*trans_src_cloud);
     // _downSizeFilterCloud.setInputCloud(trans_src_cloud);
      //_downSizeFilterCloud.filter(*filter_src_cloud);
      registration->setInputSource(trans_src_cloud);
      Eigen::Matrix4f guess= (new_keyframe->_node->estimate().inverse() * candidate->_node->estimate()).matrix().cast<float>();
      //guess(2, 3) = 0.0;
      registration->align(*aligned, guess);
      std::cout << "." << std::flush;

      double score = registration->getFitnessScore();
      if(!registration->hasConverged() || score > best_score) {
        continue;
      }
      std::cout<<"score"<<(ros::Time::now()-clo_start).toSec()*1000<<"ms"<<std::endl;
      best_score = score;
      best_matched = candidate;
      relative_pose = registration->getFinalTransformation().cast<double>();
      std::cout<<(ros::Time::now()-clo_start).toSec()*1000<<"ms"<<std::endl;
    }

    auto t2 = ros::Time::now();
    std::cout << " done" << std::endl;
    std::cout << "best_score: " << boost::format("%.3f") % best_score << "    time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

    if(best_score > fitness_score_thresh) {
      std::cout << "loop not found..." << std::endl;
      return nullptr;
    }

    std::cout << "loop found!!" << std::endl;
    std::cout << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - " << Eigen::Quaterniond(relative_pose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;

    last_edge_accum_distance = new_keyframe->_accumulate_distance;
    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);*/
  }

private:
  double distance_thresh;                 // estimated distance between keyframes consisting a loop must be less than this distance
  double accum_distance_thresh;           // traveled distance between ...
  double distance_from_last_edge_thresh;  // a new loop edge must far from the last one at least this distance

  double fitness_score_thresh;            // threshold for scan matching
  double childmaster_fitness_score_thresh;

  double last_edge_accum_distance;

  int search_radius_num;

  double child_master_distance_thresh;
  bool child_master_flag;
  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::VoxelGrid<PointT> _downSizeFilterCloud;
};

}

#endif // LOOP_DETECTOR_HPP
