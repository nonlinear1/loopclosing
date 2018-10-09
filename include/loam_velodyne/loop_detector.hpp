#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <loam_velodyne/keyframe.hpp>
#include <loam_velodyne/registrations.hpp>


namespace loam {

struct Loop {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  Loop(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Matrix4d& relpose)
    : key1(key1),
      key2(key2),
      relative_pose(relpose)
  {}

public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Matrix4d relative_pose;
};

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief constructor
   * @param pnh
   */
  LoopDetector(ros::NodeHandle& pnh) {
    distance_thresh = pnh.param<double>("distance_thresh", 5.0);
    accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 8.0);
    distance_from_last_edge_thresh = pnh.param<double>("min_edge_interval", 5.0);

    fitness_score_thresh = pnh.param<double>("fitness_score_thresh", 0.5);

    registration = select_registration_method(pnh);
    last_edge_accum_distance = 0.0;
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
      auto candidates = find_candidates(keyframes, new_keyframe);
      auto loop = matching(candidates, new_keyframe);
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
  std::vector<KeyFrame::Ptr> find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
   if(new_keyframe->_accumulate_distance - last_edge_accum_distance < distance_from_last_edge_thresh) {
      return std::vector<KeyFrame::Ptr>();
    }

    std::vector<KeyFrame::Ptr> candidates;
    candidates.reserve(32);

    for(const auto& k : keyframes) {
      // traveled distance between keyframes is too small
      if(new_keyframe->_accumulate_distance - k->_accumulate_distance < accum_distance_thresh) {
        continue;
      }

      const auto& pos1 = k->_node->estimate().translation();
      const auto& pos2 = new_keyframe->_node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();
      if(dist > distance_thresh) {
        continue;
      }

      candidates.push_back(k);
    }

    return candidates;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched well, the loop is added to the pose graph
   * @param candidate_keyframes  candidate keyframes of loop start
   * @param new_keyframe         loop end keyframe
   * @param graph_slam           graph slam
   */
  Loop::Ptr matching(const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe) {
  /*  if(candidate_keyframes.empty()) {
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
    registration->setMaximumIterations(10);
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for(const auto& candidate : candidate_keyframes) {
      ros::Time clo_start=ros::Time::now();
      pcl::PointCloud<PointT>::Ptr trans_src_cloud(new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*(candidate->_cloud),*trans_src_cloud);
      registration->setInputSource(trans_src_cloud);
      Eigen::Matrix4f guess= (new_keyframe->_node->estimate().inverse() * candidate->_node->estimate()).matrix().cast<float>();
      guess(2, 3) = 0.0;
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

  double last_edge_accum_distance;

  pcl::Registration<PointT, PointT>::Ptr registration;
};

}

#endif // LOOP_DETECTOR_HPP
