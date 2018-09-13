// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam_velodyne/ScanRegistration.h"
#include "math_utils.h"

#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>
#include<time.h>

namespace loam {


RegistrationParams::RegistrationParams(const float& scanPeriod_,
                                       const int& imuHistorySize_,
                                       const int& nFeatureRegions_,
                                       const int& curvatureRegion_,
                                       const int& maxCornerSharp_,
                                       const int& maxSurfaceFlat_,
                                       const float& lessFlatFilterSize_,
                                       const float& surfaceCurvatureThreshold_)
    : scanPeriod(scanPeriod_),
      imuHistorySize(imuHistorySize_),
      nFeatureRegions(nFeatureRegions_),
      curvatureRegion(curvatureRegion_),
      maxCornerSharp(maxCornerSharp_),
      maxCornerLessSharp(10 * maxCornerSharp_),
      maxSurfaceFlat(maxSurfaceFlat_),
      lessFlatFilterSize(lessFlatFilterSize_),
      surfaceCurvatureThreshold(surfaceCurvatureThreshold_)
{

};



bool RegistrationParams::parseParams(const ros::NodeHandle& nh) {
  bool success = true;
  int iParam = 0;
  float fParam = 0;

  if (nh.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      success = false;
    } else {
      scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (nh.getParam("imuHistorySize", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid imuHistorySize parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      imuHistorySize = iParam;
      ROS_INFO("Set imuHistorySize: %d", iParam);
    }
  }

  if (nh.getParam("featureRegions", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid featureRegions parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      nFeatureRegions = iParam;
      ROS_INFO("Set nFeatureRegions: %d", iParam);
    }
  }

  if (nh.getParam("curvatureRegion", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid curvatureRegion parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      curvatureRegion = iParam;
      ROS_INFO("Set curvatureRegion: +/- %d", iParam);
    }
  }

  if (nh.getParam("maxCornerSharp", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxCornerSharp parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      maxCornerSharp = iParam;
      maxCornerLessSharp = 10 * iParam;
      ROS_INFO("Set maxCornerSharp / less sharp: %d / %d", iParam, maxCornerLessSharp);
    }
  }

  if (nh.getParam("maxCornerLessSharp", iParam)) {
    if (iParam < maxCornerSharp) {
      ROS_ERROR("Invalid maxCornerLessSharp parameter: %d (expected >= %d)", iParam, maxCornerSharp);
      success = false;
    } else {
      maxCornerLessSharp = iParam;
      ROS_INFO("Set maxCornerLessSharp: %d", iParam);
    }
  }

  if (nh.getParam("maxSurfaceFlat", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxSurfaceFlat parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
      maxSurfaceFlat = iParam;
      ROS_INFO("Set maxSurfaceFlat: %d", iParam);
    }
  }

  if (nh.getParam("surfaceCurvatureThreshold", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid surfaceCurvatureThreshold parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
      surfaceCurvatureThreshold = fParam;
      ROS_INFO("Set surfaceCurvatureThreshold: %g", fParam);
    }
  }

  if (nh.getParam("lessFlatFilterSize", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid lessFlatFilterSize parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
      lessFlatFilterSize = fParam;
      ROS_INFO("Set lessFlatFilterSize: %g", fParam);
    }
  }

  return success;
}






ScanRegistration::ScanRegistration(const RegistrationParams& config)
      : _config(config),
        _sweepStart(),
        _scanTime(),
        _imuStart(),
        _imuCur(),
        _imuIdx(0),
        _imuHistory(_config.imuHistorySize),
        _laserCloud(),
        _cornerPointsSharp(),
        _cornerPointsLessSharp(),
        _surfacePointsFlat(),
        _surfacePointsLessFlat(),
        _imuTrans(4, 1),
        _regionCurvature(),
        _regionLabel(),
        _regionSortIndices(),
        _scanNeighborPicked()
{

}



bool ScanRegistration::setup(ros::NodeHandle& node,
                             ros::NodeHandle& privateNode)
{
  if (!_config.parseParams(privateNode)) {
    return false;
  }
  _imuHistory.ensureCapacity(_config.imuHistorySize);

  // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);


  // advertise scan registration topics
  _pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  return true;
}


/*
函数功能,累计imu数据,时间,角度,加速度,速度,位置
*/
void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;//z,x,y;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);

  IMUState newState;
  newState.stamp = imuIn->header.stamp;
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);

    const IMUState& prevState = _imuHistory.last();
    float timeDiff = float((newState.stamp - prevState.stamp).toSec());
    newState.position = prevState.position
                        + (prevState.velocity * timeDiff)
                        + (0.5 * acc * timeDiff * timeDiff);
    newState.velocity = prevState.velocity
                        + acc * timeDiff;
  }

  _imuHistory.push(newState);
}


/*
复位系统得到初始imu值,并清除所有数据
*/
void ScanRegistration::reset(const ros::Time& scanTime,
                             const bool& newSweep)
{
  _scanTime = scanTime;

  // re-initialize IMU start index and state
  _imuIdx = 0;
  if (hasIMUData()) {//如果有imu,得到初始时刻数据
    interpolateIMUStateFor(0, _imuStart);
  }

  // clear internal cloud buffers at the beginning of a sweep
  if (newSweep) {
    _sweepStart = scanTime;

    // clear cloud buffers
    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();

    // clear scan indices vector
    _scanIndices.clear();
  }
}



void ScanRegistration::setIMUTransformFor(const float& relTime)
{
  interpolateIMUStateFor(relTime, _imuCur);

  float relSweepTime = (_scanTime - _sweepStart).toSec() + relTime;//_scanTime始终和_sweepStart相同
  _imuPositionShift = _imuCur.position - _imuStart.position - _imuStart.velocity * relSweepTime;//不太理解,为何要减去速度
}



void ScanRegistration::transformToStartIMU(pcl::PointXYZI& point)
{
  // rotate point to global IMU system
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);//将roll转换为旋转矩阵,必须按照先roll,再pitch,再yaw的顺序,至于绕的轴可按照自己的坐标系

  // add global IMU position shift
  point.x += _imuPositionShift.x();
  point.y += _imuPositionShift.y();
  point.z += _imuPositionShift.z();

  // rotate point back to local IMU system relative to the start IMU state
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);//负值表示逆阵
}

/*
函数功能:
_scanTime为当前点云帧的timeStamp,_imuHistory大小为200,
reset时,relTime为0,如果_scanTime<_imuHistory[0].stamp,那么输出状态为_imuHistory[0]
如果_scanTime>_imuHistory[199].stamp,那么输出状态为_imuHistory[199],不然就插值
*/

void ScanRegistration::interpolateIMUStateFor(const float &relTime,
                                              IMUState &outputState)
{
  double timeDiff = (_scanTime - _imuHistory[_imuIdx].stamp).toSec() + relTime;//_sacnTime是点云中第一个点的扫描时间
  while (_imuIdx < _imuHistory.size() - 1 && timeDiff > 0) {
    _imuIdx++;
    timeDiff = (_scanTime - _imuHistory[_imuIdx].stamp).toSec() + relTime;
  }

  if (_imuIdx == 0 || timeDiff > 0) {
    outputState = _imuHistory[_imuIdx];
  } else {
    float ratio = -timeDiff / (_imuHistory[_imuIdx].stamp - _imuHistory[_imuIdx - 1].stamp).toSec();
    //outputState=end*(ratio)+start*(1-ratio)
    IMUState::interpolate(_imuHistory[_imuIdx], _imuHistory[_imuIdx - 1], ratio, outputState);
  }
}

/*
函数功能:提取点云中的特征点,corner点存到_cornerPointsSharp中,less corner点存到_cornerPointsLessSharp中
flat点存到_surfacePointsFlat中,less flat 点下采样后存到_surfacePointsLessFlat中.
*/

void ScanRegistration::extractFeatures(const uint16_t& beginIdx)
{
  // extract features from individual scans
  size_t nScans = _scanIndices.size();
  for (size_t i = beginIdx; i < nScans; i++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
    size_t scanStartIdx = _scanIndices[i].first;
    size_t scanEndIdx = _scanIndices[i].second;
    
    // skip empty scans
    if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion) {
      continue;
    }

    // Quick&Dirty fix for relative point time calculation without IMU data
    /*float scanSize = scanEndIdx - scanStartIdx + 1;
    for (int j = scanStartIdx; j <= scanEndIdx; j++) {
      _laserCloud[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize;
    }*/
    clock_t start_clock=clock();
    // reset scan buffers
    setScanBuffersFor(scanStartIdx, scanEndIdx);
    //std::cout<<"setScanBuffersFor:"<<(double)(clock()-start_clock)/CLOCKS_PER_SEC*1000<<std::endl;
    // extract features from equally sized scan regions
    start_clock=clock();
    for (int j = 0; j < _config.nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - j)
                   + (scanEndIdx - _config.curvatureRegion) * j) / _config.nFeatureRegions;
      size_t ep = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - 1 - j)
                   + (scanEndIdx - _config.curvatureRegion) * (j + 1)) / _config.nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;
      clock_t s1=clock();
      // reset region buffers
      setRegionBuffersFor(sp, ep);
      //std::cout<<"setFor0:"<<(double)(clock()-s1)/CLOCKS_PER_SEC*1000<<std::endl;
      s1=clock();
      // extract corner features
      int largestPickedNum = 0;
      for (size_t k = regionSize; k > 0 && largestPickedNum < _config.maxCornerLessSharp;) {
        size_t idx = _regionSortIndices[--k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] > _config.surfaceCurvatureThreshold) {

          largestPickedNum++;
          if (largestPickedNum <= _config.maxCornerSharp) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            _cornerPointsSharp.push_back(_laserCloud[idx]);
          } else {
            _regionLabel[regionIdx] = CORNER_LESS_SHARP;
          }
          _cornerPointsLessSharp.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx);
        }
      }

      // extract flat surface features
      int smallestPickedNum = 0;
      for (int k = 0; k < regionSize && smallestPickedNum < _config.maxSurfaceFlat; k++) {
        size_t idx = _regionSortIndices[k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold) {

          smallestPickedNum++;
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx);
        }
      }

      // extract less flat surface features
      for (int k = 0; k < regionSize; k++) {
        if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
          surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
        }
      }
      //std::cout<<"setFor:"<<(double)(clock()-s1)/CLOCKS_PER_SEC*1000<<std::endl;
    }
    //std::cout<<"setRegionBuffersFor:"<<(double)(clock()-start_clock)/CLOCKS_PER_SEC*1000<<std::endl;
    // down size less flat surface point cloud of current scan
    start_clock=clock();
    pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;//less fat点太多进行下采样
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(_config.lessFlatFilterSize, _config.lessFlatFilterSize, _config.lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    _surfacePointsLessFlat += surfPointsLessFlatScanDS;
    //std::cout<<"down:"<<(double)(clock()-start_clock)/CLOCKS_PER_SEC*1000<<std::endl;
  }
  //std::cout<<"maxflatnum:"<<_config.maxSurfaceFlat<<std::endl;
   // std::cout<<"FLAT:"<<_surfacePointsFlat.size()<<"less flat:"<<_surfacePointsLessFlat.size()<<std::endl;
}

/*函数功能:计算startIdx,endIdx中间点的曲率存储在_regionCurvature中,并按照从小到大的顺序把索引存在_regionSortIndices中
*/
void ScanRegistration::setRegionBuffersFor(const size_t& startIdx,
                                           const size_t& endIdx)
{
  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize);
  _regionSortIndices.resize(regionSize);
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT);

  // calculate point curvatures and reset sort indices
  float pointWeight = -2 * _config.curvatureRegion;

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;

    for (int j = 1; j <= _config.curvatureRegion; j++) {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    _regionSortIndices[regionIdx] = i;
  }
  std::sort(_regionSortIndices.begin(),_regionSortIndices.end(),
  [this,startIdx](size_t i1,size_t i2){return this->_regionCurvature[i1-startIdx]<this->_regionCurvature[i2-startIdx];});
  // sort point curvatures
  /*for (size_t i = 1; i < regionSize; i++) {
    for (size_t j = i; j >= 1; j--) {
      if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
      }
    }
  }*/
  /*std::cout<<"_regionSortIndices:";
  for(int dr=0;dr<5;dr++)
  {
    std::cout<<_regionCurvature[_regionSortIndices[dr]-startIdx]<<" ";
  }
  std::cout<<std::endl;*/
}



void ScanRegistration::setScanBuffersFor(const size_t& startIdx,
                                         const size_t& endIdx)
{
  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;
  _scanNeighborPicked.assign(scanSize, 0);

  // mark unreliable points as picked
  for (size_t i = startIdx + _config.curvatureRegion; i < endIdx - _config.curvatureRegion; i++) {
    const pcl::PointXYZI& previousPoint = (_laserCloud[i - 1]);
    const pcl::PointXYZI& point = (_laserCloud[i]);
    const pcl::PointXYZI& nextPoint = (_laserCloud[i + 1]);

    float diffNext = calcSquaredDiff(nextPoint, point);

    if (diffNext > 0.1) {
      float depth1 = calcPointDistance(point);
      float depth2 = calcPointDistance(nextPoint);

      if (depth1 > depth2) {//选择近点并且距离比小
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx - _config.curvatureRegion], _config.curvatureRegion + 1, 1);

          continue;
        }
      } else {
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx + 1], _config.curvatureRegion + 1, 1);
        }
      }
    }

    float diffPrevious = calcSquaredDiff(point, previousPoint);
    float dis = calcSquaredPointDistance(point);

    if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {//去掉与扫描线平行的平面点
      _scanNeighborPicked[i - startIdx] = 1;
    }
  }
}



void ScanRegistration::markAsPicked(const size_t& cloudIdx,
                                    const size_t& scanIdx)
{
  _scanNeighborPicked[scanIdx] = 1;

  for (int i = 1; i <= _config.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx + i] = 1;
  }

  for (int i = 1; i <= _config.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx - i] = 1;
  }
}



void ScanRegistration::publishResult()
{
  // publish full resolution and feature point clouds
  _laserCloud.is_dense=false;
  _cornerPointsSharp.is_dense=false;
  _cornerPointsLessSharp.is_dense=false;
  _surfacePointsFlat.is_dense=false;
  _surfacePointsLessFlat.is_dense=false;
  publishCloudMsg(_pubLaserCloud,            _laserCloud,            _sweepStart, "/camera");
  publishCloudMsg(_pubCornerPointsSharp,     _cornerPointsSharp,     _sweepStart, "/camera");
  publishCloudMsg(_pubCornerPointsLessSharp, _cornerPointsLessSharp, _sweepStart, "/camera");
  publishCloudMsg(_pubSurfPointsFlat,        _surfacePointsFlat,     _sweepStart, "/camera");
  publishCloudMsg(_pubSurfPointsLessFlat,    _surfacePointsLessFlat, _sweepStart, "/camera");


  // publish corresponding IMU transformation information
  _imuTrans[0].x = _imuStart.pitch.rad();
  _imuTrans[0].y = _imuStart.yaw.rad();
  _imuTrans[0].z = _imuStart.roll.rad();

  _imuTrans[1].x = _imuCur.pitch.rad();
  _imuTrans[1].y = _imuCur.yaw.rad();
  _imuTrans[1].z = _imuCur.roll.rad();

  //[P_s,1]^T=[R_rs^-1*R_rc,R_rs^-1 *(t_rc-t_rs );0,1]*[P_c,1]
  Vector3 imuShiftFromStart = _imuPositionShift;//R_rs^-1 *(t_rc-t_rs )
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[2].x = imuShiftFromStart.x();
  _imuTrans[2].y = imuShiftFromStart.y();
  _imuTrans[2].z = imuShiftFromStart.z();

  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity;
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[3].x = imuVelocityFromStart.x();
  _imuTrans[3].y = imuVelocityFromStart.y();
  _imuTrans[3].z = imuVelocityFromStart.z();

  publishCloudMsg(_pubImuTrans, _imuTrans, _sweepStart, "/camera");
}

} // end namespace loam
