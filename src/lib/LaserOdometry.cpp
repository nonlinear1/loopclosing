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

#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/common.h"
#include "math_utils.h"

#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/Dense>

#include <time.h>
namespace loam {

using std::sin;
using std::cos;
using std::asin;
using std::atan2;
using std::sqrt;
using std::fabs;
using std::pow;


LaserOdometry::LaserOdometry(const float& scanPeriod,
                             const uint16_t& ioRatio,
                             const size_t& maxIterations)
      : _scanPeriod(scanPeriod),
        _ioRatio(ioRatio),
        _systemInited(false),
        _frameCount(0),
        _maxIterations(maxIterations),
        _deltaTAbort(0.1),
        _deltaRAbort(0.1),
        _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
        _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
        _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
        _surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
        _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
        _coeffSel(new pcl::PointCloud<pcl::PointXYZI>()),
        _laser_flat_cloud(new pcl::PointCloud<pcl::PointXYZI>())

{
  // initialize odometry and odometry tf messages
  _laserOdometryMsg.header.frame_id = "/camera_init";
  _laserOdometryMsg.child_frame_id = "/laser_odom";

  _laserOdometryTrans.frame_id_ = "/camera_init";
  _laserOdometryTrans.child_frame_id_ = "/laser_odom";
}



bool LaserOdometry::setup(ros::NodeHandle &node,
                          ros::NodeHandle &privateNode)
{
  // fetch laser odometry params
  float fParam;
  int iParam;

  if (privateNode.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (privateNode.getParam("ioRatio", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid ioRatio parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      _ioRatio = iParam;
      ROS_INFO("Set ioRatio: %d", iParam);
    }
  }

  if (privateNode.getParam("maxIterations", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      _maxIterations = iParam;
      ROS_INFO("Set maxIterations: %d", iParam);
    }
  }

  if (privateNode.getParam("deltaTAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _deltaTAbort = fParam;
      ROS_INFO("Set deltaTAbort: %g", fParam);
    }
  }

  if (privateNode.getParam("deltaRAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _deltaRAbort = fParam;
      ROS_INFO("Set deltaRAbort: %g", fParam);
    }
  }


  // advertise laser odometry topics
  _pubLaserCloudCornerLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
  _pubLaserCloudSurfLast   = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
  _pubLaserCloudFullRes    = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);
  _pubLaserOdometry        = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
  _pub_laser_flat_cloud=node.advertise<sensor_msgs::PointCloud2>("/laser_flat_cloud",2);

  // subscribe to scan registration topics
  _subCornerPointsSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_sharp", 2, &LaserOdometry::laserCloudSharpHandler, this);

  _subCornerPointsLessSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_sharp", 2, &LaserOdometry::laserCloudLessSharpHandler, this);

  _subSurfPointsFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_flat", 2, &LaserOdometry::laserCloudFlatHandler, this);

  _subSurfPointsLessFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_flat", 2, &LaserOdometry::laserCloudLessFlatHandler, this);

  _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_2", 2, &LaserOdometry::laserCloudFullResHandler, this);

  _subImuTrans = node.subscribe<sensor_msgs::PointCloud2>
      ("/imu_trans", 5, &LaserOdometry::imuTransHandler, this);

  return true;
}
//_transform为T_es,R_es=R_z*R_x*R_y;
//p_s=T_si*p_i=T_is^-1*p_i=[R_is^-1,-R_is^-1*t_is]*p_i
//p_s=R_is^-1*(p_i-t_is)
//R_is^-1=s*R_es^-1=s*R_y^-1*R_x^-1*R_z^-1
//t_is=s*t_es

/*void LaserOdometry::EtransformToStart(const pcl::PointXYZI pi, pcl::PointXYZI po)
{
  float rx = _transform.rot_x.rad();
  float ry = _transform.rot_y.rad();
  float rz = _transform.rot_z.rad();

  float px=_transform.pos.x();
  float py=_transform.pos.y();
  float pz=_transform.pos.z();

  Eigen::Isometry3d T=Eigen::Isometry3d::Identity( );

    Eigen::AngleAxisd angle_x(rx,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd angle_y(ry,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd angle_z(rz,Eigen::Vector3d::UnitZ());


    Eigen::Matrix3d rotation_matrix=angle_z.toRotationMatrix()*angle_x.toRotationMatrix()*angle_y.toRotationMatrix();
   // std::cout<<"rotation_matrix"<<rotation_matrix.eulerAngles(2,0,1)<<std::endl;//z,x,y
    //std::cout<<"zangles"<<angle_x.toRotationMatrix()<<std::endl;
    Eigen::Vector3d x_pos(px,py,pz);
    Eigen::Vector3d x_pi_pos(pi.x,pi.y,pi.z);
    T.rotate(rotation_matrix);
    T.pretranslate(x_pos);

    Eigen::Isometry3d T_inverse=T.inverse();
    Eigen::Vector3d angles=T_inverse.rotation().eulerAngles(1,0,2);//y x z
    Eigen::Vector3d translate=T_inverse.translation();

    float s = (pi.intensity - int(pi.intensity));//将0.1转为1s
    angles=s*angles;
    translate=s*translate;


    angle_x=Eigen::AngleAxisd(angles(1),Eigen::Vector3d::UnitX());
    angle_y=Eigen::AngleAxisd(angles(0),Eigen::Vector3d::UnitY());
    angle_z=Eigen::AngleAxisd(angles(2),Eigen::Vector3d::UnitZ());

    rotation_matrix=angle_y.toRotationMatrix()*angle_x.toRotationMatrix()*angle_z.toRotationMatrix();

    Eigen::Isometry3d T1=Eigen::Isometry3d::Identity( );
    T1.rotate(rotation_matrix);
    T1.pretranslate(translate);
    Eigen::Vector3d po_pos1=T1*x_pi_pos;
    std::cout<<"eigen pos"<<po_pos1<<std::endl;
}
*/
void LaserOdometry::transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
  //LaserOdometry::EtransformToStart(pi,po);
  float s = 10 * (pi.intensity - int(pi.intensity));//将0.1转为1s

  po.x = pi.x - s * _transform.pos.x();
  po.y = pi.y - s * _transform.pos.y();
  po.z = pi.z - s * _transform.pos.z();
  po.intensity = pi.intensity;

  Angle rx = -s * _transform.rot_x.rad();
  Angle ry = -s * _transform.rot_y.rad();
  Angle rz = -s * _transform.rot_z.rad();
  rotateZXY(po, rz, rx, ry);

 // std::cout<<"s:"<<s<<std::endl;
 // std::cout<<"pi pos"<<pi.x<<" "<<pi.y<<" "<<pi.z<<std::endl;
 // std::cout<<"po pos"<<po.x<<" "<<po.y<<" "<<po.z<<std::endl;
//  std::cout<<"transform rot"<<_transform.rot_x.rad()<<" "<<_transform.rot_y.rad()<<" "<<_transform.rot_z.rad()<<std::endl;
 // std::cout<<"transform pos"<<_transform.pos.x()<<" "<<_transform.pos.y()<<" "<<_transform.pos.z()<<std::endl;
}


//先转移到开始,再转移到end
size_t LaserOdometry::transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  size_t cloudSize = cloud->points.size();

  for (size_t i = 0; i < cloudSize; i++) {
    pcl::PointXYZI& point = cloud->points[i];

    float s = 10 * (point.intensity - int(point.intensity));

    point.x -= s * _transform.pos.x();
    point.y -= s * _transform.pos.y();
    point.z -= s * _transform.pos.z();
    point.intensity = int(point.intensity);

    Angle rx = -s * _transform.rot_x.rad();
    Angle ry = -s * _transform.rot_y.rad();
    Angle rz = -s * _transform.rot_z.rad();
    rotateZXY(point, rz, rx, ry);
    rotateYXZ(point, _transform.rot_y, _transform.rot_x, _transform.rot_z);

    point.x += _transform.pos.x() - _imuShiftFromStart.x();
    point.y += _transform.pos.y() - _imuShiftFromStart.y();
    point.z += _transform.pos.z() - _imuShiftFromStart.z();

    rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart);
    rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd);
  }

  return cloudSize;
}



void LaserOdometry::pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                                      const Angle& blx, const Angle& bly, const Angle& blz,
                                      const Angle& alx, const Angle& aly, const Angle& alz,
                                      Angle &acx, Angle &acy, Angle &acz)
{
  float sbcx = bcx.sin();
  float cbcx = bcx.cos();
  float sbcy = bcy.sin();
  float cbcy = bcy.cos();
  float sbcz = bcz.sin();
  float cbcz = bcz.cos();

  float sblx = blx.sin();
  float cblx = blx.cos();
  float sbly = bly.sin();
  float cbly = bly.cos();
  float sblz = blz.sin();
  float cblz = blz.cos();

  float salx = alx.sin();
  float calx = alx.cos();
  float saly = aly.sin();
  float caly = aly.cos();
  float salz = alz.sin();
  float calz = alz.cos();

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly)
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                         - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                         - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  acx = -asin(srx);

  float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                                               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                 - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                                                 - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                 + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                                               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                 - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                                                 - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                 + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
                 - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
                              + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
                              - calx*cblx*cblz*salz)
                 + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                              + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                              + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
                 + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                              + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                              + calx*calz*cblx*cblz)
                 - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly - cbly*sblx*sblz)
                              + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
                              - calx*calz*cblx*sblz);
  acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());
}



void LaserOdometry::accumulateRotation(Angle cx, Angle cy, Angle cz,
                                       Angle lx, Angle ly, Angle lz,
                                       Angle &ox, Angle &oy, Angle &oz)
{
  float srx = lx.cos()*cx.cos()*ly.sin()*cz.sin()
            - cx.cos()*cz.cos()*lx.sin()
            - lx.cos()*ly.cos()*cx.sin();
  ox = -asin(srx);

  float srycrx = lx.sin()*(cy.cos()*cz.sin() - cz.cos()*cx.sin()*cy.sin())
               + lx.cos()*ly.sin()*(cy.cos()*cz.cos() + cx.sin()*cy.sin()*cz.sin())
               + lx.cos()*ly.cos()*cx.cos()*cy.sin();
  float crycrx = lx.cos()*ly.cos()*cx.cos()*cy.cos()
               - lx.cos()*ly.sin()*(cz.cos()*cy.sin() - cy.cos()*cx.sin()*cz.sin())
               - lx.sin()*(cy.sin()*cz.sin() + cy.cos()*cz.cos()*cx.sin());
  oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

  float srzcrx = cx.sin()*(lz.cos()*ly.sin() - ly.cos()*lx.sin()*lz.sin())
               + cx.cos()*cz.sin()*(ly.cos()*lz.cos() + lx.sin()*ly.sin()*lz.sin())
               + lx.cos()*cx.cos()*cz.cos()*lz.sin();
  float crzcrx = lx.cos()*lz.cos()*cx.cos()*cz.cos()
               - cx.cos()*cz.sin()*(ly.cos()*lz.sin() - lz.cos()*lx.sin()*ly.sin())
               - cx.sin()*(ly.sin()*lz.sin() + ly.cos()*lz.cos()*lx.sin());
  oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}



void LaserOdometry::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg)
{
  _timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp;

  _cornerPointsSharp->clear();
  pcl::fromROSMsg(*cornerPointsSharpMsg, *_cornerPointsSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);
  _newCornerPointsSharp = true;
}



void LaserOdometry::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg)
{
  _timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp;

  _cornerPointsLessSharp->clear();
  pcl::fromROSMsg(*cornerPointsLessSharpMsg, *_cornerPointsLessSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_cornerPointsLessSharp, *_cornerPointsLessSharp, indices);
  _newCornerPointsLessSharp = true;
}



void LaserOdometry::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg)
{
  _timeSurfPointsFlat = surfPointsFlatMsg->header.stamp;

  _surfPointsFlat->clear();
  pcl::fromROSMsg(*surfPointsFlatMsg, *_surfPointsFlat);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_surfPointsFlat, *_surfPointsFlat, indices);
  _newSurfPointsFlat = true;
}



void LaserOdometry::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg)
{
  _timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp;

  _surfPointsLessFlat->clear();
  pcl::fromROSMsg(*surfPointsLessFlatMsg, *_surfPointsLessFlat);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_surfPointsLessFlat, *_surfPointsLessFlat, indices);
  _newSurfPointsLessFlat = true;
}



void LaserOdometry::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
  _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;

  _laserCloud->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laserCloud, *_laserCloud, indices);
  _newLaserCloudFullRes = true;
}



void LaserOdometry::imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg)
{
  _timeImuTrans = imuTransMsg->header.stamp;

  pcl::PointCloud<pcl::PointXYZ> imuTrans;
  pcl::fromROSMsg(*imuTransMsg, imuTrans);

  _imuPitchStart = imuTrans.points[0].x;
  _imuYawStart = imuTrans.points[0].y;
  _imuRollStart = imuTrans.points[0].z;

  _imuPitchEnd = imuTrans.points[1].x;
  _imuYawEnd = imuTrans.points[1].y;
  _imuRollEnd = imuTrans.points[1].z;

  _imuShiftFromStart = imuTrans.points[2];
  _imuVeloFromStart = imuTrans.points[3];

  _newImuTrans = true;
}



void LaserOdometry::spin()
{
  ros::Rate rate(1000);
  bool status = ros::ok();

  // loop until shutdown
  while (status) {
    ros::spinOnce();

    // try processing new data
    process();
    //std::cout<<"laser odome spin"<<std::endl;
    status = ros::ok();
    rate.sleep();
  }
}



void LaserOdometry::reset()
{
  _newCornerPointsSharp = false;
  _newCornerPointsLessSharp = false;
  _newSurfPointsFlat = false;
  _newSurfPointsLessFlat = false;
  _newLaserCloudFullRes = false;
  _newImuTrans = false;
}



bool LaserOdometry::hasNewData()
{
  return _newCornerPointsSharp && _newCornerPointsLessSharp && _newSurfPointsFlat &&
         _newSurfPointsLessFlat && _newLaserCloudFullRes && _newImuTrans &&
         fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat).toSec()) < 0.005 &&
         fabs((_timeImuTrans - _timeSurfPointsLessFlat).toSec()) < 0.005;
}



void LaserOdometry::process()
{
  if (!hasNewData()) {
    // waiting for new data to arrive...
    return;
  }
   clock_t begin = clock();
  // reset flags, etc.
  reset();

  if (!_systemInited) {
    _cornerPointsLessSharp.swap(_lastCornerCloud);
    _surfPointsLessFlat.swap(_lastSurfaceCloud);

    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);

    _transformSum.rot_x += _imuPitchStart;
    _transformSum.rot_z += _imuRollStart;

    _systemInited = true;
    return;
  }

  pcl::PointXYZI coeff;
  bool isDegenerate = false;
  Eigen::Matrix<float,6,6> matP;

  _frameCount++;
  _transform.pos -= _imuVeloFromStart * _scanPeriod;


  size_t lastCornerCloudSize = _lastCornerCloud->points.size();
  size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();


  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    std::vector<int> pointSearchInd(1);
    std::vector<float> pointSearchSqDis(1);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);
    size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();
    size_t surfPointsFlatNum = _surfPointsFlat->points.size();

    _pointSearchCornerInd1.resize(cornerPointsSharpNum);
    _pointSearchCornerInd2.resize(cornerPointsSharpNum);
    _pointSearchSurfInd1.resize(surfPointsFlatNum);
    _pointSearchSurfInd2.resize(surfPointsFlatNum);
    _pointSearchSurfInd3.resize(surfPointsFlatNum);

    for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++) {
      pcl::PointXYZI pointSel, pointProj, tripod1, tripod2, tripod3;
      _laserCloudOri->clear();
      _coeffSel->clear();

      for (int i = 0; i < cornerPointsSharpNum; i++) {
        transformToStart(_cornerPointsSharp->points[i], pointSel);

        if (iterCount % 5 == 0) {
          pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices);
          _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

          int closestPointInd = -1, minPointInd2 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];
            int closestPointScan = int(_lastCornerCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25;
            for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan + 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
          }

          _pointSearchCornerInd1[i] = closestPointInd;//对一个corner点,离他最近的点
          _pointSearchCornerInd2[i] = minPointInd2;//第二个点
        }

        if (_pointSearchCornerInd2[i] >= 0) {
          tripod1 = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
          tripod2 = _lastCornerCloud->points[_pointSearchCornerInd2[i]];

          float x0 = pointSel.x;
          float y0 = pointSel.y;
          float z0 = pointSel.z;
          float x1 = tripod1.x;
          float y1 = tripod1.y;
          float z1 = tripod1.z;
          float x2 = tripod2.x;
          float y2 = tripod2.y;
          float z2 = tripod2.z;

          float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                              * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                              * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));//论文公式(2)分母

          float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));//论文公式(2)分子
          //ld2对x0,y0,z0的导数
          float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                      + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

          float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                       - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

          float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                       + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

          float ld2 = a012 / l12;

          // TODO: Why writing to a variable that's never read?
          pointProj = pointSel;
          pointProj.x -= la * ld2;
          pointProj.y -= lb * ld2;
          pointProj.z -= lc * ld2;

          float s = 1;//损失越小,s越大,可以看做平衡平面点和corner
          if (iterCount >= 5) {
            s = 1 - 1.8f * fabs(ld2);
          }

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          if (s > 0.1 && ld2 != 0) {
            _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }

      for (int i = 0; i < surfPointsFlatNum; i++) {
        transformToStart(_surfPointsFlat->points[i], pointSel);

        if (iterCount % 5 == 0) {
          _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];
            int closestPointScan = int(_lastSurfaceCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
            for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
              if (int(_lastSurfaceCloud->points[j].intensity) > closestPointScan + 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) <= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastSurfaceCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) >= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
          }

          _pointSearchSurfInd1[i] = closestPointInd;
          _pointSearchSurfInd2[i] = minPointInd2;
          _pointSearchSurfInd3[i] = minPointInd3;
        }

        if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0) {
          tripod1 = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
          tripod2 = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
          tripod3 = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

          float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                     - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
          float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                     - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
          float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                     - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
          float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

          float ps = sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

          // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
          pointProj = pointSel;
          pointProj.x -= pa * pd2;
          pointProj.y -= pb * pd2;
          pointProj.z -= pc * pd2;

          float s = 1;
          if (iterCount >= 5) {
            s = 1 - 1.8f * fabs(pd2) / sqrt(calcPointDistance(pointSel));
          }

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          if (s > 0.1 && pd2 != 0) {
            _laserCloudOri->push_back(_surfPointsFlat->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }

      int pointSelNum = _laserCloudOri->points.size();
      if (pointSelNum < 10) {
        continue;
      }

      Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
      Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
      Eigen::Matrix<float,6,6> matAtA;
      Eigen::VectorXf matB(pointSelNum);
      Eigen::Matrix<float,6,1> matAtB;
      Eigen::Matrix<float,6,1> matX;

      for (int i = 0; i < pointSelNum; i++) {
        const pcl::PointXYZI& pointOri = _laserCloudOri->points[i];
        coeff = _coeffSel->points[i];

        float s = 1;

        float srx = sin(s * _transform.rot_x.rad());
        float crx = cos(s * _transform.rot_x.rad());
        float sry = sin(s * _transform.rot_y.rad());
        float cry = cos(s * _transform.rot_y.rad());
        float srz = sin(s * _transform.rot_z.rad());
        float crz = cos(s * _transform.rot_z.rad());
        float tx = s * _transform.pos.x();
        float ty = s * _transform.pos.y();
        float tz = s * _transform.pos.z();

        float arx = (-s*crx*sry*srz*pointOri.x + s*crx*crz*sry*pointOri.y + s*srx*sry*pointOri.z
                     + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
                    + (s*srx*srz*pointOri.x - s*crz*srx*pointOri.y + s*crx*pointOri.z
                       + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
                    + (s*crx*cry*srz*pointOri.x - s*crx*cry*crz*pointOri.y - s*cry*srx*pointOri.z
                       + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

        float ary = ((-s*crz*sry - s*cry*srx*srz)*pointOri.x
                     + (s*cry*crz*srx - s*sry*srz)*pointOri.y - s*crx*cry*pointOri.z
                     + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx)
                     + s*tz*crx*cry) * coeff.x
                    + ((s*cry*crz - s*srx*sry*srz)*pointOri.x
                       + (s*cry*srz + s*crz*srx*sry)*pointOri.y - s*crx*sry*pointOri.z
                       + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry)
                       - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

        float arz = ((-s*cry*srz - s*crz*srx*sry)*pointOri.x + (s*cry*crz - s*srx*sry*srz)*pointOri.y
                     + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
                    + (-s*crx*crz*pointOri.x - s*crx*srz*pointOri.y
                       + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
                    + ((s*cry*crz*srx - s*sry*srz)*pointOri.x + (s*crz*sry + s*cry*srx*srz)*pointOri.y
                       + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

        float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y
                    - s*(crz*sry + cry*srx*srz) * coeff.z;

        float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y
                    - s*(sry*srz - cry*crz*srx) * coeff.z;

        float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;

        float d2 = coeff.intensity;

        matA(i, 0) = arx;
        matA(i, 1) = ary;
        matA(i, 2) = arz;
        matA(i, 3) = atx;
        matA(i, 4) = aty;
        matA(i, 5) = atz;//雅可比矩阵
        matB(i, 0) = -0.05 * d2;//LM优化中b
      }
      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;

      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      if (iterCount == 0) {
        Eigen::Matrix<float,1,6> matE;
        Eigen::Matrix<float,6,6> matV;
        Eigen::Matrix<float,6,6> matV2;

        Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
        matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();

        matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = {10, 10, 10, 10, 10, 10};
        for (int i = 0; i < 6; i++) {
          if (matE(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
              matV2(i, j) = 0;
            }
            isDegenerate = true;
          } else {
            break;
          }
        }
        matP = matV.inverse() * matV2;
      }

      if (isDegenerate) {
        Eigen::Matrix<float,6,1> matX2;
        matX2 = matX;
        matX = matP * matX2;
      }

      _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
      _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
      _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
      _transform.pos.x() += matX(3, 0);
      _transform.pos.y() += matX(4, 0);
      _transform.pos.z() += matX(5, 0);//更新位姿

      if( !pcl_isfinite(_transform.rot_x.rad()) ) _transform.rot_x = Angle();
      if( !pcl_isfinite(_transform.rot_y.rad()) ) _transform.rot_y = Angle();
      if( !pcl_isfinite(_transform.rot_z.rad()) ) _transform.rot_z = Angle();

      if( !pcl_isfinite(_transform.pos.x()) ) _transform.pos.x() = 0.0;
      if( !pcl_isfinite(_transform.pos.y()) ) _transform.pos.y() = 0.0;
      if( !pcl_isfinite(_transform.pos.z()) ) _transform.pos.z() = 0.0;

      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort) {//迭代退出阈值
        break;
      }
    }
  }//优化完成

/*———————————— 旋转角的累计变化量(校正？) ————————————*/
// AccumulateRotation作用将局部旋转坐标转换至全局旋转坐标，
//转换方法为先计算R＝(Rcy*Rcx*Rcz)(Rly*Rlx*Rlz),然后对R中的元素解三角函数，什么事局部坐标系，什么是全局坐标系
//_transform为T_es
  Angle rx, ry, rz;
  //得到T_we=T_ws*T_se=T_ws*T_es^-1,其中_transformSum=T_ws,_transform=T_es,
  //T_es=[]
  //EtestWE();
  accumulateRotation(_transformSum.rot_x,
                     _transformSum.rot_y,
                     _transformSum.rot_z,
                     -_transform.rot_x,
                     -_transform.rot_y.rad() * 1.05,
                     -_transform.rot_z,
                     rx, ry, rz);
/*———————————— 接着转移到世界坐标系下 ————————————*/
  Vector3 v( _transform.pos.x()        - _imuShiftFromStart.x(),
             _transform.pos.y()        - _imuShiftFromStart.y(),
             _transform.pos.z() * 1.05 - _imuShiftFromStart.z() );
  rotateZXY(v, rz, rx, ry);
  Vector3 trans = _transformSum.pos - v;

  pluginIMURotation(rx, ry, rz,
                    _imuPitchStart, _imuYawStart, _imuRollStart,
                    _imuPitchEnd, _imuYawEnd, _imuRollEnd,
                    rx, ry, rz);
  /*Eigen::AngleAxisd ws_angle_x(rx.rad(),Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ws_angle_y(ry.rad(),Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd ws_angle_z(rz.rad(),Eigen::Vector3d::UnitZ());


  Eigen::Matrix3d ws_rotation_matrix=ws_angle_y.toRotationMatrix()*
  ws_angle_x.toRotationMatrix()*ws_angle_z.toRotationMatrix();*/
  //std::cout<<"ws_rotation_matrix"<<ws_rotation_matrix<<std::endl;
  //std::cout<<"--------------------------------------"<<std::endl;

  _transformSum.rot_x = rx;
  _transformSum.rot_y = ry;
  _transformSum.rot_z = rz;
  _transformSum.pos = trans;

  //std::cout<<_transformSum.rot_y.rad()<<" "<<_transformSum.rot_x.rad()<<" "<<_transformSum.rot_z.rad()<<std::endl;
  //std::cout<<_transformSum.pos.x()<<" "<<_transformSum.pos.y()<<" "<<_transformSum.pos.z()<<std::endl;


  transformToEnd(_cornerPointsLessSharp);
  transformToEnd(_surfPointsLessFlat);
  transformToEnd(_surfPointsFlat);//_laser_flat_cloud
  _cornerPointsLessSharp.swap(_lastCornerCloud);
  _surfPointsLessFlat.swap(_lastSurfaceCloud);

  lastCornerCloudSize = _lastCornerCloud->points.size();
  lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
  }
  clock_t end = clock();
  //std::cout<<"matching Running time: "<<(double)(end-begin)/CLOCKS_PER_SEC*1000<<"ms"<<std::endl;
  publishResult();
}

/*void LaserOdometry::EtestWE()
{
  //计算T_sw
  double ws_rx=_transformSum.rot_x.rad();
  double ws_ry=_transformSum.rot_y.rad();
  double ws_rz=_transformSum.rot_z.rad();

  double ws_px=_transformSum.pos.x();
  double ws_py=_transformSum.pos.y();
  double ws_pz=_transformSum.pos.z();

  Eigen::AngleAxisd ws_angle_x(ws_rx,Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ws_angle_y(ws_ry,Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd ws_angle_z(ws_rz,Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d T_sw=Eigen::Isometry3d::Identity();

  Eigen::Matrix3d ws_rotation_matrix=ws_angle_y.toRotationMatrix()*
  ws_angle_x.toRotationMatrix()*ws_angle_z.toRotationMatrix();
  T_sw.rotate(ws_rotation_matrix);
  T_sw.pretranslate(Eigen::Vector3d(ws_px,ws_py,ws_pz));

  //计算T_es
  float es_rx = -_transform.rot_x.rad();
  float es_ry = -_transform.rot_y.rad()*1.05;
  float es_rz = -_transform.rot_z.rad();

  float es_px=_transform.pos.x();
  float es_py=_transform.pos.y();
  float es_pz=_transform.pos.z()*1.05;

  Eigen::Isometry3d T_es=Eigen::Isometry3d::Identity( );

  Eigen::AngleAxisd es_angle_x(es_rx,Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd es_angle_y(es_ry,Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd es_angle_z(es_rz,Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d es_rotation_matrix=es_angle_y.toRotationMatrix()*
  es_angle_x.toRotationMatrix()*es_angle_z.toRotationMatrix();

  T_es.rotate(es_rotation_matrix);
  T_es.pretranslate(Eigen::Vector3d(es_px,es_py,es_pz));

//  Eigen::Matrix3d he_rotate=ws_rotation_matrix*es_rotation_matrix;
 // Eigen::Vector3d rot_angles=he_rotate.eulerAngles(1,0,2);
 // Eigen::Isometry3d T_we=T_sw*T_es.inverse();//T_we

 // Eigen::Vector3d angles=T_we.rotation().eulerAngles(1,0,2);
//  Eigen::Vector3d translate=T_we.translation();

  //std::cout<<"angles"<<he_rotate<<std::endl;
  //std::cout<<"translate"<<translate<<std::endl;
}
*/
/*void quaternionAngle(Twist transformSum)
{
    Eigen::AngleAxisd ws_angle_x(transformSum.rot_x.rad(),Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ws_angle_y(transformSum.rot_y.rad(),Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd ws_angle_z(transformSum.rot_z.rad(),Eigen::Vector3d::UnitZ());


    Eigen::Matrix3d ws_rotation_matrix=ws_angle_y.toRotationMatrix()*
    ws_angle_x.toRotationMatrix()*ws_angle_z.toRotationMatrix();
    Eigen::Quaterniond qut(ws_rotation_matrix);
    std::cout<<"ws_quaternion"<<qut.coeffs()<<std::endl;
}
*/
void LaserOdometry::publishResult()
{
  //std::cout<<"laser odome publish"<<std::endl;
  // publish odometry tranformations
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(_transformSum.rot_z.rad(),
                                                                              -_transformSum.rot_x.rad(),
                                                                              -_transformSum.rot_y.rad());
   //quaternionAngle(_transformSum);
  _laserOdometryMsg.header.stamp = _timeSurfPointsLessFlat;
  _laserOdometryMsg.pose.pose.orientation.x = -geoQuat.y;
  _laserOdometryMsg.pose.pose.orientation.y = -geoQuat.z;
  _laserOdometryMsg.pose.pose.orientation.z = geoQuat.x;
  _laserOdometryMsg.pose.pose.orientation.w = geoQuat.w;
  _laserOdometryMsg.pose.pose.position.x = _transformSum.pos.x();
  _laserOdometryMsg.pose.pose.position.y = _transformSum.pos.y();
  _laserOdometryMsg.pose.pose.position.z = _transformSum.pos.z();
  _pubLaserOdometry.publish(_laserOdometryMsg);
   /*std::cout<<"odometry_q:"<<"x"
            <<_laserOdometryMsg.pose.pose.orientation.x<<"y"
            <<_laserOdometryMsg.pose.pose.orientation.y<<"z"
            <<_laserOdometryMsg.pose.pose.orientation.z<<"w"
            <<_laserOdometryMsg.pose.pose.orientation.w<<std::endl;*/
  _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat;
  _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _laserOdometryTrans.setOrigin(tf::Vector3( _transformSum.pos.x(), _transformSum.pos.y(), _transformSum.pos.z()) );
  _tfBroadcaster.sendTransform(_laserOdometryTrans);//相对于世界坐标系的位姿//T_wc


  // publish cloud results according to the input output ratio
  if (_ioRatio < 2 || _frameCount % _ioRatio == 1) {
    ros::Time sweepTime = _timeSurfPointsLessFlat;
    transformToEnd(_laserCloud);  // transform full resolution cloud to sweep end before sending it

    publishCloudMsg(_pubLaserCloudCornerLast, *_lastCornerCloud, sweepTime, "/camera");
    publishCloudMsg(_pubLaserCloudSurfLast, *_lastSurfaceCloud, sweepTime, "/camera");
    publishCloudMsg(_pubLaserCloudFullRes, *_laserCloud, sweepTime, "/camera");

    publishCloudMsg(_pub_laser_flat_cloud,*_surfPointsFlat,sweepTime,"/camera");
  }
}

} // end namespace loam
