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

#include "loam_velodyne/utility.h"
#include<Eigen/Geometry>
#include<visualization_msgs/Marker.h>
class TransformFusion{

private:

    ros::NodeHandle nh;

    ros::Publisher pubLaserOdometry2;
    ros::Publisher pubVisOdom;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subOdomAftMapped;
    ros::Subscriber subLaserOdometry_inter;
  

    nav_msgs::Odometry laserOdometry2;
    tf::StampedTransform laserOdometryTrans2;
    tf::TransformBroadcaster tfBroadcaster2;

    tf::StampedTransform map_2_camera_init_Trans;
    tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

    tf::StampedTransform camera_2_base_link_Trans;
    tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

    float transformSum[6];
    float transformIncre[6];
    float transformMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];

    Eigen::Isometry3d loopOptimtrans2map;
    Eigen::Isometry3d befLooptransformSum;
    Eigen::Isometry3d befLooptransformBefMapped;
    Eigen::Isometry3d beftransformAftMapped;

    std_msgs::Header currentHeader;

public:

    TransformFusion(){

        pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &TransformFusion::laserOdometryHandler, this);
        subLaserOdometry_inter = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_inter", 5, &TransformFusion::odomAftMappedHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/trans2map", 5, &TransformFusion::loopOptimHandler, this);
        pubVisOdom=nh.advertise<visualization_msgs::Marker>("/odom_marker",2);
        laserOdometry2.header.frame_id = "/camera_init";
        laserOdometry2.child_frame_id = "/camera";

        laserOdometryTrans2.frame_id_ = "/camera_init";
        laserOdometryTrans2.child_frame_id_ = "/camera";

        map_2_camera_init_Trans.frame_id_ = "/map";
        map_2_camera_init_Trans.child_frame_id_ = "/camera_init";

        camera_2_base_link_Trans.frame_id_ = "/camera";
        camera_2_base_link_Trans.child_frame_id_ = "/base_link";

        for (int i = 0; i < 6; ++i)
        {
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }
        loopOptimtrans2map=Eigen::Isometry3d::Identity();
        befLooptransformSum=Eigen::Isometry3d::Identity();
        befLooptransformBefMapped=Eigen::Isometry3d::Identity();
        beftransformAftMapped=Eigen::Isometry3d::Identity();
    }

    void transformAssociateToMap()
    {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                                   crycrx / cos(transformMapped[0]));
        
        float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                                   crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3] 
                           - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5] 
                           - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
    {
        currentHeader = laserOdometry->header;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;

        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;

        befLooptransformSum=Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd angle_x(transformSum[0],Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd angle_y(transformSum[1],Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd angle_z(transformSum[2],Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotation_matrix=angle_y.toRotationMatrix()*angle_x.toRotationMatrix()*angle_z.toRotationMatrix();
        befLooptransformSum.rotate(rotation_matrix);
        befLooptransformSum.pretranslate(Eigen::Vector3d(transformSum[3],transformSum[4],transformSum[5]));
       /* transformAssociateToMap();

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                  (transformMapped[2], -transformMapped[0], -transformMapped[1]);*/

        Eigen::Isometry3d aftloopOptim=loopOptimtrans2map*beftransformAftMapped*befLooptransformBefMapped.inverse()*befLooptransformSum;
        Eigen::Quaterniond aftloopOptimQuad(aftloopOptim.rotation());
        Eigen::Vector3d translate=aftloopOptim.translation();

        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x =aftloopOptimQuad.x();// -geoQuat.y;
        laserOdometry2.pose.pose.orientation.y =aftloopOptimQuad.y();// -geoQuat.z;
        laserOdometry2.pose.pose.orientation.z =aftloopOptimQuad.z();// geoQuat.x;
        laserOdometry2.pose.pose.orientation.w =aftloopOptimQuad.w();// geoQuat.w;
        laserOdometry2.pose.pose.position.x =translate(0);// transformMapped[3];
        laserOdometry2.pose.pose.position.y =translate(1);// transformMapped[4];
        laserOdometry2.pose.pose.position.z =translate(2);// transformMapped[5];
        pubLaserOdometry2.publish(laserOdometry2);

        laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        //laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        //laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        laserOdometryTrans2.setRotation(tf::Quaternion(aftloopOptimQuad.x(), aftloopOptimQuad.y(), aftloopOptimQuad.z(), aftloopOptimQuad.w()));
        laserOdometryTrans2.setOrigin(tf::Vector3(translate(0), translate(1), translate(2)));
        tfBroadcaster2.sendTransform(laserOdometryTrans2);

        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = "/camera_init";
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.ns = "odometry";
        sphere_marker.id = 0;
        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.pose.position.x = translate(0);
        sphere_marker.pose.position.y = translate(1);
        sphere_marker.pose.position.z = translate(2);
        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.scale.x=5;
        sphere_marker.scale.y =5;
        sphere_marker.scale.z =5;
        sphere_marker.color.r = 1.0;
        sphere_marker.color.a = 0.3;
        pubVisOdom.publish(sphere_marker);
    }

    void loopOptimHandler(const nav_msgs::OdometryConstPtr& odom)
    {
      geometry_msgs::Quaternion geo_quat=odom->pose.pose.orientation;
      Eigen::Quaterniond eig_qua(geo_quat.w,geo_quat.x,geo_quat.y,geo_quat.z);
      Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();

      pose.rotate(eig_qua.toRotationMatrix());
      pose.pretranslate(Eigen::Vector3d(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z));
      loopOptimtrans2map=pose;
    }
    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
    {
       /* double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformAftMapped[0] = -pitch;
        transformAftMapped[1] = -yaw;
        transformAftMapped[2] = roll;

        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

        transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
        transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
        transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

        transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
        transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
        transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;*/

        geometry_msgs::Quaternion geo_quat=odomAftMapped->pose.pose.orientation;
        Eigen::Quaterniond eig_qua(geo_quat.w,geo_quat.x,geo_quat.y,geo_quat.z);
        beftransformAftMapped=Eigen::Isometry3d::Identity();

        beftransformAftMapped.rotate(eig_qua.toRotationMatrix());
        beftransformAftMapped.pretranslate(Eigen::Vector3d(odomAftMapped->pose.pose.position.x,
                                                           odomAftMapped->pose.pose.position.y,
                                                           odomAftMapped->pose.pose.position.z));

        befLooptransformBefMapped=Eigen::Isometry3d::Identity( );

        Eigen::AngleAxisd angle_x(odomAftMapped->twist.twist.angular.x,Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd angle_y(odomAftMapped->twist.twist.angular.y,Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd angle_z(odomAftMapped->twist.twist.angular.z,Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d rotation_matrix=angle_y.toRotationMatrix()*angle_x.toRotationMatrix()*angle_z.toRotationMatrix();
        befLooptransformBefMapped.rotate(rotation_matrix);
        Eigen::Vector3d translation(odomAftMapped->twist.twist.linear.x
                                    ,odomAftMapped->twist.twist.linear.y
                                    ,odomAftMapped->twist.twist.linear.z);
        befLooptransformBefMapped.pretranslate(translation);
    }
};




int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    
    TransformFusion TFusion;

    ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

    ros::spin();

    return 0;
}