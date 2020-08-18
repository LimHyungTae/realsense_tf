#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <iostream>
#include <fstream>

#include "opencv2/opencv.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"


namespace rgtlib
{
// Eigen::Matrix4f <-> cv::Mat
Eigen::Matrix4f 	cvToEigen(cv::Mat mat);
cv::Mat 		eigenToCv(Eigen::Matrix4f mat);
// 6 Degree of Freedom <-> cv::Mat
void 		cvToDoF6(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw);
cv::Mat 		doF6ToCv(float x, float y, float z, float roll, float pitch, float yaw);
// 6 Degree of Freedom <-> Eigen 4x4 Transformation matrix
Eigen::VectorXf 	eigenToDoF6(Eigen::Matrix4f mat);
Eigen::Matrix4f 	doF6ToEigen(float x, float y, float z, float roll, float pitch, float yaw);
// geometry_msgs::Pose <-> Eigen 4x4 Transformation matrix
geometry_msgs::Pose eigenToPose(Eigen::Matrix4f pose);
Eigen::Matrix4f 	poseToEigen(geometry_msgs::Pose geoPose);

Eigen::MatrixXf pointToEigen(geometry_msgs::Point geoPoint);

// sensor_msgs::PointCloud2 <-> pcl::PointCloud
template<typename T>
sensor_msgs::PointCloud2 pclToMsg(pcl::PointCloud<T> cloud)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  return msg;
}

template<typename T>
pcl::PointCloud<T> msgToPcl(sensor_msgs::PointCloud2 msg)
{
  pcl::PointCloud<T> res;
  pcl::fromROSMsg(msg,res);
  return res;
}

template<typename T>
void msgToPCPtr(sensor_msgs::PointCloud2 msg,boost::shared_ptr< pcl::PointCloud< T > > ptr)
{
  pcl::fromROSMsg(msg,*ptr);
}

// Occupancy Grid map <-> opencv image
nav_msgs::OccupancyGrid  cvToMap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt);
cv::Mat                  mapToCv(nav_msgs::OccupancyGrid occumap);

void saveMap(std::string path, nav_msgs::OccupancyGrid  map);
bool loadMap(std::string path, nav_msgs::OccupancyGrid& map);
float calcDist(geometry_msgs::Point point1,geometry_msgs::Point point2);
}


#endif

