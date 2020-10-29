#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher  PCPublisher;
ros::Publisher  PCSORPublisher;
int cnt;


void sor(pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointXYZRGB>& output, int numNeighbor, float stdThreshold){
  pcl::PointCloud<pcl::PointXYZRGB> pc_sor_filtered;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_sor_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  *ptr_sor_filtered = input;


  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (ptr_sor_filtered);
  sor.setMeanK (numNeighbor);
  sor.setStddevMulThresh (stdThreshold);
  sor.filter(*ptr_sor_filtered);

  output = *ptr_sor_filtered;
}



void callbackLaser(const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout<<++cnt<<" data is comming"<<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> cloud, output;
  pcl::fromROSMsg (*input, cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::Matrix4f rot, trans, tf_depth2body;
  // 21 degree
//  trans<< 1,            0,       0,   0.0,
//          0,       0.9335, -0.3584,   0.0,
//          0,       0.3584,  0.9335,   0.0,
//          0,            0,       0,     1;
  // 22
//  trans<< 1,            0,       0,   0.0,
//          0,       0.9271, -0.3747,   0.0,
//          0,       0.3747,  0.9271,   0.0,
//          0,            0,       0,     1;
  // 23
//  trans<< 1,            0,       0,   0.0,
//          0,       0.9205, -0.3908,   0.0,
//          0,       0.3908,  0.9205,   0.0,
//          0,            0,       0,     1;

  // 25
  rot<<   1,            0,       0,   0.0,
          0,       0.9063, -0.4227,   0.0,
          0,       0.4227,  0.9063,   0.0,
          0,            0,       0,     1;

  trans<<   0,            0,       1,   0.0889,
           -1,            0,       0,   0.0,
            0,           -1,       0,   0.1271,
            0,            0,       0,     1;
  tf_depth2body = trans * rot;

  pcl::transformPointCloud(cloud, *ptr_transformed, tf_depth2body);
  output = *ptr_transformed;

  pcl::PointCloud<pcl::PointXYZRGB> sor_output;
  auto start = ros::Time::now();
  sor(output, sor_output, 30, 0.1);
  auto end = ros::Time::now();
  std::cout<<end-start<<std::endl;

  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(output, cloud_ROS);
  cloud_ROS.header.frame_id = input->header.frame_id;
  PCPublisher.publish(cloud_ROS);

  sensor_msgs::PointCloud2 sor_ROS;
  pcl::toROSMsg(sor_output, sor_ROS);
  sor_ROS.header.frame_id = input->header.frame_id;
  PCSORPublisher.publish(sor_ROS);
}


int main(int argc, char **argv)
{
  Eigen::Matrix4f rot, trans, tf_depth2body;
  // 20 degree
  rot<< 1,            0,       0,   0.0,
          0,       0.9396, -0.3421,   0.0,
          0,       0.3421,  0.9396,   0.0,
          0,            0,       0,     1;


  trans<<   0,            0,       1,   0.202,
           -1,            0,       0,   0.0,
            0,           -1,       0,   0.2896,
            0,            0,       0,     1;

  tf_depth2body = trans * rot;
  std::cout<<tf_depth2body<<std::endl;

    cnt = 0;
    std::cout<<"Operate TF"<<std::endl;
    ros::init(argc, argv, "merger");
    ros::NodeHandle   nodeHandler;

    PCPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/viz_tf/tf_pc",100);
    PCSORPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/viz_tf/sor_pc",100);
    ros::Subscriber subLaser =
        nodeHandler.subscribe<sensor_msgs::PointCloud2>("//camera/depth/color/points",1000,callbackLaser);

    ros::spin();

    return 0;
}
