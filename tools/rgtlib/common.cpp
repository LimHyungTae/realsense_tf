#include "common.h"

namespace rgtlib
{
// Eigen::Matrix4f <-> cv::Mat
Eigen::Matrix4f cvToEigen(cv::Mat mat)
{
  Eigen::Matrix4f output =  Eigen::Matrix4f::Identity();
  for(int y = 0; y < 4; y++)
  {
    for(int x = 0; x < 4; x ++)
    {
      output(x,y) = mat.at<float>(x,y);
    }
  }
  return output;
}

cv::Mat eigenToCv(Eigen::Matrix4f mat)
{
  cv::Mat output = cv::Mat::zeros(4,4,CV_32FC1);
  for(int y = 0; y < 4; y++)
  {
    for(int x = 0; x < 4; x ++)
    {
      output.at<float>(x,y) = mat(x,y);
    }
  }
  return output;

}
// 6 Degree of Freedom <-> cv::Mat
void cvToDoF6(cv::Mat mat, float *x, float *y, float *z, float *roll, float *pitch, float *yaw)
{
  *x = mat.at<float>(0,3);
  *y = mat.at<float>(1,3);
  *z = mat.at<float>(2,3);
  cv::Mat rot_mat = cv::Mat(mat(cv::Rect(0,0,3,3)));

  cv::Mat rot_vec;
  cv::Rodrigues(rot_mat,rot_vec);
  *roll = rot_vec.at<float>(0);
  *pitch = rot_vec.at<float>(1);
  *yaw = rot_vec.at<float>(2);
}

cv::Mat doF6ToCv(float x, float y, float z, float roll, float pitch, float yaw)
{
  cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
  rot_vec.at<float>(0) = roll;
  rot_vec.at<float>(1) = pitch;
  rot_vec.at<float>(2) = yaw;

  cv::Mat rot_mat;
  cv::Rodrigues(rot_vec,rot_mat);

  cv::Mat output = cv::Mat::zeros(4,4,CV_32FC1);

  rot_mat.copyTo(output(cv::Rect(0,0,3,3)));

  output.at<float>(0,3) = x;
  output.at<float>(1,3) = y;
  output.at<float>(2,3) = z;

  output.at<float>(3,3) = 1;

  return output;
}
// 6 Degree of Freedom <-> Eigen 4x4 Transformation matrix
Eigen::VectorXf eigenToDoF6(Eigen::Matrix4f mat)
{
  Eigen::VectorXf output(6);
  cvToDoF6(eigenToCv(mat), &output[0], &output[1], &output[2], &output[3], &output[4], &output[5]);
  return output;
}

Eigen::Matrix4f doF6ToEigen(float x, float y, float z, float roll, float pitch, float yaw)
{
  Eigen::Matrix4f output = cvToEigen(doF6ToCv(x,y,z,roll,pitch,yaw));
  return output;
}

// geometry_msgs::Pose <-> Eigen 4x4 Transformation matrix
geometry_msgs::Pose eigenToPose(Eigen::Matrix4f pose)
{
  geometry_msgs::Pose geoPose;

  tf::Matrix3x3 m;
  m.setValue((double)pose(0,0),
             (double)pose(0,1),
             (double)pose(0,2),
             (double)pose(1,0),
             (double)pose(1,1),
             (double)pose(1,2),
             (double)pose(2,0),
             (double)pose(2,1),
             (double)pose(2,2));

  tf::Quaternion q;
  m.getRotation(q);
  geoPose.orientation.x = q.getX();
  geoPose.orientation.y = q.getY();
  geoPose.orientation.z = q.getZ();
  geoPose.orientation.w = q.getW();

  geoPose.position.x = pose(0,3);
  geoPose.position.y = pose(1,3);
  geoPose.position.z = pose(2,3);

  return geoPose;
}
Eigen::Matrix4f poseToEigen(geometry_msgs::Pose geoPose)
{
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
  tf::Matrix3x3 m(q);
  output(0,0) = m[0][0];
  output(0,1) = m[0][1];
  output(0,2) = m[0][2];
  output(1,0) = m[1][0];
  output(1,1) = m[1][1];
  output(1,2) = m[1][2];
  output(2,0) = m[2][0];
  output(2,1) = m[2][1];
  output(2,2) = m[2][2];
  output(3,3) = 1;

  output(0,3) = geoPose.position.x;
  output(1,3) = geoPose.position.y;
  output(2,3) = geoPose.position.z;

  return output;
}

Eigen::MatrixXf pointToEigen(geometry_msgs::Point geoPoint)
    {
      Eigen::MatrixXf result(4,1);
      result << geoPoint.x, geoPoint.y, geoPoint.z, 1;
      return result;
    }


// Occupancy Grid map <-> opencv image
nav_msgs::OccupancyGrid cvToMap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt)
{
  nav_msgs::OccupancyGrid m_gridmap;
  m_gridmap.info.resolution = resolution;
  geometry_msgs::Pose origin;
  origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
  origin.orientation.w = 1;
  m_gridmap.info.origin = origin;
  m_gridmap.info.width = cvimg.size().width;
  m_gridmap.info.height = cvimg.size().height;
  //ROS_INFO("[CV 2 OCCUGRID CONVERSION] PUT CV DATA");
  for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);
  //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID SIZE : %d",m_gridmap.data.size());
  //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);

  for(int y = 0; y < cvimg.size().height; y++)
  {
    for(int x = 0; x < cvimg.size().width; x++)
    {
      int tmpdata = cvimg.at<unsigned char>(y,x);
      int ttmpdata = -1; //Unknown
      if(tmpdata >= 150) //free
      {
        ttmpdata = (tmpdata - 250) / -2;
        if(ttmpdata < 0)
          ttmpdata = 0;
      }
      else if(tmpdata <= 98)
      {
        ttmpdata = (tmpdata - 200) / -2;
        if(ttmpdata > 100)
          ttmpdata = 100;
      }
      m_gridmap.data.at(x + m_gridmap.info.width * y) = ttmpdata;
    }
  }
  return m_gridmap;
}

cv::Mat mapToCv(nav_msgs::OccupancyGrid occumap)
{
  // unkown = -1 -> 99~149, free : 0:50 ->250:150, occupied :51:100 -> 0:98
  double resolution = occumap.info.resolution;
  cv::Point origin_cvpt(-occumap.info.origin.position.x / resolution,
                        -occumap.info.origin.position.y / resolution);
  cv::Size img_size;
  cv::Mat cvimg = cv::Mat::zeros( occumap.info.height,occumap.info.width,CV_8UC1);
  //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] GRID SIZE : %d",occumap.data.size());
  //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] CV ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);
  for(int pt = 0;pt < occumap.data.size();pt++)
  {
    int pt_y = pt / occumap.info.width;
    int pt_x = pt % occumap.info.width;
    int value = occumap.data.at(pt);
    unsigned char img_value;
    if(value == -1) img_value = 120;
    else if (value <= 50) img_value = 250 - 2 * value;
    else if (value >=51) img_value = 200 - 2 * value;
    cvimg.at<unsigned char>(pt_y,pt_x) = img_value;
  }
  return cvimg;
}

void saveMap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in)
{
  cv::Mat occumat = mapToCv(gridmap_in);
  std::stringstream strm_png;
  strm_png << filepath << ".png";
  std::stringstream strm_info;
  strm_info << filepath << ".csv";

  cv::imwrite(strm_png.str(),occumat);

  std::ofstream filesave(strm_info.str().c_str());
  if(filesave.is_open())
  {
    filesave << gridmap_in.info.resolution << "\n";
    filesave << gridmap_in.info.origin.position.x << "\n";
    filesave << gridmap_in.info.origin.position.y << "\n";
  }
  filesave.close();
}

bool loadMap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out)
{
  std::stringstream strm_png;
  strm_png << filepath <<".png";
  std::stringstream strm_info;
  strm_info << filepath << ".csv";
  cv::Mat occumat = cv::imread(strm_png.str(),cv::IMREAD_GRAYSCALE);
  std::ifstream fileload(strm_info.str().c_str());
  float resolution,origin_x,origin_y;
  std::vector<std::string>   output;
  std::string line;
  if(!fileload.is_open())
  {
    std::cout << "[Error]: Cannot open occupancy map. Check the path " <<filepath<< std::endl;
    return false;
  }
  while(std::getline(fileload,line)) output.push_back(line);
  if(output.size()!=3)
  {
    std::cout << "[Error]: Cannot open occupancy map (arguments)" << std::endl;
    return false;
  }
  resolution = std::atof(output.at(0).c_str());
  origin_x = std::atof(output.at(1).c_str());
  origin_y = std::atof(output.at(2).c_str());
  gridmap_out = cvToMap(occumat, resolution,cv::Point(- origin_x / resolution,-origin_y / resolution));
  gridmap_out.header.frame_id = "map";

  return true;
}
float calcDist(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
  return sqrt(pow(point1.x-point2.x,2.0)+pow(point1.y-point2.y,2.0)+pow(point1.z-point2.z,2.0));
}
}

