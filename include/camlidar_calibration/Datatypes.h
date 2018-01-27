#pragma once
#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

typedef struct _camlidar_topic_t camlidar_topic_t;
struct _camlidar_topic_t {
    std::string cam_topic;
    std::string lidar_topic;
};

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

typedef pair<cv::Mat, PointCloud> CamlidarData;
typedef vector<CamlidarData> CamlidarBucket;

typedef float NumType;
typedef Eigen::Matrix<NumType, 1, 3> Matrix1x3;
typedef Eigen::Matrix<NumType, 3, 4> Matrix3x4;
