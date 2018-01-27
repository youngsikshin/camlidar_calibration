#pragma once

#include <map>
#include <Eigen/Core>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <camlidar_calibration/Datatypes.h>
#include <camlidar_calibration/PinholeModel.h>

class GeometricCalib
{
public:
  GeometricCalib(CamlidarBucket& camlidar_bucket, PinholeModel::Ptr pinhole_model, Eigen::Matrix4f extrinsic);
  ~GeometricCalib();

  void marker_extraction();
  void show_prepared_data();

private:
  typedef vector<PointCloud> PointClusters;
  CamlidarBucket& camlidar_bucket_;
  PinholeModel::Ptr pinhole_model_;
  Eigen::Matrix4f extrinsic_;

  vector<cv::Mat> marker_images_;
  vector<PointClusters> pc_clusters_;
  vector<Eigen::Vector4f> plane_params_;
  vector<Eigen::Vector3f> plane_center_;
  vector<Eigen::Vector3f> marker_normals_;
  vector<Eigen::Vector3f> marker_positions_;
  map<int, int> plane_marker_pair_;

  bool aruco_marker_extraction(CamlidarData& camlidar_data);
  bool region_growing_segmentation(PointCloud& pc);
  void calc_extrinsic();

};
