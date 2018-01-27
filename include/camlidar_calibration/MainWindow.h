#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPixmap>
#include <QImage>

#include <thread>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#include <camlidar_calibration/Datatypes.h>
#include <camlidar_calibration/Utility.h>
#include <camlidar_calibration/RosClient.h>
#include <camlidar_calibration/CameraModel.h>
#include <camlidar_calibration/PinholeModel.h>
#include <camlidar_calibration/GeometricCalib.h>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void on_select_button_clicked();

private slots:
  void on_preprocessing_button_clicked();

private slots:
  void on_h_spinbox_valueChanged(double arg1);

private slots:
  void on_p_spinbox_valueChanged(double arg1);

private slots:
  void on_r_spinbox_valueChanged(double arg1);

private slots:
  void on_z_spinbox_valueChanged(double arg1);

private slots:
  void on_y_spinbox_valueChanged(double arg1);

private slots:
  void on_x_spinbox_valueChanged(double arg1);

private slots:
  void on_start_stop_button_clicked();

private:
  Ui::MainWindow *ui;

  RosClient ros_client_;
  GeometricCalib* geometric_calib_;

  std::thread ros_thread_;

  QPixmap image_;
  CamlidarBucket camlidar_bucket_;

  CameraModel::Ptr camera_;
  PinholeModel::Ptr pinhole_model_;

  Eigen::Matrix4f extrinsic_;

  double x_, y_, z_, r_, p_, h_;
  bool is_extracted_marker_;
  bool is_segmented_points_;

  void set_pixmap();
  void set_spinbox_values();

private slots:
  void get_camlidar();
};

#endif // MAINWINDOW_H
