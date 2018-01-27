#include <camlidar_calibration/MainWindow.h>

#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow), x_(0.0), y_(0.0), z_(0.0), r_(0.0), p_(0.0), h_(0.0)
{
  ui->setupUi(this);

  connect(&ros_client_, SIGNAL(camlidar_data_signal()), this, SLOT(get_camlidar()));

  ros_thread_ = std::thread(&RosClient::start, &ros_client_);

  camera_.reset(new PinholeModel(ros_client_.width(),
                                 ros_client_.height(),
                                 ros_client_.fx(),
                                 ros_client_.fy(),
                                 ros_client_.cx(),
                                 ros_client_.cy(),
                                 ros_client_.d0(),
                                 ros_client_.d1(),
                                 ros_client_.d2(),
                                 ros_client_.d3(),
                                 ros_client_.d4()));

  pinhole_model_ = static_pointer_cast<PinholeModel> (camera_);

  Matrix3x4 extrinsics3x4 = ros_client_.extrinsics();
  extrinsic_.block<3, 4>(0, 0) = extrinsics3x4;
  set_spinbox_values();

  is_extracted_marker_ = false;
  is_segmented_points_ = false;
  geometric_calib_ = new GeometricCalib(camlidar_bucket_, pinhole_model_, extrinsic_);
}

MainWindow::~MainWindow()
{
  ros::shutdown();
  if(ros_thread_.joinable())
    ros_thread_.join();

  delete ui;
}

void MainWindow::get_camlidar()
{
    CamlidarData& camlidar_data = ros_client_.camlidar_data();

    cv::Mat& image = camlidar_data.first;
    PointCloud& pc = camlidar_data.second;

    PointCloud tf_pc = pc;
    pcl::transformPointCloud(pc, tf_pc, extrinsic_);

    camera_->undistort_image(image, image);

//    int64_t tic = timestamp_now();
//    if(aruco_marker_extraction()) {

//        points_segmentation(pc);
//    }
//    int64_t toc = timestamp_now();

//    cerr << (toc-tic)*1e-6 << endl;

    cv::Mat img_with_points;
    image.copyTo(img_with_points);

    for(auto point:tf_pc) {

        auto uv = camera_->xyz_to_uv(point);

        if(!camera_->is_in_image(uv, 2) || point.z <= 0.0 || point.z > 30.0)
            continue;

        const float u_f = uv(0);
        const float v_f = uv(1);
        const int u_i = static_cast<int> (u_f);
        const int v_i = static_cast<int> (v_f);

        cv::circle(img_with_points, cv::Point(u_i, v_i), 1.0, cv::Scalar( 0.0, 0.0, 255.0 ), -1);

    }


    QImage qimage;
    cvmat_to_qimage(img_with_points, qimage);

    image_ = QPixmap::fromImage(qimage);
    ui->image_label->setPixmap(image_.scaledToWidth(ui->image_label->width()));
}

void MainWindow::set_pixmap()
{

}

void MainWindow::set_spinbox_values()
{
    Eigen::Matrix3f R = extrinsic_.block<3, 3>(0, 0);
    Eigen::Vector3f rph = R.eulerAngles(0, 1, 2);

    x_ = extrinsic_(0,3);
    y_ = extrinsic_(1,3);
    z_ = extrinsic_(2,3);

    r_ = rph(0)*180.0/M_PI;
    p_ = rph(1)*180.0/M_PI;
    h_ = rph(2)*180.0/M_PI;

    ui->x_spinbox->setValue(x_);
    ui->y_spinbox->setValue(y_);
    ui->z_spinbox->setValue(z_);

    ui->r_spinbox->setValue(r_);
    ui->p_spinbox->setValue(p_);
    ui->h_spinbox->setValue(h_);
}

void MainWindow::on_start_stop_button_clicked()
{
    if(!ui->start_stop_button->text().compare("Stop")) {
        ui->start_stop_button->setText("Start");
        ros_client_.stop();
        ros_thread_.join();
    }
    else if(!ui->start_stop_button->text().compare("Start")) {
        ui->start_stop_button->setText("Stop");
//        ros_client_.start();
        ros_thread_ = std::thread(&RosClient::start, &ros_client_);
    }
}

void MainWindow::on_x_spinbox_valueChanged(double arg1)
{
    x_ = arg1;
}

void MainWindow::on_y_spinbox_valueChanged(double arg1)
{
    y_ = arg1;
}

void MainWindow::on_z_spinbox_valueChanged(double arg1)
{
    z_ = arg1;
}

void MainWindow::on_r_spinbox_valueChanged(double arg1)
{
    r_ = arg1;
}

void MainWindow::on_p_spinbox_valueChanged(double arg1)
{
    p_ = arg1;
}

void MainWindow::on_h_spinbox_valueChanged(double arg1)
{
    h_ = arg1;
}

void MainWindow::on_preprocessing_button_clicked()
{
    if(!is_extracted_marker_)
      geometric_calib_->marker_extraction();
}

void MainWindow::on_select_button_clicked()
{
  CamlidarData camlidar_data = ros_client_.camlidar_data();
  camlidar_bucket_.push_back(camlidar_data);
}
