#pragma once

#include <QObject>

#include <string>
#include <atomic>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <camlidar_calibration/Datatypes.h>

using namespace std;

class RosClient : public QObject
{
Q_OBJECT

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> ApproximateSyncPolicy;

public:
    RosClient();
    ~RosClient();
    void run();
    void start();
    void stop();
    void camlidar_callback(const sensor_msgs::ImageConstPtr& cam, const sensor_msgs::PointCloud2ConstPtr& lidar);

    inline float& width() { return width_; }
    inline float& height() { return height_; }
    inline float& fx() { return fx_; }
    inline float& fy() { return fy_; }
    inline float& cx() { return cx_; }
    inline float& cy() { return cy_; }
    inline double& d0() { return d_[0]; }
    inline double& d1() { return d_[1]; }
    inline double& d2() { return d_[2]; }
    inline double& d3() { return d_[3]; }
    inline double& d4() { return d_[4]; }
    inline Matrix3x4& extrinsics() { return extrinsics_; }

    CamlidarData& camlidar_data() { return camlidar_data_; }


private:
    atomic<bool> status_;

    camlidar_topic_t camlidar_topic_;

    ros::NodeHandle node_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;

    message_filters::Synchronizer<ApproximateSyncPolicy> *sync_;

    CamlidarData camlidar_data_;

    void read_parameters();

    float width_, height_;
    float fx_, fy_, cx_, cy_;
    double d_[5];

    Matrix3x4 extrinsics_;

signals:
    void camlidar_data_signal(void);

};

