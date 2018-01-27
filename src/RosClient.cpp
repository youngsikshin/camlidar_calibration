#include <camlidar_calibration/RosClient.h>

RosClient::RosClient()
{
    status_ = true;

    read_parameters();

    image_sub_.subscribe(node_, camlidar_topic_.cam_topic, 100);
    lidar_sub_.subscribe(node_, camlidar_topic_.lidar_topic, 10);

    sync_ = new message_filters::Synchronizer<ApproximateSyncPolicy> (ApproximateSyncPolicy(100), image_sub_, lidar_sub_);
    sync_->registerCallback(boost::bind(&RosClient::camlidar_callback, this, _1, _2));

}

RosClient::~RosClient()
{

}

void RosClient::start()
{
  status_ = true;

  run();
}

void RosClient::stop()
{
  status_ = false;
}

void RosClient::run()
{
  while(ros::ok() && status_)
      ros::spinOnce();
}

void RosClient::camlidar_callback(const sensor_msgs::ImageConstPtr& cam, const sensor_msgs::PointCloud2ConstPtr& lidar)
{
  // camera processing
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(cam, cam->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  sensor_msgs::PointCloud2ConstPtr pc_ptr;
  pc_ptr = lidar;

  PointCloud pc;
  pcl::fromROSMsg(*pc_ptr, pc);

  cv::Mat rgb;
  cvtColor(cv_ptr->image, rgb, CV_BayerRG2BGR);

  camlidar_data_ = CamlidarData(rgb, pc);
  emit camlidar_data_signal();

//  cv::namedWindow("test");
//  cv::imshow("test",rgb);
//  cv::waitKey(10);

//  cerr << pc[0].x << ", " << pc[0].y << ", " << pc[0].z << endl;
}

void RosClient::read_parameters()
{
  string pkg_path = ros::package::getPath("camlidar_calibration");
  string params_path = pkg_path+"/params/";

  cv::FileStorage f_ros_settings(params_path+"camlidar_param.yaml", cv::FileStorage::READ);

  camlidar_topic_.cam_topic = string(f_ros_settings["Ros.Camera"]);
  camlidar_topic_.lidar_topic = string(f_ros_settings["Ros.Velodyne"]);

  width_ = f_ros_settings["Camera.width"];
  height_ = f_ros_settings["Camera.height"];
  fx_ = f_ros_settings["Camera.fx"];
  fy_ = f_ros_settings["Camera.fy"];
  cx_ = f_ros_settings["Camera.cx"];
  cy_ = f_ros_settings["Camera.cy"];
  float k1 = f_ros_settings["Camera.k1"];
  float k2 = f_ros_settings["Camera.k2"];
  float p1 = f_ros_settings["Camera.p1"];
  float p2 = f_ros_settings["Camera.p2"];
  d_[0] = f_ros_settings["Camera.d0"];
  d_[1] = f_ros_settings["Camera.d1"];
  d_[2] = f_ros_settings["Camera.d2"];
  d_[3] = f_ros_settings["Camera.d3"];
  d_[4] = f_ros_settings["Camera.d4"];
  int RGB_sel = f_ros_settings["Camera.RGB"];

  cv::Mat T;
  f_ros_settings["extrinsicMatrix"] >> T;
  cv::cv2eigen(T, extrinsics_);

}
