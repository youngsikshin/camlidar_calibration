#include<camlidar_calibration/GeometricCalib.h>

GeometricCalib::GeometricCalib(CamlidarBucket& camlidar_bucket, PinholeModel::Ptr pinhole_model, Eigen::Matrix4f extrinsic)
    : camlidar_bucket_(camlidar_bucket)
{
    pinhole_model_ = pinhole_model;
    extrinsic_ = extrinsic;

    cerr << extrinsic_ << endl;
}

GeometricCalib::~GeometricCalib()
{

}

void GeometricCalib::marker_extraction()
{
    for(auto iter = camlidar_bucket_.begin(); iter!=camlidar_bucket_.end();) {

        if(region_growing_segmentation(iter->second)) {
            if(aruco_marker_extraction(*iter)) {
                cerr << "[GeometricCalib]\t Marker extraction and plane segmentation are success!" << endl;
            }
            else {
                iter = camlidar_bucket_.erase(iter);
                marker_images_.erase(marker_images_.end());
            }

            cerr << "[GeometricCalib]\t Run regiongrowing." << endl;

        }
        else {
            cerr << "[GeometricCalib]\t Erase invalid data pair." << endl;
            iter = camlidar_bucket_.erase(iter);
        }

        iter++;

    }

//    show_prepared_data();
//    calc_extrinsic();

    cerr << marker_images_.size() << endl;
    cerr << pc_clusters_.size() << endl;
    cerr << plane_params_.size() << endl;
}

void GeometricCalib::show_prepared_data()
{

    for(int i=0; i < marker_images_.size(); ++i) {
        cv::Mat& img = marker_images_[i];

        PointClusters& clusters = pc_clusters_[i];

        int cluster_idx = 0;
        for(int j=0; j < clusters.size(); ++j) {
            PointCloud& pc = clusters[j];
            PointCloud pc_c;
            pcl::transformPointCloud(pc, pc_c, extrinsic_);

            float sum_x=0;
            float sum_y=0;
            float sum_z=0;
            for(auto point:pc_c) {
                sum_x += point.x;
                sum_y += point.y;
                sum_z += point.z;
            }

            Eigen::Vector3f plane_center(sum_x/pc.size(), sum_y/pc.size(), sum_z/pc.size());

            float min_dist = 100.0;
            int min_idx = -1;

            for(int k=0; k < marker_normals_.size(); ++k) {
                auto d = -(marker_normals_[k](0)*marker_positions_[k](0) + marker_normals_[k](1) * marker_positions_[k](1) + marker_normals_[k](2) * marker_positions_[k](2));

                auto marker_dist = marker_normals_[k].dot(plane_center) + d;

                if(marker_dist < min_dist) {
                    min_dist = marker_dist;
                    min_idx = k;
                }
            }
            plane_marker_pair_.insert(pair<int,int>(j,min_idx));

            ++cluster_idx;

            auto plane_param = plane_params_[cluster_idx-1];
            cerr << "nv" << cluster_idx << " = [ " << plane_param(0) << " "
                                                   << plane_param(1) << " "
                                                   << plane_param(2) << " ]';" << endl;

            cerr << "dv" << cluster_idx << " = " << plane_param(3) << std::endl;
            cerr << "pv" << cluster_idx << " = [ " << sum_x/pc.size() << " " << sum_y/pc.size() << " " << sum_z/pc.size() << " ]';" << endl;
        }

        for(int j=0; j < clusters.size(); ++j) {
            PointCloud& pc = clusters[j];
            PointCloud pc_c;
            pcl::transformPointCloud(pc, pc_c, extrinsic_);
            for(auto point:pc_c) {

                auto uv = pinhole_model_->xyz_to_uv(point);

                if(!pinhole_model_->is_in_image(uv, 2) || point.z <= 0.0 || point.z > 30.0)
                    continue;

                const float u_f = uv(0);
                const float v_f = uv(1);
                const int u_i = static_cast<int> (u_f);
                const int v_i = static_cast<int> (v_f);

                if(plane_marker_pair_[j] == 0)
                    cv::circle(img, cv::Point(u_i, v_i), 1.0, cv::Scalar( 0.0, 0.0, 255.0 ), -1);
                else if(plane_marker_pair_[j]==1)
                    cv::circle(img, cv::Point(u_i, v_i), 1.0, cv::Scalar( 0.0, 255.0, 0.0 ), -1);
                else if(plane_marker_pair_[j]==2)
                    cv::circle(img, cv::Point(u_i, v_i), 1.0, cv::Scalar( 255.0, 0.0, 0.0 ), -1);

            }

        }


        cv::namedWindow("test");
        cv::imshow("test", img);
        cv::waitKey(1000);
    }

}

void GeometricCalib::calc_extrinsic()
{
    Eigen::Vector3f nv1(plane_params_[0](0), plane_params_[0](1), plane_params_[0](2));
    Eigen::Vector3f nv2(plane_params_[1](0), plane_params_[1](1), plane_params_[1](2));
    Eigen::Vector3f nv3(plane_params_[2](0), plane_params_[2](1), plane_params_[2](2));

    Eigen::Vector3f nc1 = marker_normals_[plane_marker_pair_[0]];
    Eigen::Vector3f nc2 = marker_normals_[plane_marker_pair_[1]];
    Eigen::Vector3f nc3 = marker_normals_[plane_marker_pair_[2]];

    Eigen::Vector3f pc1 = marker_positions_[plane_marker_pair_[0]];
    Eigen::Vector3f pc2 = marker_positions_[plane_marker_pair_[1]];
    Eigen::Vector3f pc3 = marker_positions_[plane_marker_pair_[2]];

    Eigen::Matrix3f cov = nc1 * nv1.transpose() + nc2 * nv2.transpose() + nc3 * nv3.transpose();

//    JacobiSVD<MatrixXf> svd(cov, ComputeThinU | ComputeThinV);

    cerr << cov << endl << endl;
//    cerr << svd.matrixU() << endl;
}

bool GeometricCalib::aruco_marker_extraction(CamlidarData& camlidar_data)
{
    // prepare image
    cv::Mat& image = camlidar_data.first;

    cv::namedWindow("test");
    cv::imshow("test", image);
    cv::waitKey(1000);

    cv::Mat gray_image;
    image.convertTo(gray_image, CV_8UC1, 1.0);//, 1.0/255);

    // aruco detector
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);     //DICT_6X6_1000         DICT_ARUCO_ORIGINAL
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

    std::vector< int > ids;
    std::vector< std::vector< cv::Point2f > > corners, rejected;
    cv::aruco::detectMarkers(gray_image, dictionary, corners, ids);

    cv::Mat& camera_matrix = pinhole_model_->camera_matrix();
    cv::Mat dist_coeffs(5, 1, CV_64FC1, cv::Scalar::all(0));

    vector<cv::Vec3d> rvecs, tvecs;

    cv::Mat marker_img;
    image.copyTo(marker_img);

    if(ids.size() == 3) {

        cv::aruco::drawDetectedMarkers(marker_img, corners, ids);
        cv::aruco::estimatePoseSingleMarkers(corners, 0.1492, camera_matrix, dist_coeffs, rvecs, tvecs);

        for(size_t i = 0; i < ids.size(); ++i) {

            if (ids[i] == 213 || ids[i] == 100 || ids[i] == 120 || ids[i] == 140 || ids[i] == 160 || ids[i] == 180) {
                cv::aruco::drawAxis(marker_img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);

                cv::Mat Rot;
                cv::Rodrigues(rvecs[i],Rot);

                Eigen::Matrix3f R;
                cv::cv2eigen(Rot, R);

                Eigen::Vector3f t(tvecs[i](0), tvecs[i](1), tvecs[i](2));

                Eigen::Vector3f normal(Rot.at<double> (0,2), Rot.at<double> (1,2), Rot.at<double> (2,2));

                marker_normals_.push_back(normal);
                marker_positions_.push_back(t);

                cerr << "ID: " << ids[i] << endl;
                cerr << "nc" << i+1 << " = [ " << Rot.at<double> (0,2) << " " << Rot.at<double> (1,2) << " " << Rot.at<double> (2,2) << "]';" << endl;
                cerr << "pc" << i+1 << " = [ " << t(0) << ", " << t(1) << ", " << t(2) << "]';" << endl;
            }

        }

        marker_images_.push_back((marker_img));
        return true;

    }

    return false;
}

bool GeometricCalib::region_growing_segmentation(PointCloud& pc)
{
//    pcl::transformPointCloud(pc, pc, extrinsic_);
    PointCloud::Ptr src = pc.makeShared();


    pcl::search::Search<Point>::Ptr tree = boost::shared_ptr<pcl::search::Search<Point>> (new pcl::search::KdTree<Point>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(src);
    normal_estimator.setKSearch(200);
    normal_estimator.compute(*normals);

    pcl::RegionGrowing<Point, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(src);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(0.05);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    cerr << "[GeometricCalib]\t cluster size : " << clusters.size() << endl;

    if(clusters.size() == 3) {
        // divide pc
        vector<PointCloud> pc_clusters;

        for(int i=0; i < clusters.size(); ++i) {
            auto cluster = clusters[i];

            PointCloud pc_cluster;
            for(auto idx:cluster.indices) {
                Point& point = pc[idx];
                pc_cluster.push_back(point);
            }

            PointCloud::Ptr ptr_pc_cluster = pc_cluster.makeShared();
            // get plane parameters
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<Point> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.008);

            seg.setInputCloud (ptr_pc_cluster);
            seg.segment (*inliers, *coefficients);

            PointCloud inlier_cluster;
            float sum_x=0;
            float sum_y=0;
            float sum_z=0;

            for(auto idx:inliers->indices) {
                Point& point = pc_cluster[idx];
                sum_x += point.x; sum_y += point.y; sum_z += point.z;
                inlier_cluster.push_back(point);
            }

            Eigen::Vector4f plane_params;

            plane_params << coefficients->values[0],
                            coefficients->values[1],
                            coefficients->values[2],
                            coefficients->values[3];

            if (coefficients->values[3] < 0.0)
                plane_params = -1.0*plane_params;

            cerr << coefficients->values[0]
                 << coefficients->values[1]
                 << coefficients->values[2]
                 << coefficients->values[3] << endl;

            pc_clusters.push_back(inlier_cluster);
            plane_params_.push_back(plane_params);

//            Eigen::Vector3f plane_center(sum_x/inlier_cluster.size(), sum_y/inlier_cluster.size(), sum_z/inlier_cluster.size());
//            plane_center_.push_back(plane_center);
        }


//        for(int i=0; i<3; ++i) {
//            auto plane_param = plane_params_[i];
//            auto plane_center = plane_center_[i];
//            cerr << "plane equation : " << plane_param(0) * plane_center(0) + plane_param(1) * plane_center(1) + plane_param(2) * plane_center(2) + plane_param(3) << endl;
//        }

        pc_clusters_.push_back(pc_clusters);

        return true;
    } else {
        return false;
    }

}
