#pragma once

#include <chrono>
#include <opencv2/opencv.hpp>
#include <QImage>


int64_t timestamp_now();
void cvmat_to_qimage(cv::Mat& src, QImage& dst);
