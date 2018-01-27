#include <camlidar_calibration/Utility.h>

int64_t timestamp_now()
{
    std::chrono::system_clock::time_point now = std::chrono::high_resolution_clock::now();
    std::chrono::system_clock::duration duration_now = now.time_since_epoch();

    return std::chrono::duration_cast<std::chrono::microseconds>(duration_now).count();
}

void cvmat_to_qimage(cv::Mat& src, QImage& dst)
{
    switch (src.type()) {
    case CV_8UC1:
      dst = QImage(src.data, src.cols, src.rows, static_cast<int> (src.step), QImage::Format_Grayscale8);
      break;
    case CV_8UC3:
      dst = QImage(src.data, src.cols, src.rows, static_cast<int> (src.step), QImage::Format_RGB888);
      dst = dst.rgbSwapped();
    default:
      break;
    }
}
