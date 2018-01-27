#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>

using namespace std;

int main()
{
    cout << "Generating charuco!" << endl;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
    cv::Mat board_image;
    board->draw(cv::Size(600,500), board_image, 10, 1);

    cv::imwrite("charuco_original.png",board_image);
    return 0;
}
