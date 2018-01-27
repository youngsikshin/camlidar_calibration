#include <camlidar_calibration//MainWindow.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "camlidar_calibration_node");

    MainWindow w;
    w.show();

    return a.exec();
}
