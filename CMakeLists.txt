cmake_minimum_required(VERSION 2.8.3)
project(camlidar_calibration)

add_definitions(-std=c++11)
set(CMAKE_AUTOMOC ON)
# Find includes in corresponding build directories
# Adds in each directory ${CMAKE_CURRENT_SOURCE_DIR} and ${CMAKE_CURRENT_BINARY_DIR}
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (HEADER
    include/camlidar_calibration/MainWindow.h
    include/camlidar_calibration/RosClient.h
    include/camlidar_calibration/CameraModel.h
    include/camlidar_calibration/PinholeModel.h
    include/camlidar_calibration/Datatypes.h
    include/camlidar_calibration/Utility.h
    include/camlidar_calibration/GeometricCalib.h
)

set (SRC
    src/MainWindow.cpp
    src/main.cpp
    src/RosClient.cpp
    src/PinholeModel.cpp
    src/Utility.cpp
    src/GeometricCalib.cpp
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#find_package (Ceres REQUIRED)
#find_package(Glog)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

find_package (Qt5Widgets)
find_package (Qt5Gui)
find_package (Qt5OpenGL)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
#    std_msgs
    pcl_ros
    pcl_conversions
    image_transport
    cv_bridge
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp
#      std_msgs
      image_transport
      cv_bridge
      pcl_ros
      pcl_conversions
)

qt5_wrap_ui (camlidar_calibration_gui_moc ${SRC_DIR}/MainWindow.ui)
#qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${SRC_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
#    ${CERES_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIR}
)

add_executable (camlidar_calibration
    ${HEADER}
    ${SRC}
    ${camlidar_calibration_gui_moc}
)
target_link_libraries (camlidar_calibration
    Qt5::Widgets
    Qt5::Gui
#    Qt5::OpenGL
    ${catkin_LIBRARIES}
#    ${CERES_LIBRARIES}
    ${Glog_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Sophus_LIBS}
)

add_executable (gen_charuco
    src/gen_charuco.cpp
)

target_link_libraries (gen_charuco
    ${OpenCV_LIBS}
)

install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
