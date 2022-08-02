#ifndef HKCAMINTERFACE_H
#define HKCAMINTERFACE_H

#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace std;

extern string CONFIG_PATH;
extern string PRE_PATH;

extern sensor_msgs::ImagePtr pRgb;
extern sensor_msgs::ImagePtr pDepth;

class HKcamInterface{
public:
    // *** 构造函数 ***
    HKcamInterface();

    // *** 析构函数 ***
    ~HKcamInterface();

    // *** 获取图像 ***
    vector<sensor_msgs::ImagePtr> fetchFrame();

    // *** 获取相机信息 ***
    sensor_msgs::CameraInfo getHKCameraInfo();

    void stop();

private:
    // *** 句柄 ***
    void* handle = NULL;

    // *** 开始检测标记 ***
    bool m_flagOnce;
    bool m_flagStop;
};

#endif