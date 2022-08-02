#ifndef SED_HK_H
#define SED_HK_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "../include/yaml-cpp/yaml.h"

using namespace std;

// *** 配置文件路径 ***
extern string CONFIG_PATH;
extern string PRE_PATH;

// *** 相机话题　***
extern string rgb_topic;
extern string depth_topic;

// *** 图尺寸 ***
extern int img_width;
extern int img_height;

// *** 目标框尺寸 ***
extern int roi_x;
extern int roi_y;
extern int roi_width;
extern int roi_height;

// *** 直线检测标记 ***
extern bool test_line;

class Sed_hk{
public:
    // *** 构造函数 ***
    Sed_hk();

    // *** 析构函数 ***
    ~Sed_hk();
    
    // *** 初始化函数 ***
    void initialProcess();

    // *** 彩色图回调函数　***
    void rgbCb(const sensor_msgs::ImageConstPtr &msg);

    // *** 深度图回调函数　***
    void depthCb(const sensor_msgs::ImageConstPtr &msg);

    // *** 深度图像处理函数 ***
    void depthProcess();

    // *** 左右遍历函数 ***
    void edgeDetect(int ry);

    // *** 纵坐标遍历函数 ***
    vector<int> pointAround(vector<int> &edge, bool &flag, int &count, int &ry, int &num, int spX);

    // *** 直线拟合函数 ***
    vector<int> lineProcess();

    // *** 绘图函数 ***
    void paint();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // *** 创建订阅对象　***
    image_transport::Subscriber rgbSub;
    image_transport::Subscriber depthSub;

    // *** 图像指针　***
    cv_bridge::CvImagePtr rgbPtr;
    cv_bridge::CvImagePtr depthPtr;

    // *** 遍历间隔 ***
    int m_gapRow;
    int m_gapCol;
    int m_gapVal;
    int m_biasCol;
    
    // *** 边缘点参数 ***
    int m_numL;
    int m_numR;
    int m_numPoint;

    // *** 坐标偏差 ***
    int m_biasX;
    int m_biasY;

    // *** 深度差阈值　***
    float m_deVal;

    // *** 直线拟合参数 ***
    double m_param;
    double m_reps;
    double m_aeps;

    // *** 边缘标记 ***
    bool m_flagEdge;

    // *** 起始点 ***
    vector<int> m_sp;

    // *** 边缘点向量　***
    vector<int> m_edgeL;
    vector<int> m_edgeR;

    // *** 直线端点坐标 ***
    vector<int> m_line; 

    // *** 标准图像 ***
    cv::Mat m_rgbImage;
    cv::Mat m_depthImage;
    cv::Mat m_binaryImage;
};

#endif