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

    // *** 返回实时标记 ***
    bool isRT();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // *** 创建订阅对象　***
    image_transport::Subscriber rgbSub;
    image_transport::Subscriber depthSub;

    // *** 图像指针　***
    cv_bridge::CvImagePtr rgbPtr;
    cv_bridge::CvImagePtr depthPtr;

    // *** 配置文件路径 ***
    string m_configPath;
    string m_prePath;

    // *** 相机话题　***
    string m_rgbTopic;
    string m_depthTopic;

    // *** 图尺寸 ***
    int m_imgWidth;
    int m_imgHeight;

    // *** 目标框尺寸 ***
    int m_roiX;
    int m_roiY;
    int m_roiWidth;
    int m_roiHeight;

    // *** 遍历间隔 ***
    int m_gapRow;
    int m_gapCol;
    int m_gapVal;
    int m_biasRow;
    
    // *** 边缘点参数 ***
    int m_numL;
    int m_numR;
    int m_numPoint;

    // *** 纵坐标范围 ***
    int m_rangeVal;

    // *** 坐标偏差 ***
    int m_biasX;
    int m_biasY;

    // *** 深度差阈值　***
    float m_deVal;

    // *** 直线拟合参数 ***
    double m_param;
    double m_reps;
    double m_aeps;

    // *** 实时检测标记 ***
    bool m_flagRT;

    // *** 直线检测标记 ***
    bool m_flagLine;

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