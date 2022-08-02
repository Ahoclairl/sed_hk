// *** 这是一个循环请求消息的客户端 ***
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "../include/HKcamInterface.h"
#include "sed_hk/image.h"

using namespace std;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "sed_hk_client");
    ros::NodeHandle nh;

    ros::ServiceClient client;
    ROS_INFO("进入客户端");
    HKcamInterface hkcam;
    vector<sensor_msgs::ImagePtr> pImage = hkcam.fetchFrame();
    while(pImage.empty()){
        pImage = hkcam.fetchFrame();
    }
    sensor_msgs::CameraInfo camInfo = hkcam.getHKCameraInfo();

    client = nh.serviceClient<sed_hk::image>("imgProcess");

    // *** 组织请求数据 ***
    sed_hk::image msgs;

    msgs.request.rgb = *pImage[0];
    msgs.request.depth = *pImage[1];
    msgs.request.camInfo = camInfo;

    // *** 优化启动顺序，即可以先启动客户端，使其挂起等候服务器启动 ***
    // client.waitForExistence(); // 第一种方法
    ros::service::waitForService("imgProcess"); // 第二种方法，需要调用ros::service中的waitForServicce()函数，参数为话题

    ros::Rate rate(30);

    // *** 发送请求，返回bool值进行判断 ***
    bool flag = client.call(msgs); // client.call()函数表示客户端发送了消息之后，返还的消息包被存在()内的值
    if(flag)
    {   
        // *** 循环发布请求 ***
        while(flag)
        {
            // *** 调用图像采集函数 ***
            pImage = hkcam.fetchFrame();
            camInfo = hkcam.getHKCameraInfo();
            msgs.request.rgb = *pImage[0];
            msgs.request.depth = *pImage[1];
            msgs.request.camInfo = camInfo;
            flag = client.call(msgs); // client.call()函数表示客户端发送了消息之后，返还的消息包被存在()内的值
            rate.sleep();
            ros::spinOnce();
        }
    }
    else{
        ROS_INFO("接收失败...");
    }

    return 0;
}