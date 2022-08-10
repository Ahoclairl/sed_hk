// *** 导入ros及服务消息相关包 ***
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "../include/sed_hk.h"
#include "../common/common.hpp"
#include "sed_hk/image.h"

using namespace std;

image_transport::CameraPublisher rgbPub;
image_transport::CameraPublisher depthPub;

// 4. 设置回调函数
bool imageProcess(sed_hk::image::Request &request, sed_hk::image::Response &response)
{
    // sensor_msgs::CameraInfo camInfo = getHKCameraInfo();
    rgbPub.publish(request.rgb, request.camInfo);
    depthPub.publish(request.depth, request.camInfo);

    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2. 初始化ros节点
    ros::init(argc, argv, "sed_hk_server");
    ros::NodeHandle nh;
    // 3. 建立服务器
    ros::ServiceServer server;
    server = nh.advertiseService("imgProcess", imageProcess);
    // 5. 发布话题
    image_transport::ImageTransport it(nh);
    string rgbTopic = "/hkcamera/rgb";
    string depthTopic = "/hkcamera/depth";
    rgbPub = it.advertiseCamera(rgbTopic, 1);
    depthPub = it.advertiseCamera(depthTopic, 1);
    ROS_INFO("———————————— 进入 sed_hk 图像检测");
    Sed_hk sed;
    if(sed.isRT()){
        ROS_INFO("———————————— 订阅话题成功，开始实时检测");
    }else{
        ROS_INFO("———————————— 订阅话题成功，在图形界面按 's' 键开始检测");
    }
    // 6. spin()
    ros::spin();
    return 0;
}
