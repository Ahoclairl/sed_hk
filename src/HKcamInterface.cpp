#include "../include/HKcamInterface.h"
#include "../common/common.hpp"
#include "../common/RenderImage.hpp"
#include "../yaml-cpp/yaml.h"

// *** 相机相关变量 ***
RIFrameInfo depth   = { 0 };
RIFrameInfo rgb     = { 0 };

MV3D_RGBD_FRAME_DATA stFrameData{0};

// *** 构造函数 ***
HKcamInterface::HKcamInterface(string &PRE_PATH){
    m_configPath = PRE_PATH + "/config/cam.yaml";
    // *** 读取配置参数 ***
    YAML::Node config = YAML::LoadFile(m_configPath);
    m_imgSizeL = config["m_imgSizeL"].as<double>();
    m_imgSizeS = config["m_imgSizeS"].as<double>();
    m_exposureTime = config["m_exposureTime"].as<float>();
    m_flagLargeImg = config["m_flagLargeImg"].as<bool>();

    MV3D_RGBD_VERSION_INFO stVersion;
    ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion) );
    LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);
    ASSERT_OK(MV3D_RGBD_Initialize());

    unsigned int nDevNum = 0;

    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum));
    LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
    ASSERT(nDevNum);

    std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++)
    {  
        LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName);
    }
    LOG("---------------------------------------------------------------");

    unsigned int nIndex  = 0;
    while (true)
    {
        LOG("Please enter the index of the camera to be connected��\n");
        scanf("%d",&nIndex);
        LOG("Connected camera index:%d \r\n", nIndex);

        if ((nDevNum  <= nIndex) || (0 > nIndex))
        {
            LOG("enter error!\r\n");
        }
        else
        {
            break;
        }
    }
    LOG("---------------------------------------------------------------\r\n");

    // *** 打开相机 ***
    ASSERT_OK(MV3D_RGBD_OpenDevice(&m_handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    // *** 重设相机分辨率 ***
    MV3D_RGBD_PARAM pstValue;

    pstValue.enParamType = ParamType_Enum;
    pstValue.ParamInfo.stEnumParam.nCurValue = m_flagLargeImg == true ? m_imgSizeL : m_imgSizeS;
    MV3D_RGBD_SetParam(m_handle, MV3D_RGBD_ENUM_RESOLUTION, &pstValue);
    LOGD("Resize success.");

    // *** 设置相机曝光参数 ***
    pstValue.enParamType = ParamType_Float;
    pstValue.ParamInfo.stFloatParam.fCurValue = m_exposureTime;
    MV3D_RGBD_SetParam(m_handle, MV3D_RGBD_FLOAT_EXPOSURETIME, &pstValue);

    // *** 开始图像采集 ***
    ASSERT_OK(MV3D_RGBD_Start(m_handle));
    LOGD("———————————— 设备开始工作，按 'c' 或 'q' 关闭设备");

    m_flagOnce = true;
    m_flagStop = false;
}

// *** 析构函数 ***
HKcamInterface::~HKcamInterface(){

}

// *** 获取图像 ***
vector<sensor_msgs::ImagePtr> HKcamInterface::fetchFrame(){
    if(m_flagOnce && !m_flagStop){
        LOGD("———————————— 相机初始化完成，开始获取图像");
        m_flagOnce = false;
    }

    // *** 图像信息 ***
    m_nRet = MV3D_RGBD_FetchFrame(m_handle, &stFrameData, 500);

    if (MV3D_RGBD_OK == m_nRet)
    {
        parseFrame(&stFrameData, &depth, &rgb);
        cv::Mat rgbImg = cv::Mat(cv::Size(stFrameData.stImageData->nWidth, stFrameData.stImageData->nHeight), CV_8UC3, rgb.pData);
        cv::Mat depthImg = cv::Mat(cv::Size(stFrameData.stImageData->nWidth, stFrameData.stImageData->nHeight), CV_16UC1, depth.pData);
        
        /* 流行的图像编码
            mono8：CV_8UC1，即8位单通道（灰度图）
            mono16：CV_16UC1，即16位单通道（灰度图）
            bgr8：CV_8UC3，即8位蓝绿红色序彩色图
            rgb8：CV_8UC3，即8位红绿蓝色序彩色图
            brga8：CV_8UC4，即带有alpha通道的8位BGR彩色图
            rcba8：CV_8UC4，即带有alpha通道的8位RGB彩色图
        */

        m_pRgb = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgbImg).toImageMsg();
        m_pDepth = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthImg).toImageMsg();

        if (_kbhit())
        {
            if ('c' == _getch() || 'q' == _getch())
            {
                LOGD("recieve exit cmd!");
                m_flagStop = true;
            }
        }
        if(m_flagStop){
            stop();
        }
        return {m_pRgb, m_pDepth};
    }
    return {};
}

sensor_msgs::CameraInfo HKcamInterface::getHKCameraInfo(){
    sensor_msgs::CameraInfo cam;
    MV3D_RGBD_CALIB_INFO cam_info;
    // *** 相机畸变系数 ***
    vector<double> D;
    for(double s : cam_info.stDistortion.fData){
        D.push_back((double)(s));
    }
    // *** 相机内参 ***
    boost::array<double, 9> K = {
        (double)(cam_info.stIntrinsic.fData[0]), (double)(cam_info.stIntrinsic.fData[1]), (double)(cam_info.stIntrinsic.fData[2]),
        (double)(cam_info.stIntrinsic.fData[3]), (double)(cam_info.stIntrinsic.fData[4]), (double)(cam_info.stIntrinsic.fData[5]),
        (double)(cam_info.stIntrinsic.fData[6]), (double)(cam_info.stIntrinsic.fData[7]), (double)(cam_info.stIntrinsic.fData[8]),
    };
    // *** 投影矩阵 ***
    boost::array<double, 12> P = {
        (double)(cam_info.stIntrinsic.fData[0]), (double)(cam_info.stIntrinsic.fData[1]), (double)(cam_info.stIntrinsic.fData[2]), 0.000000,
        (double)(cam_info.stIntrinsic.fData[3]), (double)(cam_info.stIntrinsic.fData[4]), (double)(cam_info.stIntrinsic.fData[5]), 0.000000,
        (double)(cam_info.stIntrinsic.fData[6]), (double)(cam_info.stIntrinsic.fData[7]), (double)(cam_info.stIntrinsic.fData[8]), 0.000000
    };
    // *** 旋转矩阵 ***
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam.width = m_flagLargeImg == true ? 1280 : 640;
    cam.height = m_flagLargeImg == true ? 720 : 360;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;
    cam.header.frame_id = "hk_cam";
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    return cam;
}

void HKcamInterface::stop(){
    ASSERT_OK(MV3D_RGBD_Stop(m_handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&m_handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
}
