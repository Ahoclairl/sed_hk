#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/yaml-cpp/yaml.h"
#include "../common/common.hpp"
#include "../common/RenderImage.hpp"

static string CONFIG_PATH = "config/config.yaml";

// *** 图尺寸 ***
int img_width;
int img_height;

// *** 目标框尺寸 ***
int roi_x;
int roi_y;
int roi_width;
int roi_height;

// *** 直线检测标记 ***
bool test_line;

// *** 实时图 ***
cv::Mat rgb_image;
cv::Mat depth_image;

class hkcamInterface{
public:
    // *** 构造函数 ***
    hkcamInterface(){
        initialProcess();
    }

    // *** 析构函数 ***
    ~hkcamInterface(){}
    
    // *** 初始化函数 ***
    void initialProcess(){
        // *** 配置文件检测 ***
        ifstream readConfig(CONFIG_PATH);
        if(!readConfig.is_open()){
            cout << "配置文件路径错误！" << endl;
            return;
        }
        
        YAML::Node config = YAML::LoadFile(CONFIG_PATH);
        img_width = config["img_width"].as<int>();
        img_height = config["img_height"].as<int>();
        roi_x = config["roi_x"].as<int>();
        roi_y = config["roi_y"].as<int>();
        roi_width = config["roi_width"].as<int>();
        roi_height = config["roi_height"].as<int>();
        test_line = config["test_line"].as<bool>();
        m_deVal = config["m_deVal"].as<float>();
        m_gapRow = config["m_gapRow"].as<int>();
        m_gapCol = config["m_gapCol"].as<int>();
        m_gapVal = config["m_gapVal"].as<int>();
        m_biasCol = config["m_biasCol"].as<int>();
        m_numPoint = config["m_numPoint"].as<int>();
        m_biasX = config["m_biasX"].as<int>();
        m_biasY = config["m_biasY"].as<int>();
        m_param = config["m_param"].as<double>();
        m_reps = config["m_reps"].as<double>();
        m_aeps = config["m_aeps"].as<double>();

        m_flagEdge = false;
        m_sp = vector<int>(2);
        m_numL = 0;
        m_numR = 0;
        m_edgeL = vector<int>(m_numPoint);
        m_edgeR = vector<int>(m_numPoint);
        m_line = vector<int>(4);

        m_binaryImage = cv::Mat::zeros(cv::Size(roi_width, roi_height), CV_8UC1);
    }

    // *** 图像输入函数 ***
    void inputProcess(cv::Mat &rgbImg, cv::Mat &depthImg){
        rgbImg.copyTo(m_rgbImage);
        depthImg.copyTo(m_depthImage);
    }

    // *** 深度图像处理函数 ***
    void depthProcess(){
        // *** 调整至标准尺寸 ***
        cv::resize(m_depthImage, m_depthImage, cv::Size(img_width, img_height), CV_INTER_LINEAR);
        // *** 计算起始点 ***
        bool flagSp = true;
        int count = 0;
        int ry = 0;
        int num = 0;
        m_sp = pointAround(m_edgeL, flagSp, count, ry, num, roi_width / 2);
        if(test_line){
            cout << "———————————— 单帧直线检测完成" << endl;
        }else{
            cout << "———————————— 单帧曲线检测完成" << endl;
        }
        // *** 计算边缘点 ***
        edgeDetect(m_sp[1] - roi_y);
        cv::imshow("BINARY", m_binaryImage);
        m_flagEdge = true;
        if(test_line){
            m_line = lineProcess();
        }
        paint();
        initialProcess();
    }

    // *** 左右遍历函数 ***
    void edgeDetect(int ry){
        m_edgeL[0] = m_sp[0];
        m_edgeL[1] = m_sp[1];
        m_edgeR[0] = m_sp[0];
        m_edgeR[1] = m_sp[1];
        // *** 左右延伸时参考的纵坐标 ***
        int rLy = ry;
        int rRy = ry;
        // *** 边界计数器，用来计算间隔数，超过 m_gapVal 即认为到达边界 ***
        int countL = 0;
        int countR = 0;
        // *** 误差带边界 ***
        vector<int> xL;
        vector<int> xR;
        // *** 循环外左右限位标记 ***
        bool flagL = true;
        bool flagR = true;
        // *** 大循环从矩形图中心开始向左右延伸 ***
        for(int i = m_gapRow; i < roi_width / 2; i += m_gapRow){
            // *** 一旦循环外左右限位标记都置否，说明无法再延伸，即刻跳出大循环 ***
            if(!flagL && !flagR){
                break;
            }
            // *** 自下往上判断，一旦形成一条误差带，立刻向两旁跳转 ***
            if(flagL){
                // *** 调用 单点纵坐标遍历函数 来确定左右端点 ***
                xL = pointAround(m_edgeL, flagL, countL, rLy, m_numL, roi_width / 2 - i);
                if(m_numL == 40){
                    flagL = false;
                }
            }
            if(flagR){
                xR = pointAround(m_edgeR, flagR, countR, rRy, m_numR, roi_width / 2 + i);
                if(m_numR == 40){
                    flagR = false;
                }
            }
        }
        // *** 在 m_binaryImage 灰度图上绘制中线 ***
        cv::line(m_binaryImage, cv::Point(roi_width / 2, 0), cv::Point(roi_width / 2, roi_height), cv::Scalar(255), 2);
        return;
    }

    // *** 纵坐标遍历函数 ***
    vector<int> pointAround(vector<int> &edge, bool &flag, int &count, int &ry, int &num, int spX){
        // *** 初始化纵坐标范围 ***
        int start = ry + 20 + 2 * count < roi_height ? ry + 20 + 2 * count : roi_height - 1;
        int end = ry - 20 - 2 * count > 0 ? ry - 20 - 2 * count : 0;
        if(spX == roi_width / 2){
            start = roi_height;
            end = 0;
        }
        // *** 图像指针 ***
        ushort *pA;
        ushort *pB;
        uchar *pC;
        for(int j = start; j >= end; j -= m_gapCol){
            pA = m_depthImage.ptr<ushort>(roi_y + j - m_biasCol);
            pB = m_depthImage.ptr<ushort>(roi_y + j);
            // *** depth 为当前坐标下的深度值 ***
            float depthA = 0.001 * pA[roi_x + spX];
            float depthB = 0.001 * pB[roi_x + spX];
            // *** 如果有任何数据为 0, 直接跳过本轮循环 ***
            if(!depthA || !depthB){
                // cout << "存在无效数据" << endl;
                continue;
            }
            // *** dErr 为当前坐标下的有效深度差 ***
            float dErr = depthA - depthB;
            // cout << dErr << endl;
            if(dErr < m_deVal){
                // *** 间隔计数置 0 ，重新开始计数 ***
                count = 0;
                ry = j;
                // *** 储存 m_edge 信息 ***
                if(spX != roi_width / 2 && ((spX > roi_width / 2 && spX - m_biasX > roi_width / 2) || spX < roi_width / 2)){
                    num++;
                    edge[2 * num] = roi_x + spX - m_biasX;
                    edge[2 * num + 1] = roi_y + j - m_biasY;
                }
                // *** 在 m_binaryOutput 上绘制误差带 ***
                for(int k = 0; k < 5; k++){
                    if(j - k >= 0 && j - k < roi_height){
                        pC = m_binaryImage.ptr<uchar>(j - k);
                        pC[spX] = 255;
                        pC[spX - m_gapRow / 2] = 255;
                        pC[spX + m_gapRow / 2] = 255;
                    }else{
                        break;
                    }
                }
                return {roi_x + spX, roi_y + j};
            }

            if(j == end || spX < 30 || spX > roi_width - 30){
                count++;
                if(count == m_gapVal || spX < 30 || spX > roi_width - 30){
                    // *** 当间隔计数到达阈值 m_gapVal ，可认为达到端点，该方向循环停止 ***
                    flag = false;
                    return {roi_x + spX, roi_y + j};
                }
            }
        }
        return {-1, -1};
    }

    vector<int> lineProcess(){
        vector<cv::Point2f> points;
        cv::Vec4f lines;
        for(int i = 0; i < m_numL; i++){
            points.push_back(cv::Point2f(m_edgeL[2 * i], m_edgeL[2 * i + 1]));            
        }
        for(int i = 1; i < m_numR; i++){
            points.push_back(cv::Point2f(m_edgeR[2 * i], m_edgeR[2 * i + 1]));            
        }

        cv::fitLine(points, lines, cv::DIST_L2, m_param, m_reps, m_aeps);

        // y = k * x + b;
        // x = (y - b) / k;
        // b = y - k * x;
        
        float k = lines[1] / lines[0];
        float b = lines[3] - (k * lines[2]);
        //线段起点x
        int x1 = m_edgeL[2 * m_numL];
        int y1 = k * x1 + b;
        //线段终点x
        int x2 = m_edgeR[2 * m_numR];
        int y2 = k * x2 + b;
        return {x1, y1, x2, y2};
    }

    // *** 绘图函数 ***
    void paint(){
        // *** roi ***
        cv::rectangle(rgb_image, cv::Rect(roi_x, roi_y, roi_width, roi_height), cv::Scalar(0, 255, 0), 1);
        // *** 相机中线 ***
        cv::line(rgb_image, cv::Point(img_width / 2, roi_y), cv::Point(img_width / 2, roi_y + roi_height), cv::Scalar(255, 0, 0), 1);
        // *** 边缘线　***
        if(m_flagEdge){
            cv::circle(m_rgbImage, cv::Point(m_sp[0], m_sp[1]), 3, cv::Scalar(0, 0, 255), -1);
            if(test_line){
                cv::line(m_rgbImage, cv::Point(m_line[0], m_line[1]), cv::Point(m_line[2], m_line[3]), cv::Scalar(255, 0, 0), 2);
                cv::circle(m_rgbImage, cv::Point(m_line[0], m_line[1]), 3, cv::Scalar(0, 0, 255), -1);
                cv::circle(m_rgbImage, cv::Point(m_line[2], m_line[3]), 3, cv::Scalar(0, 0, 255), -1);
            }else{
                for(int i = 1; i < m_numL; i++){
                    if(m_edgeL[2 * i] != -1 && m_edgeL[2 * i + 2] != -1){
                        cv::line(m_rgbImage, cv::Point(m_edgeL[2 * i], m_edgeL[2 * i + 1]), cv::Point(m_edgeL[2 * i + 2], m_edgeL[2 * i + 3]), cv::Scalar(255, 0, 0), 2);
                    }
                    if(i == m_numL - 1){
                        cv::circle(m_rgbImage, cv::Point(m_edgeL[2 * i + 2], m_edgeL[2 * i + 3]), 3, cv::Scalar(0, 0, 255), -1);
                    }
                }
                for(int i = 1; i < m_numR; i++){
                    if(m_edgeR[2 * i] != -1 && m_edgeR[2 * i + 2] != -1){
                        cv::line(m_rgbImage, cv::Point(m_edgeR[2 * i], m_edgeR[2 * i + 1]), cv::Point(m_edgeR[2 * i + 2], m_edgeR[2 * i + 3]), cv::Scalar(255, 0, 0), 2);
                    }
                    if(i == m_numR - 1){
                        cv::circle(m_rgbImage, cv::Point(m_edgeR[2 * i + 2], m_edgeR[2 * i + 3]), 3, cv::Scalar(0, 0, 255), -1);
                    }
                }
                cv::line(m_rgbImage, cv::Point(m_edgeL[2], m_edgeL[3]), cv::Point(m_edgeR[2], m_edgeR[3]), cv::Scalar(255, 0, 0), 2);
            }
            // *** 输出边缘线 ***
            cv::imshow("EDGE", m_rgbImage(cv::Rect(roi_x, roi_y, roi_width, roi_height)));
        }
    }

private:
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

int main(int argc,char** argv)
{
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

    void* handle = NULL;

    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    ASSERT_OK(MV3D_RGBD_Start(handle));
    LOGD("Start work success.And press c or q to exit!");

    BOOL bExit_Main = FALSE;
    MV3D_RGBD_FRAME_DATA stFrameData = {0};
    BOOL flagOnce = true;
    hkcamInterface hkcam;
    while (!bExit_Main)
    {
        if(flagOnce){
            LOGD("———————————— 进入Sed_hk检测");
            LOGD("———————————— 初始化完成后按 's' 开始检测");
                
            // *** 查看内参 ***
            // MV3D_RGBD_CALIB_INFO *pstCalibInfo;
            // int test = MV3D_RGBD_GetCalibInfo(handle, 2, pstCalibInfo);
            // LOGD("GetDevice calib success.");
            // for(float s : pstCalibInfo->stIntrinsic.fData){
            //     cout << "相机内参：" << s << endl;
            // }

            flagOnce = false;
        }

        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        
        if (MV3D_RGBD_OK == nRet)
        {
            // LOGD("MV3D_RGBD_FetchFrame success.");
            RIFrameInfo depth   = { 0 };
            RIFrameInfo rgb     = { 0 };
            parseFrame(&stFrameData, &depth, &rgb);
            rgb_image = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, rgb.pData);
            depth_image = cv::Mat(cv::Size(img_width, img_height), CV_16UC1, depth.pData);
            hkcam.paint();
            cv::imshow("RGB_WINDOW", rgb_image);
            cv::imshow("DEPTH_WINDOW", depth_image * 10);
            cv::waitKey(1);
            if(cv::waitKey(33) == 's'){
                hkcam.inputProcess(rgb_image, depth_image);
                hkcam.depthProcess();

            }
        }

        if (_kbhit())
        {
            if ('c' == _getch() || 'q' == _getch())
            {
                LOGD("recieve exit cmd!");
                bExit_Main = TRUE;
            }
        }
    }

    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("Main done!");
    return  0;
}

