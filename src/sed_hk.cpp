#include "../include/sed_hk.h"

// *** 配置路径 ***
string CONFIG_PATH;
string PRE_PATH;

// *** 相机话题　***
string rgb_topic;
string depth_topic;

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

// *** 构造函数 ***
Sed_hk::Sed_hk():it_(nh_){
    initialProcess();
    rgbSub = it_.subscribe(rgb_topic, 1,
        &Sed_hk::rgbCb, this);
    depthSub = it_.subscribe(depth_topic, 1,
        &Sed_hk::depthCb, this);
}

// *** 析构函数 ***
Sed_hk::~Sed_hk(){}

// *** 初始化函数 ***
void Sed_hk::initialProcess(){
    // *** 配置文件检测 ***
    PRE_PATH = ros::package::getPath("sed_hk");
    nh_.setParam("prePath", PRE_PATH);
    CONFIG_PATH = PRE_PATH + "/config/config.yaml";
    ifstream readConfig(CONFIG_PATH);
    if(!readConfig.is_open()){
        cout << "配置文件路径错误！" << endl;
        return;
    }
    
    YAML::Node config = YAML::LoadFile(CONFIG_PATH);
    rgb_topic = config["rgb_topic"].as<string>();
    depth_topic = config["depth_topic"].as<string>();
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


// *** 彩色图回调函数　***
void Sed_hk::rgbCb(const sensor_msgs::ImageConstPtr &msg){
    try
    {
        // *** 与发布话题时的编码 "rgb8" 相反，这里的编码选择 "BGR8" ***
        rgbPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("rgbCb Error! %s", e.what());
    }
    // *** roi ***
    cv::rectangle(rgbPtr->image, cv::Rect(roi_x, roi_y, roi_width, roi_height), cv::Scalar(0, 255, 0), 1);
    // *** 相机中线 ***
    cv::line(rgbPtr->image, cv::Point(img_width / 2, roi_y), cv::Point(img_width / 2, roi_y + roi_height), cv::Scalar(255, 0, 0), 1);
    cv::imshow("RGB_WINDOW", rgbPtr->image);
    cv::waitKey(1);
}

// *** 深度图回调函数　***
void Sed_hk::depthCb(const sensor_msgs::ImageConstPtr &msg){
    try
    {
        depthPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("depthCb Error! %s", e.what());
    }
    cv::imshow("DEPTH_WINDOW", 10 * depthPtr->image);
    cv::waitKey(1);
    // *** 配合主函数提示信息，按 's' 检测 ***
    if(cv::waitKey(33) == 's'){
        depthProcess();
    }
}

// *** 深度图像处理函数 ***
void Sed_hk::depthProcess(){
    rgbPtr->image.copyTo(m_rgbImage);
    depthPtr->image.copyTo(m_depthImage);
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
void Sed_hk::edgeDetect(int ry){
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
vector<int> Sed_hk::pointAround(vector<int> &edge, bool &flag, int &count, int &ry, int &num, int spX){
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

// *** 直线拟合函数 ***
vector<int> Sed_hk::lineProcess(){
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
void Sed_hk::paint(){
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
        // *** 输出深度图 ***
        cv::imshow("DEPTH", 10 * m_depthImage(cv::Rect(roi_x, roi_y, roi_width, roi_height)));
        // *** 输出边缘线 ***
        cv::imshow("EDGE", m_rgbImage(cv::Rect(roi_x, roi_y, roi_width, roi_height)));
    }
}
