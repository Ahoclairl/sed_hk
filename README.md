# SED_HK

**适用平台：LINUX系统下的ROS平台**

基于海康 MV-EB435i 型号 RGB-D 相机 SDK 二次开发的**ROS平台下落差环境边缘检测(Staircase Environment Detector)算法说明**

该算法主要功能：

a. 将 MV-EB435i 相机采集的**彩色图**和**深度图**分别以 **720p** 和 **360p** 分辨率发布到ROS平台

b. 在地面深度信息部分丢失的条件下完成落差边缘计算

c. 通过深度差算法与直线拟合检测与地面有一定垂直关系的障碍物**直线边缘**

d. 通过深度差算法检测与地面有一定垂直关系的障碍物**曲线边缘**

具体算法原理参见：

![Example](https://github.com/Ahoclairl/sed_hk/blob/master/pic/%E5%AE%9E%E6%B5%8B.png "边缘检测实例")

## 1.安装步骤

a. 打开安装包，将主目录 *“/sed_hk”* 添加进工作空间下的 *“src”* 文件夹

b. 设置主目录中的 *CMakeLists.txt* 文件，检查 *cv_bridge* 文件夹路径和 *Open_CV* 版本号

c. 回到工作目录，打开终端进行编译

    ```
    catkin_make
    ```

## 2.启动步骤

安装完成后将 MV-EB435i 通过 *USB3.0* 数据线连接电脑，输入以下代码检测设备：

    ```
    lsusb
    ```

检测到设备后**先进入工作空间，启动相应 launch 文件**：

    ```
    roslaunch sed_hk sed_hk.launch
    ```

看到提示语后再在工作目录下新建终端，**并以 root 权限进入终端**，输入以下代码运行程序：

    ```
    source ./devel/setup.bash
    rosrun sed_hk sed_hk_client
    ```

**当需要关闭程序时，先在客户端终端下按 *c* 或 *q*，待相机和客户端停止运作后再关闭服务器**

## 3.使用说明

修改 *config/cam.yaml: flag_largeImg* 可以切换 **720p / 360p** 图像模式，**true** 为 **720p**，**false** 为 **360p**

**在 *config/cam.yaml* 中修改分辨率后，还需要修改 *config/config.yaml* 中对应的图像及目标框参数**

修改 *config/config.yaml: test_line* 可以切换 **曲线 / 直线** 检测模式，**true** 为直线检测，**false** 为曲线检测

修改 *config/config.yaml: m_biasX / m_biasY* 可以修正深度图与彩色图检测点的偏移

## 4.注意事项

a. 运行程序前要**确保深度图和彩色图的尺寸相同**，可以进入 *“/sed_hk/config/config.yaml”* 修改全局变量中图片与目标框的尺寸

b. **该算法需要保证相机与墙面保持一定平行度**

c. 存在由于光照条件或曝光原因导致有效深度 (*如阶梯边缘*) 丢失而造成的检测偏移