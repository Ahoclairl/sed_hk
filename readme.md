# SED_HK

基于海康 MV-EB435i 型号 RGB-D 相机 SDK 二次开发的**落差环境边缘检测(Staircase Environment Detector)**算法说明

![Example](https://github.com/Ahoclairl/sed_hk/blob/master/pic/%E5%AE%9E%E6%B5%8B.png) 
## 1.安装步骤

a. 解压后打开，进入主目录 */sed_hk*

b. 在主目录下打开终端

c. 编译

    ```
    mkdir build

    cd build

    cmake ..

    make
    ```

安装完成后将 MV-EB435i 通过 **USB3.0** 数据线连接电脑，输入以下代码检测设备：

    ```
    lsusb
    ```

检测到设备后在主目录下**以 root 权限进入终端**，输入以下代码运行程序：

    ```
    ./build/sed
    ```

出现如下提示即可顺利运行程序：

![test](https://github.com/Ahoclairl/sed_hk/blob/master/pic/%E4%BB%A3%E7%A0%81%E6%B5%8B%E8%AF%95.png)
## 2.使用说明

算法原理参见：

修改 *test_line* 可以切换**曲线 / 直线**检测模式，*true* 为直线检测，*false* 为曲线检测

修改 *m_biasX / m_biasY* 可以修正深度图与彩色图检测点的偏移
## 3.注意事项

a. 运行程序前要**确保深度图和彩色图的尺寸相同**，可以进入 */sed_hk/config/config.yaml* 修改全局变量中图片与目标框的尺寸

b. **该算法需要保证相机与墙面保持一定平行度**

c. 存在由于光照条件或曝光原因导致有效深度*（如阶梯边缘）*丢失而造成的检测偏移