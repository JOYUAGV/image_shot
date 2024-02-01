## 背景
为了在ros上实现调用外部摄像头完成图像采集，实现跟手机、相机等相同的拍照功能，特基于ros的话题通讯机制，开发一个功能包（image_shot），通过订阅相机话题，采用键盘按键（回车键）交互，对图像（包括rgb、depth）进行采集！
## 环境与依赖
**系统环境依赖: Linux + ROS**
  
  推荐:
  
  Ubuntu 18.04 - 装有ROS melodic desktop-full 或者 Ubuntu 20.04 - 装有ROS noetic desktop-full (安装详见[http://ros.org](http://ros.org))

**依赖的库: libopencv-dev** 
```
sudo apt-get install libopencv-dev
```

## 下载与编译
**克隆代码**
```
cd rosworkspace/src 
git clone https://github.com/JOYUAGV/image_shot.git
```
**编译**
```
cd ..
catkin_make
```
## 参数配置

为保证功能包顺利采集图像，请对文件"config/image_shot.yaml"进行配置！

|参数    |功能                       |默认值|
|-------------|-------------------------------|------------------------|
|rgbTopicName |rgb图像话题名 |/camera/color/image_raw |
|depthTopicName |depth图像话题名 |/camera/aligned_depth_to_color/image_raw |
|rgb_prefix |rgb图像命名前缀 |rgb_ |
|depth_prefix |深度图像命名前缀|dep_ |
|data_set |所拍摄的数据集文件夹名 |test |
|delta_stamp |用于判断rgb与深度图像是否为同一帧的时间差阈值 (单位：Sec) |0.001 |
|queue_depth |订阅话题的缓存大小 |100 |
|control_frequency |用于检测按键扫描的频率 (单位：Hz) |50.0 |

## 运行
**运行之前**

请启动你的web相机或rgbd相机并发布相应的rgb或者深度图像话题，以下为相关例子：

Case 0: 启动web相机 （以笔记本自带相机为例，驱动安装详见[https://github.com/ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam)）
```
roslaunch usb_cam usb_cam_bringup.launch
```
Case 1: 启动rgbd相机（以realsense d435系列相机为例，驱动安装详见[https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)）
```
roslaunch realsense2_camera rs_aligned_depth.launch
```
**启动该节点**

现在你可以启动该节点并按下“回车键”采集相应的图片！
```
roslaunch image_shot image_shot.launch
```
**注意**: 在你完成图像采集之前，请不要随意点击你的鼠标，否则，你将无法成功采集到任何图像！
## 结果

请在如下文件夹中查看你所采集的图像 "image_shot/image_sets/data_set"！