#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH; // 焦距
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK; // 鱼眼相机mask图的位置
extern std::vector<std::string> CAM_NAMES; //相机参数配置文件名
extern int MAX_CNT; // 特征点最大个数
extern int MIN_DIST; // 特征点之间最小间隔
extern int MIN_DIST_INITIALIZED;
extern int WINDOW_SIZE; 
extern int FREQ; // 控制图像光流跟踪的频率
extern double F_THRESHOLD; // ransac的门限
extern int SHOW_TRACK; // 是否发布跟踪点的图像
extern int STEREO_TRACK; // 双目跟踪=1
extern int EQUALIZE; // 若光太亮或太暗则为1，进行直方图均衡化
extern int FISHEYE; // 鱼眼相机=1
extern bool PUB_THIS_FRAME; // 是否发布特征点

void readParameters(ros::NodeHandle &n);
