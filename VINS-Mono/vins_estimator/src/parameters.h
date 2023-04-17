#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH; // 深度初始值
extern double MIN_PARALLAX; // 关键帧选择阈值 （单位：像素）
extern int ESTIMATE_EXTRINSIC; // imu和camera的外参是否准确：准确，不准确，没有

// 加速度计和陀螺仪噪声和随机偏置标准差
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC; // 从camera到imu的旋转矩阵
extern std::vector<Eigen::Vector3d> TIC; // 从camera到imu的平移向量
extern Eigen::Vector3d G; // 重力

extern double BIAS_ACC_THRESHOLD; // Ba阈值
extern double BIAS_GYR_THRESHOLD; // Bg阈值
extern double SOLVER_TIME; // 最大解算时间 （保证实时性）
extern int NUM_ITERATIONS; // 最大解算器迭代次数
extern std::string EX_CALIB_RESULT_PATH; // camera与imu外参的输出路径 extrinsic_parameter.csv
extern std::string VINS_RESULT_PATH; // vins_result_no_loop.csv
extern std::string IMU_TOPIC; // imu topic
extern double TD; // IMU和camera的时间差. unit: s. readed image clock + td = real image clock (IMU clock)
extern double TR; // 每帧时间
extern int ESTIMATE_TD; // 在线校准imu和camera时间
extern int ROLLING_SHUTTER; // 1：卷帘快门相机；0：全局快门相机
extern double ROW, COL; // 图片的宽高


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
