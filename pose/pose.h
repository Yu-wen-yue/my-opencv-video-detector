#ifndef POSE_H
#define POSE_H

#include <opencv2/opencv.hpp>

#include "detector.h"

// 相机内参和畸变系数（题目直接给的，见培训文档“选做部分”）。
struct CameraParams {
    cv::Mat cameraMatrix; // 3x3 内参矩阵 K
    cv::Mat distCoeffs;   // 1x5 畸变系数 [k1, k2, p1, p2, k3]
};

// 一次 PnP 解算的结果。
struct PoseResult {
    bool valid = false;
    cv::Vec3d rvec;           // 旋转向量：把装甲板坐标系转到相机坐标系
    cv::Vec3d tvec;           // 平移向量：装甲板中心在相机坐标系下的坐标，单位 mm
    double reprojError = 0.0; // 四个端点的平均重投影误差，单位像素
    double yaw = 0.0;         // 装甲板正面法向的水平偏角，单位度
    double pitch = 0.0;       // 装甲板正面法向的竖直偏角，单位度
    double roll = 0.0;        // 装甲板在画面内的倾斜角，单位度
};

// 返回题目给出的相机参数（内参 + 畸变）。
CameraParams makeCameraParams();

// 用装甲板的四个端点做 PnP 解算。成功返回 true，失败时 pose.valid 为 false。
bool solveArmorPose(const Armor& armor, const CameraParams& cam, PoseResult& pose);

// 以装甲板中心为原点画 FLU 三维坐标轴：x 红（板前方）、y 绿（板左侧）、z 蓝（板上方）。
void drawPoseAxes(cv::Mat& frame, const PoseResult& pose, const CameraParams& cam);

#endif // POSE_H
