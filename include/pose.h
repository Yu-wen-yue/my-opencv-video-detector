#ifndef POSE_H
#define POSE_H

#include <array>

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

    // 下面三个角都是“相对视线”的量：0 度 = 装甲板正对相机，
    // 与装甲板在画面里的位置无关（以前直接拿法向对相机光轴取角，
    // 板子偏在画面一侧时即使正对相机也会读出一个偏角）。
    double yaw = 0.0;   // 水平偏角：正面法向朝画面右侧偏为正，单位度
    double pitch = 0.0; // 竖直偏角：正面法向朝画面上方偏为正，单位度
    double roll = 0.0;  // 画面内的倾斜角：顺时针为正（板上方朝右倒），单位度
};

// 返回题目给出的相机参数（内参 + 畸变）。
CameraParams makeCameraParams();

// 用装甲板的四个端点做 PnP 解算。成功返回 true，失败时 pose.valid 为 false。
bool solveArmorPose(const Armor& armor, const CameraParams& cam, PoseResult& pose);

// 以装甲板中心为原点画 FLU 三维坐标轴：x 红（板前方）、y 绿（板左侧）、z 蓝（板上方）。
void drawPoseAxes(cv::Mat& frame, const PoseResult& pose, const CameraParams& cam);

// ---------------------------------------------------------------------------
// 角度读数的平滑（只影响显示的数字，不影响坐标轴）。
//
// 为什么需要它：装甲板很小（远处只有 40x16 像素）、又基本正对相机时，
// yaw / pitch 这两个量几乎不可观测 —— 四个角点差 0.5~1 个像素，解出来的角度
// 就会跳几度到几十度，而且平面四点 PnP 本身还有两个“投影完全一样”的解，
// 相邻帧会在两个解之间来回跳。距离由整体尺度决定，所以距离读数不受影响。
//
// 这里用最近 kWindow 帧的**中值**滤波：中值比平均更抗这种整块的跳变。
// 代价是约 (kWindow-1)/2 帧的延迟（25 FPS、窗口 5 -> 约 0.08 秒）。
// ---------------------------------------------------------------------------
class AngleFilter {
public:
    // 丢目标时调用，避免把上一次的结果和这一次混在一起。
    void reset() {
        count_ = 0;
        next_ = 0;
    }

    // 传入一帧的解算结果，就地把它里面的三个角度换成滤波后的值。
    void apply(PoseResult& pose);

private:
    static constexpr int kWindow = 5; // 取最近 5 帧

    // 把缓冲区里有效的前 count 个数排序后取中位数。
    static double median(std::array<double, kWindow> values, int count);

    std::array<double, kWindow> yaw_{};
    std::array<double, kWindow> pitch_{};
    std::array<double, kWindow> roll_{};
    int count_ = 0; // 已经存了几个有效值（最多 kWindow 个）
    int next_ = 0;  // 下一个写的位置
};

#endif // POSE_H
