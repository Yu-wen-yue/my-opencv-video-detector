#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

// 一根灯条的信息
struct LightBar {
    cv::RotatedRect rect;   // 灯条的最小外接旋转矩形
    cv::Point2f top;        // 灯条上端点（图像中 y 较小的点）
    cv::Point2f bottom;     // 灯条下端点（图像中 y 较大的点）
    float length = 0.0f;    // 灯条长度（旋转矩形的长边）
    float angle = 0.0f;     // 灯条长轴相对“竖直方向”的夹角，单位：度
};

// 一块装甲板的信息（由左右两根灯条组成）
struct Armor {
    cv::Point2f corners[4]; // 按顺序：左上、右上、右下、左下
    cv::Point2f center;     // 装甲板中心
    bool valid = false;     // 是否成功配对
};

// 第一步~第三步：在一帧图像中找出所有灯条，并计算它们的上下端点。
// mask 是输出的二值图，方便在窗口里观察二值化效果。
std::vector<LightBar> findLightBars(const cv::Mat& frame, cv::Mat& mask);

// 第四步：从灯条列表中挑出最像“一块装甲板”的左右两根灯条。
bool pairLightBars(const std::vector<LightBar>& bars, Armor& armor);

// 第五步：把装甲板的四个端点按顺序连成闭合矩形并绘制出来。
void drawArmor(cv::Mat& frame, const Armor& armor);

#endif // DETECTOR_H
