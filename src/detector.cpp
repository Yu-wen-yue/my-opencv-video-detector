#include "detector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

// ============================================================================
// 所有可调参数都放在这里。第一次运行时如果效果不好，优先改这几个数。
// ============================================================================

// ---- 二值化参数 ----
// 注意：这套参数是针对 videos/test.avi 这种“很暗、灯条偏橙红”的画面调的。
// 换视频后如果检测不到，优先降低 kSaturationMin / kValueMin / kBrightnessMin。
constexpr int kSaturationMin = 90;    // 饱和度下限：过滤掉白色数字、灰色背景
constexpr int kValueMin      = 40;    // 亮度下限（HSV 的 V 通道）
constexpr int kBrightnessMin = 60;    // 亮度二值化门限，越高越严格
constexpr int kRedHueMax     = 20;    // 红色/橙红色色相上限（OpenCV 的 H 范围是 0~180）

// ---- 灯条轮廓筛选参数 ----
constexpr double kMinContourArea = 10.0; // 轮廓最小面积（像素）
constexpr float  kMinAspectRatio = 2.0f; // 长宽比下限，灯条是细长的
constexpr float  kMinFillRatio   = 0.4f; // 填充率下限，灯条是实心的
constexpr float  kMaxTiltDeg     = 30.0f; // 灯条长轴与竖直方向的最大夹角
constexpr float  kMinBarLength   = 10.0f; // 灯条最短长度（像素）
constexpr float  kMinBarWidth    = 2.0f;  // 灯条最细宽度（像素）
constexpr double kMinMeanBrightness = 105.0; // 灯条区域平均亮度下限，用来排除地面反光

// ---- 两根灯条配对参数 ----
constexpr float kMaxPairAngleDiff   = 15.0f; // 两根灯条长轴夹角最大差
constexpr float kMinLengthRatio     = 0.6f;  // 短灯条/长灯条 的最小比值
constexpr float kMaxCenterYDiffRatio = 0.5f; // 中心 y 差 <= 0.5 * 平均灯条长度
constexpr float kMinGapRatio        = 1.0f;  // 两灯条中心水平距 / 平均长度 的下限
constexpr float kMaxGapRatio        = 4.0f;  // 两灯条中心水平距 / 平均长度 的上限
constexpr float kIdealGapRatio      = 2.0f;  // 最理想的间距比例，用于打分

// 求两条短边的中点时用的小工具
cv::Point2f midpoint(const cv::Point2f& a, const cv::Point2f& b) {
    return cv::Point2f((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
}

} // namespace

// ============================================================================
// 第一步 ~ 第三步：二值化 -> 找轮廓 -> 计算灯条上下端点
// ============================================================================
std::vector<LightBar> findLightBars(const cv::Mat& frame, cv::Mat& mask) {
    std::vector<LightBar> bars;
    mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    if (frame.empty()) {
        return bars;
    }

    // ------------------------------------------------------------------
    // 第一步：设置图像掩码，对图像进行二值化
    // ------------------------------------------------------------------
    // 把 BGR 转成 HSV。
    // H = 颜色（0~180），S = 饱和度（越鲜艳越大），V = 亮度（越亮越大）。
    // RoboMaster 的灯条“又亮又鲜艳”，用 S 和 V 就能把白色数字、灰色背景滤掉。
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 红色在 HSV 里跨越了 0 度，所以要分两段取，再合并。
    cv::Mat redLow, redHigh, blue, colorMask;
    cv::inRange(hsv, cv::Scalar(0,   kSaturationMin, kValueMin),
                     cv::Scalar(kRedHueMax, 255, 255), redLow);
    cv::inRange(hsv, cv::Scalar(170, kSaturationMin, kValueMin),
                     cv::Scalar(180, 255, 255), redHigh);
    // 蓝色灯条（对方机器人）
    cv::inRange(hsv, cv::Scalar(100, kSaturationMin, kValueMin),
                     cv::Scalar(130, 255, 255), blue);

    cv::bitwise_or(redLow, redHigh, colorMask); // 两段红色合并
    cv::bitwise_or(colorMask, blue, colorMask); // 再把蓝色并进来

    // 再用 cv::threshold 对 V（亮度）通道做一次二值化，
    // 然后和颜色掩码取交集：只保留“颜色够红/够蓝，而且足够亮”的像素。
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsv, hsvChannels);
    cv::Mat bright;
    cv::threshold(hsvChannels[2], bright, kBrightnessMin, 255, cv::THRESH_BINARY);
    cv::bitwise_and(colorMask, bright, mask);

    // 形态学处理：只做“闭运算”，把灯条内部的小空洞补上。
    // 这里不用“开运算”，因为视频里的灯条只有 3~5 像素宽，开运算会把灯条直接腐蚀掉。
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // ------------------------------------------------------------------
    // 第二步：识别灯条轮廓
    // ------------------------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        const double contourArea = cv::contourArea(contour);
        if (contourArea < kMinContourArea) {
            continue; // 太小的噪点，丢掉
        }
        if (contour.size() < 4) {
            continue; // 点数太少的退化轮廓，交给 minAreaRect 可能出问题
        }

        // 计算这块轮廓区域的平均亮度（V 通道）。
        // 视频里地面会映出灯条的反光，反光虽然形状也像灯条，但明显更暗。
        cv::Mat contourMask = cv::Mat::zeros(frame.size(), CV_8UC1);
        const std::vector<std::vector<cv::Point>> singleContour{contour};
        cv::drawContours(contourMask, singleContour, 0, cv::Scalar(255), cv::FILLED);
        const double meanBrightness = cv::mean(hsvChannels[2], contourMask)[0];
        if (meanBrightness < kMinMeanBrightness) {
            continue; // 太暗，多半是反光
        }

        // minAreaRect：给轮廓套一个“可以旋转的矩形”，很适合细长的灯条。
        cv::RotatedRect rect = cv::minAreaRect(contour);
        const float rectWidth  = rect.size.width;
        const float rectHeight = rect.size.height;
        if (rectWidth <= 0.0f || rectHeight <= 0.0f) {
            continue;
        }

        // OpenCV 不保证 width 和 height 哪个是长边，这里统一一下：
        // length = 长边，width = 短边。
        const float length = std::max(rectWidth, rectHeight);
        const float width  = std::min(rectWidth, rectHeight);

        if (length < kMinBarLength || width < kMinBarWidth) {
            continue;
        }
        if (length / width < kMinAspectRatio) {
            continue; // 不够细长，不是灯条
        }

        // 填充率 = 轮廓真实面积 / 外接矩形面积。
        // 灯条几乎是实心矩形，填充率接近 1；弯曲的反光、数字填充率会低很多。
        const float fillRatio = static_cast<float>(contourArea) / (rectWidth * rectHeight);
        if (fillRatio < kMinFillRatio) {
            continue;
        }

        // ------------------------------------------------------------------
        // 第三步：根据灯条轮廓计算上下两个端点的坐标
        // ------------------------------------------------------------------
        // rect.points() 按顺序返回旋转矩形的 4 个顶点：
        //   pts[0] -> pts[1] -> pts[2] -> pts[3] -> 回到 pts[0]
        // 灯条的两端，正好是“两条短边”的中点。
        cv::Point2f pts[4];
        rect.points(pts);

        const float edge01 = cv::norm(pts[0] - pts[1]);
        const float edge12 = cv::norm(pts[1] - pts[2]);

        cv::Point2f endpointA;
        cv::Point2f endpointB;
        if (edge01 < edge12) {
            // 短边是 pts[0]-pts[1] 和 pts[2]-pts[3]
            endpointA = midpoint(pts[0], pts[1]);
            endpointB = midpoint(pts[2], pts[3]);
        } else {
            // 短边是 pts[1]-pts[2] 和 pts[3]-pts[0]
            endpointA = midpoint(pts[1], pts[2]);
            endpointB = midpoint(pts[3], pts[0]);
        }

        LightBar bar;
        bar.rect = rect;

        // 图像坐标系里 y 向下增大，所以 y 小的是“上端点”。
        if (endpointA.y <= endpointB.y) {
            bar.top = endpointA;
            bar.bottom = endpointB;
        } else {
            bar.top = endpointB;
            bar.bottom = endpointA;
        }

        // 灯条长轴方向：从 top 指向 bottom。
        const cv::Point2f axis = bar.bottom - bar.top;
        bar.length = cv::norm(axis);
        if (bar.length <= 0.0f) {
            continue;
        }

        // 计算长轴偏离竖直方向多少度：
        // 完全竖直时 atan2(0, |dy|) = 0；越倾斜角度越大。
        bar.angle = std::atan2(std::abs(axis.x), std::abs(axis.y)) * 180.0f /
                    static_cast<float>(CV_PI);
        if (bar.angle > kMaxTiltDeg) {
            continue; // 横着的反光不是灯条
        }

        bars.push_back(bar);
    }

    return bars;
}

// ============================================================================
// 第四步：两根灯条配对成一块装甲板
// ============================================================================
bool pairLightBars(const std::vector<LightBar>& bars, Armor& armor) {
    armor.valid = false;
    if (bars.size() < 2) {
        return false;
    }

    float bestScore = std::numeric_limits<float>::max();
    int bestLeft = -1;
    int bestRight = -1;

    for (std::size_t i = 0; i < bars.size(); ++i) {
        for (std::size_t j = i + 1; j < bars.size(); ++j) {
            // 先按中心 x 坐标确定谁在左、谁在右。
            const LightBar* left  = &bars[i];
            const LightBar* right = &bars[j];
            if (left->rect.center.x > right->rect.center.x) {
                std::swap(left, right);
            }

            const float avgLength = (left->length + right->length) * 0.5f;
            if (avgLength <= 0.0f) {
                continue;
            }

            // 条件 1：两根灯条要基本平行。
            const float angleDiff = std::abs(left->angle - right->angle);
            if (angleDiff > kMaxPairAngleDiff) {
                continue;
            }

            // 条件 2：两根灯条长度要接近。
            const float lengthRatio =
                std::min(left->length, right->length) / std::max(left->length, right->length);
            if (lengthRatio < kMinLengthRatio) {
                continue;
            }

            // 条件 3：中心高度要接近。
            const float centerYDiff = std::abs(left->rect.center.y - right->rect.center.y);
            if (centerYDiff > kMaxCenterYDiffRatio * avgLength) {
                continue;
            }

            // 条件 4：上下端点也要分别对齐。
            const float topDiff    = std::abs(left->top.y - right->top.y);
            const float bottomDiff = std::abs(left->bottom.y - right->bottom.y);
            if (topDiff > kMaxCenterYDiffRatio * avgLength ||
                bottomDiff > kMaxCenterYDiffRatio * avgLength) {
                continue;
            }

            // 条件 5：两根灯条的水平间距要合理。
            const float gap = right->rect.center.x - left->rect.center.x;
            const float gapRatio = gap / avgLength;
            if (gapRatio < kMinGapRatio || gapRatio > kMaxGapRatio) {
                continue;
            }

            // 打分：分数越小，这一对越像装甲板。
            // 各项都做了归一化，方便直接相加。
            const float score =
                angleDiff / kMaxPairAngleDiff +        // 越平行越好
                std::abs(gapRatio - kIdealGapRatio) +  // 间距越接近理想值越好
                centerYDiff / avgLength +              // 高度越一致越好
                std::abs(1.0f - lengthRatio);          // 长度越接近越好

            if (score < bestScore) {
                bestScore = score;
                bestLeft  = static_cast<int>(i);
                bestRight = static_cast<int>(j);
            }
        }
    }

    if (bestLeft < 0 || bestRight < 0) {
        return false; // 没有找到合格的组合
    }

    const LightBar* left  = &bars[bestLeft];
    const LightBar* right = &bars[bestRight];
    if (left->rect.center.x > right->rect.center.x) {
        std::swap(left, right);
    }

    // 五个步骤中的最后一步所需数据：
    // 四个端点按“左上 -> 右上 -> 右下 -> 左下”的顺序排列。
    armor.corners[0] = left->top;
    armor.corners[1] = right->top;
    armor.corners[2] = right->bottom;
    armor.corners[3] = left->bottom;

    armor.center = (armor.corners[0] + armor.corners[1] +
                    armor.corners[2] + armor.corners[3]) * 0.25f;
    armor.valid = true;
    return true;
}

// ============================================================================
// 绘制：把四个端点按顺序连线，画出一个闭合矩形
// ============================================================================
void drawArmor(cv::Mat& frame, const Armor& armor) {
    if (!armor.valid) {
        return;
    }

    // polylines 需要整数坐标点，这里把 Point2f 转成 Point。
    std::vector<cv::Point> points;
    points.reserve(4);
    for (const auto& corner : armor.corners) {
        points.emplace_back(cvRound(corner.x), cvRound(corner.y));
    }

    // 绿色闭合矩形，线宽 2。
    cv::polylines(frame, points, /*isClosed=*/true, cv::Scalar(0, 255, 0), 2);

    // 四个端点画绿色实心小圆，中心画红色实心圆，方便观察。
    for (const auto& point : points) {
        cv::circle(frame, point, 4, cv::Scalar(0, 255, 0), cv::FILLED);
    }
    cv::circle(frame,
               cv::Point(cvRound(armor.center.x), cvRound(armor.center.y)),
               5, cv::Scalar(0, 0, 255), cv::FILLED);

    cv::putText(frame, "ARMOR",
                cv::Point(points[0].x, std::max(points[0].y - 10, 15)),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
}
