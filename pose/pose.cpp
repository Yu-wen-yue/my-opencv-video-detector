#include "pose.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

// ============================================================================
// 装甲板的物理尺寸，单位：mm。
//
// 重要：PnP 只能解出“相对于模型尺寸”的距离。这两个数如果和真实装甲板不一致，
// 解出来的距离会按同样比例放大/缩小（坐标轴的方向不受影响）。
// 这里用的是小装甲板的常用尺寸：两根灯条中心线相距 135mm，灯条长 55mm。
// 如果你量出来的装甲板不是这个大小，改这两个数即可。
// ============================================================================
constexpr float kArmorWidth     = 135.0f; // 两根灯条中心线之间的距离
constexpr float kLightBarLength = 55.0f;  // 灯条长度

// 画坐标轴时每条轴的长度（mm），比灯条短一点，画面才不会太乱。
constexpr float kAxisLength = 45.0f;

// 平均重投影误差超过这么多像素，就认为这一帧没解算成功。
constexpr double kMaxReprojError = 5.0;

// ----------------------------------------------------------------------------
// 装甲板坐标系（任务要求的 FLU）：
//   x 轴指向装甲板正前方（也就是正对镜头的那一面朝外的方向）
//   y 轴指向装甲板左侧
//   z 轴指向装甲板上方
// 原点放在装甲板中心。
//
// 相机坐标系还是 OpenCV 的惯例：x 向右、y 向下、z 沿光轴向前。
//
// 装甲板四个端点和 detector.h 里 Armor::corners 的顺序一一对应
// （左上、右上、右下、左下，都是“图像里”的左右）：
//   图像左上是装甲板的右侧（装甲板正面朝向我们，就像两个人面对面，
//   它的左手边出现在我们的右边），所以左上对应的 y 是负的。
// ----------------------------------------------------------------------------
std::vector<cv::Point3f> armorModelPoints() {
    const float halfWidth = kArmorWidth * 0.5f;
    const float halfLen   = kLightBarLength * 0.5f;
    return {
        {0.0f, -halfWidth, +halfLen}, // 0 左上（图像左）
        {0.0f, +halfWidth, +halfLen}, // 1 右上（图像右）
        {0.0f, +halfWidth, -halfLen}, // 2 右下
        {0.0f, -halfWidth, -halfLen}, // 3 左下
    };
}

// 三个角度都是从旋转矩阵 R 里拆出来的，0 度表示装甲板正对相机、且没有倾斜。
//   n  = R 的第一列 = 装甲板 x 轴（正面法向）在相机坐标系下的方向
//   up = R 的第三列 = 装甲板 z 轴（板上方）在相机坐标系下的方向
void computeAngles(const cv::Mat& R, PoseResult& pose) {
    const cv::Vec3d n(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0));
    const cv::Vec3d up(R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));

    const double kRad2Deg = 180.0 / CV_PI;
    // 正对相机时 n = (0, 0, -1)，所以用 -n[2] 做“正前方”的参考。
    pose.yaw   = std::atan2(n[0], -n[2]) * kRad2Deg;
    pose.pitch = std::asin(std::max(-1.0, std::min(1.0, -n[1]))) * kRad2Deg;
    // 板上方在画面里应该是竖直向上的（-y），偏多少就是画面内的倾斜角。
    pose.roll = std::atan2(up[0], -up[1]) * kRad2Deg;
}

// 画一条带箭头的坐标轴，并在箭头上写字母。
void drawAxis(cv::Mat& frame, const cv::Point& origin, const cv::Point2f& tip,
              const cv::Scalar& color, const char* label) {
    const cv::Point end(cvRound(tip.x), cvRound(tip.y));
    cv::arrowedLine(frame, origin, end, color, 2, cv::LINE_AA, 0, 0.25);
    cv::putText(frame, label, end + cv::Point(4, -4),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
}

} // namespace

CameraParams makeCameraParams() {
    CameraParams cam;
    // 内参（行优先）：fx, 0, cx / 0, fy, cy / 0, 0, 1
    cam.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        9.28130989e+02, 0.0,            3.77572945e+02,
        0.0,            9.30138391e+02, 2.83892859e+02,
        0.0,            0.0,            1.0);
    // 畸变：k1, k2, p1, p2, k3
    cam.distCoeffs = (cv::Mat_<double>(1, 5) <<
        -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00);
    return cam;
}

bool solveArmorPose(const Armor& armor, const CameraParams& cam, PoseResult& pose) {
    pose = PoseResult{};
    if (!armor.valid) {
        return false;
    }

    const std::vector<cv::Point2f> imagePoints(armor.corners, armor.corners + 4);
    const std::vector<cv::Point3f> objectPoints = armorModelPoints();

    // 先把像素坐标去畸变到“理想针孔”的像素坐标。
    // solvePnP 内部对畸变的处理并不完整，先自己去畸变、再按无畸变相机解算，
    // 是工程上最常用也最稳的做法（P 传内参矩阵，输出仍然是像素坐标）。
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(imagePoints, undistorted, cam.cameraMatrix, cam.distCoeffs,
                        cv::noArray(), cam.cameraMatrix);

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    // 四个点都在同一平面上，属于“平面 PnP”，用 IPPE 方法最合适。
    bool ok = cv::solvePnP(objectPoints, undistorted, cam.cameraMatrix, cv::Mat(),
                           rvec, tvec, false, cv::SOLVEPNP_IPPE);
    if (!ok) {
        // IPPE 在点几乎共线之类的退化情况下会失败，退回经典的迭代法。
        ok = cv::solvePnP(objectPoints, undistorted, cam.cameraMatrix, cv::Mat(),
                          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    if (!ok || tvec[2] <= 0.0) {
        return false; // 解不出，或者装甲板跑到相机后面去了
    }

    // 用解出来的位姿把模型点投回图像（这次带上畸变），和实际端点比对，
    // 误差太大说明这次解算是错的，宁可这一帧不画。
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cam.cameraMatrix, cam.distCoeffs, projected);

    double errorSum = 0.0;
    for (std::size_t i = 0; i < imagePoints.size(); ++i) {
        errorSum += cv::norm(projected[i] - imagePoints[i]);
    }
    pose.reprojError = errorSum / static_cast<double>(imagePoints.size());
    if (pose.reprojError > kMaxReprojError) {
        return false;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    computeAngles(R, pose);

    pose.rvec = rvec;
    pose.tvec = tvec;
    pose.valid = true;
    return true;
}

void drawPoseAxes(cv::Mat& frame, const PoseResult& pose, const CameraParams& cam) {
    if (!pose.valid || frame.empty()) {
        return;
    }

    // 装甲板坐标系下的原点和三条轴的终点。
    const std::vector<cv::Point3f> axisPoints = {
        {0.0f, 0.0f, 0.0f},
        {kAxisLength, 0.0f, 0.0f}, // +x：装甲板正前方
        {0.0f, kAxisLength, 0.0f}, // +y：装甲板左侧
        {0.0f, 0.0f, kAxisLength}, // +z：装甲板上方
    };

    std::vector<cv::Point2f> projected;
    // 带畸变系数投影，画出来的位置才和真实画面一致。
    cv::projectPoints(axisPoints, pose.rvec, pose.tvec, cam.cameraMatrix, cam.distCoeffs,
                      projected);

    const cv::Point origin(cvRound(projected[0].x), cvRound(projected[0].y));
    // 颜色和培训文档里的示意图一致：x 红、y 绿、z 蓝。
    drawAxis(frame, origin, projected[1], cv::Scalar(0, 0, 255), "X");
    drawAxis(frame, origin, projected[2], cv::Scalar(0, 255, 0), "Y");
    drawAxis(frame, origin, projected[3], cv::Scalar(255, 0, 0), "Z");
    cv::circle(frame, origin, 3, cv::Scalar(255, 255, 255), cv::FILLED);

    // 左上角写一行位姿摘要：距离 + 偏航角 + 俯仰角。
    char text[96];
    std::snprintf(text, sizeof(text), "d=%.2fm yaw=%+.1f pitch=%+.1f", cv::norm(pose.tvec) / 1000.0,
                  pose.yaw, pose.pitch);
    cv::putText(frame, text, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}
