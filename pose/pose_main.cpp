// 选做任务：灯条识别 + PnP 位姿解算 + 画出三维坐标轴。
//
// 这个程序是独立的一份，和原来的灯条识别程序（main.cpp）互不影响：
//   * 检测部分直接复用 detector.h / detector.cpp，一行都没改；
//   * 输出文件也换成了 videos/result_pose.avi，不会覆盖原来的 result.avi。
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "detector.h"
#include "pose.h"

int main(int argc, char** argv) {
    // CMake 会把项目根目录通过 PROJECT_ROOT 传进来，和 main.cpp 的做法一致。
#ifdef PROJECT_ROOT
    const std::string projectRoot = PROJECT_ROOT;
#else
    const std::string projectRoot = ".";
#endif

    // 也支持手动指定输入/输出路径：
    //   ./tjurm-pose 输入视频 输出视频
    const std::string videoPath =
        (argc > 1) ? argv[1] : projectRoot + "/videos/test.avi";
    const std::string outputPath =
        (argc > 2) ? argv[2] : projectRoot + "/videos/result_pose.avi";

    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "打不开视频：" << videoPath << std::endl;
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 1.0 || fps > 240.0) {
        std::cout << "视频帧率异常（" << fps << "），改用 30 FPS。" << std::endl;
        fps = 30.0;
    }

    const int totalFrames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "视频能打开，但读不到第一帧。" << std::endl;
        return -1;
    }

    std::cout << "视频：" << videoPath << std::endl;
    std::cout << "分辨率：" << frame.cols << " x " << frame.rows
              << "，FPS：" << fps
              << "，总帧数：" << totalFrames << std::endl;

    cv::VideoWriter writer(
        outputPath,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps,
        frame.size());
    if (!writer.isOpened()) {
        std::cerr << "无法创建输出视频：" << outputPath << std::endl;
        return -1;
    }

    const char* display = std::getenv("DISPLAY");
    const char* wayland = std::getenv("WAYLAND_DISPLAY");
    const bool showWindow =
        (display != nullptr && display[0] != '\0') ||
        (wayland != nullptr && wayland[0] != '\0');
    const int waitDelayMs = std::max(1, static_cast<int>(1000.0 / fps));

    if (!showWindow) {
        std::cout << "未检测到图形界面，只保存视频，不弹出窗口。" << std::endl;
    } else {
        std::cout << "按 q 或 ESC 可以提前结束。" << std::endl;
    }

    // 题目直接给出的相机内参和畸变系数。
    const CameraParams cam = makeCameraParams();

    int frameIndex = 0;
    int detectedFrames = 0; // 成功配对出装甲板的帧数
    int posedFrames = 0;    // 成功解算出位姿的帧数

    while (!frame.empty()) {
        ++frameIndex;

        cv::Mat mask;
        const std::vector<LightBar> bars = findLightBars(frame, mask);
        Armor armor;
        const bool found = pairLightBars(bars, armor);

        PoseResult pose;
        if (found) {
            ++detectedFrames;

            // 先画坐标轴，再画绿色矩形：这样“ARMOR”这几个字会盖在坐标轴上面，
            // 装甲板比较小的时候才不会糊成一团。
            if (solveArmorPose(armor, cam, pose)) {
                drawPoseAxes(frame, pose, cam);
                ++posedFrames;
            }
            drawArmor(frame, armor);
        }

        writer.write(frame);

        if (showWindow) {
            cv::imshow("Pose", frame);
            cv::imshow("Mask", mask);
            const int key = cv::waitKey(waitDelayMs) & 0xFF;
            if (key == 'q' || key == 27) {
                break;
            }
        }

        // 每 30 帧打印一次进度和位姿，方便在终端里看解算结果。
        if (frameIndex % 30 == 0) {
            std::cout << "已处理 " << frameIndex << " 帧，灯条数：" << bars.size();
            if (!found) {
                std::cout << "，未配对成功";
            } else if (!pose.valid) {
                std::cout << "，已框出装甲板，但位姿解算失败";
            } else {
                std::cout << "，距离 " << cv::norm(pose.tvec) / 1000.0 << " m"
                          << "，t=(" << pose.tvec[0] << ", " << pose.tvec[1] << ", "
                          << pose.tvec[2] << ") mm"
                          << "，yaw=" << pose.yaw << " pitch=" << pose.pitch
                          << " roll=" << pose.roll
                          << "，重投影误差 " << pose.reprojError << " px";
            }
            std::cout << std::endl;
        }

        if (!cap.read(frame)) {
            frame.release();
        }
    }

    writer.release();
    cap.release();
    if (showWindow) {
        cv::destroyAllWindows();
    }

    std::cout << "处理完成：共 " << frameIndex << " 帧，成功识别装甲板 "
              << detectedFrames << " 帧，成功解算位姿 " << posedFrames << " 帧。" << std::endl;
    std::cout << "输出视频：" << outputPath << std::endl;

    return 0;
}
