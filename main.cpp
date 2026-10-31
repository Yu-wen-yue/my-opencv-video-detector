#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "detector.h"
#include "pose.h"

// 同一个视频会处理出三个版本：
//   videos/result.avi       只画装甲板的绿色矩形框（第一步~第五步的结果）
//   videos/result_axes.avi  只画装甲板中心的三维坐标轴（选做任务：PnP 位姿）
//   videos/result_pose.avi  矩形框 + 坐标轴
int main(int argc, char** argv) {
    // CMake 会把项目根目录通过 PROJECT_ROOT 传进来。
    // 这样无论你在 build/ 里运行，还是在项目根目录运行，都能找到视频。
#ifdef PROJECT_ROOT
    const std::string projectRoot = PROJECT_ROOT;
#else
    const std::string projectRoot = ".";
#endif

    // 也支持手动指定输入视频和输出目录：
    //   ./tjurm-test2 输入视频 输出目录
    const std::string videoPath =
        (argc > 1) ? argv[1] : projectRoot + "/videos/test.avi";
    const std::string outputDir =
        (argc > 2) ? argv[2] : projectRoot + "/videos";

    const std::string boxPath  = outputDir + "/result.avi";
    const std::string axesPath = outputDir + "/result_axes.avi";
    const std::string posePath = outputDir + "/result_pose.avi";

    // ------------------------------------------------------------------
    // 1. 打开视频
    // ------------------------------------------------------------------
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "打不开视频：" << videoPath << std::endl;
        std::cerr << "请检查文件是否存在，路径是否正确。" << std::endl;
        return -1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 1.0 || fps > 240.0) {
        std::cout << "视频帧率异常（" << fps << "），改用 30 FPS。" << std::endl;
        fps = 30.0;
    }

    const int totalFrames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    // 先读第一帧，用它的尺寸来创建输出视频。
    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "视频能打开，但读不到第一帧。" << std::endl;
        return -1;
    }

    std::cout << "视频：" << videoPath << std::endl;
    std::cout << "分辨率：" << frame.cols << " x " << frame.rows
              << "，FPS：" << fps
              << "，总帧数：" << totalFrames << std::endl;

    // ------------------------------------------------------------------
    // 2. 创建三个输出视频
    // ------------------------------------------------------------------
    // MJPG + .avi 是 OpenCV 自带支持的组合，不需要额外安装编码器。
    const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoWriter writerBox(boxPath, fourcc, fps, frame.size());
    cv::VideoWriter writerAxes(axesPath, fourcc, fps, frame.size());
    cv::VideoWriter writerPose(posePath, fourcc, fps, frame.size());

    if (!writerBox.isOpened() || !writerAxes.isOpened() || !writerPose.isOpened()) {
        std::cerr << "无法创建输出视频，请检查目录是否存在、是否有写权限：" << outputDir
                  << std::endl;
        return -1;
    }

    // WSLg 下一般有 DISPLAY，可以弹窗；纯命令行环境则只保存视频，避免报错。
    const char* display = std::getenv("DISPLAY");
    const char* wayland = std::getenv("WAYLAND_DISPLAY");
    const bool showWindow =
        (display != nullptr && display[0] != '\0') ||
        (wayland != nullptr && wayland[0] != '\0');

    // 按视频帧率控制窗口刷新速度，避免 101 帧一闪而过。
    // 例如 25 FPS -> 每帧等待 40 毫秒。
    const int waitDelayMs = std::max(1, static_cast<int>(1000.0 / fps));

    if (!showWindow) {
        std::cout << "未检测到图形界面，只保存视频，不弹出窗口。" << std::endl;
    } else {
        std::cout << "按 q 或 ESC 可以提前结束。" << std::endl;
    }

    // 题目直接给出的相机内参和畸变系数（选做任务的位姿解算要用）。
    const CameraParams cam = makeCameraParams();

    // 角度读数的中值滤波（原因见 pose.h 里 AngleFilter 的注释）：
    // 板子小、又基本正对相机时，yaw/pitch 逐帧会跳几度到几十度，直接显示没法看。
    // 它只平滑显示的数字，坐标轴仍然用当帧的解算结果画。
    AngleFilter angleFilter;

    // ------------------------------------------------------------------
    // 3. 逐帧处理
    // ------------------------------------------------------------------
    int frameIndex = 0;
    int detectedFrames = 0; // 成功配对出装甲板的帧数
    int posedFrames = 0;    // 成功解算出位姿的帧数

    while (!frame.empty()) {
        ++frameIndex;

        cv::Mat mask;
        // 找灯条 + 配对成装甲板
        const std::vector<LightBar> bars = findLightBars(frame, mask);
        Armor armor;
        const bool found = pairLightBars(bars, armor);

        // 一帧要输出三个版本，所以都在副本上画，互不影响：
        //   frameBox   -> 只画矩形框
        //   frameAxes  -> 只画坐标轴
        //   frame      -> 矩形框 + 坐标轴
        cv::Mat frameBox  = frame.clone();
        cv::Mat frameAxes = frame.clone();

        PoseResult pose;
        if (found) {
            ++detectedFrames;

            // 先画坐标轴，再画矩形框：这样“ARMOR”几个字会盖在坐标轴上面，
            // 装甲板比较小的时候才不会糊成一团。
            if (solveArmorPose(armor, cam, pose)) {
                ++posedFrames;
                angleFilter.apply(pose); // 平滑 yaw / pitch / roll
                drawPoseAxes(frameAxes, pose, cam);
                drawPoseAxes(frame, pose, cam);
            }

            drawArmor(frameBox, armor);
            drawArmor(frame, armor);
        } else {
            // 这一帧没找到装甲板：清掉滤波历史，免得下次出现时和旧数据混在一起。
            angleFilter.reset();
        }

        // 三个版本分别写入各自的视频
        writerBox.write(frameBox);
        writerAxes.write(frameAxes);
        writerPose.write(frame);

        // 实时显示处理结果和二值化掩码
        if (showWindow) {
            cv::imshow("Result", frame);
            cv::imshow("Mask", mask);

            const int key = cv::waitKey(waitDelayMs) & 0xFF;
            if (key == 'q' || key == 27) { // 27 是 ESC
                break;
            }
        }

        // 每 30 帧打印一次进度，方便观察运行状态
        if (frameIndex % 30 == 0) {
            std::cout << "已处理 " << frameIndex << " 帧，灯条数：" << bars.size();
            if (!found) {
                std::cout << "，未配对成功";
            } else if (!pose.valid) {
                std::cout << "，已框出装甲板，但位姿解算失败";
            } else {
                std::cout << "，距离 " << cv::norm(pose.tvec) / 1000.0 << " m";
            }
            std::cout << std::endl;
        }

        // 读下一帧；读不到就退出循环
        if (!cap.read(frame)) {
            frame.release();
        }
    }

    // ------------------------------------------------------------------
    // 4. 收尾
    // ------------------------------------------------------------------
    writerBox.release();
    writerAxes.release();
    writerPose.release();
    cap.release();
    if (showWindow) {
        cv::destroyAllWindows();
    }

    std::cout << "处理完成：共 " << frameIndex << " 帧，成功识别装甲板 "
              << detectedFrames << " 帧，成功解算位姿 " << posedFrames << " 帧。" << std::endl;
    std::cout << "输出视频：" << std::endl;
    std::cout << "  只画矩形框：      " << boxPath << std::endl;
    std::cout << "  只画坐标轴：      " << axesPath << std::endl;
    std::cout << "  矩形框 + 坐标轴：" << posePath << std::endl;

    return 0;
}
