#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "detector.h"

int main(int argc, char** argv) {
    // CMake 会把项目根目录通过 PROJECT_ROOT 传进来。
    // 这样无论你在 build/ 里运行，还是在项目根目录运行，都能找到视频。
#ifdef PROJECT_ROOT
    const std::string projectRoot = PROJECT_ROOT;
#else
    const std::string projectRoot = ".";
#endif

    // 也支持手动指定输入/输出路径：
    //   ./tjurm-test2 输入视频 输出视频
    const std::string videoPath =
        (argc > 1) ? argv[1] : projectRoot + "/videos/test.avi";
    const std::string outputPath =
        (argc > 2) ? argv[2] : projectRoot + "/videos/result.avi";

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
    // 2. 创建输出视频
    // ------------------------------------------------------------------
    // MJPG + .avi 是 OpenCV 自带支持的组合，不需要额外安装编码器。
    cv::VideoWriter writer(
        outputPath,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps,
        frame.size());

    if (!writer.isOpened()) {
        std::cerr << "无法创建输出视频：" << outputPath << std::endl;
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

    // ------------------------------------------------------------------
    // 3. 逐帧处理
    // ------------------------------------------------------------------
    int frameIndex = 0;
    int detectedFrames = 0;
    bool found = false;

    while (!frame.empty()) {
        ++frameIndex;

        cv::Mat mask;
        // 找灯条 + 配对成装甲板
        const std::vector<LightBar> bars = findLightBars(frame, mask);
        Armor armor;
        found = pairLightBars(bars, armor);

        // 如果配对成功，就在画面上画绿色矩形
        if (found) {
            drawArmor(frame, armor);
            ++detectedFrames;
        }

        // 把处理后的这一帧写入输出视频
        writer.write(frame);

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
            std::cout << "已处理 " << frameIndex << " 帧，灯条数：" << bars.size()
                      << (found ? "，已框出装甲板" : "，未配对成功") << std::endl;
        }

        // 读下一帧；读不到就退出循环
        if (!cap.read(frame)) {
            frame.release();
        }
    }

    // ------------------------------------------------------------------
    // 4. 收尾
    // ------------------------------------------------------------------
    writer.release();
    cap.release();
    if (showWindow) {
        cv::destroyAllWindows();
    }

    std::cout << "处理完成：共 " << frameIndex << " 帧，成功识别 "
              << detectedFrames << " 帧。" << std::endl;
    std::cout << "输出视频：" << outputPath << std::endl;

    return 0;
}
