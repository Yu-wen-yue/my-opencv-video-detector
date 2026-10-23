# 前言

声明：代码及除此“前言”外的README内容由AI辅助生成，本人将逐步消化与验证其核心逻辑。

当前进度：
   已完成代码逻辑的整体复盘
   已完成“找端点”函数及其内部所使用的cv函数的学习

# RoboMaster 装甲板灯条识别

读取 `videos/test.avi`，逐帧识别装甲板两侧的灯条，
计算每根灯条的上下端点，并用绿色闭合矩形把四个端点连接起来，
同时输出 `videos/result.avi`。

## 目录结构

```
.
├── CMakeLists.txt          # CMake 构建脚本
├── main.cpp                # 读视频、逐帧调用检测、写输出视频、显示窗口
├── include/
│   └── detector.h          # LightBar / Armor 结构体和函数声明
├── src/
│   └── detector.cpp        # 灯条识别核心算法（二值化、轮廓、配对）
├── videos/
│   ├── test.avi            # 输入视频
│   └── result.avi          # 程序生成的输出视频
└── .vscode/
    └── c_cpp_properties.json  # 让 VS Code 认识 OpenCV 头文件，去掉红色波浪线
```

## 编译与运行（在 WSL 终端中）

```bash
cd /mnt/d/AAAAAAATJU/TJU_Competition/ROBOMASTER/my-opencv-video-detector
rm -rf build
mkdir build && cd build
cmake -DOpenCV_DIR=/usr/local/opencv4.5.4/lib/cmake/opencv4 ..
make -j$(nproc)
./tjurm-test2
```

运行结束后，在 Windows 里双击打开：

```
D:\AAAAAAATJU\TJU_Competition\ROBOMASTER\my-opencv-video-detector\videos\result.avi
```

也可以手动指定输入输出视频：

```bash
./tjurm-test2 /路径/输入.avi /路径/输出.avi
```

## 算法流程

1. **二值化**：BGR → HSV；用 `cv::inRange` 提取红色（H 0~20、170~180）和蓝色（H 100~130）
   灯条；再用 `cv::threshold` 对 V（亮度）通道做二值化，两者取交集。
   这样能过滤白色数字、灰色背景。
2. **找轮廓**：`cv::findContours` 找外轮廓，`cv::minAreaRect` 给每个轮廓套一个旋转矩形。
   用面积、长宽比、填充率、倾斜角度筛掉反光和噪点。
3. **算端点**：取旋转矩形两条短边的中点，就是灯条的上、下端点；
   按 y 坐标排序，y 小的是上端点。
4. **配对**：两根灯条必须基本平行、长度接近、上下端分别对齐、水平间距合理；
   用打分函数选出最像装甲板的一对。
5. **绘制**：四个端点按“左上 → 右上 → 右下 → 左下”的顺序用 `cv::polylines` 连成闭合矩形，
   写出 `result.avi`，并在有图形界面时用 `cv::imshow` 实时显示。

## 参数在哪里调

打开 `src/detector.cpp`，文件最上方有一组 `constexpr` 常量：

- `kSaturationMin`：饱和度下限。检测不到灯条时降低它；把白色数字也框进来时提高它。
- `kValueMin` / `kBrightnessMin`：亮度下限。画面暗时降低。
- `kRedHueMax`：红色色相上限。灯条偏橙红时提高（本视频用到 20）。
- `kMinMeanBrightness`：灯条区域平均亮度下限，用来排除地面反光。
- `kMinAspectRatio`、`kMinFillRatio`：形状筛选，反光碎片多时提高。
- 配对参数：`kMaxPairAngleDiff`、`kMaxCenterYDiffRatio`、`kMinGapRatio`、`kMaxGapRatio`。

调参时同时看程序弹出的 `Mask` 窗口：理想情况下，画面里只应该剩两根干净的白色竖条。

## 对应任务要求的函数

| 任务要求             | 代码位置                             |
| -------------------- | ------------------------------------ |
| `cv::VideoCapture` | `main.cpp` 打开视频                |
| `cv::threshold`    | `detector.cpp` 亮度二值化          |
| `cv::findContours` | `detector.cpp` 找灯条轮廓          |
| `cv::minAreaRect`  | `detector.cpp` 求灯条旋转矩形      |
| `cv::imshow`       | `main.cpp` 显示 Result / Mask 窗口 |

## 常见问题

- **VS Code 里 `#include <opencv2/opencv.hpp>` 有红色波浪线**：
  只要 CMake 能编译通过就不用管；本项目的 `.vscode/c_cpp_properties.json`
  已经把 `/usr/local/opencv4.5.4/include/opencv4` 加进了 `includePath`。
  注意 VS Code 需要连接 WSL 打开项目，这个 Linux 路径才有效。
- **弹不出窗口**：确认使用 WSLg（Windows 11 默认支持）；没有图形界面时程序会自动只保存视频。
- **result.avi 是 0 字节**：检查 `videos` 目录是否有写权限，或输出路径是否写错。
