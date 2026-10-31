# 前言

声明：代码及除此“前言”外的README内容由AI辅助生成，本人将逐步消化与验证其核心逻辑。

当前进度：
   已完成代码逻辑的整体复盘
   已完成“找端点”函数及其内部所使用的cv函数的学习

# RoboMaster 装甲板灯条识别

读取 `videos/test.avi`，逐帧识别装甲板两侧的灯条，
计算每根灯条的上下端点，用绿色闭合矩形把四个端点连接起来，
再用 PnP 解算位姿、在装甲板中心画出三维坐标轴。

一次运行同时输出三个视频：

| 输出文件 | 内容 |
| --- | --- |
| `videos/result.avi` | 只画装甲板的绿色矩形框 |
| `videos/result_axes.avi` | 只画装甲板中心的三维坐标轴（FLU：x 红 / y 绿 / z 蓝） |
| `videos/result_pose.avi` | 矩形框 + 坐标轴 |

## 目录结构

```
.
├── CMakeLists.txt          # CMake 构建脚本
├── main.cpp                # 读视频、逐帧检测 + 位姿解算、写三个输出视频、显示窗口
├── include/
│   ├── detector.h          # LightBar / Armor 结构体和函数声明
│   └── pose.h              # 相机参数、位姿结果结构体、AngleFilter、函数声明
├── src/
│   ├── detector.cpp        # 灯条识别核心算法（二值化、轮廓、配对）
│   └── pose.cpp            # solvePnP 解算 + projectPoints 画坐标轴
├── videos/
│   ├── test.avi            # 输入视频
│   ├── result.avi          # 输出：只画矩形框
│   ├── result_axes.avi     # 输出：只画坐标轴
│   ├── result_pose.avi     # 输出：矩形框 + 坐标轴
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

运行结束后，在 Windows 里双击打开（三个视频任选）：

```
D:\AAAAAAATJU\TJU_Competition\ROBOMASTER\my-opencv-video-detector\videos\result.avi
D:\AAAAAAATJU\TJU_Competition\ROBOMASTER\my-opencv-video-detector\videos\result_axes.avi
D:\AAAAAAATJU\TJU_Competition\ROBOMASTER\my-opencv-video-detector\videos\result_pose.avi
```

也可以手动指定输入视频和输出目录：

```bash
./tjurm-test2 /路径/输入.avi /路径/输出目录
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
5. **绘制**：四个端点按“左上 → 右上 → 右下 → 左下”的顺序用 `cv::polylines` 连成闭合矩形；
6. **位姿**（选做）：用四个端点做 PnP 解算，把三维坐标轴投到装甲板中心，
   并在有图形界面时用 `cv::imshow` 实时显示。
   每一步的结果分别写进 `result.avi` / `result_axes.avi` / `result_pose.avi`。

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
| `cv::solvePnP`（选做）     | `src/pose.cpp` 解算装甲板位姿 |
| `cv::projectPoints`（选做） | `src/pose.cpp` 画三维坐标轴 |

## 选做任务：PnP 位姿解算 + 三维坐标轴

位姿解算的代码放在 `include/pose.h` + `src/pose.cpp`，由 `main.cpp` 直接调用：每帧检测出装甲板后，
先 `solveArmorPose()` 解出位姿，再 `drawPoseAxes()` 把坐标轴画到装甲板中心。

同一帧在三个副本上分别作图，所以一次运行输出三个视频：

```cpp
cv::Mat frameBox  = frame.clone();   // 只画矩形框
cv::Mat frameAxes = frame.clone();   // 只画坐标轴
// frame 本身作为“矩形框 + 坐标轴”的版本
```

- `videos/result.avi`：只画矩形框（第一步~第五步的结果）；
- `videos/result_axes.avi`：只画坐标轴；
- `videos/result_pose.avi`：矩形框 + 坐标轴。

坐标轴是 FLU 轴序：**x 红**（装甲板正前方）、**y 绿**（装甲板左侧）、
**z 蓝**（装甲板上方），和培训文档里的示意图配色一致；画面左上角还写着距离、
偏航角（yaw）和俯仰角（pitch）。没解出位姿的帧（重投影误差太大，或者装甲板
跑到相机后面）就只画矩形框。

### 代码分工

| 文件 | 作用 |
| --- | --- |
| `include/pose.h` | 相机参数 `CameraParams`、位姿结果 `PoseResult`、`AngleFilter`、函数声明 |
| `src/pose.cpp` | 内参、`cv::solvePnP` 解算、`cv::projectPoints` 画坐标轴、角度与滤波 |
| `main.cpp` | 调用 `solveArmorPose()` / `drawPoseAxes()`，写出三个版本的视频 |

检测部分直接复用原来的 `findLightBars()` 和 `pairLightBars()`，没有改动。

### 角度标签：相对视线，并且做了中值滤波

左上角的 `yaw` / `pitch` 是**相对视线（LOS, line of sight）**的偏角：0° 表示装甲板
法向正对着相机。先取 `los = -tvec / |tvec|`（相机指向装甲板的方向），再在垂直于
`los` 的平面里取两个基向量 `ex = (0,-1,0) × los`、`ey = los × ex`，最后
`yaw = atan2(n·ex, n·los)`、`pitch = atan2(n·ey, n·los)`，其中 `n` 是装甲板法向。
这样装甲板出现在画面偏左/偏右时读数不受影响 —— 如果直接相对相机光轴算，
板子在画面边缘、明明正对相机也会被读出十几度。

但这两个角在 1.7 m 的距离上**天生不稳**：板子只有 70×30 像素左右，转 10° 时远边
只比近边短 1.4%（约 0.4 像素），和角点本身的 0.5 像素量化同一个量级，所以解出的
角度会来回跳。显示用的值因此做了 **5 帧中值滤波**（`AngleFilter`，`include/pose.h`）：
连续 5 帧取中位数，单个坏帧不会直接跳到读数上，实测逐帧抖动（相邻帧差的绝对值）

| | 均值 | p95 | 最大 |
| --- | --- | --- | --- |
| yaw 原始 → 滤波 | 4.2° → 1.7° | 11.6° → 10.7° | 33.9° → 26.1° |
| pitch 原始 → 滤波 | 6.7° → 2.3° | 27.1° → 17.4° | 42.5° → 23.2° |

滤波只作用在**文字读数**上，`cv::projectPoints` 画坐标轴用的仍是每帧解出的真实位姿；
连续几帧没有装甲板时 `AngleFilter` 会自动清空，不会带着旧数据。

### 解算流程

1. `pairLightBars()` 给出四个端点（左上、右上、右下、左下）；
2. `cv::undistortPoints()` 先把四个端点去畸变，换成“理想针孔相机”下的像素坐标
   —— `solvePnP` 内部对畸变的处理并不完整，先把畸变去掉再解算是最稳的做法；
3. `cv::solvePnP()` 用 `SOLVEPNP_IPPE` 方法解算（四个点共面，属于平面 PnP）；
   万一 IPPE 在退化情况下失败，会退回 `SOLVEPNP_ITERATIVE`；
4. 解出 `rvec`（旋转）和 `tvec`（平移，单位 mm）后，用 `cv::projectPoints()`
   把装甲板坐标系的原点和三条轴的终点投回图像（这次带上畸变系数，画的位置才和画面一致），
   再 `cv::arrowedLine` 画箭头；
5. 用四个端点的**重投影误差**检查这次解算靠不靠谱：把模型点按解出来的位姿投回去，
   和实际端点比，平均偏差超过 5 像素就认为这一帧解错了，不画坐标轴。

### 坐标系和模型点（这里最容易写错）

装甲板坐标系用任务要求的 FLU，原点在装甲板中心：

- **x**：装甲板正前方（也就是正对镜头那一面朝外的方向）
- **y**：装甲板左侧
- **z**：装甲板上方

相机坐标系还是 OpenCV 的惯例：x 向右、y 向下、z 沿光轴向前。

四个模型点必须和 `Armor::corners` 的顺序（图像里的左上、右上、右下、左下）一一对应：

```cpp
corners[0] 左上 -> (0, -w/2, +h/2)
corners[1] 右上 -> (0, +w/2, +h/2)
corners[2] 右下 -> (0, +w/2, -h/2)
corners[3] 左下 -> (0, -w/2, -h/2)   // w = 灯条中心距，h = 灯条长度
```

注意 y 的符号：装甲板正面朝着我们，就像两个人面对面，**它的左手边出现在画面右边**，
所以“图像左上”那个点的 y 是负的。画出来的绿色 y 轴指向画面右侧，
和培训文档里的示意图一致；如果符号写反，解出来的位姿会差 180°（z 轴会朝下）。

一个自查的办法：解完之后看画出来的坐标轴 —— z 一定朝上、y 朝装甲板左侧（画面右）、
x 因为指向镜头，在装甲板靠近画面中心时会缩得很短，这都是对的。

### 物理尺寸（决定距离的那两个数）

`src/pose.cpp` 最上面：

```cpp
constexpr float kArmorWidth     = 135.0f; // 两根灯条中心线之间的距离，单位 mm
constexpr float kLightBarLength = 55.0f;  // 灯条长度，单位 mm
constexpr float kAxisLength     = 45.0f;  // 画出来每条轴的长度，单位 mm
```

PnP 只能解出“相对于模型尺寸”的距离：这两个数如果和真实装甲板不一致，
算出来的距离会按同样比例缩小/放大，**姿态角不受影响**。
灯条中心距 110~135mm 在不同开源代码里都有人用，这里取 135mm、灯条 55mm，
和 `test.avi` 里量到的像素宽高比（≈2.4）也是对得上的；
换成大装甲板或别的板子时，改这两个数就行。

### 相机参数

内参和畸变直接用了题目给的值（`makeCameraParams()`）：

```
fx=928.13  fy=930.14  cx=377.57  cy=283.89
k1=-0.2544 k2=0.5694 p1=0.00365 p2=-0.00109 k3=-1.3385
```

注意主点是 (377.6, 283.9)，比 720×540 画面的中心 (360, 270) 偏了十几个像素，
说明这组参数大概不是按 720×540 标定的（可能原图的尺寸/裁剪不一样）。
这里按题目原样使用，没有做缩放，所以解出来的距离会有一点系统误差，
但坐标轴的方向基本不受影响。

### 实测结果（`videos/test.avi`）

- 101 帧全部配对成功，也全部解算成功；
- 四个端点的平均重投影误差约 **0.1 像素**（说明模型和检测到的端点吻合得很好）；
- 距离从 2.6 m（远处的装甲板）到 1.73 m（近处的“4”号装甲板）。

## 常见问题

- **VS Code 里 `#include <opencv2/opencv.hpp>` 有红色波浪线**：
  只要 CMake 能编译通过就不用管；本项目的 `.vscode/c_cpp_properties.json`
  已经把 `/usr/local/opencv4.5.4/include/opencv4` 加进了 `includePath`。
  注意 VS Code 需要连接 WSL 打开项目，这个 Linux 路径才有效。
- **弹不出窗口**：确认使用 WSLg（Windows 11 默认支持）；没有图形界面时程序会自动只保存视频。
- **result.avi 是 0 字节**：检查 `videos` 目录是否有写权限，或输出路径是否写错。
