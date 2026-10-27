cmake_minimum_required(VERSION 3.5)
project(tjurm-test2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# calib3d 是选做任务（cv::solvePnP / cv::projectPoints）需要的模块
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio calib3d)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
file(GLOB SOURCES "src/*.cpp")

add_executable(tjurm-test2 main.cpp ${SOURCES})

target_link_libraries(tjurm-test2 PRIVATE ${OpenCV_LIBRARIES})

# 把项目根目录传给程序，避免从 build/ 目录运行时找不到 videos/test.avi
target_compile_definitions(tjurm-test2 PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

# ---------------------------------------------------------------------------
# 选做任务：PnP 位姿解算 + 三维坐标轴。
# 单独编成一个可执行文件 pose/tjurm-pose，检测部分复用 src/detector.cpp，
# 原来的 tjurm-test2 完全不受影响。
# ---------------------------------------------------------------------------
add_executable(tjurm-pose
    pose/pose_main.cpp
    pose/pose.cpp
    src/detector.cpp)

target_include_directories(tjurm-pose PRIVATE pose include)
target_link_libraries(tjurm-pose PRIVATE ${OpenCV_LIBRARIES})
target_compile_definitions(tjurm-pose PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
