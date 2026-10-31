cmake_minimum_required(VERSION 3.5)
project(tjurm-test2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# calib3d 是选做任务（cv::solvePnP / cv::projectPoints）需要的模块
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio calib3d)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
# src/ 下的 detector.cpp、pose.cpp 都会被这里扫到，不用再单独列
file(GLOB SOURCES "src/*.cpp")

add_executable(tjurm-test2 main.cpp ${SOURCES})

target_link_libraries(tjurm-test2 PRIVATE ${OpenCV_LIBRARIES})

# 把项目根目录传给程序，避免从 build/ 目录运行时找不到 videos/test.avi
target_compile_definitions(tjurm-test2 PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
