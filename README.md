# Inspection_robot_H2

> 加氢站燃爆智能巡检机器人 · Hydrogen Station Explosion Intelligent Inspection Robot

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Ubuntu%2020.04-orange.svg)]()
[![ROS](https://img.shields.io/badge/ROS-Noetic-green.svg)]()
[![Qt](https://img.shields.io/badge/Qt-5.12%2B-brightgreen.svg)]()
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)]()

---

## 📑 目录

- [文件结构](#文件结构)
- [项目简介](#项目简介)
- [系统架构](#系统架构)
- [核心功能](#核心功能)
  - [1. 自主导航与 SLAM](#1-自主导航与-slam)
  - [2. 机械臂检测](#2-机械臂检测)
  - [3. 多模态传感检测](#3-多模态传感检测)
  - [4. 视觉目标检测](#4-视觉目标检测)
  - [5. 巡检任务编排](#5-巡检任务编排)
- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [算法概览](#算法概览)
- [项目部署](#项目部署)
- [界面预览](#界面预览)
- [实验验证](#实验验证)
- [引用](#引用)
- [许可证](#许可证)

---

## 文件结构

```
Inspection_robot_H2/
├── src/                           # 上位机控制程序 (Qt/C++)
│   ├── AGV_test.pro               # Qt 项目文件
│   ├── main.cpp                   # 程序入口
│   ├── mainwindow.cpp             # 主窗口 (核心集成)
│   ├── mainwindow.h               # 主窗口头文件
│   ├── mainwindow.ui              # UI 布局文件
│   ├── agv.cpp / agv.h           # AGV 底盘 CAN 总线控制
│   ├── aubo_robot.cpp / aubo_robot.h  # AUBO 机械臂控制
│   ├── sensormonitor.cpp / .h    # 传感器数据采集与显示
│   ├── opencamera.cpp             # ZED 相机初始化和帧采集
│   ├── shipinliu.cpp              # 视频流处理与 HSV 目标检测
│   ├── enter_save.cpp             # 图像保存工具
│   ├── LED.cpp / LED.h           # LED 指示灯自定义控件
│   ├── particleprogress.cpp / .h # 粒子特效进度条控件
│   ├── DataThread.h               # 传感器数据采集线程
│   ├── zcan.h                     # CAN 协议帧定义
│   ├── util.h                     # 工具宏与辅助函数
│   ├── demo_simple_goal.cpp       # 独立巡检导航 ROS 节点
│   └── include/
│       └── Camera.hpp             # Stereolabs ZED SDK C++ 封装
├── algorithms/                    # 核心算法
│   ├── laserMapping.cpp           # Fast-LIO LiDAR SLAM
│   ├── IMU_Processing.hpp         # IMU 前向传播 + 点云去畸变
│   ├── nav_point.cpp              # 队列式多点导航
│   └── shipinliu.cpp              # HSV 色彩空间视觉检测
├── docs/                          # 文档
│   ├── architecture.md            # 系统架构详解
│   ├── hardware.xlsx              # 硬件清单与接线说明
│   ├── deployment.md              # 部署指南（从零到运行）
│   └── images/                    # 上位机界面截图
├── assets/
│   └── robot.jpg                  # 机器人实物照片
├── .gitignore
├── LICENSE                        # MIT
└── README.md                      # 项目总览
└── README_Reocalization.md        # 项目重定位部分
```

---

## 项目简介

**Inspection_robot_H2** 是一款专为加氢站防爆场景打造的自主巡检机器人系统。机器人搭载 Livox MID360 激光雷达，基于 **Fast-LIO** 算法实现全域 3D SLAM 建图与自主导航；融合 **ZED 2 双目相机**、**UTi586A 红外热像仪**、**氢气浓度传感器**和**紫外火焰探测器**等多模态传感器，可对场站内设备高温异常、明火隐患、氢气泄漏等燃爆风险进行全方位检测与预警。上位机基于 **Qt 5 + C++17** 开发，将 AGV 底盘控制、AUBO 机械臂操控、传感器数据采集、实时视频目标检测、ROS 导航栈管理统一集成到一个图形化界面中。

系统已在实际加氢站场景完成多轮实验验证，包括氢气泄漏检测、红外测温、紫外火焰响应、全局地图构建与自主导航等关键功能测试。

---

## 系统架构

![系统架构图](docs/images/jiagou.png)

---

## 核心功能

### 1. 自主导航与 SLAM

- 基于 Livox MID360 激光雷达 + Fast-LIO 紧耦合框架，对加氢站全场区进行**实时 3D 点云建图**与**高频率位姿估计**（50 Hz+），为后续巡检导航提供厘米级定位基准
- 采用迭代误差状态扩展卡尔曼滤波（ES-EKF）融合 IMU 高频角速度/加速度数据与 LiDAR 点云，抑制长时间运行中的惯导漂移，确保防爆区内位姿一致性
- ikd-Tree 增量式地图管理结构，在加氢站大型场站尺度下支持点云的动态插入、按需删除与高效最近邻搜索
- 基于 FOV 的局部地图分割策略，将全局地图按视场范围剪裁为局部子图，降低计算开销，适配大规模场站环境
- move_base 全局路径规划 + DWA 局部避障，覆盖 B/C/D/E（自主定义） 多区域 50+ 预定义巡检点位，支持多点队列式自主导航

### 2. 机械臂检测

- 集成 **AUBO-i16** 6 自由度协作机械臂，通过 TCP/IP通信，支持登录、上电自检与实时位姿回调，末端搭载红外/视觉传感器对加氢站关键设备进行多角度贴近探查
- 支持关节空间运动（J1~J6 独立控制，速度/加速度可独立配置）与笛卡尔空间运动（XYZ + RPY），配合逆运动学（IK）解算，灵活适配不同巡检点位的检测姿态需求
- 内置多套预编检测动作序列（`point_move_b8` 复位、`point_nav` 区域检测等），AGV 导航到位后自动触发执行，检测完毕自动复位，覆盖黄埔实验室 B/C/D 区及小虎岛一期等多个场站场景
- 关节速度通过 UI 滑块 0~100% 连续可调，启停加减速平滑，满足防爆场站对机械臂运动安全性与平稳性的严格要求

### 3. 多模态传感检测

| 传感器 | 检测对象 | 通信协议 | 采样方式 |
|--------|----------|----------|----------|
| 氢气传感器 | H₂ 浓度 | RS485  | 1Hz 轮询 |
| 紫外火焰传感器 | 明火/火花 | 4-20ma转RS485  | 1Hz 轮询 |
| ZED 2 双目相机 | 管道/法兰 | USB 3.0 | 实时视频流 |
| UTi586A 红外热像仪 | 管道设备温度异常 | Ethernet | 按需采集 |

### 4. 视觉目标检测

- 基于 ZED 2 双目相机实时视频流（HD720），对加氢站**管道法兰、仪表、管道**等关键设备进行在线视觉目标检测
- 完整处理流水线：高斯模糊（3×3）降噪 → BGR→HSV 色彩空间转换 → `inRange` 色相/饱和度/明度三通道联合分割 → 高光剔除（V 通道 ≥230 且 S 通道 ≤65 判定为高光区域并掩码扣除）→ 形态学开/闭运算（椭圆核 3×3），去除噪点并弥合目标断裂区域
- `connectedComponentsWithStats` 连通域分析，对每个候选区域施加**面积**、**宽高**、**长宽比**、**矩形度**五维几何约束，滤除背景干扰
- 6 路 OpenCV Trackbar 实时可调（H/S/V 上下限 + 高光 V/S 阈值），适应不同光照条件与检测目标的现场快速标定
- 支持一键保存检测结果：标注帧图像 + 4 通道中间掩码（色彩掩码/高光掩码/融合掩码/形态学结果）+ CSV 坐标文件（中心坐标 + 边界框）

### 5. 巡检任务编排

- 按区域（B/C/D/E，对应不同场景，可以自主设计）组织巡检点位
- 支持手动逐点巡检与自动序列巡检两种模式
- AGV 自动导航到位 → 机械臂执行检测动作 → 传感器记录数据
- 实时显示机器人位姿、传感器趋势图、导航状态

---

## 硬件平台

| 序号 | 组件 | 型号 | 通信接口 | 说明 |
|------|------|------|----------|------|
| 1 | 防爆底盘 | IICT6 | CAN Bus (USB-CAN) | 标准 CAN 2.0 帧 |
| 2 | 协作机械臂 | AUBO-i16 | TCP/IP | 负载 16kg |
| 3 | 激光雷达 | Livox MID360 | Ethernet | 360° FOV |
| 4 | 双目相机 | Stereolabs ZED 2 | USB 3.0 | 1080p 深度 |
| 5 | 红外热像仪 | UTi586A | Ethernet | -20~650°C |
| 6 | 氢气传感器 | — | RS485 | Modbus RTU |
| 7 | 火焰传感器 | 紫外 | RS485 | 185~260nm |
| 8 | 工控机 | — | — | Ubuntu 20.04 |

---

## 软件架构

### 技术栈

| 层级 | 模块 | 核心文件 | 说明 |
|:---:|------|----------|------|
| **应用层**<br>(Qt 5) | 主窗口 | `mainwindow.cpp` |所有子系统集成枢纽 |
| | AGV 底盘 | `agv.cpp` `zcan.h` | CAN Bus 通信，运动控制与巡检序列 |
| | 机械臂 | `aubo_robot.cpp` | AUBO-i16 TCP/IP 控制，关节/笛卡尔运动 |
| | 传感器 | `sensormonitor.cpp` | H₂ + 火焰传感器串口轮询 |
| | 相机 | `opencamera.cpp` `Camera.hpp` | ZED 2 视频流采集与 HSV 检测 |
| **中间件** | ROS Noetic | `roscpp` `tf2` `actionlib` | 导航栈：Fast-LIO + move_base |
| **操作系统** | Linux | — | Ubuntu 20.04 LTS (x86_64) |

### 关键设计

- **单窗口核心**：`mainwindow.cpp` 作为集成枢纽，管理所有子系统生命周期
- **定时器驱动**：AGV 序列、传感器轮询、视频帧处理、ROS 状态更新均通过 QTimer 调度
- **多线程**：传感数据采集（DataThread）+ 视频处理（VideoProcessingThread）独立于 UI 线程
- **进程管理**：ROS 节点通过 QProcess 管理，标准化启停控制与日志输出
- **信号槽解耦**：线程间通过 Qt 信号槽传递数据，UI 与逻辑分离

---

## 算法概览

### Fast-LIO（激光 SLAM）
- **文件**: `algorithms/laserMapping.cpp`
- **方法**: 迭代误差状态扩展卡尔曼滤波（ES-EKF）
- **数据结构**: ikd-Tree 增量式 KD 树
- **配准**: 点到平面 ICP
- **状态维度**: 21 维（位姿 + 速度 + 偏置 + 重力）

### IMU 预处理
- **文件**: `algorithms/IMU_Processing.hpp`
- **功能**: IMU 初始化、前向传播、点云运动畸变补偿（反向传播）

### 多点导航
- **文件**: `algorithms/nav_point.cpp`
- **功能**: 基于 RViz 点击 + move_base actionlib 的队列式多目标导航

### HSV 视觉检测
- **文件**: `algorithms/shipinliu.cpp`
- **功能**: 高斯模糊 → HSV 阈值 → 高光去除 → 形态学 → 连通域分析 → 几何过滤

---



## 项目部署

### 环境要求

- **OS**: Ubuntu 20.04 LTS (x86_64)
- **ROS**: Noetic (desktop-full)
- **Qt**: 5.12+
- **OpenCV**: 4.x
- **CUDA**: 11.x（可选，ZED SDK 加速用）

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt install build-essential cmake git qt5-default qtcreator
sudo apt install libopencv-dev
sudo apt install ros-noetic-roscpp ros-noetic-geometry-msgs \
  ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-actionlib \
  ros-noetic-move-base-msgs ros-noetic-nav-msgs
```

### 2. 安装 ZED SDK

从 [Stereolabs 官网](https://www.stereolabs.com/developers/release/) 下载对应 Ubuntu 20.04 的 ZED SDK，按官方文档安装后验证：

```bash
/usr/local/zed/tools/ZED_Diagnostic
```

### 3. 安装 AUBO Robot SDK

联系遨博（AUBO）机器人官方获取 Linux C++ SDK，将 SDK 的 `inc/` 和 `lib/` 分别放入：

```
src/dependents/robotSDK/inc/
src/dependents/robotSDK/lib/linux_x64/
src/dependents/log4cplus/linux_x64/lib/
```

### 4. 安装 USB-CAN 驱动

将 `libusbcanfd.so` 放入 `src/lib/`，或通过 apt 安装 `libusb-1.0-0-dev`。

### 5. 搭建 ROS 工作空间

```bash
mkdir -p ~/ws_sentry/src
cd ~/ws_sentry/src
# 放入以下 ROS 包：
#   livox_ros_driver2      — Livox MID360 雷达驱动
#   fast_lio_localization  — Fast-LIO 定位与建图
#   sentry_nav             — move_base 导航配置
#   agv_navigation         — AGV 巡检任务节点
cd ~/ws_sentry && catkin_make
echo "source ~/ws_sentry/devel/setup.bash" >> ~/.bashrc
```
- 具体的导航部署参考 [README_Reocalization.md](README_Reocalization.md)。

### 6. 编译上位机

```bash
cd src
qmake AGV_test.pro
make -j$(nproc)
```

### 7. 运行

```bash
export ROS_WORKSPACE=~/ws_sentry
./AGV_test
```

### 8. 操作流程

1. 启动雷达驱动 → 等待激光雷达就绪
2. 启动 Fast-LIO 定位 → 等待初始化完成
3. 发布初始位姿 → 在 RViz 中确认
4. 启动 move_base → 启动路径规划
5. 启动 AGV → 开始执行巡检任务
6. 登录机械臂 → 上电 → 巡检点开始检测

详细说明请参阅 [部署指南](docs/deployment.md)。

---

## 界面预览

### 上位机主界面

![上位机主界面](docs/images/interface.png)

### 部署场景

| 室外场景 | 室内场景 |
|:---:|:---:|
| ![室外部署](docs/images/interface1.png) | ![室内部署](docs/images/interface3.png) |

---

## 实验验证

本项目已在以下场景完成实验验证：

- **广州能源研究所**（2025年8月）：氢气泄漏检测、紫外火焰响应测试、红外测温实验
- **黄埔实验室**（2025年12月）：全局 3D 地图构建（ PCD 点云）、自主导航巡检、多传感器融合验证
- **小虎岛电氢智慧能源站**（2026年1月-2026年3月）：多点连续导航、全局路径规划稳定性测试、多传感器融合验证、视觉检测、机械臂控制

---

## 引用

若本项目对您的研究有帮助，欢迎引用：

```bibtex
@software{Inspection_robot_H2,
  author       = {ZNzn825},
  title        = {Inspection\_robot\_H2: Hydrogen Station Intelligent Inspection Robot},
  year         = {2025},
  url          = {https://github.com/ZNzn825/Inspection_robot_H2},
  note         = {面向加氢站防爆场景的自主巡检机器人系统}
}
```

---

## 许可证

本项目采用 [MIT License](LICENSE) 开源，允许自由使用、修改和分发。

## 致谢

本项目在研发过程中得到了广州能源研究所、黄埔实验室的支持，在此表示感谢。

---

**Star ⭐ 是对我最好的鼓励！**
