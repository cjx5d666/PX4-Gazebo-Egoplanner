# PX4-Gazebo-Egoplanner
# 从零开始搭建魔改自主导航仿真系统  
ROS Noetic + PX4 v1.13 + Ego-Planner + 自定义桥接

本文档记录在 **Ubuntu 20.04 LTS** 上完整搭建一套魔改自主导航仿真环境的过程，包括 ROS Noetic、PX4 v1.13 SITL、Ego-Planner、自定义点云清洗、TF 链、px4_bridge.py 安全桥接、mission_manager.py 任务调度等。

**目标**：实现无人机在 Gazebo 中自主避障、轨迹跟踪，并支持自定义 SDF（带深度相机）。

**适用人群**：ROS/PX4 仿真开发者，熟悉 Python 和基本 Linux 命令。

**硬件推荐**：8核16G+ 内存，SSD，物理机或 VMware/VirtualBox 虚拟机。

## 目录

- [0. 系统前置准备](#0-系统前置准备)
- [1. 换源（大陆/香港必做）](#1-换源大陆香港必做)
- [2. 安装 ROS Noetic](#2-安装-ros-noetic)
- [3. 安装 MAVROS + GeographicLib](#3-安装-mavros--geographiclib)
- [4. 安装 PX4 v1.13.0](#4-安装-px4-v1130)
- [5. 安装 Ego-Planner](#5-安装-ego-planner)
- [6. PX4 参数优化（关键，解决 OFFBOARD/failsafe 问题）](#6-px4-参数优化关键解决-offboardfailsafe-问题)
- [7. 添加自定义魔改文件并编译](#7-添加自定义魔改文件并编译)
- [8. 完整启动顺序 & 测试](#8-完整启动顺序--测试)
- [常见问题 & 调试经验](#常见问题--调试经验)

## 0. 系统前置准备

```bash
sudo apt update && sudo apt upgrade -y && sudo apt autoremove -y

sudo apt install -y git curl wget vim net-tools lsb-release gnupg software-properties-common build-essential

# 关闭自动锁屏（仿真时避免中断）
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'

sudo reboot  # 重启一次，确保系统更新生效
