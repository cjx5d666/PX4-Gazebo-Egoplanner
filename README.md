# 从零开始搭建魔改自主导航仿真系统

> **核心环境**：Ubuntu 20.04 LTS + ROS Noetic + PX4 v1.13 + Ego-Planner + 自定义桥接

本文档记录了完整搭建一套魔改自主导航仿真环境的过程。该方案集成了 PX4 SITL、Ego-Planner 路径规划、自定义点云清洗及任务调度模块，支持自定义带深度相机的 SDF 模型。

**目标**：实现无人机在 Gazebo 中的自主避障与轨迹跟踪。
**适用人群**：ROS/PX4 开发者，需具备基础 Linux 及 Python 知识。
**硬件建议**：8核16G+ 内存，SSD 硬盘。

---

## 目录

* [0. 系统前置准备](https://www.google.com/search?q=%230-%E7%B3%BB%E7%BB%9F%E5%89%8D%E7%BD%AE%E5%87%86%E5%A4%87)
* [1. 软件源配置](https://www.google.com/search?q=%231-%E8%BD%AF%E4%BB%B6%E6%BA%90%E9%85%8D%E7%BD%AE)
* [2. 安装 ROS Noetic](https://www.google.com/search?q=%232-%E5%AE%89%E8%A3%85-ros-noetic)
* [3. 安装 MAVROS & GeographicLib](https://www.google.com/search?q=%233-%E5%AE%89%E8%A3%85-mavros--geographiclib)
* [4. 安装 PX4 v1.13.0](https://www.google.com/search?q=%234-%E5%AE%89%E8%A3%85-px4-v1130)
* [5. 安装 Ego-Planner](https://www.google.com/search?q=%235-%E5%AE%89%E8%A3%85-ego-planner)
* [6. PX4 关键参数优化](https://www.google.com/search?q=%236-px4-%E5%85%B3%E9%94%AE%E5%8F%82%E6%95%B0%E4%BC%98%E5%8C%96)
* [7. 添加自定义魔改文件](https://www.google.com/search?q=%237-%E6%B7%BB%E5%8A%A0%E8%87%AA%E5%AE%9A%E4%B9%89%E9%AD%94%E6%94%B9%E6%96%87%E4%BB%B6)
* [8. 启动顺序与测试](https://www.google.com/search?q=%238-%E5%90%AF%E5%8A%A8%E9%A1%BA%E5%BA%8F%E4%B8%8E%E6%B5%8B%E8%AF%95)
* [常见问题排查](https://www.google.com/search?q=%23%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E6%8E%92%E6%9F%A5)

---

## 0. 系统前置准备

```bash
# 系统更新
sudo apt update && sudo apt upgrade -y && sudo apt autoremove -y

# 安装基础工具
sudo apt install -y git curl wget vim net-tools lsb-release gnupg software-properties-common build-essential

# 关闭自动锁屏（防止仿真过程中系统休眠导致数据中断）
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'

sudo reboot  # 重启以应用更新

```

---

## 1. 软件源配置

```bash
# 备份并替换为清华大学开源镜像站源
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak

sudo tee /etc/apt/sources.list << 'EOF'
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse
EOF

sudo apt update && sudo apt upgrade -y

```

---

## 2. 安装 ROS Noetic

```bash
# 添加 ROS 官方源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full -y

# 安装构建依赖
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools -y

# 初始化 rosdep (若连接失败可使用清华镜像)
sudo rosdep init
# 备选清华镜像方案: 
# sudo sed -i 's/raw.githubusercontent.com/mirrors.tuna.tsinghua.edu.cn\/github-raw/g' /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

---

## 3. 安装 MAVROS & GeographicLib

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs -y

# 下载数据集（必须使用 sudo 执行）
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

```

---

## 4. 安装 PX4 v1.13.0

```bash
# 安装依赖
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl protobuf-compiler libeigen3-dev libopencv-dev ninja-build exiftool astyle python3-pip python3-setuptools python3-jinja2 python3-future python3-pygments python3-serial python3-lxml python3-yaml openjdk-11-jdk gazebo11 libgazebo11-dev

# 克隆 v1.13.0 版本
git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.13.0 --depth 1 ~/PX4-Autopilot
cd ~/PX4-Autopilot
git submodule update --init --recursive --depth 1

# 运行官方环境配置脚本
bash ./Tools/setup/ubuntu.sh

# 预编译
make distclean
DONT_RUN=1 make px4_sitl_default gazebo

```

### 修改 `mavros_posix_sitl.launch` (配置 TF 链)

编辑 `~/PX4-Autopilot/launch/mavros_posix_sitl.launch`，在 `</launch>` 闭合标签前插入以下内容：

```xml
<group ns="mavros">
    <param name="local_position/tf/send" value="true" />
    <param name="local_position/tf/frame_id" value="map" />
    <param name="local_position/tf/child_frame_id" value="base_link" />
    <param name="global_position/tf/send" value="false" />
</group>

```

### 环境变量配置 (`.bashrc`)

> **注意**：顺序非常关键。`catkin_ws` 的 source 需放在 PX4 之前。

```bash
# 1. 基础 ROS 环境
source /opt/ros/noetic/setup.bash

# 2. Ego-Planner 工作空间
source ~/catkin_ws/devel/setup.bash 2>/dev/null || true

# 3. PX4 环境 (最后导出，确保 ROS_PACKAGE_PATH 追加正确)
export PX4_DIR="$HOME/PX4-Autopilot"
source "$PX4_DIR/Tools/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default" > /dev/null
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo"

```

---

## 5. 安装 Ego-Planner

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git

sudo apt install libarmadillo-dev -y
cd ~/catkin_ws

# 序列化编译防止报错
catkin_make -DCMAKE_BUILD_TYPE=Release -DPKG=quadrotor_msgs
catkin_make -DCMAKE_BUILD_TYPE=Release

```

---

## 6. PX4 关键参数优化

在 PX4 SITL 启动后的 `pxh>` 终端执行以下指令，解决 **OFFBOARD 切换失败**或 **RC Failsafe** 问题：

| 参数 | 值 | 描述 |
| --- | --- | --- |
| `NAV_RCL_ACT` | 0 | 禁用遥控器丢失保护 |
| `NAV_DLL_ACT` | 0 | 禁用数传丢失保护 |
| `COM_OBL_ACT` | 0 | 禁用 Offboard 丢失动作 |
| `COM_RCL_EXCEPT` | 4 | **关键**：仿真中允许在无遥控器时切入 OFFBOARD |
| `COM_OF_LOSS_T` | 10.0 | Offboard 丢失超时时间设为 10s |

**执行命令**：`param set <参数名> <值>`，最后执行 `param save`。

---

## 7. 添加自定义魔改文件

根据 GitHub 仓库说明，将对应文件分发至以下目录：

* **模型文件**：`iris_depth_camera.sdf` -> `~/PX4-Autopilot/Tools/sitl_gazebo/models/iris_depth_camera/`
* **启动配置**：`px4_single.launch` & `advanced_param_px4.xml` -> `~/catkin_ws/src/ego-planner/src/planner/plan_manage/launch/`
* **核心脚本**：`fix_cloud.py` / `px4_bridge.py` / `mission_manager.py` -> `~/catkin_ws/`

---

## 8. 启动顺序与测试

请按以下顺序在**不同终端**执行：

1. **基础环境**：
```bash
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris sdf:=$(rospack find mavlink_sitl_gazebo)/models/iris_depth_camera/iris_depth_camera.sdf

```


2. **起飞指令**：在 `pxh>` 终端输入 `commander takeoff`。
3. **运行桥接模块**：
```bash
python3 ~/catkin_ws/px4_bridge.py
python3 ~/catkin_ws/fix_cloud.py

```


4. **启动规划器**：
```bash
roslaunch ego_planner px4_single.launch

```


5. **任务调度与切换**：
* 启动：`python3 ~/catkin_ws/mission_manager.py`
* 切换 OFFBOARD：`commander mode offboard`



---

## 常见问题排查

* **无法切入 OFFBOARD**：检查 `COM_RCL_EXCEPT` 是否已设为 4，并确保 `px4_bridge.py` 正在以高频发送 setpoint。
* **RViz 显示空白**：需手动 `Add` -> `TF` 以及 `PointCloud2`（Topic 选择 `/depth_camera/points`）。
* **路径找不到**：检查 `.bashrc` 中的 `ROS_PACKAGE_PATH` 是否包含了 `PX4-Autopilot` 路径。

---
