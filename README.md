# Franka环境配置
以下环境配置均基于[ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)，机械臂基于[Franka Emika Panda](https://github.com/Liujian1997/Franka_env-Installation/Franka_Emika_Panda_Instruction_Handbook_CN.pdf)
## 版本兼容

![](https://github.com/Liujian1997/Franka_env-Installation/blob/main/img/Snipaste_2024-07-11_12-21-44.png)

## 从源代码构建

从源代码构建，请先卸载现有的之前（方法一）安装的 libfranka 和 franka_ros 以避免冲突：
```bash
sudo apt remove "*libfranka*"
```

## 构建libfranka

要构建 libfranka，请从 Ubuntu 的包管理器安装以下依赖项：
```bash
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
```
在安装完成 ROS Noetic 之后，在选择的一个目录中的 Catkin 工作区：
```bash
cd /path/to/desired/folder # 自己的目录
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.sh
catkin_init_workspace src
```

然后，通过 libfranka 从 GitHub 克隆来下载源代码：

```bash
git clone --recursive https://github.com/frankaemika/libfranka # only for panda
cd libfranka
```

默认情况下，这将检出最新版本的 libfranka. 如果要构建特定版本 libfranka，请查看相应的Git 标签：

```bash
git checkout <version>
git submodule update
```
在源目录中，创建一个构建目录并运行 CMake：

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```
## 构建ROS包和Moveit包

然后从 GitHub 克隆 franka_ros 和 Moveit 存储库：
```bash
cd ../../src
git clone --recursive https://github.com/frankaemika/franka_ros # franka_ros
git clone https://github.com/moveit/moveit_tutorials.git -b master # Moveit
git clone https://github.com/moveit/panda_moveit_config.git -b noetic-devel # Moveit
# 如果有其他需要编译的放在这一起编译，也可以单独编译，我们这里同时下载相机的存储库，不需要可以不下载
git clone https://github.com/m-tartari/realsense_gazebo_description.git # Camera
git clone https://github.com/m-tartari/realsense_gazebo_plugin.git # Camera plugin
```
默认情况下，这将检出最新版本的 franka_ros，如果要构建特定版本 franka_ros，请查看相应的 Git 标签：
```bash
cd franka_ros
git checkout <version>
```
安装任何缺少的依赖项并构建包：（注意：以下命令中注意将 /path/to/libfranka/build 替换成自己的 libfranka build 构建路径。）
```bash
cd /your path/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build  # 注意修改路径
source devel/setup.sh
```

## 如需安装RT内核
参考[链接](https://franka.cn/FCI/installation_linux.html#setting-up-the-real-time-kernel)