# Franka环境配置
以下环境配置均基于[ROS Noetic (Ubuntu版本)](https://wiki.ros.org/noetic/Installation/Ubuntu)，机械臂基于[Franka Emika Panda](https://github.com/Liujian1997/Franka_env-Installation/Franka_Emika_Panda_Instruction_Handbook_CN.pdf)
- [Franka环境配置](#franka环境配置)
  - [参考链接](#参考链接)
  - [Pinocchio安装](#pinocchio安装)
  - [版本兼容](#版本兼容)
  - [构建libfranka](#构建libfranka)
  - [构建ROS包和Moveit包](#构建ros包和moveit包)
  - [如需安装RT内核](#如需安装rt内核)
  - [Tips：](#tips)
    - [实体通信：](#实体通信)
    - [服务器下多个Gazebo一起调试](#服务器下多个gazebo一起调试)
    - [ROS节点通信（分布式）](#ros节点通信分布式)

<!-- <small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small> -->

## 参考链接
- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [Franka FCI 中文版](https://franka.cn/FCI/overview.html)
- [Moveit Noetic](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html#)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

## Pinocchio安装
安装依赖
```bash
sudo apt install -qqy lsb-release curl
```
系统中注册 robotpkg
```bash
sudo mkdir -p /etc/apt/keyrings
 curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
     | sudo tee /etc/apt/keyrings/robotpkg.asc
```
把 robotpkg 添加到apt源码库中
```bash
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list
```
更新软件列表
```bash
sudo apt update
```
直接使用以下指令安装pinocchio
```bash
# sudo apt install -qqy robotpkg-py3*-pinocchio 
# 注意：可能出现版本对应不上
sudo apt install robotpkg-py38-pinocchio=2.7.0 robotpkg-pinocchio=2.7.0 robotpkg-py38-eigenpy=3.5.0
```
设置环境变量`vim ~/.bashrc`
```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```
更新环境变量`source ~/.bashrc`
> Test：卸载后安装通过
> 
> ```bash
> sudo apt remove robotpkg-py3*-pinocchio
> ```
>
> ![](img/Snipaste_2024-07-11_15-32-31.png)
> 
> 重新安装通过，会自动安装3.8
>
> ![](img/Snipaste_2024-07-11_15-41-55.png)
>
> 会报错！！！！！！！！！！！！！！！！！！！！！
>
> ```bash
> sudo apt install robotpkg-py38-pinocchio=2.7.0 robotpkg-pinocchio=2.7.0 robotpkg-py38-eigenpy=3.5.0
> ```

## 版本兼容

![](img/Snipaste_2024-07-11_12-21-44.png)

## 构建libfranka
从源代码构建，请先卸载现有的之前（方法一）安装的 libfranka 和 franka_ros 以避免冲突：
```bash
sudo apt remove "*libfranka*"
```
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
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.9.1 # Robot system 4.2.2
cd libfranka
```

(非必须)默认情况下，这将检出最新版本的 libfranka. 如果要构建特定版本 libfranka，请查看相应的 Git 标签：

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
(非必须)默认情况下，这将检出最新版本的 franka_ros，如果要构建特定版本 franka_ros，请查看相应的 Git 标签：
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
配置成功测试：（注：运行前先运行`source devel/setup.sh`避免出现以下错误，可以加到环境变量里）

![](img/微信图片_20240711203743.png)

```bash
roslaunch panda_moveit_config demo.launch
# 另起一个终端
rosrun moveit_tutorials move_group_python_interface_tutorial.py
```

## 如需安装RT内核
参考[链接](https://franka.cn/FCI/installation_linux.html#setting-up-the-real-time-kernel)

注意：`PREEMPT_RT` 内核不支持 NVIDIA 二进制驱动程序。

安装必要的依赖项：
```bash
sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
```
对于使用内核版本 5.9.1 测试通过的 Ubuntu 20.04：
```bash
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign
```
解压：
```bash
xz -d *.xz
```

---
## Tips：
### 实体通信：
进入 `libfranka/build/examples`
```bash
./communication_test Arm_IP # 替换为机器人IP
```
如果测过不通过则不能进行下一步

### 服务器下多个Gazebo一起调试
开启不同URI的ROS MASTER
```bash
roscore -p YOUR_Port # 不同用户设置不同端口号，默认为11311
```
在执行上述命令后，再开启一个终端，执行`vim ~/.bashrc`
```bash
export ROS_MASTER_URI='http://localhost:YOUR_Port' ## 替换自己的端口号
export GAZEBO_MASTER_URI=http://localhost:YOUR_Gazebo_Port ## 与ros master类似,重新设置，默认在地址为11345，多人时会冲突
```
### ROS节点通信（分布式）
在[上一步](#服务器下多个gazebo一起调试)的基础上替换IP，前提是相互之间**可以`Ping`通**。

**主机**
```bash
export ROS_MASTER_URI='http://主机IP:YOUR_Port' ## 替换自己的端口号
export ROS_HOSTNAME=主机IP
```
**从机**
```bash
export ROS_MASTER_URI='http://主机IP:YOUR_Port' ## 替换自己的端口号
export ROS_HOSTNAME=从机IP # 注意这里是从机IP
```