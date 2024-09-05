# Mujoco安装记录

## 1. 安装Mujoco

### 1.1 下载Mujoco

下载地址：https://www.roboti.us/index.html

### 1.2 安装Mujoco

#### 1.2.1 安装Mujoco

```shell
# Mujoco dependencies
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf 
mkdir ~/.mujoco
cd ~/.mujoco
wget https://github.com/deepmind/mujoco/releases/tag/2.1.0
tar -zxvf mujoco210-linux-x86_64.tar.gz

# Add to PATH
code ~/.bashrc
export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin
source ~/.bashrc

# Test
cd ~/.mujoco/mujoco210/bin
./simulate ../model/humanoid.xml

# Error 因为使用SSH连接的服务器没有图形化界面导致，实际已经可以使用，参考：https://zhuanlan.zhihu.com/p/486957504 评论区
MuJoCo Pro version 2.10
ERROR: could not initialize GLFW

Press Enter to exit ...
```


#### 1.2.2 安装Mujoco的Python库

直接用diffusion policy的conda环境

https://github.com/real-stanford/diffusion_policy?tab=readme-ov-file#%EF%B8%8F-simulation

https://blog.csdn.net/qq_47997583/article/details/125400418

```shell
gedit ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia 
```