# 服务器使用注意事项

- [服务器使用注意事项](#服务器使用注意事项)
  - [严禁事项](#严禁事项)
  - [使用方法](#使用方法)
  - [注意事项](#注意事项)
  - [假期使用](#假期使用)

## 严禁事项

> [!CAUTION]
> 下面五条一经发现，严肃处理：
>
> - 严禁挖矿与非法外借！
> - 严禁恶意抢占显卡、无意顶掉别人程序！
> - 所有账号均没有管理员权限！
> - Home路径使用空间应小于100G，所有数据权重应该优先放到/data下面！
<!-- > > 如果显存小于24G，请使用4090-24G服务器，不要占用L40资源！ -->

## 使用方法

> [!NOTE]
>
> - IP看群通知，账号：root，密码：tj@123，端口：每个人不一样，不要登陆别人账号，不要登陆别人账号，不要登陆别人账号，重要的事情说三遍！
> - 自己可以成功登陆后可以修改密码。如何修改自己的密码，请自行百度。
> - 默认是root用户，任何命令前面都不用加 `sudo` ，默认即为最高权限。

## 注意事项

1. 命令：禁用 `reboot`，`shutdown` 等命令。

2. 显卡：使用之前务必 `nvidia-smi` 检查使用情况（有时程序可能暂时释放了GPU，因此应该将 `nvidia-smi` 与 `top` 同时使用，分析当前的服务器使用情况）。选择未被占用的显卡运行自己的程序，请不要在别人正在使用的GPU上运行（确保你的GPU编号选择的代码是正确）。

3. 远程：远程终端开启运行程序后，请定期关闭终端并重启新的终端，避免在该终端中的未成功杀死（悬挂状态）的进程空占系统资源。

4. 网络：linux机器间使用 `SCP` 指令上传或下载数据的时候，务必用-l参数限速，如果不限速，可能会导致网络阻塞，并可能杀死所有远程非 `nohup` 运行的实验程序。下载同理。

5. 内存：服务器内存避免占用总量超过80%，跑程序之前请务必查看当前系统占用内存与剩余内存，防止自己的程序启动后，由于分配较大内存使得系统内存溢出，导致程序均被杀死。

6. 分布式：如果有大规模程序运行需要，比如需要多卡分布式计算超过1天以上的，请与服务器管理员协调。

7. 环境配置：`Conda/Cuda` 都安装了，请使用 `Conda/Cuda` 环境，不要使用系统环境，因为系统环境可能存在版本冲突，导致程序无法运行。`Conda` 需要在设置以下环境变量，`cuda` 默认是非编译版本，如果需要用 `cuda` 编译，比如 `flash attention` 自行下载安装编译版本的 `cuda`，可以参考[之前的内容](./conda%2Bcuda环境配置.md)。

> [!NOTE]
>
> ```shell
> # >>> conda initialize >>>
> # !! Contents within this block are managed by 'conda init' !!
> __conda_setup="$('/opt/conda/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
> if [ $? -eq 0 ]; then
>     eval "$__conda_setup"
> else
>     if [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
>         . "/opt/conda/etc/profile.d/conda.sh"
>     else
>         export PATH="/opt/conda/bin:$PATH"
>     fi
> fi
> unset __conda_setup
> # <<< conda initialize <<
> ```


## 假期使用

> [!NOTE]

1. 登录跳板机（如果有其他方式），检查 SSH 服务的配置文件 /etc/ssh/sshd_config：

```shell
PasswordAuthentication yes  # 允许密码认证

PubkeyAuthentication yes # 允许公钥认证

```

2. 重启 SSH 服务以应用更改：

```shell
service ssh restart
```

3. 登录指令：

```shell
# lqk@1.tcp.cpolar.cn:21805 跳板机的地址和端口
# root@172.168.0.2 -p your_port  原来的登录地址和端口,172.168.0.2为万兆口，192.168.1.186为千兆口
ssh -J lqk@1.tcp.cpolar.cn:21805 root@172.168.0.2 -p your_port
```