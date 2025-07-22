# 使用 Docker 配置和运行本程序

本指南将详细介绍如何在 Ubuntu 系统下使用 Docker 配置环境、编译镜像，并运行容器以启动本程序。

## 1. 安装 Docker 及 nvidia-docker2

首先，确保你的系统已安装 Docker 和 nvidia-docker2（如需 GPU 支持）。如果尚未安装，可以按照如下步骤操作：

```bash
# 添加 Docker 官方 GPG 密钥
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# 添加 Docker APT 软件源
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 更新软件包并安装 Docker
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

# 添加 nvidia-docker2（仅需 GPU 支持时）
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## 2. 配置 Docker 国内加速源（可选）

为加快镜像下载速度，建议配置国内镜像源

```bash
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json <<EOF
{
    "registry-mirrors": [
        "https://docker.xuanyuan.me"
    ]
}
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker
```

## 3. 准备项目文件与 ROS 镜像

1. **项目目录结构建议如下：** 

   ```haskell
   your_project/
   ├── Dockerfile
   ├── data/
   ├── src
   	├── Ground-Fusion++
   	├── gnss_comm
   	├── livox_ros_driver
   └── figure/
   ```

2. **提前拉取 ROS 镜像：**

   为加快后续构建速度，建议先将所需的基础镜像拉取到本地：

   ```bash
   docker pull j3soon/ros-noetic-desktop-full
   ```

## 4. 构建 Docker 镜像

在包含 Dockerfile 的目录下执行以下命令，生成名为 `my` 的镜像：

```bash
cd /home/supercoconut/Myfile/docker    # 切换到 Dockerfile 所在文件夹
docker build -t my .
```

## 5. 启动容器并挂载数据目录

1. **允许 Docker 访问 X11（以支持 GUI 应用）：**

   ```bash
   xhost +local:docker
   ```

2. **运行容器，并将本地数据目录挂载到容器内：**

   ```bash
   docker run -it --rm \
       --env="DISPLAY=$DISPLAY" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --device=/dev/dri \
       -v /home/supercoconut/Myfile/docker/data:/root/data \
       my /bin/bash
   ```

   - `--env="DISPLAY=$DISPLAY"` 允许容器内应用访问主机的 X11。
   - `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"` 挂载 X11 套接字，支持 GUI。
   - `--device=/dev/dri` 支持硬件加速（如有需要）。
   - `-v 本地路径:容器内路径` 挂载数据目录。



## 6. 启动程序

- docker容器中对应的目录如下

```haskell
ws/
├── data/
├── src
	├── Ground-Fusion++
	├── gnss_comm
	├── livox_ros_driver
└── thirdparty/
```

- 启动程序示例为：

  - 开启一个容器终端

  ```bash
  cd /root/ws
  export MESA_GL_VERSION_OVERRIDE=3.3
  source devel/setup.bash
  roslaunch groundfusion2 run_m3dgr.launch
  ```

  - 开启另外一个容器终端

  ```bash
  cd /root/data
  rosbag play xxx.bag
  ```





