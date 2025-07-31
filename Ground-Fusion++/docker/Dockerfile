# 使用更可靠的基础镜像
FROM j3soon/ros-noetic-desktop-full:latest

COPY ros.asc /tmp/ros.asc

# 1. 修复软件源 - 完全重新配置
RUN \
    # 备份原始源列表
    cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    # 使用阿里云Ubuntu源
    sed -i 's@http://.*.ubuntu.com@http://mirrors.aliyun.com@g' /etc/apt/sources.list && \
    # 删除所有可能存在的ROS源
    rm -f /etc/apt/sources.list.d/ros*.list && \
    # 删除旧密钥
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg && \
    # 安装必要工具
    apt-get update -o Acquire::Retries=5 && \
    apt-get install -y --no-install-recommends curl gnupg2 ca-certificates lsb-release && \
    # 获取最新ROS密钥 (从官方源)
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.asc && \
    # 添加清华ROS源 (国内镜像)
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list && \
    # 再次更新
    apt-get update -o Acquire::Retries=5 && \
    # 清理
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 2. 设置非交互环境
ENV DEBIAN_FRONTEND=noninteractive


WORKDIR /root
COPY ./thirdparty ./thirdparty


RUN chmod +x /root/thirdparty/install.sh && \
    /root/thirdparty/install.sh


RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# 先编译其中第一个包
WORKDIR /root/ws/src
COPY ./src/livox_ros_driver ./livox_ros_driver

WORKDIR /root/ws
RUN /bin/bash -c " \
      source /opt/ros/noetic/setup.bash && \
      catkin_make --pkg livox_ros_driver"


WORKDIR /root/ws/src
COPY ./src/gnss_comm ./gnss_comm
COPY ./src/Ground-Fusion++ ./Ground-Fusion++


# 6. 构建主工程
WORKDIR /root/ws
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && catkin_make"]

#WORKDIR /root/ws/src
#COPY ./src/ImMesh ./ImMesh


# 7. 清理
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# 8. 启动配置
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
