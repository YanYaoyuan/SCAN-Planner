FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i \
      -e 's|http://ports.ubuntu.com/ubuntu-ports|http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
      -e 's|http://archive.ubuntu.com/ubuntu|http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
      /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      libboost-python-dev \
      libblas-dev \
      libeigen3-dev \
      liblapack-dev \
      libopencv-dev \
      libpcl-dev \
      python3-catkin-pkg \
      python3-argcomplete \
      python3-dateutil \
      python3-dev \
      python3-distlib \
      python3-docutils \
      python3-empy \
      python3-notify2 \
      python3-numpy \
      python3-opencv \
      python3-packaging \
      python3-pip \
      python3-pytest \
      python3-pytest-cov \
      python3-pytest-repeat \
      python3-pytest-rerunfailures \
      python3-setuptools \
      python3-tomli \
      python3-yaml \
    && apt-get clean \
    && find /var/lib/apt/lists -mindepth 1 -delete

# dog3 厂家 ROS 的 rosidl_parser 运行时需要 lark，但基础系统未随 ROS 安装。
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      libconsole-bridge-dev \
      libcurl4-openssl-dev \
      liblttng-ust-dev \
      liborocos-kdl-dev \
      libspdlog-dev \
      libtinyxml2-dev \
      libyaml-cpp-dev \
      python3-lark \
    && apt-get clean \
    && find /var/lib/apt/lists -mindepth 1 -delete \
    && mkdir -p /sysroot \
    && ln -s /usr /sysroot/usr

COPY wheels /opt/colcon-wheels

# dog3 无法稳定访问 PyPI。colcon 本身是纯 Python 包，部署电脑提前下载后
# 在此离线安装；其系统依赖已经由上面的 Ubuntu 软件包提供。
RUN python3 -m pip install --no-index --no-deps /opt/colcon-wheels/colcon*.whl \
    && python3 -m pip check \
    && find /opt/colcon-wheels -mindepth 1 -delete

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

CMD ["bash"]
