#!/bin/bash
set -e

echo "===== Livox 雷达工作空间自动安装脚本 ====="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查 Ubuntu 版本
if [ "$(lsb_release -sc)" != "jammy" ]; then
    echo -e "${YELLOW}警告：此脚本为 Ubuntu 22.04 (Jammy) 设计${NC}"
    read -p "继续安装? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 1. 安装 ROS Noetic
echo -e "${GREEN}[1/6] 检查 ROS Noetic...${NC}"
if ! command -v roscore &> /dev/null; then
    echo "安装 ROS Noetic..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install -y ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
else
    echo "ROS Noetic 已安装"
fi

# 2. 安装依赖
echo -e "${GREEN}[2/6] 安装依赖包...${NC}"
sudo apt install -y \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    cmake \
    git \
    python3-catkin-tools

# 3. 安装 Livox SDK
echo -e "${GREEN}[3/6] 安装 Livox SDK (第一代)...${NC}"
if [ ! -f "/usr/local/lib/liblivox_sdk_static.a" ]; then
    cd /tmp
    git clone https://github.com/Livox-SDK/Livox-SDK.git
    cd Livox-SDK
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    cd ~
else
    echo "Livox SDK 已安装"
fi

# 4. 安装 Livox SDK2
echo -e "${GREEN}[4/6] 安装 Livox SDK2 (第二代)...${NC}"
if [ ! -f "/usr/local/lib/liblivox_lidar_sdk_shared.so" ]; then
    cd /tmp
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    cd ~
else
    echo "Livox SDK2 已安装"
fi

# 5. 编译工作空间
echo -e "${GREEN}[5/6] 编译工作空间...${NC}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 退出 conda 环境（如果存在）
if command -v conda &> /dev/null; then
    conda deactivate 2>/dev/null || true
fi

rm -rf build/ devel/
catkin_make -DCMAKE_CXX_STANDARD=17 \
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
            -DROS_EDITION=ROS1

# 6. 配置环境变量
echo -e "${GREEN}[6/6] 配置环境变量...${NC}"
if ! grep -q "source $SCRIPT_DIR/devel/setup.bash" ~/.bashrc; then
    echo "source $SCRIPT_DIR/devel/setup.bash" >> ~/.bashrc
    echo "环境变量已添加到 ~/.bashrc"
fi

echo -e "${GREEN}===== 安装完成！ =====${NC}"
echo -e "${YELLOW}请运行: source ~/.bashrc${NC}"
echo -e "${YELLOW}或重新打开终端${NC}"
