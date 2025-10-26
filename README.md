# Livox MID-70 雷达 ROS 工作空间

## 📝 说明

本仓库包含完整配置好的 Livox 雷达 ROS 驱动，已针对 **Livox MID-70** 进行测试和优化。

**基于以下官方驱动修改**：
- [Livox ROS Driver (第一代)](https://github.com/Livox-SDK/livox_ros_driver) - BSD 3-Clause License
- [Livox ROS Driver2 (第二代)](https://github.com/Livox-SDK/livox_ros_driver2) - BSD 3-Clause License

## 🔧 主要修改

### 第一代驱动
- ✅ 配置文件已优化
- ✅ CMake 版本兼容性修复

### 第二代驱动
- ✅ 添加 C++17 支持
- ✅ 配置文件模板
- ✅ 修复编译警告

## 🚀 快速开始

### 系统要求
- Ubuntu 22.04
- ROS Noetic

### 一键安装
```bash
# 1. 克隆仓库
git clone xxx
cd ~/ws_livox

# 2. 运行安装脚本
chmod +x setup.sh
./setup.sh

# 3. 刷新环境
source ~/.bashrc
```

### 网络配置

将电脑有线网卡设置为静态 IP：
- IP: `192.168.1.5`
- 子网掩码: `255.255.255.0`
- 雷达默认 IP: `192.168.1.111`

**图形界面配置**：
```
设置 → 网络 → 有线连接 → 齿轮图标 → IPv4 → 手动
```

### 配置你的雷达

#### 方法1：使用第一代驱动（推荐用于初学者）

编辑配置文件：
```bash
nano src/livox_ros_driver/livox_ros_driver/config/livox_lidar_config.json
```

修改广播码为你的雷达二维码：
```json
{
    "lidar_config": [
        {
            "broadcast_code": "3GGDM3E0020011",  // 改成你的
            "enable_connect": true,
            ...
        }
    ]
}
```

启动：
```bash
roslaunch livox_ros_driver livox_lidar_rviz.launch
```

#### 方法2：使用第二代驱动（推荐）

编辑配置文件：
```bash
nano src/livox_ros_driver2/config/MID360_config.json
```

确认 IP 正确：
```json
{
  "lidar_config": [
    {
      "ip": "192.168.1.111",
      ...
    }
  ]
}
```

启动：
```bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

## 📂 项目结构
```
ws_livox/
├── src/
│   ├── livox_ros_driver/          # 第一代驱动（已配置）
│   │   └── livox_ros_driver/
│   │       └── config/
│   │           └── livox_lidar_config.json
│   └── livox_ros_driver2/         # 第二代驱动（已配置）
│       ├── config/
│       │   └── MID360_config.json
│       └── CMakeLists.txt         # 已添加 C++17 支持
├── setup.sh                        # 自动安装脚本
├── setup_network.sh                # 网络配置脚本
├── README.md
└── LICENSE
```

## 🐛 故障排查

### 问题1：看不到点云
```bash
# 检查网络
ping 192.168.1.111

# 检查 ROS 话题
rostopic list
rostopic hz /livox/lidar

# 查看日志
rosnode info /livox_lidar_publisher
```

### 问题2：编译错误
```bash
# 退出 conda 环境
conda deactivate

# 清理重新编译
cd ~/ws_livox
rm -rf build/ devel/
./setup.sh
```

### 问题3：C++ 版本错误

第二代驱动已配置 C++17，如果仍有问题：
```bash
catkin_make -DCMAKE_CXX_STANDARD=17 \
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
            -DROS_EDITION=ROS1
```

## 📸 效果展示

（点云截图未添加）

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 开源协议

- 本项目配置脚本：MIT License
- Livox 驱动：BSD 3-Clause License（保留原始协议）

## 📮 联系方式

- 作者：pc
- Email: 18928974030@163.com

## ⭐ 致谢

感谢 Livox 官方提供的优秀开源驱动！

---

**注意**：本仓库仅用于学习交流，商业使用请遵守 Livox 官方协议。
