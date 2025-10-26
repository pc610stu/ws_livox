# Livox 雷达 ROS 工作空间

## 硬件支持
- Livox MID-70
- Livox MID-360
- 其他 Livox 雷达

## 系统要求
- Ubuntu 22.04
- ROS Noetic

## 快速开始

### 1. 克隆仓库
```bash
git clone <your-repo-url> ~/ws_livox
cd ~/ws_livox
```

### 2. 自动安装
```bash
./setup.sh
```

### 3. 配置雷达

#### 网络配置
- 将电脑有线网卡设置为静态IP: `192.168.1.5/24`
- 雷达默认IP: `192.168.1.111`

#### 第一代驱动配置
编辑配置文件：
```bash
nano src/livox_ros_driver/livox_ros_driver/config/livox_lidar_config.json
```

将 `YOUR_BROADCAST_CODE_HERE` 替换为你的雷达广播码（雷达上的二维码）

#### 第二代驱动配置
编辑配置文件：
```bash
nano src/livox_ros_driver2/config/MID360_config.json
```

确认 IP 地址正确

### 4. 启动驱动

#### 第一代驱动
```bash
roslaunch livox_ros_driver livox_lidar_rviz.launch
```

#### 第二代驱动
```bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

## 故障排查

### 看不到点云
1. 检查网络连接：`ping 192.168.1.111`
2. 检查广播码是否正确
3. 查看ROS日志：`rosnode info /livox_lidar_publisher`

### 编译错误
1. 确保退出 conda 环境：`conda deactivate`
2. 重新编译：`./setup.sh`

## 项目结构
```
ws_livox/
├── src/
│   ├── livox_ros_driver/      # 第一代驱动
│   └── livox_ros_driver2/     # 第二代驱动
├── setup.sh                    # 自动安装脚本
├── config_template_driver1.json
├── config_template_driver2.json
└── README.md
```

## 维护者
- pc <18928974030@163.com>

## 许可证
- Livox SDK: 遵循 Livox 许可证
- ROS 驱动: 遵循相应开源许可证
