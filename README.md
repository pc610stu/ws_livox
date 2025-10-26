# Livox MID-70 工作空间

个人备份，已配置好的 Livox 雷达驱动。

## 使用
```bash
git clone <repo-url> ~/ws_livox
cd ~/ws_livox
# 安装 SDK 后
catkin_make -DCMAKE_CXX_STANDARD=17 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DROS_EDITION=ROS1
source devel/setup.bash
```

## 配置

- 第一代驱动配置：`src/livox_ros_driver/livox_ros_driver/config/livox_lidar_config.json`
- 第二代驱动配置：`src/livox_ros_driver2/config/MID360_config.json`
- 广播码：`3GGDM3E00200111`

## 启动
```bash
roslaunch livox_ros_driver livox_lidar_rviz.launch
# 或
roslaunch livox_ros_driver2 rviz_MID360.launch
```
