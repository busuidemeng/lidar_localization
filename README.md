# auto_driving_localization_new

## 大问题
1. 建图时加入imu约束会糊(大概解决了)

## 小问题
1. 滑窗优化中的约束问题(已解决)
2. gnss原点加载问题(已解决)
3. 闭环模块中有个2.0超参数的问题(已解决)
4. ESKF中的init_pose_可能是有问题的(已解决)
5. ESKF中的运动约束问题，感觉葛垚代码有点问题。以及运动约束是否加入角速度判断(已解决)
6. ESKF中融合模式只有：地图匹配+imu融合、地图匹配+速度观测+imu融合、地图匹配+运动约束+imu融合、地图匹配(仅使用位置观测)+速度观测+imu融合、地图匹配(仅使用位置观测)+imu融合。并没加入gnss观测。(已解决)
7. g2o的头文件中，自定义变量下划线的问题(已解决)



## 编译方式

```bash
# 编译建图模块
catkin_make --cmake-args -D BUILD_MAPPING=ON
# 使用滤波器定位
catkin_make --cmake-args -D BUILD_MAPPING=OFF -D USE_KALMAN=ON
# 使用优化器定位
catkin_make --cmake-args -D BUILD_MAPPING=OFF -D USE_KALMAN=OFF
```

## 运行方式

### 基于kitti数据集的建图

```bash
# 配置lio_back_end.yaml，使能需要的约束和其对应的权重

#  启动节点
source devel/setup.bash
roslaunch lidar_localization lio_mapping.launch

# 播放数据集

# 优化轨迹（多执行几次）
source devel/setup.bash
rosservice call /optimize_map

# 保存地图
rosservice call /save_map

# 保存gnss原点（方便定位使用）
rosservice call /save_origin
```

### 基于urbanNav数据集的建图

urbanNav数据集[下载地址](https://github.com/weisongwen/UrbanNavDataset)，目前仅支持：

1. UrbanNav-HK-Data20190428
2. UrbanNav-HK-Data20200314

```bash
# 配置hk_data_pretreat.yaml，设置外参

# 配置hk_lio_back_end.yaml，使能需要的约束和其对应的权重

#  启动节点
source devel/setup.bash
roslaunch lidar_localization hk_lio_mapping.launch

# 播放数据集

# 优化轨迹（多执行几次）
source devel/setup.bash
rosservice call /optimize_map

# 保存地图
rosservice call /save_map
```

### 基于仿真数据的建图

```bash
# 配置simulator.yaml，设置仿真数据噪声大小

# 配置lio_back_end_sim.yaml，使能需要的约束和其对应的权重

#  启动节点
source devel/setup.bash
roslaunch lidar_localization lio_mapping.launch

# ctrl+c退出，轨迹会自动保存
```

### 基于kitti数据集的滤波定位

```bash
# 配置kitti_filtering.ymal，配置地图加载路径、定位方式、滤波器权重

#  启动节点
source devel/setup.bash
roslaunch lidar_localization lio_mapping.launch

# 播放数据集

# 保存轨迹
source devel/setup.bash
rosservice call /save_odometry
```
### 基于仿真数据的滤波定位

```bash
# 配置gnss_ins_sim_filtering.ymal，配置定位方式、滤波器权重

#  启动节点
source devel/setup.bash
roslaunch lidar_localization gnss_ins_sim_localization.launch

# 播放数据集

# 保存轨迹
source devel/setup.bash
rosservice call /save_odometry
```


### 基于kitti数据集的优化定位

目前只支持地图匹配+imu预积分的融合

```bash
# 配置sliding_window.ymal，配置地图加载路径、使能需要的约束和其对应的权重

#  启动节点
source devel/setup.bash
roslaunch lidar_localization lio_localization.launch

# 播放数据集

# 保存轨迹
source devel/setup.bash
rosservice call /save_odometry
```
