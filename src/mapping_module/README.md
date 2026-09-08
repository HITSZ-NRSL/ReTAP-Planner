## 功能介绍
### 全局地图：
- 订阅关键帧话题(参数```global_map/keyframe_topic```)，监听```lidar_frame->map_frame```的TF，合并关键帧到全局八叉树地图，并发布话题"mapping_module/global_octree"。将每一帧关键帧点云存盘，并更新记录record.txt。
- 订阅回环优化后的位姿图 ```global_map/loop_closure_topic```，根据最新的关键帧位姿，重新更新全局八叉树地图，并更新record.txt。

### 局部地图：
- 订阅去畸变后的点云帧话题(参数```local_map/lidar_topic```)，监听```lidar_frame->map_frame```的TF，合并到局部八叉树地图，滑窗更新机器人邻域包围框中的八叉树，并发布话题"mapping_module/local_octree"

### 地图保存：
- ```rosservice call /mapping_module/save_octomap ~/path_to_save_maps```

## Compliation
- ``` sudo apt install ros-melodic-octomap-msgs ros-melodic-octomap-ros libompl-dev libusb-dev binutils-dev ```

## 测试
  - ```roslaunch mapping_module mr1000_mapping.launch``` 

## SLAM测试
- 启动建图，配合fast-lio
  - ``` roslaunch mapping_module mapping_fast_lio_exp.launch ```
- **注意!!!**：
  - 用```rosbag play```测试时，需要加```--clock```参数，使用仿真时间；同时在```mapping```的```launch```文件中，打开```use_sim_time```参数
  - 用```rosbag play```连接```fast_lio```测试时，其```launch```文件中也应打开```use_sim_time```
  - 用真实雷达连接```fast_lio```测试时，```launch```文件中应关闭```use_sim_time```