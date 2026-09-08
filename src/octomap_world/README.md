## 读取八叉树并发布```publish_loaded_octree.cpp```
- 载入本地文件
- 发布到指定话题
- 文件名、话题名通过```rosparam```获取

## 订阅八叉树并存盘```save_advertised_octree.cpp```
- 订阅指定话题
- 存到指定路径
- 文件名、话题名通过```rosparam```获取

## 测试两个节点
- ```roslaunch octomap_world octomap_test.launch```
- ```rviz```中显示加载的地图