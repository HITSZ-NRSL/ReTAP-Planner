## Install
- 安装qhull，编译后build目录不要删
```bash
cd your_workspace/src/path_planning/lib/qhull/build
cmake ..
make -j
```
- 其他依赖
```
sudo apt install libompl-dev
```

## Global Planning
- 全局规划
```bash
roslaunch path_planning global_planning_exp.launch
```

## Startup
- 在rviz中先用publish point给定位置，再用2D nav goal给定方向，即可规划该位姿下的构型。
```bash
roslaunch path_planning local_planning_hybrid_astar_exp.launch
```