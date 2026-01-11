
# A-Star-ROS2

**基于 ROS 2 的 A* 自主避障寻路算法**

> 🚀 本项目基于 ROS 2 框架实现了 A* (A-Star) 路径规划器，能够在已知障碍物环境（如栅格地图或点云）下进行自主避障与最优路径生成。

## 📦 环境依赖 (Prerequisites)

*   **OS**: Ubuntu 20.04 / 22.04
*   **ROS 2**: Humble / Foxy / Iron
*   **Visualization**: RViz2

## 🛠️ 编译与构建 (Build)

请确保您已创建了 ROS 2 工作空间（workspace），并将本项目克隆到 `src` 目录下：

```bash
# 1. 进入工作空间的 src 目录
cd ~/ros2_ws/src

# 2. 克隆代码
git clone https://github.com/JackJu-HIT/A-star-ROS2.git

# 3. 返回工作空间根目录并编译
cd ~/ros2_ws
colcon build --packages-select a_star_planner

# 4. 设置环境变量
source install/setup.bash
```

## 🚀 运行指南 (Usage)

请按照以下顺序启动节点和可视化工具：

### 1. 启动路径规划节点
```bash
# 推荐使用 ros2 run 方式运行
ros2 run a_star_planner a_star_plan
```

### 2. 启动可视化界面
打开一个新的终端（**注意：新终端也需要 source 环境**）：
```bash
cd ~/ros2_ws
source install/setup.bash
rviz2
```
*建议加载项目目录下的 `.rviz` 配置文件（如有），以便正确显示 Marker 和 Topic。*

### 3. 初始化与规划
1.  **加载环境**：节点启动后，程序会自动加载默认的全局轨迹与障碍物信息。
2.  **设置起点**：在 RViz 工具栏中点击 **2D Pose Estimate**。
3.  **触发规划**：在地图可行区域点击设置机器人的初始位置，算法将自动计算并生成避障路径。

## 📸 运行效果 (Results)

![A* 算法运行效果图](https://github.com/JackJu-HIT/A-star-ROS2/blob/master/a_star_planner/results.png?raw=true)

## 🙌 致谢 (Acknowledgements)

本项目核心算法逻辑参考并修改自以下优秀的开源项目，特此致谢：

*   **Ego-Planner**: [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)

## 💬 关注与交流 (Contact)

如果您对机器人规划控制感兴趣，欢迎关注我的频道获取更多技术分享：

*   📱 **微信公众号**：机器人规划与控制研究所
*   📺 **Bilibili**：[机器人算法研究所](https://space.bilibili.com/your-uid-here) *(建议此处替换为您的B站主页链接)*

