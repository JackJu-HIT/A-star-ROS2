

# A-Star-ROS2
**基于 ROS 2 的 A* 自主避障寻路算法**

这是一个基于 ROS 2 框架实现的 A* (A-Star) 路径规划器，支持在已知障碍物环境下的自主避障与路径生成。

## 📦 环境依赖
*   **ROS 2** (Humble/Foxy/Iron 等)
*   **RViz2**

## 🛠️ 编译与构建 (Build)

请确保你已创建了 ROS 2 工作空间（workspace），并将本项目放入 `src` 目录下：

```bash
cd ~/ros2_ws/src
# 克隆代码到此处 
# git clone [<your-repo-url>](https://github.com/JackJu-HIT/A-star-ROS2.git)

cd ~/ros2_ws
colcon build 
source install/setup.bash
```

## 🚀 运行指南 (Usage)

请按照以下顺序启动节点和可视化工具：

### 1. 启动路径规划节点
```bash
# 推荐使用 ros2 run 方式运行
ros2 run a_star_planner a_star_plan

# 或者直接运行编译后的二进制文件 (不推荐)
# ./build/a_star_planner/a_star_plan
```

### 2. 启动可视化界面
打开一个新的终端并运行 RViz2：
```bash
rviz2
```

### 3. 配置地图与障碍物
在节点运行后，程序会加载默认的全局轨迹与障碍物信息（或通过参数文件/代码配置）。

### 4. 设置起始点
在 RViz 工具栏中使用 **2D Pose Estimate** 工具点击地图，设置机器人的初始位置，算法将自动开始规划路径。

## 📸 运行效果 (Results)

![A* 算法运行效果图](https://github.com/JackJu-HIT/A-star-ROS2/blob/master/a_star_planner/results.png?raw=true)
