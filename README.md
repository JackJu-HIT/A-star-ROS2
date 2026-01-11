# A-Star-ROS2: 机器人自主避障寻路算法

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Language-C++](https://img.shields.io/badge/Language-C%2B%2B-red)](https://en.cppreference.com/)
[![License-MIT](https://img.shields.io/badge/License-MIT-green)](https://opensource.org/licenses/MIT)

> **本项目基于 ROS 2 框架实现了高效的 A* (A-Star) 路径规划器。** 它能够实时处理环境中的障碍物信息（栅格地图/点云），并生成一条安全、最优的自主避障路径，适用于轮式机器人、无人机等移动平台的局部或全局规划。

---

## 📦 环境要求 (Prerequisites)

*   **操作系统**: Ubuntu 22.04 (推荐) / 20.04
*   **ROS 2 版本**: Humble (推荐) / Foxy
*   **依赖库**: 
    *   [Eigen 3](https://eigen.tuxfamily.org/)
    *   PCL (Point Cloud Library)
    *   Standard ROS 2 geometry/nav messages

---

## 🛠️ 编译与构建 (Build)

```bash
# 1. 创建并进入工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. 克隆本项目
git clone https://github.com/JackJu-HIT/A-star-ROS2.git

# 3. 编译指定包
cd ~/ros2_ws
colcon build --symlink-install --packages-select a_star_planner

# 4. 激活环境
source install/setup.bash
```

---

## 🚀 快速启动 (Quick Start)

### 1. 运行路径规划节点
```bash
ros2 run a_star_planner a_star_plan
```

### 2. 启动 RViz2 可视化
在新的终端中运行：
```bash
rviz2 -d src/A-star-ROS2/a_star_planner/config/default.rviz
```
*(注：如果项目提供了配置文件，直接加载即可；否则需手动添加以下话题)*

---

## 📊 话题接口 (Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/visual_local_trajectory` | `nav_msgs/Path` | **规划结果**：A* 算法生成的最终避障轨迹 |
| `/visual_global_path` | `nav_msgs/Path` | **全局基准**：起始点到终点的原始参考直线 |
| `/visual_local_obstacles` | `sensor_msgs/PointCloud2` | **环境感知**：当前规划器识别到的局部障碍物点云 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | **交互接口**：通过 RViz 接收机器人起点位置 |

---

## 🎮 操作说明 (Usage)

1.  **加载地图**：启动节点后，系统会默认生成虚拟障碍物环境。
2.  **设置起点**：点击 RViz 工具栏顶部的 **"2D Pose Estimate"** 按钮。
3.  **生成路径**：在地图上任意可行区域（空白处）点击，规划器将以此点作为起点，自动计算避障路径并实时发布。
4.  **调整环境**：可以通过配置文件动态调整障碍物膨胀半径（Inflation Radius）以适应不同尺寸的机器人。

---

## 📸 运行效果 (Results)

![A* 算法运行效果图](https://github.com/JackJu-HIT/A-star-ROS2/blob/master/a_star_planner/results.png?raw=true)

---

## 🙌 致谢 (Acknowledgements)

本项目的开发参考了以下优秀开源项目，在此表示由衷的感谢：

*   **EGO-Planner**:  [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)

---

## 💬 交流与反馈 (Contact)

如果您对机器人**规划、控制**感兴趣，欢迎通过以下方式交流学习：

*   📱 **微信公众号**：[机器人规划与控制研究所](https://mp.weixin.qq.com/s/DnsGCi86n4Fzjbb18bKR3g) - 分享前沿算法与工程实践。
*   📺 **Bilibili**：[机器人算法研究所](https://space.bilibili.com/3493138800925925) 
*   📧 **Email**: [juchunyu@qq.com]
