# Gsing Dog Simulation 2026

## 🚀 运行方法

### 1. 首先编译工作空间

```bash
# 进入你的项目目录（这就是你的工作空间）
cd /home/hyper/program/ROS/2026_Gsing_Dog_simulation-main

# 激活Python虚拟环境
source /home/hyper/program/ROS/ros2_jazzy_venv/bin/activate

# 激活系统ROS2
source /opt/ros/jazzy/setup.bash

# 编译（如果第一次运行或修改了C++代码）
colcon build --symlink-install

# 激活编译后的工作空间
source install/setup.bash





# 进入项目目录
cd /home/hyper/program/ROS/2026_Gsing_Dog_simulation-main

# 激活Python虚拟环境
source /home/hyper/program/ROS/ros2_jazzy_venv/bin/activate

# 激活系统ROS2
source /opt/ros/jazzy/setup.bash

# 激活工作空间（编译后的环境）
source install/setup.bash

# 启动仿真
ros2 launch simulation sim.launch.py






# 进入项目目录
cd /home/hyper/program/ROS/2026_Gsing_Dog_simulation-main

# 激活环境
source /home/hyper/program/ROS/ros2_jazzy_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 运行路径规划
ros2 run simulation path_planner.py
