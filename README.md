# CARLA 自动驾驶与路径规划仿真

## 项目简介

本项目包含基于 **CARLA Simulator** 的自动驾驶行为测试和路线规划脚本。通过调用 CARLA 的 Python API 与 `GlobalRoutePlanner`，实现了从简单的路径寻点漫游，到使用基础三角函数算法进行车辆控制的进阶演练。

项目内附带了对应的运行录像，以直观展示代码运行效果。

## 项目结构

```text
.
├── test3-对应录像1.py    # 基础路线规划与视角跟随
├── 录像1.mp4             # 对应 test3 的运行效果演示
├── Final-对应录像2.py    # 基于三角函数算法的车辆自动控制与遥测显示
└── 录像2.mp4             # 对应 Final 版本的运行效果演示
```

## 核心文件说明



### 1. `test3-对应录像1.py`
- **目标**：让车辆在选择好起点和终点路线后，自动沿路线移动，并在摄像头视角中标出规划的路线路点。
- **实现方式**：
  - 使用 CARLA 的 `GlobalRoutePlanner` 搜索全图最长可行路线。
  - 创建并挂载 RGB 相机到车辆 `*car1*` 上，使用 OpenCV 实时显示画面。
  - 通过不断更新 `vehicle.set_transform()` 瞬移车辆（未涉及真实的动力学控制算法），以最高效率走完路点。
  

![1](assets/1.gif)

### 2. `Final-对应录像2.py`

- **目标**：在获取路线后，车辆通过自身当前的坐标和朝向，通过算法运算控制车辆的油门和转向。
- **实现方式**：
  - 加载车辆 `*mini*` 并挂载 RGB 单目相机。
  - **转向控制**：通过 `get_angle` 函数，计算车辆当前朝向向量与目标路点之间的角度夹角（基本三角关系算法），并转化为 `steer` 控制信号。
  - **油门控制**：通过 `maintain_speed` 函数实现基础巡航，接近设定阈值时主动降速，保证过弯能力。
  - **UI 显示**：结合 OpenCV 在实时流画面上叠加 Telemetry（遥测）数据，如当前车速（Speed）、目标转向角（Steering angle）及下一个路点编号（Next waypoint）。
  - 通过 `vehicle.apply_control(carla.VehicleControl(...))` 真实控制车辆的底盘响应。

![2](assets/2.gif)

## 环境依赖

- **CARLA Simulator** (如 0.9.14)
- **Python 环境** (推荐 3.7 或直接使用 CARLA 针对 Python 的编译版)
- **Python 库**：
  - `carla` (CARLA 的 Python API)
  - `opencv-python` (`cv2` 用于画面显示和 GUI 文字叠加)
  - `numpy` (图像矩阵处理)

## 运行指南

1. 启动 CARLA 服务端 (例如运行 `CarlaUE4.exe` 或 Linux `./CarlaUE4.sh`)，默认监听 `localhost:2000`。
2. 确保你的 Python 环境中包含了 CARLA PythonAPI 包。如果你的 CARLA 安装路径不同，请在脚本中修改对应的库检索路径，例如：
   ```python
   # 脚本内预设路径为 C:\CARLA_0.9.14\WindowsNoEditor\PythonAPI\carla
   sys.path.append('你的 CARLA PythonAPI 路径')
   ```
3. 运行所需的测试脚本：
   ```bash
   python test3-对应录像1.py
   # 或
   python Final-对应录像2.py
   ```
4. 如果需要退出 `Final-对应录像2.py` 的仿真画面，在 OpenCV 窗口激活时按下 `q` 键即可自动刹车并销毁测试车辆。

