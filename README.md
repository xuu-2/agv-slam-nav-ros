# AGV 自主导航系统

基于 ROS 的四轮 AGV（自动导引车）建图与导航工程。融合 **速腾聚创 3D 激光雷达**、**维特智能 IMU**、**里程计**，支持 Cartographer / Gmapping 建图，以及 AMCL + move_base 自主导航。

## 功能概览

| 能力 | 说明 |
|------|------|
| 3D → 2D 感知 | `pointcloud_to_laserscan` 将雷达点云转为 2D 激光扫描 |
| 位姿融合 | IMU + 里程计，可选 `robot_pose_ekf` 扩展卡尔曼滤波 |
| SLAM 建图 | 支持 Gmapping / Cartographer（3D）等 |
| 定位导航 | AMCL 定位 + move_base 打点导航（DWA / TEB） |
| 地图管理 | 多张已建地图（`.pgm` / `.yaml`），脚本保存 Cartographer 地图 |

## 目录结构

```
AGV/
├── agv.sh                  # 一键启动：雷达 + 底盘融合 + Cartographer 定位 + move_base
├── start_navigation.sh     # 建图流程启动脚本
├── save_map.sh             # Cartographer 地图保存（pbstream → ros map）
├── 代码说明.txt            # 简要功能对照说明
└── agv_hqcartest/          # ROS catkin 工作空间主体
    └── src/
        ├── car_top/                # 底盘驱动、串口、里程计/IMU 发布、建图导航 launch
        ├── hq_car_description/     # 四轮车 URDF 模型
        ├── wit_ros_imu/            # 维特智能 IMU ROS 驱动
        ├── pointcloud_to_laserscan/# 3D 点云转 2D LaserScan
        ├── nav/                    # 导航参数、AMCL / move_base launch、RViz 配置
        ├── robot_pose_ekf/         # 位姿 EKF 融合
        ├── teb_local_planner/      # TEB 局部路径规划器
        └── navigation/             # ROS Navigation 栈（move_base、amcl、costmap 等）
```

> **说明：** 完整运行还依赖本仓库外的工作空间：`rslidar_sdk`（雷达驱动）、`cartographer`（建图定位）。脚本中路径默认为 `~/rslidar_sdk`、`~/cartographer`、`~/hqcartest`。

## 功能包说明

### `car_top`（核心）

底盘与上层控制节点：

| 可执行文件 | 作用 |
|------------|------|
| `car_top` | 主控 / 融合相关节点 |
| `vel_top` | 速度相关控制 |
| `pub_car_message` | 底盘消息发布 |
| `pub_imu` | IMU 数据发布 |
| `pub_car_odom` | 里程计发布 |

主要 Launch：

- `build_map.launch` — 建图（可切换 gmapping / hector / cartographer / karto）
- `cartographer_movebase.launch` — Cartographer 模式：底盘 + 点云转扫描 + IMU + 加载地图
- `car_top.launch` — 底盘基础启动

地图文件位于 `car_top/map/`（如 `map16`、`map3333` 等），当前默认加载 `map16.yaml`。

### `hq_car_description`

四轮差速车 URDF（`four_whell.urdf`），供 TF、可视化与导航 footprint 使用。

### `wit_ros_imu`

维特智能 IMU 驱动（串口 / Modbus / CAN 等），提供姿态与 TF。详见包内 `使用说明.md`。

### `pointcloud_to_laserscan`

将 3D 点云（如 `/scan_matched_points2`）投影为 2D `LaserScan`，供 2D 导航栈使用。

### `nav`

导航配置与启动：

- AMCL 定位
- move_base（代价地图、全局规划、DWA 局部规划等参数）
- RViz：`demo_nav.rviz`、`record_trac.rviz`

### `navigation` / `teb_local_planner` / `robot_pose_ekf`

标准 ROS Navigation 栈、TEB 局部规划器、机器人位姿 EKF，供建图与导航链路调用。

## 系统依赖

- **OS：** Ubuntu（推荐 16.04 / 18.04 / 20.04，与所用 ROS 发行版匹配）
- **ROS：** Kinetic / Melodic / Noetic（按实际环境选择）
- **外部工作空间：**
  - [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk) — 速腾 3D 雷达
  - Cartographer ROS — 3D SLAM / 定位（脚本使用改写过的配置）
- **硬件：** 四轮 AGV 底盘、3D 激光雷达、维特 IMU、串口通信

## 编译

将本仓库中的 `agv_hqcartest` 放到 catkin 工作空间（或重命名为 `hqcartest` 以匹配脚本路径）：

```bash
# 示例：工作空间为 ~/hqcartest
cd ~/hqcartest
# 确保 src 下包含本仓库的功能包
catkin_make
# 或: catkin build
source devel/setup.bash
```

依赖示例（按 ROS 发行版替换 `noetic`）：

```bash
sudo apt-get install ros-noetic-navigation ros-noetic-robot-pose-ekf \
  ros-noetic-gmapping ros-noetic-map-server ros-noetic-amcl
```

## 使用方法

脚本使用 `gnome-terminal` 多标签启动，请先根据本机路径修改 `~/rslidar_sdk`、`~/hqcartest`、`~/cartographer`。

### 1. 建图

```bash
chmod +x start_navigation.sh
./start_navigation.sh
```

典型流程：底盘 `car_top` → 雷达 `rslidar_sdk` → IMU → Cartographer `backpack_3d` 建图。

### 2. 保存地图（Cartographer）

```bash
chmod +x save_map.sh
# 按需修改脚本中的 pbstream / map 输出路径
./save_map.sh
```

脚本会调用：

1. `/finish_trajectory`
2. `/write_state` 写出 `.pbstream`
3. `cartographer_pbstream_to_ros_map` 转为 `.pgm` / `.yaml`

### 3. 定位 + 导航

```bash
chmod +x agv.sh
./agv.sh
```

依次启动：

1. 雷达驱动 `rslidar_sdk/start.launch`
2. `car_top/cartographer_movebase.launch`（数据融合 + 地图）
3. Cartographer 3D 定位 `demo_backpack_3d_localization.launch`
4. `nav/move_base.launch`（打点导航）

也可单独启动：

```bash
roslaunch car_top build_map.launch          # 建图
roslaunch nav car_run.launch                # AMCL + move_base 导航
roslaunch car_top cartographer_movebase.launch
```

在 RViz 中使用 **2D Nav Goal** 发送目标点即可导航。

## 数据流（简图）

```
雷达点云 ──► Cartographer / pointcloud_to_laserscan ──► LaserScan
IMU + 里程计 ──► (可选 EKF) ──► odom / TF
地图 map_server + AMCL / Cartographer 定位 ──► move_base ──► cmd_vel ──► 底盘
```

## 注意事项

1. **路径硬编码：** `agv.sh`、`start_navigation.sh`、`save_map.sh` 中的绝对路径需改成你的本机路径。
2. **串口权限：** IMU / 底盘常见设备为 `/dev/ttyUSB0`，需 `sudo chmod 777 /dev/ttyUSB0` 或配置 udev。
3. **地图切换：** 修改 `car_top/launch/include/read_map.launch` 中的 `map` 参数（默认 `map16.yaml`）。
4. **Cartographer 与 EKF：** 使用 Cartographer 时通常不再启用 `robot_pose_ekf`（相关 launch 已用参数控制）。
5. **仓库体积：** `car_top/map/` 含多张栅格地图，上传 GitHub 前可考虑 Git LFS 或精简不需要的地图。

## 相关文件对照

| 文件 | 作用 |
|------|------|
| `agv.sh` | AGV 运行（定位 + 导航） |
| `start_navigation.sh` | 建图启动 |
| `save_map.sh` | 地图保存 |
| `代码说明.txt` | 功能包与脚本速查 |

## License

各子包 License 以对应 `package.xml` / `LICENSE` 为准；部分第三方包（如 `teb_local_planner`、`navigation`）遵循其原仓库协议。

---

如需补充仿真、具体硬件型号或参数调优说明，可在 Issue / Wiki 中继续完善。
