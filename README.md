# 水下机器人自主巡检 · ROS 2 / Stonefish

基于 [HERO Lab Stonefish](https://github.com/HERO-Lab-POSTECH/stonefish_sim) 的个人水下自主巡检项目。BlueROV2 在真实水动力仿真中绕开立柱和箱体，通过实际相机图像发现巡检面板，视觉伺服接近、稳定观察，最后返回起点。

该版本是可复现的工程基线：已知场地 A* 规划、HSV 几何标记识别、PID 位姿控制、实际推进器分配和水流扰动。导航状态来自仿真真值里程计；没有宣称实现水下 SLAM、未知障碍探索、学习策略、机械臂操作或实机迁移。

## 环境与运行

Ubuntu 22.04、ROS 2 Humble、OpenGL 4.3+、Python 3.10。开发验证机器为 RTX 5090 D。请克隆到 **ASCII 路径**，ROS Humble 的消息生成器不能正确读取中文构建路径。

```bash
git clone --recurse-submodules https://github.com/sencangWei/underwater-inspection-ros2.git
cd underwater-inspection-ros2
bash scripts/build.sh
source scripts/env.sh
python3 scripts/run_demo.py --output evidence/my-run
```

`scripts/env.sh` 使用系统 Python/NumPy，设置独立 ROS_DOMAIN_ID=73，并选择 NVIDIA OpenGL；不会修改全局 Python 包。GLM 固定版本安装到 `.local`，Stonefish 使用用户目录安装前缀。原生依赖需要事先安装：

```bash
sudo apt install build-essential cmake libsdl2-dev libfreetype6-dev libopengl-dev \
  python3-colcon-common-extensions python3-opencv python3-scipy \
  python3-transforms3d python3-yaml python3-pytest \
  ros-humble-desktop ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-cv-bridge ros-humble-image-transport
```

本仓库锁定上游核心 `eb258a83e25888a091105b8c4d77b962732f880a`（1.3.0）和集成 `8915a77ae6d0c220ee3464993d920b578433271f`。构建脚本应用 `patches/stonefish_runtime.patch`，修复上游退出时 Node 被重复删除的错误；保留上游 GPL 许可证和来源。

## 任务与证据

场景和任务参数位于 `src/underwater_inspection/config/mission.yaml`。默认起点 NED `(-5,0,2.5)`，面板 `(5.5,0,2.5)`，水流 `(0,0.1,0)` m/s，要求在相机前方约 1.3 m 连续稳定观察 8 秒。

```text
TRANSIT_AVOID_OBSTACLES → SEARCH_PANEL → VISUAL_APPROACH
    → INSPECT_HOLD → RETURN_HOME → COMPLETE
```

每次运行生成阶段事件、10 Hz 控制日志、相机帧时间戳、实际传感器视频、截图、巡检原图与 summary.json；失败也保留原因和日志。显示窗口由独立 viewer 进程负责，避免 GUI 阻塞控制任务。

时间：上游没有 `/clock` 发布器，传感器时间戳使用 ROS 系统时间。本版各控制节点统一系统时间，报告使用 wall seconds，视频按真实帧时间戳对齐。

复盘见 [docs/复盘.md](docs/复盘.md)，上游比较见 [docs/上游选择.md](docs/上游选择.md)。最终实际运行结果和 Demo 将在验证后列出。

## 基础检查

```bash
source scripts/env.sh
PYTHONPATH=src/underwater_inspection python3 -m pytest -q src/underwater_inspection/test
```

测试检查绕障路线整段安全间距、阻塞起点拒绝、背景排除/图像测距，并使用实际水下 RGB 帧验证黄色面板因光吸收变绿后仍可识别。

## 后续研究方向

1. 用 DVL/IMU/深度融合替代真值定位，并比较漂移和任务成功率。
2. 声呐在线障碍检测与重新规划，加入未知障碍和传感器掉帧恢复。
3. 利用本项目记录的感知/动作数据训练局部接近策略，与当前视觉伺服基线比较；逐步加入随机水流与能见度测试。
4. 巡检成功后扩展浮动平台机械臂的触碰/阀门操作。
