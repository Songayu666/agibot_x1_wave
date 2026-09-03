# 智元灵犀 X1 拟人动作控制

[English](README.md)

本项目基于智元灵犀 X1 推理工程扩展，保留原有零位、站立、行走、MuJoCo 仿真和 DCU/EtherCAT 真机控制链路，并增加 16 个上肢及问候动作。

项目运行于 x86-64 Ubuntu 22.04，使用 ROS 2 Humble、GCC 13 和 CMake 3.26+，以 [AimRT](https://aimrt.org/) 为中间件，通过 ONNX Runtime 运行强化学习策略，并使用 MuJoCo 进行仿真。

> **当前验证状态：** ROS 2 动作触发和状态转换已经正常工作，但 `wave_forearm` 在 MuJoCo 中开始执行时会使机器人倒地。因此新增动作目前**尚未通过仿真验收，禁止部署到真机**。详见[开发记录](DEVELOPMENT_NOTES.zh_CN.md#8-当前仿真问题)。

![智元灵犀 X1](doc/x1.jpg)

## 已实现动作

| 动作 | ROS 2 话题 | 执行命令 |
| --- | --- | --- |
| 前臂挥手 | `/wave_forearm` | `./wave_forearm.sh` |
| 整臂挥手 | `/wave_arm` | `./wave_arm.sh` |
| 手腕挥手 | `/wave_wrist` | `./wave_wrist.sh` |
| 招手并模拟点头 | `/greet_nod` | `./run_action.sh greet_nod` |
| 敬礼 | `/salute` | `./run_action.sh salute` |
| 双手抱拳 | `/fist_greeting` | `./run_action.sh fist_greeting` |
| 鼓掌 | `/clap` | `./run_action.sh clap` |
| 双臂比心 | `/heart` | `./run_action.sh heart` |
| 欢迎手势 | `/welcome` | `./run_action.sh welcome` |
| 左右指向 | `/point_left`、`/point_right` | `./run_action.sh point_left` |
| 擦汗 | `/wipe_sweat` | `./run_action.sh wipe_sweat` |
| 挠头 | `/scratch_head` | `./run_action.sh scratch_head` |
| 双臂舞蹈 | `/arm_dance` | `./run_action.sh arm_dance` |
| 鞠躬挥手 | `/bow_wave` | `./run_action.sh bow_wave` |
| 组合问候 | `/greeting_combo` | `./run_action.sh greeting_combo` |

机器人没有可控头部关节，因此 `greet_nod` 通过小幅腰部俯仰近似点头。项目不控制手指造型，鼓掌、比心、指向和抱拳均通过手臂整体姿态近似表达。

## 软件架构

```text
手柄或 ROS 2 话题
→ AimRT 状态机
→ PD 或强化学习控制器
→ /joint_cmd
→ MuJoCo SimModule 或真机 DcuDriverModule
```

新增动作只能从 `stand` 状态触发。动作开始时，控制器读取实时关节位置，将其作为轨迹起点和返回姿态。Ruckig 用于限制关键帧之间的速度、加速度和 jerk。

## 环境要求

- x86-64 Ubuntu 22.04
- ROS 2 Humble
- GCC/G++ 13
- CMake 3.26+
- ONNX Runtime
- `libglfw3-dev` 和 `libdart-external-lodepng-dev`
- 手柄不是必需品，也可以直接发布 ROS 2 状态话题

真机运行还需要 X1 硬件、正确配置的 EtherCAT 接口、实时 Linux 环境、保护架或悬吊设备以及物理急停。

## 编译

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source url_gitee.bashrc
./build.sh $DOWNLOAD_FLAGS
chmod +x ./*.sh
```

修改 `src/module/**/cfg/` 下的配置后必须重新构建，因为运行配置会在构建时复制到 `build/cfg/`。

## 无手柄运行 MuJoCo 仿真

终端 A：

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/ros2_setup.sh
export LD_LIBRARY_PATH="$PWD/build/install/lib:${LD_LIBRARY_PATH:-}"
cd build
./run_sim.sh
```

仿真初始状态为 `idle`，其 PD 刚度为 0，因此机器人可能在启动时倒地。随后在终端 B 切换到 `zero`：

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/ros2_setup.sh
ros2 topic pub --once /zero_mode std_msgs/msg/Float32 '{data: 1.0}'
```

确认终端 A 出现 `Trigger event: [/zero_mode] -> zero` 后，在 MuJoCo 中点击一次 **Simulation → Reset**，然后进入 `stand`：

```bash
ros2 topic pub --once /stand_mode std_msgs/msg/Float32 '{data: 1.0}'
```

正确启动顺序是：

```text
启动仿真 → zero → MuJoCo Reset → stand
```

进入 `stand` 后不要再次 Reset，否则仿真物理状态与控制器状态可能不同步。动作通过安全验证后，可以执行：

```bash
./wave_forearm.sh
./return_to_stand.sh
```

目前除非正在定位已知倒地问题，否则应在执行动作前停止。

## 真机运行

新增动作必须先在 MuJoCo 中重复测试，确认无倒地、自碰撞、关节突跳和控制错误后，才能考虑部署真机。基础真机控制链路启动方式：

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/ros2_setup.sh
cd build
./run.sh
```

启动前应核对 `src/module/dcu_driver_module/cfg/dcu_x1.yaml` 中的 EtherCAT 网卡名称。`/stand_mode` 不能代替物理急停。

## 关键文件

```text
src/module/control_module/                   状态机及 PD/强化学习控制
src/module/control_module/cfg/rl_x1.yaml     真机控制配置
src/module/control_module/cfg/rl_x1_sim.yaml 仿真控制配置
src/module/sim_module/                       MuJoCo 仿真与模型
src/module/dcu_driver_module/                DCU/EtherCAT 真机驱动
src/module/joy_stick_module/                 手柄输入
run_action.sh                                动作统一入口
WAVE_GUIDE.zh_CN.md                          动作与安全指南
DEVELOPMENT_NOTES.zh_CN.md                   开发及问题排查记录
```

本地 `build/` 目录包含大量构建产物，不应提交到 GitHub。

## 文档

- [English README](README.md)
- [动作与安全指南](WAVE_GUIDE.zh_CN.md)
- [开发记录](DEVELOPMENT_NOTES.zh_CN.md)
- [原始 X1 开发指南](doc/tutorials.zh_CN.md)
- [手柄模块](doc/joy_stick_module/joy_stick_module.zh_CN.md)
- [DCU 驱动模块](doc/dcu_driver_module/dcu_driver_module.zh_CN.md)
- [强化学习控制模块](doc/rl_control_module/rl_control_module.zh_CN.md)

## 安全要求

- 每个动作必须先在 MuJoCo 中反复测试，再考虑部署真机；
- 真机首次测试必须使用保护架或悬吊；
- 另一名操作人员应始终握住物理急停；
- 出现振荡、异响、关节突跳、失稳、跟踪异常或 DCU 掉线时立即急停；
- 未重新完成仿真验收前，不提高真机关节限制、速度、加速度、jerk 或 PD 增益；
- 近头动作和双臂接近动作必须逐帧检查自碰撞。

## 许可证

本研究项目运行于 [AimRT](https://aimrt.org/) 框架之上，使用[木兰宽松许可证第 2 版](LICENSE.txt)发布，不对特定用途的适用性提供保证。
