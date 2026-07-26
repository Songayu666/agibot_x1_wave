# AgiBot X1 拟人动作控制

基于智元灵犀 X1 推理工程扩展的拟人动作控制项目。项目保留原有站立、行走、MuJoCo 仿真和 DCU/EtherCAT 真机控制链路，并增加右臂挥手、双臂交互、腰部协同及组合问候等 16 个动作。

> 当前项目在 **Ubuntu 22.04 + ROS 2 Humble** 环境中编译和运行。Windows 主要用于管理文件及通过 Git 上传代码，不能直接运行 `run_sim.sh` 或 `run.sh`。

## 已实现动作

| 动作 | ROS 2 话题 | 快捷脚本 | 主要运动部位 |
| --- | --- | --- | --- |
| 前臂挥手 | `/wave_forearm` | `wave_forearm.sh` | 右肩、右肘，以前臂摆动为主 |
| 整臂挥手 | `/wave_arm` | `wave_arm.sh` | 右肩与右肘协同 |
| 手腕挥手 | `/wave_wrist` | `wave_wrist.sh` | 右臂保持抬起，右腕滚转 |
| 招手并模拟点头 | `/greet_nod` | `greet_nod.sh` | 右臂与腰部俯仰 |
| 敬礼 | `/salute` | `salute.sh` | 右肩、右肘、右腕 |
| 双手抱拳 | `/fist_greeting` | `fist_greeting.sh` | 双肩、双肘、双腕 |
| 鼓掌 | `/clap` | `clap.sh` | 双臂在胸前往复靠近 |
| 双手比心 | `/heart` | `heart.sh` | 双臂形成近似心形轮廓 |
| 欢迎手势 | `/welcome` | `welcome.sh` | 双臂展开后收回 |
| 指向左侧 | `/point_left` | `point_left.sh` | 左肩、左肘、左腕 |
| 指向右侧 | `/point_right` | `point_right.sh` | 右肩、右肘、右腕 |
| 擦汗 | `/wipe_sweat` | `wipe_sweat.sh` | 右臂在额头前方移动 |
| 挠头 | `/scratch_head` | `scratch_head.sh` | 右臂在头部附近小幅运动 |
| 双臂连续舞蹈 | `/arm_dance` | `arm_dance.sh` | 双臂与腰部偏航、侧倾 |
| 鞠躬并挥手 | `/bow_wave` | `bow_wave.sh` | 腰部俯仰与右臂 |
| 组合问候 | `/greeting_combo` | `greeting_combo.sh` | 双臂连续完成挥手、敬礼和展开 |

`greet_nod` 当前没有控制真实头部关节，而是通过小幅腰部俯仰近似表现点头。抱拳、鼓掌和比心不控制手指或夹爪，只使用手臂整体姿态近似表达。

## 控制方法

动作采用人工设计的关节空间关键帧轨迹。每个动作在 YAML 中配置：

- 参与运动的关节列表；
- 各阶段目标关节角；
- 关节上下限；
- stiffness 与 damping；
- 最大速度、最大加速度和最大 jerk。

控制器使用动态自由度 Ruckig 在相邻关键帧之间生成平滑的目标位置序列，并逐周期下发目标位置及刚度、阻尼参数。动作触发时会记录当前关节姿态，并在轨迹结尾返回该姿态。

动作只能从 `stand` 或动作自身状态触发。测试结束后可执行：

```bash
./return_to_stand.sh
```

恢复标准站立状态。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- GCC 13
- CMake 3.26 或更高版本
- ONNX Runtime
- MuJoCo及原始 X1 推理工程所需依赖

真机运行还需要 X1 硬件、DCU/EtherCAT 通信环境、保护架和硬件急停。

## 编译

```bash
cd ~/agibot_x1_wave
source /opt/ros/humble/setup.bash
source url_gitee.bashrc
./build.sh $DOWNLOAD_FLAGS
chmod +x ./*.sh
```

修改 `src/module/control_module/cfg/` 中的 YAML 后必须重新编译，因为构建过程会将配置复制到 `build/`。

## MuJoCo仿真

终端 A：

```bash
cd ~/agibot_x1_wave/build
./run_sim.sh
```

通过手柄依次进入 `zero` 和 `stand`，确认机器人稳定站立。不要进入 `walk`。

终端 B：

```bash
cd ~/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/install/ros2_setup.sh
./wave_forearm.sh
```

也可以通过统一入口运行复杂动作：

```bash
./run_action.sh welcome
./return_to_stand.sh

./run_action.sh arm_dance
./return_to_stand.sh

./run_action.sh greeting_combo
./return_to_stand.sh
```

## 真机运行

完成仿真验证后，在保护架、硬件急停和现场安全人员就绪的条件下启动：

```bash
cd ~/agibot_x1_wave/build
./run.sh
```

在另一个已加载 ROS 2 环境的终端执行动作脚本。首次测试建议从 `wave_forearm`、`welcome` 等幅度较小或远离头部的动作开始。

> 真机出现振荡、异响、姿态失稳、通信掉线或跟踪异常时，应立即使用硬件急停。`stand` 指令不能替代硬件急停。

## 推荐测试顺序

1. `wave_forearm`
2. `wave_arm`
3. `wave_wrist`
4. `welcome`
5. `point_left`、`point_right`
6. `salute`
7. `fist_greeting`、`heart`
8. `clap`
9. `greet_nod`
10. `wipe_sweat`、`scratch_head`
11. `arm_dance`
12. `bow_wave`
13. `greeting_combo`

每个动作应先在 MuJoCo 中连续测试，确认没有穿模、突跳、明显失衡或控制报错，再部署到真机。

## 动作配置位置

- 真机配置：`src/module/control_module/cfg/rl_x1.yaml`
- 仿真配置：`src/module/control_module/cfg/rl_x1_sim.yaml`
- 动作触发入口：`run_action.sh`
- 独立动作脚本：项目根目录下的 `*.sh`

关键帧格式：

```text
[到下一关键帧的最短时间（秒）, 关节角1（弧度）, 关节角2（弧度）, ...]
```

新增动作时，需要同步完成：

1. 在 `rl_x1.yaml` 中增加动作控制器和状态；
2. 将动作加入 `stand.pre_states`；
3. 在 `rl_x1_sim.yaml` 中增加状态，并把控制器加入 `controller_imports`；
4. 在 `run_action.sh` 白名单中加入动作名；
5. 创建快捷脚本；
6. 重新编译并先完成仿真验证。

更详细的测试及安全说明见 [`WAVE_GUIDE.zh_CN.md`](WAVE_GUIDE.zh_CN.md)。

## 已知问题

- 进入 `walk` 后，髋部 EtherCAT/DCU 可能掉线。当前上肢动作测试均在原地 `stand` 状态进行。
- 当前项目没有可用的头部控制接口，点头动作由腰部俯仰近似实现。
- 当前夹爪不能复现灵巧手指型，抱拳、比心、鼓掌和指向动作属于视觉近似。

## 目录说明

```text
agibot_x1_wave/
├── src/                         核心源码、控制模块、仿真模块和驱动模块
├── doc/                         原工程说明及硬件资料
├── cmake/                       CMake辅助脚本
├── build.sh                     构建脚本
├── run_action.sh                动作统一触发入口
├── return_to_stand.sh           恢复站立
├── *_action.sh / wave_*.sh      动作快捷脚本
├── WAVE_GUIDE.zh_CN.md          动作测试和安全指南
└── LICENSE.txt                  开源许可证
```

`build/` 是本地编译产物，体积较大，不应提交到 GitHub。

## 许可证

本项目沿用仓库中的 [`LICENSE.txt`](LICENSE.txt)。使用和分发前请阅读许可证及原始项目的相关说明。

