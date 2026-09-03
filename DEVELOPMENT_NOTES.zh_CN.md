# AgiBot X1 Wave 开发记录

本文记录 `agibot_x1_wave` 的实现目标、关键文件、构建与运行方式、历史问题及当前验证状态，便于后续维护和复盘。

> 当前项目位于 `/home/zhongde/agibot_ws/src/agibot_x1_wave`，运行环境为 x86-64 Ubuntu 22.04、ROS 2 Humble、G++ 13 和 CMake 3.28.4。本机可以直接编译并运行 MuJoCo 仿真；只有连接并正确配置 X1 真机硬件后才能运行真机控制。本文不再使用旧环境中的 `/mnt/mydisk/robot_ws` 路径，也不将 Windows 作为当前项目运行或 Git 管理环境。

## 1. 实现目标

项目从官方 `agibot_x1_infer` 控制工程复制为独立项目，原项目保持不变。在原有零位、站立和行走能力基础上，增加以下三种右手挥手动作：

| ROS 2 话题 | 脚本 | 动作说明 |
| --- | --- | --- |
| `/wave_forearm` | `wave_forearm.sh` | 抬起右臂，主要使用前臂姿态挥手 |
| `/wave_arm` | `wave_arm.sh` | 使用整条右臂进行较大幅度挥动 |
| `/wave_wrist` | `wave_wrist.sh` | 保持手臂抬起，主要使用右腕滚转关节挥动 |

三个动作只能从 `stand` 状态触发。设计动作流程为：

```text
当前右臂姿态
→ 平滑抬手
→ 往复挥动 3 次
→ 平滑返回触发前姿态
→ 保持当前姿态
```

动作完成后，通过 `/stand_mode` 或 `return_to_stand.sh` 恢复标准站立状态。

项目随后还增加了敬礼、鼓掌、比心、指向、擦汗、挠头、手臂舞蹈、鞠躬挥手和组合问候等动作。完整列表参见 `WAVE_GUIDE.zh_CN.md`。

## 2. 控制逻辑

控制数据流如下：

```text
手柄或 ROS 2 话题
→ AimRT 状态机
→ PD/强化学习控制器
→ /joint_cmd
→ MuJoCo SimModule 或真机 DcuDriverModule
```

### 状态机

`ControlModule` 根据 YAML 中的 `robot_states` 注册动作话题。收到触发消息后，状态机会检查当前状态是否包含在目标动作的 `pre_states` 中。挥手动作的前置状态为 `stand`，因此在 `idle`、`zero` 或 `walk` 状态发送挥手命令不会进入动作。

进入动作状态后，控制模块组合以下控制器：

- `pd_zero`：为全身关节提供基础控制；
- `pd_stand`：维持站立姿态；
- 对应动作控制器，例如 `pd_wave_forearm`：覆盖指定右臂关节。

动作控制器启动时会读取实时关节反馈，将当前姿态作为轨迹起点和终点，通过 Ruckig 限制速度、加速度和 jerk。

### 仿真与真机模块

- 仿真模式加载 `JoyStickModule + ControlModule + SimModule`；
- 真机模式加载 `JoyStickModule + ControlModule + DcuDriverModule`；
- `SimModule` 使用 MuJoCo 模型计算动力学；
- `DcuDriverModule` 通过 EtherCAT/DCU 与真机执行器通信；
- 行走控制器通过 ONNX Runtime 加载强化学习策略。

## 3. 关键文件

### 控制器源码

```text
src/module/control_module/
├── include/control_module/
├── src/control_module.cc
├── src/pd_controller.cc
├── src/rl_controller.cc
└── src/utilities.cc
```

主要职责包括：

- 订阅模式和动作话题；
- 执行状态转换检查；
- 接收 IMU 和关节反馈；
- 运行 PD 或强化学习控制器；
- 使用 Ruckig 生成动作轨迹；
- 发布 `/joint_cmd`。

### 配置文件

| 用途 | 源配置 | 当前构建后配置 |
| --- | --- | --- |
| 仿真控制 | `src/module/control_module/cfg/rl_x1_sim.yaml` | `build/cfg/control_module/rl_x1_sim.yaml` |
| 真机控制 | `src/module/control_module/cfg/rl_x1.yaml` | `build/cfg/control_module/rl_x1.yaml` |
| DCU/EtherCAT | `src/module/dcu_driver_module/cfg/dcu_x1.yaml` | `build/cfg/dcu_driver_module/dcu_x1.yaml` |
| MuJoCo | `src/module/sim_module/cfg/sim_x1.yaml` | `build/cfg/sim_module/sim_x1.yaml` |
| 手柄 | `src/module/joy_stick_module/cfg/joy_x1.yaml` | `build/cfg/joy_stick_module/joy_x1.yaml` |

构建过程中也会生成 `build/install/bin/cfg/` 下的安装副本，但当前 `build/run_sim.sh` 和 `build/run.sh` 从 `build/cfg/` 加载配置。修改 `src/module/**/cfg/` 后必须重新构建，避免源码配置与运行配置不一致。

### 启动与环境脚本

```text
build/run_sim.sh       # 启动 MuJoCo 仿真
build/run.sh           # 启动真机控制
build/ros2_setup.sh    # 加载构建生成的 ROS 2 消息环境
```

## 4. 动作脚本

动作脚本通过 ROS 2 发布一次 `std_msgs/msg/Float32` 消息。例如 `wave_forearm.sh`：

```bash
#!/usr/bin/env bash
set -euo pipefail
ros2 topic pub --once /wave_forearm std_msgs/msg/Float32 '{data: 1.0}'
```

其他基础脚本使用相同结构：

```bash
./wave_arm.sh
./wave_wrist.sh
./return_to_stand.sh
```

复杂动作可以通过统一入口执行：

```bash
./run_action.sh welcome
./run_action.sh salute
./run_action.sh greeting_combo
```

脚本需要具有执行权限：

```bash
chmod +x ./*.sh
```

## 5. 当前环境检查结果

当前电脑已经确认：

```text
系统：Ubuntu 22.04.5 LTS
架构：x86_64
ROS 2：Humble
Python：/usr/bin/python3，Python 3.10.12
编译器：G++ 13.4
CMake：3.28.4
手柄设备：/dev/input/js0（非必须，可直接使用 ROS 2 话题）
图形环境：X11
```

项目已存在完整的 `build/` 目录。设置 `build/install/lib` 为运行时动态库搜索路径后，`aimrt_main` 和 `libpkg1.so` 没有未解析的动态库依赖。

## 6. 构建方法

项目顶层 `CMakeLists.txt` 的最低要求是 CMake 3.24；项目文档建议使用 CMake 3.26 或更高版本。当前 CMake 3.28.4 满足要求。

```bash
conda deactivate 2>/dev/null || true

cd /home/zhongde/agibot_ws/src/agibot_x1_wave

source /opt/ros/humble/setup.bash
source url_gitee.bashrc

./build.sh $DOWNLOAD_FLAGS
chmod +x ./*.sh
```

如果构建系统错误选择 Conda Python，可临时恢复系统路径：

```bash
export PATH=/home/zhongde/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
hash -r

which python3
python3 --version
```

预期 Python 为 `/usr/bin/python3`、版本为 3.10.x。

## 7. 无手柄仿真流程

### 终端 A：启动仿真

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/ros2_setup.sh
export LD_LIBRARY_PATH="$PWD/build/install/lib:${LD_LIBRARY_PATH:-}"

cd build
./run_sim.sh
```

仿真初始状态为 `idle`，其 PD 刚度为 0，因此机器人在重力作用下倒地是当前工程的预期启动现象。

### 终端 B：进入 zero

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/ros2_setup.sh

ros2 topic pub --once /zero_mode std_msgs/msg/Float32 '{data: 1.0}'
```

确认终端 A 出现：

```text
Trigger event: [/zero_mode] -> zero
```

随后在 MuJoCo 中执行一次：

```text
Simulation → Reset
```

正确顺序是 `zero → Reset → stand`。Reset 用于恢复仿真的物理姿态，应在控制状态已经切换到 `zero` 后执行；进入 `stand` 或开始动作后不要再次 Reset，否则物理状态和控制状态可能不同步。

### 终端 B：进入 stand

```bash
ros2 topic pub --once /stand_mode std_msgs/msg/Float32 '{data: 1.0}'
```

确认日志出现：

```text
Trigger event: [/stand_mode] -> stand
```

等待机器人稳定站立后，才可以发送动作命令。

### 执行动作

```bash
./wave_forearm.sh
```

动作完成并确认姿态稳定后执行：

```bash
./return_to_stand.sh
```

每个动作之间必须观察机器人，不要一次性连续发送多个动作命令。

## 8. 当前仿真问题

截至本记录更新时，已确认以下状态转换成功：

```text
idle → zero → stand → wave_forearm
```

日志确认 `/wave_forearm` 已被控制器接收，但机器人在挥手动作开始时倒地。因此目前只能说明话题订阅和状态机工作正常，不能说明挥手轨迹已经通过仿真验收。

初步排查范围包括：

- 抬臂轨迹改变重心，而站立控制器没有动态补偿；
- 动作的速度、加速度或 jerk 对当前 MuJoCo 模型过大；
- 仿真 PD 刚度和阻尼不匹配；
- 手臂可能与躯干发生碰撞；
- 动作状态切换时多个 PD 控制器重新初始化产生瞬态冲击。

在问题定位完成前：

- 不继续测试幅度更大的 `wave_arm`、`arm_dance` 等动作；
- 不将该动作部署到真机；
- 不将 `Trigger event` 日志视为动作验收通过；
- 应先降低动作幅度和速度，在 MuJoCo 中逐步验证。

## 9. 历史问题与解决方法

### 9.1 ROS 2 错误使用 Conda Python

历史报错：

```text
Found Python3: /home/sylvia/anaconda3/bin/python3
Python 3.12.7
ModuleNotFoundError: No module named 'em'
```

原因是 ROS 2 Humble 使用 Ubuntu 22.04 的系统 Python 3.10，而 CMake 错误选择了 Conda Python 3.12。

处理方式：

```bash
conda deactivate
export PATH=/home/zhongde/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
hash -r

which python3
/usr/bin/python3 -c "import em; print(em.__file__)"
```

### 9.2 CMake 版本不足

历史环境中的 CMake 3.22.1 低于项目 `CMakeLists.txt` 要求的 3.24。当前机器使用 `/home/zhongde/.local/bin/cmake`，版本为 3.28.4，已经解决。

```bash
which cmake
cmake --version
```

### 9.3 `libmujoco.so` 文件无效

历史副本中的 `libmujoco.so` 曾是 18 字节 ASCII 文本，链接时会出现：

```text
file format not recognized
treating as linker script
syntax error
```

目前有效库已经恢复：

```bash
file src/module/sim_module/third_party/lib/libmujoco.so
```

预期结果包含：

```text
ELF 64-bit LSB shared object, x86-64
```

无效文件保留为 `libmujoco.so.invalid`，仅用于问题追溯，不应参与链接或运行。

### 9.4 AimRT 找不到 ROS 2 或项目动态库

真机启动脚本会对 `aimrt_main` 设置 `cap_net_raw`。带文件能力的程序可能进入安全执行模式并忽略 `LD_LIBRARY_PATH`，导致运行时找不到 ROS 2 或项目动态库。

仿真运行可使用：

```bash
export LD_LIBRARY_PATH="/home/zhongde/agibot_ws/src/agibot_x1_wave/build/install/lib:${LD_LIBRARY_PATH:-}"
```

真机环境如果受到安全执行模式影响，需要将明确的库路径写入动态链接器配置：

```bash
printf '%s\n' \
  '/home/zhongde/agibot_ws/src/agibot_x1_wave/build/install/lib' \
  '/opt/ros/humble/lib' \
  '/opt/ros/humble/lib/x86_64-linux-gnu' \
  | sudo tee /etc/ld.so.conf.d/agibot_x1_wave.conf

sudo ldconfig
```

不要写入已经不存在的旧路径。执行前应再次确认目录真实存在。

### 9.5 EtherCAT 找不到从站

历史报错：

```text
EtherCAT init on enp2s0 succeeded.
No slaves found!
XyberController start failed.
```

原因是 DCU 配置中的网卡名称与实际连接机器人的网卡不一致。当前源码和构建配置均使用：

```yaml
ethercat:
  ifname: enx98fc8413080c
```

真机启动前必须重新确认网卡名称，因为 USB 网卡名称可能随硬件或系统配置变化：

```bash
ip -br link
rg -n 'ifname:' src/module/dcu_driver_module/cfg/dcu_x1.yaml
```

修改源码配置后重新构建，再检查运行配置：

```bash
rg -n 'ifname:' build/cfg/dcu_driver_module/dcu_x1.yaml
```

## 10. 真机启动流程

> 当前挥手动作尚未通过 MuJoCo 验收，因此以下内容仅记录基础启动流程，不构成当前执行真机挥手的许可或建议。

真机启动前应完成实时内核、动态库、EtherCAT 网卡、DCU 从站、急停和保护架检查。

### 终端 A：启动真机控制系统

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/ros2_setup.sh

cd build
./run.sh
```

### 终端 B：检查系统

```bash
cd /home/zhongde/agibot_ws/src/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/ros2_setup.sh

ros2 topic list | rg 'joint_states|imu/data|zero_mode|stand_mode|wave'
timeout 5 ros2 topic hz /joint_states
ros2 topic info /wave_forearm -v
```

基础系统正常后才能通过手柄或 ROS 2 话题进入 `zero → stand`。不要进入 `walk`，避免将已知的髋部 DCU 掉线问题混入动作验证。

## 11. 安全限制

- 所有新增动作只能从 `stand` 状态触发；
- 任何新增动作都必须先通过 MuJoCo 重复测试，再考虑真机；
- 真机首次测试必须使用保护架或悬吊；
- 另一名操作人员应始终握住物理急停；
- `return_to_stand.sh` 不能代替物理急停；
- 出现振荡、异响、关节突跳、跟踪误差增大、机身失稳或 DCU 掉线时立即急停；
- 不直接扩大真机配置中的关节角度、速度、加速度或 jerk 限制；
- 近头动作和双手接近动作必须逐帧检查是否发生自碰撞。

## 12. GitHub 仓库

项目已经推送至公开仓库：

```text
https://github.com/Songayu666/agibot_x1_wave
```

提交前检查：

```bash
git status
git diff --check
```

确认修改后再执行：

```bash
git add <需要提交的文件>
git commit -m "Update development notes"
git push origin main
```
