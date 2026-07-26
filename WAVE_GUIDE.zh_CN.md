# X1 右手挥手项目

本项目从官方 `agibot_x1_infer-main` 独立复制，原项目未被修改。它保留原有站立和行走功能，并增加三个只能从 `stand` 状态进入的右手动作：

- `/wave_forearm`：推荐动作。手臂抬到头侧，主要以前臂姿态往复挥手。
- `/wave_arm`：整条右臂进行幅度较大的挥动。
- `/wave_wrist`：手臂保持抬起，主要使用右腕滚转关节挥动；不控制夹爪。

每次触发都会从当前右臂姿态平滑抬手、挥动三次，再平滑回到触发前的姿态。动作结束后控制器保持该姿态；发送 `/stand_mode` 回到标准站立状态。

## 1. 编译

环境：Ubuntu 22.04、ROS 2 Humble、GCC 13、CMake 3.26+，其余依赖与原仓库一致。

```bash
cd ~/path/to/agibot_x1_wave
source /opt/ros/humble/setup.bash
source url_gitee.bashrc
./build.sh $DOWNLOAD_FLAGS
chmod +x ./*.sh
```

每次修改 `src/module/control_module/cfg/` 后都要重新执行构建，因为构建过程会把配置复制到 `build/cfg/control_module/`。

## 2. 仿真验证（必须先完成）

终端 A：

```bash
cd ~/path/to/agibot_x1_wave/build
./run_sim.sh
```

用手柄依次进入 `zero` 和 `stand`。确认机器人已经稳定站立，不要进入 `walk`。

终端 B：

```bash
cd ~/path/to/agibot_x1_wave
source /opt/ros/humble/setup.bash
source build/install/ros2_setup.sh 2>/dev/null || true
./wave_forearm.sh
```

确认动作完成后执行：

```bash
./return_to_stand.sh
```

然后分别测试：

```bash
./wave_arm.sh
./return_to_stand.sh
./wave_wrist.sh
./return_to_stand.sh
```

如果脚本找不到消息类型，先在构建目录执行 `source install/ros2_setup.sh`，再运行相同的 `ros2 topic pub` 命令。

## 3. 仿真验收条件

只有全部满足后才能测试真机：

1. 机器人必须处于 `stand`，从其他状态发送挥手话题不会切换状态。
2. 右臂抬起和放下没有突跳，没有穿过胸部或头部。
3. 三种动作各连续测试至少 10 次，无异常姿态、仿真发散或控制进程报错。
4. 动作结束后右臂回到触发前姿态；`return_to_stand.sh` 可正常恢复站立状态。
5. 测试期间不触发 `/walk_mode`，避免已知的髋部 DCU 掉线问题混入验证。

## 4. 真机测试

真机配置 `rl_x1.yaml` 使用了更低的 PD 刚度和更保守的运动限制。首次测试仍须按以下顺序：

1. 使用保护架或悬吊，急停由另一人持有，清空右臂运动范围内人员和物体。
2. 启动后进入 `zero`，再进入 `stand`，至少观察 30 秒，确认髋部 DCU 和双腿状态稳定。
3. 首次只测试 `wave_forearm`。操作人员站在机器人后侧，不要站在右臂扫掠范围内。
4. 出现振荡、异响、跟踪误差突然增大、DCU 掉线或机身失稳时立刻急停；不要尝试用 `/stand_mode` 替代硬件急停。
5. `wave_forearm` 连续 10 次通过后，再测试 `wave_arm`；最后测试 `wave_wrist`。

启动真机：

```bash
cd ~/path/to/agibot_x1_wave/build
./run.sh
```

另开已加载 ROS 2 环境的终端运行根目录下的动作脚本。

## 5. 动作参数位置

- 仿真：[src/module/control_module/cfg/rl_x1_sim.yaml](src/module/control_module/cfg/rl_x1_sim.yaml)
- 真机：[src/module/control_module/cfg/rl_x1.yaml](src/module/control_module/cfg/rl_x1.yaml)

每个轨迹点格式为：

```text
[到下一点的最短时间（秒）, 关节角1（弧度）, 关节角2（弧度）, ...]
```

不要直接在真机配置中扩大角度或提高 `max_velocity`、`max_acceleration`、`max_jerk`。先修改仿真配置并重新完成验收。

## 7. 复杂动作

所有复杂动作都只能从 `stand` 状态触发。每执行完一个动作，先运行
`./return_to_stand.sh`，确认恢复稳定站立，再测试下一个动作。

| 脚本 | 动作 | 重要说明 |
| --- | --- | --- |
| `greet_nod.sh` | 招手并模拟点头 | 当前没有头部关节，用小幅腰部俯仰代替点头 |
| `salute.sh` | 右手敬礼 | 手与头部保持安全间隙 |
| `fist_greeting.sh` | 双手抱拳 | 夹爪不能做手指造型，双手不真实接触 |
| `clap.sh` | 鼓掌三次 | 视觉鼓掌，目标点保留间隙 |
| `heart.sh` | 双臂比心 | 形成手臂心形轮廓，不控制手指 |
| `welcome.sh` | 双臂展开再收回 | 建议作为第一个双臂测试动作 |
| `point_left.sh` | 左臂指向左侧 | 夹爪只近似手指方向 |
| `point_right.sh` | 右臂指向右侧 | 夹爪只近似手指方向 |
| `wipe_sweat.sh` | 擦汗 | 与面部保持间隙 |
| `scratch_head.sh` | 挠头 | 属于近头动作，最后测试 |
| `arm_dance.sh` | 双臂和腰部连续舞蹈 | 动作较长，观察重心和足底状态 |
| `bow_wave.sh` | 小幅鞠躬后挥手 | 腰部参与，真机必须使用保护架 |
| `greeting_combo.sh` | 挥手、敬礼、展开双臂 | 自动串联后回到触发前姿态 |

例如：

```bash
./welcome.sh
./return_to_stand.sh
./salute.sh
./return_to_stand.sh
```

也可以使用统一入口：

```bash
./run_action.sh welcome
./run_action.sh greeting_combo
```

推荐仿真测试顺序：

1. `welcome`
2. `point_left`、`point_right`
3. `salute`
4. `fist_greeting`、`heart`
5. `clap`
6. `greet_nod`
7. `wipe_sweat`、`scratch_head`
8. `arm_dance`
9. `bow_wave`
10. `greeting_combo`

近头动作和双手接近动作必须逐帧观察 MuJoCo 仿真。如果发现手臂穿过身体、
头部或双手互相碰撞，不得部署真机，应先缩小对应轨迹角度。

## 6. 本项目的控制器修正

- Ruckig 插值支持任意关节数量，不再硬编码为 3 自由度。
- 初始化时校验轨迹行宽、正时长、运动限制和动作关节范围。
- 每次触发都从实时反馈姿态起步，并在结尾回到该姿态。
- 三种动作使用独立话题，且 ROS 2 外部发布可以被 AimRT 控制模块接收。
