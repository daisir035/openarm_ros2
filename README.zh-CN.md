# OpenArm ROS 2：ZK 单电机混合测试

简体中文 | [English](README.md)

本分支用于只有一台真实 ZK 电机时验证完整 ROS 2 控制链路：

- 左臂关节 1 使用 `left_can_interface` 上的 ZK ID 1 真电机。
- 左臂关节 2-7、左夹爪、整条右臂和右夹爪使用假硬件。
- 双臂轨迹控制器、夹爪控制器、MoveIt 2 和 RViz 仍按完整双臂模型启动。
- 测试模式不会执行整臂自动回零；启动后 ID 1 先保持当前位置。

本文固定使用 Ubuntu 22.04 和 ROS 2 Humble。

## 1. 安装依赖

先按 [ROS 2 Humble 官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配置软件源，然后执行：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  libcli11-dev \
  can-utils

source /opt/ros/humble/setup.bash
test -f /etc/ros/rosdep/sources.list.d/20-default.list || sudo rosdep init
rosdep update
```

## 2. 获取完整源码

三个仓库必须位于同一个 workspace 的 `src` 目录，不能只构建 `openarm_ros2`：

```bash
mkdir -p ~/openarm_ws/src
cd ~/openarm_ws/src

git clone https://github.com/daisir035/openarm_ros2.git
git -C openarm_ros2 switch test/zk-id1-hybrid
git clone https://github.com/daisir035/openarm_description.git
vcs import . < openarm_ros2/openarm.repos
```

目录应为：

```text
~/openarm_ws/src/openarm_ros2
~/openarm_ws/src/openarm_can
~/openarm_ws/src/openarm_description
```

## 3. 安装依赖并构建

```bash
cd ~/openarm_ws
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y

colcon build --symlink-install
source install/setup.bash
```

正常结果是构建以 `Summary` 无失败结束。检查新参数是否已经安装：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py --show-args \
  | grep single_motor
```

每个新终端都需要执行：

```bash
source /opt/ros/humble/setup.bash
source ~/openarm_ws/install/setup.bash
```

## 4. 先验证纯假硬件

此步骤不访问 CAN，也不会驱动电机：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=openarm_v2.0 \
  use_fake_hardware:=true
```

另开终端检查：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

以下 5 个控制器都应显示 `active`：

```text
joint_state_broadcaster
left_joint_trajectory_controller
right_joint_trajectory_controller
left_gripper_controller
right_gripper_controller
```

验证完成后，在启动终端按 `Ctrl+C` 停止。

## 5. 连接单台真电机

> 安全警告：启动会使能 ZK ID 1。请先完成电机标零，卸除负载或固定机构，清空运动范围，并把急停或断电装置放在手边。

接线约定：

- 只连接一台电机，电机 ID 必须为 1。
- 电机接到后续传给 `left_can_interface` 的 CAN 口。
- 即使实际接在 `can0`，它在模型中仍表示左臂关节 1。
- 总线必须使用 CAN FD，标称速率 1 Mbps，数据速率 5 Mbps。

以下示例使用 `can0`：

```bash
ip -brief link | grep can
openarm-can-cli -i can0 can_configure
ip -details -statistics link show can0
```

输出中应看到 `UP`、`fd on`、`bitrate 1000000` 和 `dbitrate 5000000`。

部分 `gs_usb` 适配器不支持 `restart-ms`。如果上面的工具提示
`Device doesn't support restart from Bus Off`，改为手动配置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.75 \
  dbitrate 5000000 fd on dsample-point 0.75 dsjw 2
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

## 6. 启动单电机混合模式

先确保上一节的假硬件进程已停止，再执行：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=openarm_v2.0 \
  single_motor_test:=true \
  left_can_interface:=can0
```

`single_motor_test:=true` 会自动启用混合硬件，不需要再传 `use_fake_hardware:=false`。默认测试增益为：

```text
single_motor_kp=10.0
single_motor_kd=1.0
```

需要调节时直接覆盖，例如：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  single_motor_test:=true \
  left_can_interface:=can0 \
  single_motor_kp:=5.0 \
  single_motor_kd:=0.5
```

## 7. 启动后验收

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/openarm_ws/install/setup.bash

ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
ip -details -statistics link show can0
```

成功标准：

- `openarm_left_hardware_interface` 为 `active`。
- `openarm_right_hardware_interface` 为 `active`，类型是 `GenericSystem`。
- 5 个控制器全部为 `active`。
- `/joint_states` 中有完整双臂和夹爪状态。
- 启动日志显示 `single_motor_test=true`，且没有 `received 0/1`。
- CAN 的 `bus-off`、`errors` 和 `dropped` 没有持续增加。

确认上述状态后，再在 RViz 中给左臂关节 1 设置很小的位移并先 `Plan`；确认方向和幅度正确后才 `Execute`。其余关节只改变假硬件状态，不会发送 CAN 指令。

## 8. 使用 MoveIt 2

MoveIt 启动文件会自己启动 ros2_control，不要和上一节的 bringup 同时运行：

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v2.0 \
  single_motor_test:=true \
  left_can_interface:=can0
```

## 9. 停止与确认

在启动终端按 `Ctrl+C`，等待日志出现硬件停用信息。随后检查：

```bash
ros2 node list
pgrep -af 'ros2_control_node|robot_state_publisher|rviz2|move_group'
```

两条命令都没有列出本次启动的进程才算完全停止。最后可关闭 CAN：

```bash
sudo ip link set can0 down
```

## RViz 关节控制与示教

MoveIt 的 RViz 配置会自动加载 `关节控制` 面板。面板读取当前关节状态和
URDF 限位，通过现有左右臂轨迹控制器发送分段目标。先使用上面的纯假硬件
命令验证，再连接真机。

录制关节轨迹：

```bash
ros2 run openarmx_teach record_joint_states_always --rate 10 --outfile demo.yaml
```

回放全部关节：

```bash
ros2 run openarmx_teach play_joint_trajectory demo.yaml --all-joints
```

详细说明见 `openarmx_joint_slider_panel/README_CN.md` 和
`openarmx_teach/README_CN.md`。

## 故障排查

### 找不到 `OpenArmCANConfig.cmake`

`openarm_can` 没有放在同一 workspace。确认 `~/openarm_ws/src/openarm_can` 存在，再从 workspace 根目录重新构建。

### 找不到 `single_motor_test` 参数

当前终端加载了旧构建结果。执行：

```bash
cd ~/openarm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-clean-cache
source install/setup.bash
```

### `Cannot find device "can0"`

用 `ip -brief link` 查实际接口名，并把同一名称传给 `openarm-can-cli` 和 `left_can_interface`。

### `Single-motor test received 0/1 motor states`

控制器会停止激活，不会把缺失电机当成假电机。检查 ID 1、供电、共地、终端电阻、CAN FD 配置和波特率：

```bash
ip -details -statistics link show can0
candump can0
```

### 控制器加载失败或配置失败

先查看 `ros2_control_node` 最早出现的错误；后续 spawner 报错通常只是结果。确认没有另一个 `/controller_manager` 正在运行：

```bash
ros2 node list
pgrep -af 'ros2_control_node|controller_manager'
```

## 参考资料

- [OpenArm ROS 2 安装](https://docs.openarm.dev/1.0/software/ros2/install)
- [OpenArm ROS 2 Control](https://docs.openarm.dev/1.0/software/ros2/control)
- [ROS 2 Humble 安装](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

## 许可证

主项目使用 Apache License 2.0，详见 [LICENSE](LICENSE)。迁移的
`openarmx_joint_slider_panel` 和 `openarmx_teach` 独立保留 CC BY-NC-SA
4.0 许可证。
