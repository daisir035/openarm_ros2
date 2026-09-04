# OpenArm 关节滑块面板

这是一个 RViz2 面板，用于调试 OpenArm 双臂和夹爪。面板从
`/joint_states` 同步当前位置，从 `robot_description` 读取关节限位，并将
滑块目标分成小步轨迹发送，避免目标位置突变。

## 构建

```bash
cd ~/Desktop/arm/openarm
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select openarmx_joint_slider_panel
source install/setup.bash
```

## 使用

先启动带轨迹控制器的仿真：

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v2.0 use_fake_hardware:=true
```

MoveIt 的 RViz 配置会自动加载 `关节控制` 面板。也可以在 RViz 的
`Panels -> Add New Panel` 中选择
`openarmx_joint_slider_panel/JointSliderPanel`。

面板使用以下话题：

- `/left_joint_trajectory_controller/joint_trajectory`
- `/right_joint_trajectory_controller/joint_trajectory`
- `/left_gripper_controller/joint_trajectory`
- `/right_gripper_controller/joint_trajectory`

拖动滑块会直接向已激活的控制器发送命令。连接真机前应先在纯假硬件
模式确认关节方向、限位和 Home/Hands Up 目标。

## 许可证

本包保留来源项目的 CC BY-NC-SA 4.0 许可证，详见 [LICENSE](LICENSE)。
