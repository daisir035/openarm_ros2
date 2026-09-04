# OpenArm 轨迹录制与回放

该包从 `/joint_states` 录制关节位置到 YAML，并通过左右臂的
`FollowJointTrajectory` action 回放。支持左右臂筛选、夹爪、速度倍率和
ROS 命名空间。

## 构建

```bash
cd ~/Desktop/arm/openarm
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select openarmx_teach
source install/setup.bash
```

## 录制

```bash
ros2 run openarmx_teach record_joint_states_always \
  --rate 10 --outfile demo.yaml
```

终端按键：空格或 `p` 开始/暂停，`c` 清空，`w` 保存退出，`q` 不保存退出。

## 回放

启动 `joint_trajectory_controller` 后执行：

```bash
ros2 run openarmx_teach play_joint_trajectory demo.yaml --all-joints
```

只回放一侧或调整速度：

```bash
ros2 run openarmx_teach play_joint_trajectory demo.yaml --left-arm
ros2 run openarmx_teach play_joint_trajectory demo.yaml --all-joints --rate-scale 0.5
```

第一次回放应使用纯假硬件。录制文件来自不同模型时，不要直接下发真机。

## 许可证

本包保留来源项目的 CC BY-NC-SA 4.0 许可证，详见 [LICENSE](LICENSE)。
