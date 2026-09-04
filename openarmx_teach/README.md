# OpenArm trajectory record and playback

Record `/joint_states` to YAML:

```bash
ros2 run openarmx_teach record_joint_states_always --rate 10 --outfile demo.yaml
```

Play the recording through the active joint trajectory controllers:

```bash
ros2 run openarmx_teach play_joint_trajectory demo.yaml --all-joints
```

Test playback with fake hardware before commanding a real robot. See
`README_CN.md` for the full Chinese quick start.

The package retains the source project's CC BY-NC-SA 4.0 license.
