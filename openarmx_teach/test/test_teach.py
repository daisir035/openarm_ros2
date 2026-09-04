from sensor_msgs.msg import JointState

from openarmx_teach.play_joint_trajectory import (
    controller_trajectories,
    filter_joints,
)
from openarmx_teach.record_joint_states_always import JointStatesBuffer


def test_recorded_joint_order_and_playback_filter():
    buffer = JointStatesBuffer()
    msg = JointState()
    msg.name = ["openarm_left_joint2", "openarm_left_joint1"]
    msg.position = [0.2, 0.1]
    buffer.update(msg)

    assert buffer.snapshot() == [0.2, 0.1]

    names, points = filter_joints(
        buffer.joint_names,
        [{"positions": buffer.snapshot(), "time_from_start": 0.1}],
        ["openarm_left_joint1"],
    )
    assert names == ["openarm_left_joint1"]
    assert points == [{"positions": [0.1], "time_from_start": 0.1}]


def test_gripper_uses_trajectory_controller_action():
    names = ["openarm_left_finger_joint1"]
    points = [{"positions": [0.02], "time_from_start": 0.1}]

    trajectories = controller_trajectories([""], names, points)

    assert trajectories == [(
        "/left_gripper_controller/follow_joint_trajectory",
        names,
        points,
    )]
