# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# Licensed under CC BY-NC-SA 4.0.

import argparse
import sys
from typing import Dict, List, Tuple

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_yaml(file_path: str) -> Dict:
    with open(file_path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def filter_joints(
        joint_names: List[str], points: List[Dict],
        target_joints: List[str]) -> Tuple[List[str], List[Dict]]:
    """Return trajectory data ordered by target_joints."""
    if not target_joints:
        return joint_names, points

    for name in target_joints:
        if name not in joint_names:
            print(f"Warning: Joint '{name}' not found in recorded joints")

    filtered_names = [name for name in target_joints if name in joint_names]
    if not filtered_names:
        return [], []

    indices = [joint_names.index(name) for name in filtered_names]
    filtered_points = []
    for point in points:
        positions = point.get('positions', [])
        if len(positions) < len(joint_names):
            continue
        filtered_points.append({
            'positions': [positions[index] for index in indices],
            'time_from_start': point.get('time_from_start', 0.0),
        })
    return filtered_names, filtered_points


def make_goal(
        joint_names: List[str], points: List[Dict],
        rate_scale: float) -> FollowJointTrajectory.Goal:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = joint_names
    for point in points:
        message = JointTrajectoryPoint()
        message.positions = [float(value) for value in point['positions']]
        seconds = float(point['time_from_start']) / rate_scale
        message.time_from_start = rclpy.duration.Duration(
            seconds=seconds).to_msg()
        goal.trajectory.points.append(message)
    return goal


class TrajectoryPlayer(Node):
    def __init__(self) -> None:
        super().__init__('play_joint_trajectory')

    def play(self, trajectories: List[Tuple[str, List[str], List[Dict]]],
             rate_scale: float) -> bool:
        pending = []
        for action_name, joint_names, points in trajectories:
            client = ActionClient(self, FollowJointTrajectory, action_name)
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(
                    f'Action server {action_name} not available')
                return False
            future = client.send_goal_async(
                make_goal(joint_names, points, rate_scale))
            pending.append((action_name, client, future))

        result_futures = []
        success = True
        for action_name, client, future in pending:
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f'Goal rejected by {action_name}')
                success = False
                continue
            result_futures.append(
                (action_name, client, handle.get_result_async()))

        for action_name, client, future in result_futures:
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is None or result.result.error_code != 0:
                self.get_logger().error(f'Trajectory failed on {action_name}')
                success = False
            else:
                self.get_logger().info(
                    f'Trajectory completed on {action_name}')
        return success


def normalize_prefix(raw: str) -> str:
    prefix = raw.strip()
    return '/' + prefix.strip('/') if prefix else ''


def selected_joints(args, recorded_names: List[str]) -> List[str]:
    if args.joints:
        return args.joints
    if args.left_arm:
        return [f'openarm_left_joint{i}' for i in range(1, 8)]
    if args.right_arm:
        return [f'openarm_right_joint{i}' for i in range(1, 8)]
    if args.both_arms:
        return (
            [f'openarm_left_joint{i}' for i in range(1, 8)] +
            [f'openarm_right_joint{i}' for i in range(1, 8)])
    return recorded_names


def controller_trajectories(
        prefixes: List[str], joint_names: List[str],
        points: List[Dict]) -> List[Tuple[str, List[str], List[Dict]]]:
    groups = (
        ('left_joint_trajectory_controller',
         [name for name in joint_names if name.startswith('openarm_left_joint')]),
        ('right_joint_trajectory_controller',
         [name for name in joint_names if name.startswith('openarm_right_joint')]),
        ('left_gripper_controller',
         [name for name in joint_names if name.startswith('openarm_left_finger')]),
        ('right_gripper_controller',
         [name for name in joint_names if name.startswith('openarm_right_finger')]),
    )
    trajectories = []
    for prefix in prefixes:
        for controller, target_names in groups:
            if not target_names:
                continue
            names, filtered_points = filter_joints(
                joint_names, points, target_names)
            if names and filtered_points:
                action = f'{prefix}/{controller}/follow_joint_trajectory'
                trajectories.append((action, names, filtered_points))
    return trajectories


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Play an OpenArm joint trajectory YAML file')
    parser.add_argument('file', help='YAML file produced by the recorder')
    parser.add_argument('--action', help='Use one FollowJointTrajectory action')
    parser.add_argument('--rate-scale', type=float, default=1.0)
    parser.add_argument('--joints', nargs='*')
    parser.add_argument('--left-arm', action='store_true')
    parser.add_argument('--right-arm', action='store_true')
    parser.add_argument('--both-arms', action='store_true')
    parser.add_argument('--all-joints', action='store_true')
    parser.add_argument('--arm-prefix', nargs='+', default=[''])
    args = parser.parse_args()

    if args.rate_scale <= 0:
        parser.error('--rate-scale must be greater than zero')

    data = load_yaml(args.file) or {}
    joint_names = data.get('joint_names', [])
    points = data.get('points', [])
    if not joint_names or not points:
        parser.error('YAML must contain non-empty joint_names and points')

    joint_names, points = filter_joints(
        joint_names, points, selected_joints(args, joint_names))
    if not joint_names or not points:
        parser.error('no usable trajectory points remain after filtering')

    prefixes = list(dict.fromkeys(
        normalize_prefix(prefix) for prefix in args.arm_prefix))
    if args.action:
        prefix = prefixes[0]
        action = args.action if args.action.startswith('/') else '/' + args.action
        trajectories = [(
            action if prefix and action.startswith(prefix + '/') else prefix + action,
            joint_names,
            points,
        )]
    else:
        trajectories = controller_trajectories(prefixes, joint_names, points)
    if not trajectories:
        parser.error('recording contains no supported OpenArm joint groups')

    rclpy.init()
    player = TrajectoryPlayer()
    try:
        success = player.play(trajectories, args.rate_scale)
    finally:
        player.destroy_node()
        rclpy.shutdown()
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
