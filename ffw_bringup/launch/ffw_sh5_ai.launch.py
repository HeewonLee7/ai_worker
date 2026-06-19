# Authors: Heewon Lee

"""SH5 teleop (real) with lg2 (gripper) leader: exo drives arms/head/lift,
Manus drives hands -- synchronized so the hand only moves while the exo is active.

Adds an exo_sync_gate node so the gripper-based lg2 leader can drive the SH5
(hand) arms AND so the Manus hand stream is gated by the exo trigger:

  - ARM: the lg2 leader bundles gripper_l_joint1 / gripper_r_joint1 into the arm
    trajectories, which the arm controllers reject ("doesn't match the
    controller's joints"), so the arms never move. The gate strips those gripper
    joints and forwards the remaining arm joints to the arm controllers.

  - HAND: Manus starts publishing hand commands the instant it connects, so the
    hand would move before the operator engages teleop -- polluting recorded data.
    The exo arm broadcaster only publishes while AutoMode is ACTIVE (operator held
    the gripper trigger), so "arm topic is flowing" == "exo is ON". The gate
    forwards Manus hand commands ONLY while the exo arm is active, and stops
    forwarding once the exo goes silent (OFF). Arm and hand thus start/stop
    together.

It includes the _handfix follower (arm + hand remaps removed) instead of the
original follower.

Install once (in container): sudo apt install ros-jazzy-rmw-cyclonedds-cpp

Run :
    ros2 launch ffw_bringup ffw_sh5_ai.launch.py     # robot
    ROS_DOMAIN_ID=11 pixi run -e cyclonedds teleop   # Manus (manus-core-ros2)

Needs CycloneDDS + domain 11 (Manus=Humble, robot=Jazzy; Fast DDS won't interop).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Relay node run with `python3 -c` (kept inline so no extra installed file is
# needed). See module docstring for the arm/hand behavior.
EXO_SYNC_GATE_CODE = '''
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# (leader_topic -> controller_topic). Arm trajectories are stripped of gripper
# joints and always forwarded (the leader only publishes them while active).
ARM_RELAYS = [
    ("/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
     "/arm_l_controller/joint_trajectory"),
    ("/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
     "/arm_r_controller/joint_trajectory"),
]
# Manus hand topics -> hand controllers. Gated: forwarded only while the exo arm
# is active.
HAND_RELAYS = [
    ("/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory",
     "/hand_l_controller/joint_trajectory"),
    ("/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory",
     "/hand_r_controller/joint_trajectory"),
]
DROP_PREFIXES = ("gripper",)
# Exo is considered OFF if no arm command arrived within this window. ON has no
# such delay: the broadcaster publishes the arm topic in the very cycle it goes
# ACTIVE, so the gate opens within one 100 Hz cycle (~10 ms) of the arm turning
# on. OFF is the only case that needs a timeout (absence of messages); the arm
# leader is local at 100 Hz, so 0.1 s (10 cycles) detects OFF in ~100 ms while
# tolerating jitter. Lower it for snappier OFF, raise it if OFF ever flickers.
ACTIVE_TIMEOUT = 0.1

STATE = {"last_active": 0.0}


def strip_joints(msg):
    keep = [i for i, n in enumerate(msg.joint_names)
            if not n.startswith(DROP_PREFIXES)]
    if len(keep) == len(msg.joint_names):
        return msg
    out = JointTrajectory()
    out.header = msg.header
    out.joint_names = [msg.joint_names[i] for i in keep]
    for p in msg.points:
        q = JointTrajectoryPoint()
        q.positions = [p.positions[i] for i in keep] if p.positions else []
        q.velocities = [p.velocities[i] for i in keep] if p.velocities else []
        q.accelerations = [p.accelerations[i] for i in keep] if p.accelerations else []
        q.effort = [p.effort[i] for i in keep] if p.effort else []
        q.time_from_start = p.time_from_start
        out.points.append(q)
    return out


def main():
    rclpy.init()
    node = Node("exo_sync_gate")
    for in_t, out_t in ARM_RELAYS:
        pub = node.create_publisher(JointTrajectory, out_t, 10)

        def arm_cb(msg, pub=pub):
            STATE["last_active"] = time.monotonic()
            pub.publish(strip_joints(msg))

        node.create_subscription(JointTrajectory, in_t, arm_cb, 10)
    for in_t, out_t in HAND_RELAYS:
        pub = node.create_publisher(JointTrajectory, out_t, 10)

        def hand_cb(msg, pub=pub):
            if time.monotonic() - STATE["last_active"] <= ACTIVE_TIMEOUT:
                pub.publish(msg)

        node.create_subscription(JointTrajectory, in_t, hand_cb, 10)
    rclpy.spin(node)


main()
'''


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')

    # Match the Manus side: all nodes on domain 11 + CycloneDDS.
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '11')
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_launch_dir, 'ffw_sh5_follower_ai_handfix.launch.py')),
        launch_arguments={'launch_cameras': 'true', 'init_position': 'true'}.items()
    )
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_lg2_leader_ai.launch.py'))
    )

    # Inherits ROS_DOMAIN_ID / RMW from the env vars set above.
    exo_sync_gate = ExecuteProcess(
        cmd=['python3', '-c', EXO_SYNC_GATE_CODE],
        output='screen',
    )

    return LaunchDescription([
        set_domain_id,
        set_rmw,
        follower,
        exo_sync_gate,
        TimerAction(period=30.0, actions=[leader]),
    ])
