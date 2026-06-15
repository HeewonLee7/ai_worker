# Authors: Heewon Lee

"""SH5 teleop (real): exo drives arms/head/lift, Manus drives hands.

Install once (in container): sudo apt install ros-jazzy-rmw-cyclonedds-cpp

Run :
    ros2 launch ffw_bringup ffw_sh5_ai.launch.py     # robot
    ROS_DOMAIN_ID=11 pixi run -e cyclonedds teleop   # Manus (manus-core-ros2)

WARNING: if you start Manus while the robot is already running, the robot starts
moving the instant it connects.

Needs CycloneDDS + domain 11 (Manus=Humble, robot=Jazzy; Fast DDS won't interop).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')

    # Match the Manus side: all nodes on domain 11 + CycloneDDS.
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '11')
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_sh5_follower_ai.launch.py')),
        launch_arguments={'launch_cameras': 'true', 'init_position': 'true'}.items()
    )
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir,
                                                   'ffw_lg2_leader_ai.launch.py'))
    )

    return LaunchDescription([
        set_domain_id,
        set_rmw,
        follower,
        TimerAction(period=30.0, actions=[leader]),
    ])
