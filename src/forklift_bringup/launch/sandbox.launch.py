"""
Open the existing warehouse with one forklift and NO mission (free drive / test mode).

  ros2 launch forklift_bringup sandbox.launch.py                       # standard robot
  ros2 launch forklift_bringup sandbox.launch.py robot_type:=high_lift # 2 m lift
  ros2 launch forklift_bringup sandbox.launch.py robot_type:=mecanum   # fast Mecanum robot
  ros2 launch forklift_bringup sandbox.launch.py loads:=true           # with the QR boxes

It starts the world, the robot, the controllers, the sensors and the EKF. It does not start
Nav2, AMCL, the QR perception, the load handler, the mission manager or the fleet coordinator.
Drive the robot with the keyboard (stamped commands, the twist_mux teleop input):

  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -p stamped:=true -r cmd_vel:=/cmd_vel/teleop

This file is a thin wrapper around sim.launch.py (mode:=none perception:=false).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('forklift_bringup'), 'launch',
                                  'sim.launch.py'])),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'robot_type': LaunchConfiguration('robot_type'),
            'spawn_loads': LaunchConfiguration('loads'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'mode': 'none',
            'perception': 'false',
        }.items())
    return LaunchDescription([
        # The Fast DDS shared-memory transport lost service responses and discovery under the
        # load of the start-up (lifecycle and spawner timeouts, BT errors, missions of 170-250 s
        # instead of 120 s). With UDP only, the same runs took 116-126 s.
        # dds_udp_only:=false keeps the default transport.
        DeclareLaunchArgument('dds_udp_only', default_value='true'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4',
                               condition=IfCondition(LaunchConfiguration('dds_udp_only'))),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('robot_type', default_value='standard',
                              description='standard | high_lift | mecanum'),
        DeclareLaunchArgument('loads', default_value='false',
                              description='Spawn the QR-labelled boxes'),
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        sim,
    ])
