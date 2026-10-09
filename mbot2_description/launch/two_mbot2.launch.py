"""Launch two or three coordinated mBot2 robots in the branch maze."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROBOTS = (
    # name, x (m), y (m), yaw (rad)
    # Simple branch maze: both robots begin on the west-east black line.
    ('mbot1', '-5.4', '0.0', '0.0'),
    ('mbot2', '-6.1', '0.0', '0.0'),
    ('mbot3', '-6.8', '0.0', '0.0'),
)


def robot_description(xacro_file, robot_name):
    """สร้าง URDF ของหุ่นหนึ่งตัว โดยใส่ topic prefix ของตัวเอง."""
    return ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' robot_namespace:=/',
            robot_name,
        ]),
        value_type=str,
    )


def robot_actions(xacro_file, robot_name, x_position, y_position, yaw,
                  start_controllers, delay_seconds):
    """คืน launch actions สำหรับ model, TF publisher, spawn และ controller."""
    description = robot_description(xacro_file, robot_name)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': description,
                'use_sim_time': True,
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            output='screen',
            arguments=[
                '-topic',
                f'/{robot_name}/robot_description',
                '-name',
                robot_name,
                '-x',
                x_position,
                '-y',
                y_position,
                '-z',
                '0.06',
                '-Y',
                yaw,
            ],
        ),
        TimerAction(
            period=delay_seconds,
            condition=IfCondition(start_controllers),
            actions=[Node(
                package='mbot2_control',
                executable='maze_solver',
                name='maze_solver',
                namespace=robot_name,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'team_enabled': True,
                    'use_ground_truth': True,
                    'spawn_x': float(x_position),
                    'spawn_y': float(y_position),
                    'spawn_yaw': float(yaw),
                }],
            )],
        ),
    ]


def bridge_arguments(robot_names, include_clock=False):
    """สร้างรายการ bridge ครบทุก topic สำหรับ mBot2 ทุกตัว."""
    arguments = ([
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/world/simple_branch_maze/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    ] if include_clock else [])

    for robot_name in robot_names:
        prefix = f'/{robot_name}'
        arguments.extend([
            f'{prefix}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            f'{prefix}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'{prefix}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            f'{prefix}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            f'{prefix}/ultrasonic/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'{prefix}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            f'{prefix}/quad_rgb_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{prefix}/quad_rgb_2/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{prefix}/quad_rgb_3/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{prefix}/quad_rgb_4/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        ])

    return arguments


def generate_launch_description():
    """Create Gazebo, bridges, the coordinator and two/three robots."""
    start_controllers = LaunchConfiguration('start_controllers')
    robot_count = LaunchConfiguration('robot_count')
    headless = LaunchConfiguration('headless')
    three_robots = IfCondition(PythonExpression(["'", robot_count, "' == '3'"]))
    declare_start_controllers = DeclareLaunchArgument(
        'start_controllers',
        default_value='true',
        description='Start the coordinator and maze solvers automatically.',
    )
    declare_robot_count = DeclareLaunchArgument(
        'robot_count', default_value='2', description='2 or 3 robots.',
    )
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo server without the 3-D window.',
    )

    pkg_share = get_package_share_directory('mbot2_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mbot2.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'simple_branch_maze.sdf')

    set_sdf_path = SetEnvironmentVariable(
        'SDF_PATH',
        os.path.join(pkg_share, 'worlds'),
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-s -r {world_file}'}.items(),
        condition=IfCondition(headless),
    )

    actions = [declare_start_controllers, declare_robot_count, declare_headless,
               set_sdf_path, gazebo, gazebo_headless,
               Node(package='mbot2_control', executable='team_coordinator',
                    name='maze_team_coordinator', output='screen',
                    condition=IfCondition(start_controllers))]
    for index, robot in enumerate(ROBOTS):
        devices = robot_actions(xacro_file, *robot, start_controllers,
                                delay_seconds=index * 8.0)
        if index == 2:
            actions.append(GroupAction(devices, condition=three_robots))
        else:
            actions.extend(devices)

    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='two_mbot2_bridge',
            output='screen',
            arguments=bridge_arguments(['mbot1', 'mbot2'], include_clock=True),
        )
    )
    actions.append(GroupAction([
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='third_mbot2_bridge', output='screen',
             arguments=bridge_arguments(['mbot3']))], condition=three_robots))

    return LaunchDescription(actions)
