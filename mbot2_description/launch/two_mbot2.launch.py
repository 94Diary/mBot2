"""เปิด Gazebo พร้อม mBot2 สองตัวที่ใช้ ROS namespace แยกกัน."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROBOTS = (
    # name, x (m), y (m), yaw (rad)
    ('mbot1', '-0.30', '-0.20', '1.5707963'),
    ('mbot2', '0.30', '0.20', '-1.5707963'),
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


def robot_actions(xacro_file, robot_name, x_position, y_position, yaw):
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
        Node(
            package='mbot2_control',
            executable='maze_solver',
            name='maze_solver',
            namespace=robot_name,
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]


def bridge_arguments():
    """สร้างรายการ bridge ครบทุก topic สำหรับ mBot2 ทุกตัว."""
    arguments = []

    for robot_name, _, _, _ in ROBOTS:
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
    """สร้าง launch description สำหรับสนามและ mBot2 สองตัว."""
    pkg_share = get_package_share_directory('mbot2_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mbot2.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'mbot2_world.sdf')

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
    )

    actions = [set_sdf_path, gazebo]
    for robot in ROBOTS:
        actions.extend(robot_actions(xacro_file, *robot))

    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='two_mbot2_bridge',
            output='screen',
            arguments=bridge_arguments(),
        )
    )

    return LaunchDescription(actions)
