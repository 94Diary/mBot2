import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mbot2_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'mbot2.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'mbot2_world.sdf')

    # value_type=str บอก robot_state_publisher ตรงๆ ว่านี่คือข้อความ URDF
    # ไม่ใช่ YAML ป้องกัน error "Unable to parse the value of parameter
    # robot_description as yaml" ที่เกิดกับ launch_ros เวอร์ชันใหม่ๆ
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # Start Gazebo Sim (Fortress) with our world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Publish TF from the URDF + /joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn mBot2 into the running simulation from the robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'mbot2', '-z', '0.05'],
    )

    # Bridge ROS 2 <-> Gazebo Sim topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/ultrasonic/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/quad_rgb/1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/quad_rgb/2/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/quad_rgb/3/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/quad_rgb/4/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])
