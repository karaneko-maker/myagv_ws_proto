import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    urdf_file = '/home/agv1/myagv_ws/src/myagv_description/urdf/agv.urdf'

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='myagv_communication',
            executable='serial',
            name='serial_node',
            output='screen'
        ),
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }],
            output='screen'
        ),
        Node(
            package='myagv_communication',
            executable='diff_drive_controller_node',  # 実行ファイル名（.pyを除く）を入力してください
            name='diff_drive_controller_node',
            output='screen'
        ),
    ])

'''
ここに入っているファイルはcoreだから、以下の3点のファイルを起動する。
・TF関連
・serial通信
・LiDAR関連のノード
・cmd_vel-モーター制御値のコード
・
・


'''