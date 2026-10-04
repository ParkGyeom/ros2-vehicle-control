import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vehicle_control')
    default_world = os.path.join(pkg_share, 'worlds', 'diff_drive.sdf')

    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo world file (odometry 50Hz version)')

    # 터미널 A 역할: Gazebo 실행 (-r = 바로 재생)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},  # VM에서 크래시 방지
        output='screen')

    # 터미널 B 역할: ROS <-> Gazebo 브리지
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen')

    return LaunchDescription([world_arg, gazebo, bridge])
