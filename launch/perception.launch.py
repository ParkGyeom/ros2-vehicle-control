"""인식 데모: Gazebo(카메라 달린 차) + 브리지 + 검출 노드 + (선택) 장애물 정지 주행 + 결과 영상 창.

ros2 launch vehicle_control perception.launch.py                 # 시뮬 + YOLO 인식만 (차는 서 있음)
ros2 launch vehicle_control perception.launch.py drive:=true     # 장애물 정지 주행까지
ros2 launch vehicle_control perception.launch.py gui:=false      # Gazebo 창 없이 (VM에서 FPS↑)

# 거리 추정 검증: 정답 거리를 아는 빨간 상자 + 빨간색 검출기
ros2 launch vehicle_control perception.launch.py world:=distance_test.sdf detector:=red_box drive:=true
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vehicle_control')
    # 파일 이름만 주면 패키지의 worlds/에서 찾고, 절대 경로를 주면 그대로 사용
    world = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])

    args = [
        DeclareLaunchArgument('world', default_value='perception_road.sdf'),
        DeclareLaunchArgument('detector', default_value='yolo',
                              description='yolo | red_box (검증용)'),
        DeclareLaunchArgument('model', default_value='yolo26n.pt',
                              description='YOLO 가중치 (처음 실행 시 자동 다운로드)'),
        DeclareLaunchArgument('imgsz', default_value='640',
                              description='추론 해상도, CPU에서 느리면 320'),
        DeclareLaunchArgument('delay', default_value='0.0',
                              description='red_box 검출기의 인위적 지연 (s)'),
        DeclareLaunchArgument('drive', default_value='false',
                              description='true면 obstacle_stop 노드도 실행'),
        DeclareLaunchArgument('odom_prediction', default_value='true',
                              description='obstacle_stop의 지연 보상 on/off (비교 실험용)'),
        DeclareLaunchArgument('image_view', default_value='true',
                              description='검출 결과 영상 창(rqt_image_view)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false면 Gazebo 창 없이 실행 → 카메라 FPS가 올라감'),
    ]

    # VM(소프트웨어 렌더링)에서는 Gazebo 창 그리기와 카메라 렌더링이 CPU를 나눠 쓴다
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},  # VM에서 크래시 방지
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen')
    gazebo_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=UnlessCondition(LaunchConfiguration('gui')),
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen')

    yolo = Node(
        package='vehicle_control',
        executable='yolo_detector.py',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'imgsz': LaunchConfiguration('imgsz'),
        }],
        condition=LaunchConfigurationEquals('detector', 'yolo'),
        output='screen')

    red_box = Node(
        package='vehicle_control',
        executable='red_box_detector.py',
        parameters=[{'delay': LaunchConfiguration('delay')}],
        condition=LaunchConfigurationEquals('detector', 'red_box'),
        output='screen')

    obstacle_stop = Node(
        package='vehicle_control',
        executable='obstacle_stop',
        parameters=[{'odom_prediction': LaunchConfiguration('odom_prediction')}],
        condition=IfCondition(LaunchConfiguration('drive')),
        output='screen')

    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/perception/image_annotated'],
        condition=IfCondition(LaunchConfiguration('image_view')))

    return LaunchDescription(args + [
        gazebo, gazebo_headless, bridge, yolo, red_box, obstacle_stop, image_view])
