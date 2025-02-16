import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    node_params = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')


    # image
    if (launch_params['camera']=='mv'):
        image_node = ComposableNode(
            package='mindvision_camera',
            plugin='mindvision_camera::MVCameraNode',
            name='mindvision_camera',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    else:
        image_node = ComposableNode(
            package='hik_camera',
            plugin='hik_camera::HikCameraNode',
            name='hik_camera',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    # detector
    armor_detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # 使用intra cmmunication提高图像的传输速度
    def get_camera_detector_container(*detector_nodes):
        nodes_list = list(detector_nodes)
        nodes_list.append(image_node)
        container = ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=nodes_list,
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', ],
        )
        return TimerAction(
            period=2.0,
            actions=[container],
        )
    cam_detector_node = get_camera_detector_container(armor_detector_node)
    

    # tracker
    tracker_node = Node(
        package='armor_tracker',
        executable='armor_tracker_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--log-level', 'armor_tracker:='+launch_params['tracker_log_level']],
    )

    # solver
    solver_node = Node(
        package='armor_solver',
        executable='armor_solver_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--log-level', 'armor_solver:='+launch_params['solver_log_level']],
    )

    # 串口
    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+launch_params['serial_log_level']],
    )

    # 延迟启动
    delay_serial_node = TimerAction(
        period=5.0,
        actions=[serial_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=1.0,
        actions=[tracker_node],
    )

    delay_solver_node = TimerAction(
        period=1.0,
        actions=[solver_node],
    )

    delay_cam_detector_node = TimerAction(
        period=2.0,
        actions=[cam_detector_node],
    )

    return LaunchDescription([
        robot_state_publisher,
        delay_cam_detector_node,
        delay_tracker_node,
        delay_solver_node,
        delay_serial_node,
    ])



    