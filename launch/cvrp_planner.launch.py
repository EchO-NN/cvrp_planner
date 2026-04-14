from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
import os


def _forward_env(*keys):
    return {
        key: os.environ[key]
        for key in keys
        if os.environ.get(key)
    }


def _handle_params_generator_exit(event, _context, next_action):
    if event.returncode == 0:
        return [
            LogInfo(msg='params_generator 已退出，开始运行 cvrp_node'),
            next_action,
        ]
    return [
        LogInfo(
            msg=f'params_generator 异常退出，返回码 {event.returncode}，停止后续节点启动',
        ),
        EmitEvent(event=Shutdown(reason='params_generator failed')),
    ]


def _handle_cvrp_node_exit(event, _context, next_action):
    if event.returncode == 0:
        return [
            LogInfo(msg='cvrp_node 已退出，开始运行 xml_splitter'),
            next_action,
        ]
    return [
        LogInfo(
            msg=f'cvrp_node 异常退出，返回码 {event.returncode}，停止后续节点启动',
        ),
        EmitEvent(event=Shutdown(reason='cvrp_node failed')),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('cvrp_planner')
    param_file = os.path.join(pkg_share, 'params', 'params.yaml')
    gui_env = _forward_env(
        'DISPLAY',
        'XAUTHORITY',
        'WAYLAND_DISPLAY',
        'XDG_RUNTIME_DIR',
        'QT_QPA_PLATFORM',
        'ROS_LOG_DIR',
        'MPLCONFIGDIR',
    )

    first_node = Node(
        package='cvrp_planner',
        executable='params_generator',
        name='params_generator',
        output='screen',
        parameters=[param_file],
        additional_env=gui_env,
    )

    second_node = Node(
        package='cvrp_planner',
        executable='cvrp_node',
        name='cvrp_planner_node',
        output='screen',
        parameters=[param_file],
        additional_env=gui_env,
    )

    third_node = Node(
        package='cvrp_planner',
        executable='xml_splitter',
        name='xml_splitter',
        output='screen',
    )

    start_second_after_first = RegisterEventHandler(
        OnProcessExit(
            target_action=first_node,
            on_exit=lambda event, context: _handle_params_generator_exit(
                event,
                context,
                second_node,
            )
        )
    )

    start_third_after_second=RegisterEventHandler(
        OnProcessExit(
            target_action=second_node,
            on_exit=lambda event, context: _handle_cvrp_node_exit(
                event,
                context,
                third_node,
            )
        )
    )

    return LaunchDescription([
        first_node,
        start_second_after_first,
        start_third_after_second
    ])
