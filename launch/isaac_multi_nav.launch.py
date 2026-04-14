import json
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _load_planner_section(param_file):
    data = yaml.safe_load(open(param_file, 'r', encoding='utf-8')) or {}
    section = data.get('cvrp_planner_node', {}).get('ros__parameters', {})
    if not section:
        raise RuntimeError(f'参数文件缺少 cvrp_planner_node.ros__parameters: {param_file}')
    return section


def _resolve_map_path(pkg_share, map_path):
    if not map_path:
        raise RuntimeError('planner 参数里缺少 map_path')
    if os.path.isabs(map_path):
        return map_path
    return os.path.join(pkg_share, map_path)


def _build_robot_configs(planner_params):
    robot_names = json.loads(planner_params.get('robot_names', '[]'))
    robot_starts = json.loads(planner_params.get('robot_starts', '[]'))
    robot_yaws_deg = json.loads(planner_params.get('odometry_origin_yaws_deg', '[]'))
    if not robot_names:
        raise RuntimeError('robot_names 不能为空')
    if len(robot_starts) != len(robot_names):
        raise RuntimeError('robot_starts 数量必须与 robot_names 一致')
    if robot_yaws_deg and len(robot_yaws_deg) != len(robot_names):
        raise RuntimeError('odometry_origin_yaws_deg 数量必须与 robot_names 一致')

    configs = []
    for index, name in enumerate(robot_names):
        start = robot_starts[index]
        yaw_deg = float(robot_yaws_deg[index]) if robot_yaws_deg else 0.0
        configs.append(
            {
                'name': str(name).strip('/'),
                'x': float(start[0]),
                'y': float(start[1]),
                'yaw_deg': yaw_deg,
            }
        )
    return configs


def generate_launch_description():
    pkg_share = get_package_share_directory('cvrp_planner')
    bringup_dir = get_package_share_directory('nav2_bringup')
    planner_param_file = os.path.join(pkg_share, 'params', 'params.yaml')
    nav2_params_file_default = os.path.join(pkg_share, 'params', 'nav2_isaac_params.yaml')
    rviz_config_default = os.path.join(pkg_share, 'rviz', 'isaac_multi_nav.rviz')

    planner_params = _load_planner_section(planner_param_file)
    robot_configs = _build_robot_configs(planner_params)
    default_map_path = _resolve_map_path(pkg_share, planner_params.get('map_path', ''))

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Nav2 使用的 ROS map yaml 文件',
        ),
    )
    ld.add_action(
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file_default,
            description='Isaac 多机器人 Nav2 参数文件',
        ),
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用仿真时间',
        ),
    )
    ld.add_action(
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='自动激活 Nav2 生命周期节点',
        ),
    )
    ld.add_action(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 日志级别',
        ),
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否自动启动 RViz2 调试窗口',
        ),
    )
    ld.add_action(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='RViz2 配置文件',
        ),
    )

    ld.add_action(LogInfo(msg=f'加载多机器人 Nav2，地图: {default_map_path}'))
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            remappings=[
                ('/tf', '/robot_0/tf'),
                ('/tf_static', '/robot_0/tf_static'),
            ],
            condition=IfCondition(use_rviz),
        ),
    )

    for robot in robot_configs:
        namespace = robot['name']
        yaw_rad = math.radians(robot['yaw_deg'])
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=nav2_params_file,
                root_key=namespace,
                param_rewrites={
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_yaml_file,
                },
                convert_types=True,
            ),
            allow_substs=True,
        )

        robot_group = GroupAction(
            actions=[
                LogInfo(
                    msg=(
                        f'启动 {namespace} Nav2: '
                        f'map->odom=({robot["x"]:.3f}, {robot["y"]:.3f}, yaw={robot["yaw_deg"]:.1f}deg)'
                    ),
                ),
                PushRosNamespace(namespace),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_to_odom_static',
                    output='screen',
                    arguments=[
                        '--x', TextSubstitution(text=str(robot['x'])),
                        '--y', TextSubstitution(text=str(robot['y'])),
                        '--z', '0.0',
                        '--yaw', TextSubstitution(text=str(yaw_rad)),
                        '--frame-id', 'map',
                        '--child-frame-id', 'odom',
                    ],
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                ),
                Node(
                    package='cvrp_planner',
                    executable='odom_tf_bridge',
                    name='odom_tf_bridge',
                    output='screen',
                    parameters=[
                        {
                            'use_sim_time': use_sim_time,
                            'odom_topic': 'chassis/odom',
                            'parent_frame_id': 'odom',
                            'child_frame_id': 'base_link',
                        },
                    ],
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                ),
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    output='screen',
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[
                        {
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'node_names': ['map_server'],
                        },
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'launch', 'navigation_launch.py'),
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'params_file': nav2_params_file,
                        'use_composition': 'False',
                        'use_respawn': 'False',
                        'log_level': log_level,
                    }.items(),
                ),
            ],
        )
        ld.add_action(robot_group)

    return ld
