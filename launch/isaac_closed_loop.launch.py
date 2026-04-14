import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('cvrp_planner')
    nav_launch = os.path.join(pkg_share, 'launch', 'isaac_multi_nav.launch.py')
    planner_launch = os.path.join(pkg_share, 'launch', 'cvrp_planner.launch.py')

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(planner_launch)),
        ],
    )
