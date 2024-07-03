from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    package_path = get_package_share_path('acoustic_localization')
    control_param_file_path = str(package_path / 'config/control_params.yaml')

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        # Node(executable='yaw_controller.py', package='position_control'),
        # add your other controllers here, for example:
        # Node(executable='xy_controller.py', package='position_control'),
        Node(executable='control.py',
             package='acoustic_localization',
             parameters=[
                 LaunchConfiguration('control_config_file',
                                     default=control_param_file_path),
             ]),
    ])
    launch_description.add_action(group)
    return launch_description
