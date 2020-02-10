# https://github.com/yujinrobot/kobuki/tree/devel/kobuki_keyop/launch
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            descriptions='prefix for node names'),
        launch_ros.actions.Node(
            package='kobuki_keyop', node_executable='keyop_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'keyop_node'])
    ])