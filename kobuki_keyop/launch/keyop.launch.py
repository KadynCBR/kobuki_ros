# https://github.com/yujinrobot/kobuki/tree/devel/kobuki_keyop/launch
from launch import LaunchDescription
from launch_ros.actions import Node

# Parameters
linear_vel_step = 0.05
linear_vel_max = 1.5
angular_vel_step = 0.33
angular_vel_max = 6.6

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kobuki_keyop',
            node_executable='keyop_node',
            node_name='kobuki_keyop',
            remappings=[
                ('keyop/motor_power', 'mobile_base/commands/motor_power'),
                ('keyop/cmd_vel', 'mobile_base/commands/velocity')],
            parameters=[
                ('linear_vel_step', linear_vel_step),
                ('linear_vel_max', linear_vel_max),
                ('angular_vel_step', angular_vel_step),
                ('angular_vel_max', angular_vel_max)]
        ),
    ])