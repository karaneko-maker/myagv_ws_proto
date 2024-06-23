# import launch
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     ld = LaunchDescription()

#     joy_node = Node(
#         package="joy",
#         executable="joy_node"
#     )

#     teleop_twist_joy_node = Node(
#         package="myagv_joy",
#         executable="teleop_twist_joy"
#     )

#     ld.add_action(joy_node)
#     ld.add_action(teleop_twist_joy_node)

#     return ld

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen'
        ),
    ])
