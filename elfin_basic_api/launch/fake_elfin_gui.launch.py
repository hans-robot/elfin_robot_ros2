
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elfin_basic_api',
            executable='elfin_gui.py',
            name='elfin_gui_node',
            output='screen',
            parameters=[
                {'use_fake_robot': True}
            ]
        )
    ])