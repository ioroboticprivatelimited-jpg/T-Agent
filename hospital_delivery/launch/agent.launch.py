import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('hospital_delivery'),
                       'config', 'agent.yaml')
    names = ['order_manager', 't_agent', 'crowd_monitor', 'nav_executor',
             'safety_supervisor', 'bedside_verifier']
    return LaunchDescription([
        Node(package='hospital_delivery', executable=n, name=n,
             parameters=[cfg], output='screen') for n in names])
