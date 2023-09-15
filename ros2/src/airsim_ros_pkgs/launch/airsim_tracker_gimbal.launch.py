from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    gimbal_tracking_node = Node(
            package='airsim_ros_pkgs',
            executable='tracker_node',  # Изменено на 'tracker_node'
            name='gimbal_tracking_node',
            output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
  
    ld.add_action(gimbal_tracking_node)  # Add our gimbal tracking node to the launch description

    return ld
