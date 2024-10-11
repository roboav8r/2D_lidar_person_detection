import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()


    # Config files
    config = os.path.join(
        get_package_share_directory('dr_spaam_ros'),
        'config',
        'detector_params.yaml'
    )

    # Audio acquisition node
    node = Node(
        package='dr_spaam_ros',
        executable='node.py',
        name='dr_spaam_ros',
        output='screen',
        parameters=[config]
    )
    ld.add_action(node)

    return ld

# <launch>
#   <node pkg="dr_spaam_ros" type="node.py" name="dr_spaam_ros" output="screen">
#     <rosparam command="load" file="$(find dr_spaam_ros)/config/dr_spaam_ros.yaml"/>
#     <rosparam command="load" file="$(find dr_spaam_ros)/config/topics.yaml"/>
#   </node>
# </launch>
