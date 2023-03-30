from launch import LaunchDescription
from launch_ros.actions import *
from launch.actions import *
from launch.substitutions import *
from launch.event_handlers import *
from launch.events import *

def generate_launch_description():
    # node = Node(package='turtlesim',
    #             executable='turtlesim_node')

    bag_loc_value = LaunchConfiguration('bag_loc')
    bag_loc_arg = DeclareLaunchArgument('bag_loc', default_value="bags/example1/")

    ep = ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_loc_value])
    track_node = Node(package='project3', executable='track')
    event_handler = OnProcessExit(target_action=ep, on_exit=[EmitEvent(event=Shutdown())])
    terminate_at_end = RegisterEventHandler(event_handler)
    ld = LaunchDescription([ bag_loc_arg, ep, track_node, terminate_at_end ])
    return ld
