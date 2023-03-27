from launch import LaunchDescription
from launch_ros.actions import *
from launch.actions import *
from launch.substitution import *
from launch.event_handlers import *
from launch.events import *

def generate_launch_description():
    node = Node(package='turtlesim',
                executable='turtlesim_node')
    ep = ExecuteProcess(cmd=['ros2', 'bag', 'play', 'bags/example1'])
    event_handler = OnProcessExit(target_action=ep,
                              on_exit=[EmitEvent(event=Shutdown())])
    terminate_at_end = RegisterEventHandler(event_handler)
    ld = LaunchDescription([ ep, terminate_at_end ])
    return ld