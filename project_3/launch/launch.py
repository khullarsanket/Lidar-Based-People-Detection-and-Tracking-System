from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    node1 = Node(package='project_3', executable='MovingObjectLocationDetector')
    node2 = Node(package='project_3', executable='TrackMovingObject')
    bag_in_arg = DeclareLaunchArgument('bag_in') ## argument to enter at the terminal for the input bag file name
    bag_out_arg = DeclareLaunchArgument('bag_out') ## argument to enter at the terminal for the output bag file name
    
    # Using Command substitution to concatenate path and argument
    bag_file_path = Command(["echo /home/lab2004/Fall_2023/CSCE_752/Project_3/project3-bags/bags/", LaunchConfiguration('bag_in')])
    bag_record_file_path = Command(["echo /home/lab2004/Fall_2023/CSCE_752/Project_3/project3-bags/recordings/", LaunchConfiguration('bag_out')])
    
    ep = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_path],
        shell=True,  # Added this to ensure that the command is executed in the shell
        name='ros2_bag_play'  # Given a name to the process for identification
    )
    
    ep2 = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','/scan','/person_locations','/person_count','-o', bag_record_file_path],
        shell=True,  # Added this to ensure that the command is executed in the shell
        name='ros2_bag_record'  # Given a name to the process for identification
    )
    
    # Event handler to shut down the launch when the bag file play process finishes
    shutdown_on_bag_play_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=ep,
            on_exit=[EmitEvent(event=Shutdown(reason='Bag play process finished'))],
        )
    )
    
    ld = LaunchDescription([
        bag_in_arg,
        bag_out_arg,
        node1,
        node2,
        ep,
        ep2,
        shutdown_on_bag_play_exit
    ])
    
    return ld
