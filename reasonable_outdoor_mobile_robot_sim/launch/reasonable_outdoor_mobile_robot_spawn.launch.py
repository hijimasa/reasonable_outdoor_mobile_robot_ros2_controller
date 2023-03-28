import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    reasonable_outdoor_mobile_robot_description_path = os.path.join(
        get_package_share_directory('reasonable_outdoor_mobile_robot_description'))

    xacro_file = os.path.join(reasonable_outdoor_mobile_robot_description_path,
                              'robots',
                              'reasonable_outdoor_mobile_robot.urdf.xacro')
    # xacroをロード
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("reasonable_outdoor_mobile_robot_bringup"),
            "config",
            "reasonable_outdoor_mobile_robot.yaml",
        ]
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'reasonable_outdoor_mobile_robot'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_controller'],
        output='screen'
    )

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
        remappings=[
            ('/cmd_vel_stamped', '/diff_drive_controller/cmd_vel'),
        ],
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_diff_drive_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        velocity_converter,
    ])
