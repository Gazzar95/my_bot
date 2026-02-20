
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    package_name = 'my_bot'

    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml',
    )

    # 1) Robot state publisher (URDF -> TF), with sim time
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true', 'use_sim': 'true'}.items()
    )

    # 2) Gazebo + my world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'my_dev.world',
            )
        }.items()
    )

    # 3) Spawn the robot into Gazebo using the robot_description topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'],
        output='screen',
    )

    # 4) Controller spawners (do NOT start immediately)
    # joint_state_broadcaster first
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen',
    )

    # diff drive controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen',
    )

    # 5) Delay controller spawners until after the robot is spawned
    #
    # - OnProcessStart waits for spawn_entity to start
    # - TimerAction adds an extra delay (e.g. 3s) to let gazebo_ros2_control initialize
    controller_spawner_event = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(
                    period=3.0,  # seconds; tweak if needed
                    actions=[
                        joint_broad_spawner,
                        diff_drive_spawner,
                    ],
                )
            ],
        )
    )

    # 6) SLAM Toolbox (async)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': 'true'}],
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        controller_spawner_event,
    ])
