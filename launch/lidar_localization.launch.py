import os

import launch
import launch.actions
import launch.events
import launch.event_handlers

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    lidar_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','velodyne']
        )

    imu_tf = launch_ros.actions.Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','imu_link']
        )

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar_localization_ros2'),
            'param',
            'localization.yaml'))

    lidar_localization = launch_ros.actions.LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[localization_param_dir],
        remappings=[('/cloud','/lio/body/cloud'),
                    ('/imu','/livox/imu'),
                    ('/odom','/lio/robo/odom')],
        prefix=['taskset -c 5,6'],   # 绑定 CPU 5,6
        output='screen',
        respawn=True,                # 崩溃后自动重启
        respawn_delay=2.0,           # 重启间隔 2 秒
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # 进程退出/崩溃后的自动恢复处理器（配合 respawn 使用）
    # 等待新进程启动后重新触发生命周期 configure → activate
    lidar_localization_respawn_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=lidar_localization,
            on_exit=[
                launch.actions.LogInfo(
                    msg="lidar_localization 进程退出，等待 respawn 后恢复生命周期..."
                ),
                launch.actions.TimerAction(
                    period=1.5,  # 等待新进程启动并完成 DDS 注册
                    actions=[
                        launch.actions.LogInfo(
                            msg="重新触发生命周期: configure → activate"
                        ),
                        launch.actions.EmitEvent(
                            event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch.events.matches_action(
                                    lidar_localization
                                ),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                            )
                        ),
                    ],
                ),
            ],
        )
    )

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    ld.add_action(lidar_localization_respawn_handler)

    ld.add_action(lidar_localization)
    # ld.add_action(lidar_tf)
    ld.add_action(to_inactive)

    return ld