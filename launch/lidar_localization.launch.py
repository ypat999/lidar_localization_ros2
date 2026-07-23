import os

import launch
import launch.actions
import launch.event_handlers

import launch_ros
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

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
                    ('/odom','/lio/odom')],
        prefix=['taskset -c 5,6'],   # 绑定 CPU 5,6
        output='screen',
        respawn=True,                # 崩溃后自动重启
        respawn_delay=2.0,           # 重启间隔 2 秒
    )

    # ──────────────────────────────────────────────
    # 生命周期兜底脚本（shell）
    #  - 轮询等待 /lidar_localization/change_state 服务就绪
    #  - 服务出现后等 2 秒让节点稳定
    #  - 依次发送 CONFIGURE → ACTIVATE
    #  - ros2 service call 无超时限制，能等待 on_configure 加载地图完成
    #  - 120 秒总超时，期间节点崩溃会重置轮询继续等待
    # ──────────────────────────────────────────────
    lifecycle_bootstrap_cmd = [
        'bash', '-c', '''
echo "[lidar_localization] 等待 change_state 服务就绪 (最长 120s)..."
deadline=$((SECONDS + 120))
while [ $SECONDS -lt $deadline ]; do
    if ros2 service list 2>/dev/null | grep -q "/lidar_localization/change_state"; then
        echo "[lidar_localization] 服务就绪, 等待节点稳定 (2s)..."
        sleep 2

        echo "[lidar_localization] 发送 CONFIGURE (id=1)..."
        result=$(ros2 service call /lidar_localization/change_state \
            lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: ''}}" 2>&1)
        ret=$?
        echo "$result"
        if [ $ret -eq 0 ] && echo "$result" | grep -q "success.*True"; then
            sleep 1
            echo "[lidar_localization] 发送 ACTIVATE (id=3)..."
            result2=$(ros2 service call /lidar_localization/change_state \
                lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: ''}}" 2>&1)
            ret2=$?
            echo "$result2"
            if [ $ret2 -eq 0 ] && echo "$result2" | grep -q "success.*True"; then
                echo "[lidar_localization] 生命周期恢复完成"
                exit 0
            fi
        fi

        echo "[lidar_localization] 服务调用失败, 5 秒后重试..."
        sleep 5
        continue
    fi
    sleep 1
done
echo "[lidar_localization] 超时: 120 秒内服务未就绪"
exit 1
'''
    ]

    # 初次启动: 等待 lidar_localization 进程起来后触发生命周期
    lifecycle_bootstrap = launch.actions.ExecuteProcess(
        cmd=lifecycle_bootstrap_cmd,
        output='screen',
        name='lidar_localization_bootstrap'
    )

    # 崩溃恢复处理器: 进程退出后等待 respawn, 再重新触发生命周期
    lidar_localization_respawn_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=lidar_localization,
            on_exit=[
                launch.actions.LogInfo(
                    msg="[lidar_localization] 进程退出, 等待 respawn 后恢复..."
                ),
                launch.actions.TimerAction(
                    period=4.0,  # respawn_delay(2s) + 进程启动 + DDS注册
                    actions=[
                        launch.actions.ExecuteProcess(
                            cmd=lifecycle_bootstrap_cmd,
                            output='screen',
                            name='lidar_localization_recover'
                        ),
                    ],
                ),
            ],
        )
    )

    ld.add_action(lidar_localization_respawn_handler)
    ld.add_action(lidar_localization)
    ld.add_action(lifecycle_bootstrap)

    return ld
