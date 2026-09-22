import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gọi launch file MoveIt + Gazebo có sẵn của UR
    ur_gz_dir = get_package_share_directory('ur_simulation_gz')
    ur_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_gz_dir, 'launch', 'ur_sim_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3',
            'launch_rviz': 'true'
        }.items()
    )

    # Node điều khiển của sinh viên
    writer_node = Node(
        package='ur3_writer',
        executable='ur3_writer_node',
        name='ur3_writer_node',
        output='screen'
    )

    # Đợi 12 giây cho Gazebo và MoveIt khởi động hoàn tất mới chạy Node điều khiển
    delayed_writer_node = TimerAction(
        period=12.0,
        actions=[writer_node]
    )

    return LaunchDescription([
        ur_sim_launch,
        delayed_writer_node
    ])
