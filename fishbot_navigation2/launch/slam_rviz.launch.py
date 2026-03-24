import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径获取
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    # 获取 slam_toolbox 的默认路径（确保你已安装该包）
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # 2. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # SLAM 的配置文件，如果没有自定义的，可以使用 slam_toolbox 默认提供的
    slam_config_file = os.path.join(fishbot_navigation2_dir, 'config', 'mapper_params_online_async.yaml')

    # 3. 定义动作
    
    # 声明 Launch 参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # 包含 SLAM Toolbox 启动文件 (异步模式最常用)
    start_slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': slam_config_file # 指向你的 SLAM 配置文件
        }.items()
    )

    # 启动 RViz
    rviz_config_dir = os.path.join(fishbot_navigation2_dir, 'rviz', 'slam_config.rviz')
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        start_slam_toolbox_node,
        start_rviz_node
    ])