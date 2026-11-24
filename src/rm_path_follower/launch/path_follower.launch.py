from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rm_path_follower')
    mode = LaunchConfiguration('mode')
    extract_map = LaunchConfiguration('extract_map')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='auto', choices=['manual', 'auto']),
        DeclareLaunchArgument('extract_map', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('algorithm', default_value='astar'),
        DeclareLaunchArgument('goal_x', default_value='5.5'),
        DeclareLaunchArgument('goal_y', default_value='4.5'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # Simulação do robô e mundo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('rm_bot_sim'), 'launch', 'rm_bot.launch.py'])
            ),
            launch_arguments={
                'world': PathJoinSubstitution([share, 'worlds', 'main_world.sdf']),
                'rviz_config': PathJoinSubstitution([share, 'rviz', 'aula04.rviz']),
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz')
            }.items()
        ),

        # Ponte da câmera superior (sobe apenas se extract_map:=true)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/map_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen',
            condition=IfCondition(extract_map)
        ),

        # Extrator de mapa (sobe apenas se extract_map:=true)
        Node(
            package='rm_path_follower',
            executable='map_extractor',
            output='screen',
            condition=IfCondition(extract_map)
        ),

        # Nó de Planejamento (A*, etc.)
        Node(
            package='rm_path_follower',
            executable='planner_node',
            parameters=[{
                'algorithm': LaunchConfiguration('algorithm'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }],
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'auto'"]))
        ),

        # Nó Controlador Fixo (Feedback Linearization)
        Node(
            package='rm_path_follower',
            executable='controller_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'auto'"]))
        ),
    ])