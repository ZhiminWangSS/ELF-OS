from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    root=Path(__file__).resolve().parents[1];params=str(root/'config/nav2.yaml')
    nodes=[DeclareLaunchArgument('localization',default_value='true'),DeclareLaunchArgument('rviz',default_value='false')]
    nodes.extend([
        Node(package='fast_lio',executable='fastlio_mapping',name='prior_lio',output='screen',parameters=[str(root/'config/localization.yaml')],remappings=[('/Odometry','/lio/odom'),('/map_save','/localization/unused_map_save')],condition=IfCondition(LaunchConfiguration('localization'))),
        Node(package='elf_navigation',executable='prior_initializer',output='screen',parameters=[{'map_path':str(root/'maps/floor_1789552084236/map.pcd')}],condition=IfCondition(LaunchConfiguration('localization'))),
        ExecuteProcess(cmd=['/usr/bin/python3',str(root/'tools/localization_adapter.py'),'--robot',str(root/'config/robot.yaml')],output='screen',condition=IfCondition(LaunchConfiguration('localization')))
    ])
    for package,exe,name in [('nav2_map_server','map_server','map_server'),('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server'),('nav2_bt_navigator','bt_navigator','bt_navigator')]:
        nodes.append(Node(package=package,executable=exe,name=name,output='screen',parameters=[params],remappings=[('cmd_vel','/navigation/cmd_vel')]))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle',output='screen',parameters=[{'autostart':True,'node_names':['map_server','planner_server','controller_server','bt_navigator']}]))
    nodes.append(ExecuteProcess(cmd=['/usr/bin/python3',str(root/'tools/velocity_bridge.py')],output='screen'))
    nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/navigation.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))))
    return LaunchDescription(nodes)
