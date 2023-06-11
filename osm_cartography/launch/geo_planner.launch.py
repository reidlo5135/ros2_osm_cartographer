import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions

def generate_launch_description():
    map_url = os.path.join(get_package_share_directory("osm_cartography"), "tests", "prc.osm")
    rviz_config_path = os.path.join(get_package_share_directory("osm_cartography"), "rviz", "geo_planner.rviz")

    osm_server = actions.Node(
        package='osm_cartography', 
        executable='osm_server',
        output='screen'
    )

    viz_osm = actions.Node(
        package='osm_cartography',
        executable='viz_osm',
        output='screen',
        arguments=["map_url", map_url]
    )

    route_network = actions.Node(
        package='route_network',
        executable='route_network',
        output='screen',
        arguments=["map_url", map_url]
    )

    plan_route = actions.Node(
        package='route_network',
        executable='plan_route',
        output='screen'
    )

    tf_world_map = actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        output='screen',
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "map"]
    )

    tf_map_local_map = actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["622150", "3362350", "0", "0", "0", "0", "1", "map", "local_map"]
    )

    rviz2 = actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=["-d", rviz_config_path]
    )

    rviz_goal = actions.Node(
        package='route_network',
        executable='rviz_goal',
        output='screen'
    )

    return LaunchDescription(
            [osm_server, viz_osm, route_network, plan_route, tf_world_map, tf_map_local_map, rviz2, rviz_goal]
        )

def main():
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()