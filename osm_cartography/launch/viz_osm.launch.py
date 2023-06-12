import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions

def generate_launch_description():
    map_url = "file://" + os.path.join(get_package_share_directory("osm_cartography"), "tests", "prc.osm")

    osm_server = actions.Node(
        package='osm_cartography', node_executable='osm_server', output='screen')

    viz_osm = actions.Node(
        package='osm_cartography', node_executable='viz_osm', output='screen',
        parameters=[{"map_url": map_url}])

    return LaunchDescription([osm_server, viz_osm])

def main():
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()