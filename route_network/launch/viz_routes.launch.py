import sys

from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch.actions import DeclareLaunchArgument
from launch_ros import actions, get_default_launch_description


def generate_launch_description():
    url = ""
    route_network = actions.Node(
        package='route_network', node_executable='route_network', output='screen',
        arguments=["map_url", url])

    viz_routes = actions.Node(
        package='route_network', node_executable='viz_routes', output='screen',
        arguments=["map_url", url])

    return LaunchDescription(
        [DeclareLaunchArgument('url', default_value='', description="Map url"), route_network, viz_routes])

def main():
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == '__main__':
    main()