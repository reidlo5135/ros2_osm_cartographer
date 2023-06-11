import sys

from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions

def generate_launch_description():
    plan_route = actions.Node(
        package='route_network', node_executable='plan_route', output='screen')
    
    viz_plan = actions.Node(
        package='route_network', node_executable='viz_plan', output='screen')

    return LaunchDescription([plan_route, viz_plan])


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