import sys

from geodesy import bounding_box

from geographic_msgs.srv import GetGeographicMap

import rclpy
from rclpy.node import Node

class ClientNode(Node):
    def __init__(self, url):
        super().__init__("osm_client")

        self.cli = self.create_client(GetGeographicMap, 'get_geographic_map')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = self.cli.Request(url, bounding_box.makeGlobal())

        try:
            get_map = rclpy.ServiceProxy('get_geographic_map', GetGeographicMap)
            resp = get_map(url, bounding_box.makeGlobal())
            if resp.success:
                print(resp.map)
            else:
                print('get_geographic_map failed, status: ', str(resp.status))

        except rclpy.ServiceException as e:
            print("Service call failed: " + str(e))

    def send_request(self):
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    url = ''
    if len(args) == 2:
        url = args[1]
    else:
        print('usage: osm_client <map_URL>')
        sys.exit(-9)

    try:
        client = ClientNode(url)
        client.send_request()
    except rclpy.exceptions.ROSInterruptException:
        pass

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                print(response)
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
