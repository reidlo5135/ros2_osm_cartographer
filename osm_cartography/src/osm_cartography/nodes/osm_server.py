from geographic_msgs.srv import GetGeographicMap

import rclpy
from rclpy.node import Node

from osm_cartography import xml_map

class ServerNode(Node):
    def __init__(self):
        super().__init__("osm_server")
        self.srv = self.create_service(GetGeographicMap, 'get_geographic_map', self.map_server)

        self.resp = None

    def map_server(self, req, resp):
        url = str(req.url)
        self.url = url
        self.get_logger().info('[get_geographic_map] ' + url)

        if url == '' and self.resp is not None:
            return self.resp

        try:
            resp.map = xml_map.get_osm(url, req.bounds)
        except (IOError, ValueError) as e:
            self.get_logger().error(str(e))
            resp.success = False
            resp.status = str(e)
        else:
            resp.success = True
            resp.status = url
            resp.map.header.stamp = self.get_clock().now().to_msg()
            resp.map.header.frame_id = '/map'
        return resp

def main(args=None):
    rclpy.init(args=args)
    server = ServerNode()

    try:
        rclpy.spin(server)
    except rclpy.exceptions.ROSInterruptException:
        pass

    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()