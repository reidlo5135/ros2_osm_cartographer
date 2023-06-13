import sys
import uuid

import geodesy.props
import geodesy.wu_point
from geodesy import bounding_box

from geographic_msgs.msg import KeyValue
from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RouteSegment
from geographic_msgs.srv import GetGeographicMap

import rclpy
from rclpy.node import Node

from unique_identifier_msgs.msg import UUID

PKG_NAME = 'route_network'
PKG_URL = 'http://ros.org/wiki/' + PKG_NAME

def is_oneway(feature):
    return geodesy.props.match(feature, {'oneway'})

def is_route(feature):
   return geodesy.props.match(feature, {'bridge', 'highway', 'tunnel'})


def make_graph(msg):
    uu = UUID(uuid.uuid5(uuid.NAMESPACE_URL, PKG_URL + '/map/' + str(msg.id.uuid) + '/routes'))
    return RouteNetwork(header=msg.header, id=uu, bounds=msg.bounds)


def make_seg(start, end, oneway=False):
    uu = UUID(uuid.uuid5(uuid.NAMESPACE_URL, PKG_URL + '/' + str(start) + '/' + str(end)))

    seg = RouteSegment(id=uu, start=start, end=end)
    if oneway:
        seg.props.append(KeyValue(key='oneway', value='yes'))
    return seg


class RouteNetNode(Node):
    def __init__(self):
        super().__init__("route_network")
        self.config = None

        self.pub = self.create_publisher(RouteNetwork, 'route_network', 10)
        self.graph = None

        self.get_map = self.create_client(GetGeographicMap, 'get_geographic_map')

        while not self.get_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def build_graph(self, msg):
        self.map = msg
        self.map_points = geodesy.wu_point.WuPointSet(msg.points)
        self.graph = make_graph(msg)

        for feature in filter(is_route, self.map.features):
            oneway = is_oneway(feature)
            start = None
            for mbr in feature.components:
                pt = self.map_points.get(mbr.uuid)
                if pt is not None:
                    self.graph.points.append(pt.toWayPoint())
                    end = UUID(uuid=mbr.uuid)
                    if start is not None:
                        self.graph.segments.append(make_seg(start, end, oneway))
                        if not oneway:
                            self.graph.segments.append(make_seg(end, start))
                    start = end

def main(args=None):
    rclpy.init(args=args)
    node_class = RouteNetNode()
    try:
        rclpy.spin(node_class)
    except rclpy.exceptions.ROSInterruptException:
        pass

    node_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())