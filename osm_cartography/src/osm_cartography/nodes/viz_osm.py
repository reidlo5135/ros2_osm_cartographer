import sys

import geodesy.props
import geodesy.utm
import geodesy.wu_point
from geodesy import bounding_box

from geographic_msgs.msg import GeoPoint
from geographic_msgs.srv import GetGeographicMap
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

class VizNode(Node):
    def __init__(self):
        super().__init__("viz_osm")

        map_url = "file://" + self.declare_parameter("map_url").value
        # map_url = self.declare_parameter("map_url").value
        # map_url = "file:///home/wavem/ros2_ws/src/ros2_osm_cartographer/osm_cartography/tests/prc.osm"

        self.pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        self.map = None
        self.msg = None

        self.get_map = self.create_client(GetGeographicMap, 'get_geographic_map')
        while not self.get_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.timer_interval = 3
        self.marker_life = rclpy.duration.Duration(seconds=self.timer_interval + 1).to_msg()
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

        self.get_logger().info(f"Map URL: {map_url}")

        req = GetGeographicMap.Request()
        req.url = map_url
        req.bounds = bounding_box.makeGlobal()

        self.future = self.get_map.call_async(req)

    def get_markers(self, gmap):
        self.map = gmap

        self.map_points = geodesy.wu_point.WuPointSet(gmap.points)
        self.msg = MarkerArray()
        self.mark_boundaries(ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8))
        self.mark_way_points(ColorRGBA(r=1., g=1., b=0., a=0.8))

        road_props = {'bridge', 'highway', 'tunnel'}
        fargs = [(lambda f: geodesy.props.match(f, road_props),
                  ColorRGBA(r=8., g=0.2, b=0.2, a=0.8),
                  "roads_osm"),
                 (lambda f: geodesy.props.match(f, {'building'}),
                  ColorRGBA(r=0., g=0.3, b=0.7, a=0.8),
                  "buildings_osm"),
                 (lambda f: geodesy.props.match(f, {'railway'}),
                  ColorRGBA(r=0., g=0.7, b=.7, a=0.8),
                  "railroad_osm"),
                 (lambda f: geodesy.props.match(f, {'amenity', 'landuse'}),
                  ColorRGBA(r=0., g=1., b=0., a=0.5),
                  "other_osm")]
        for args in fargs:
            self.mark_features(*args)

    def mark_boundaries(self, color):
        marker = Marker(header=self.map.header,
                        ns="bounds_osm",
                        id=0,
                        type=Marker.LINE_STRIP,
                        action=Marker.ADD,
                        scale=Vector3(x=2.),
                        color=color,
                        lifetime=self.marker_life)

        bbox = self.map.bounds
        min_lat, min_lon, max_lat, max_lon = bounding_box.getLatLong(bbox)
        p0 = geodesy.utm.fromLatLong(min_lat, min_lon).toPoint()
        p1 = geodesy.utm.fromLatLong(min_lat, max_lon).toPoint()
        p2 = geodesy.utm.fromLatLong(max_lat, max_lon).toPoint()
        p3 = geodesy.utm.fromLatLong(max_lat, min_lon).toPoint()

        marker.points.append(p0)
        marker.points.append(p1)
        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.points.append(p3)
        marker.points.append(p0)
        self.msg.markers.append(marker)

    def mark_features(self, predicate, color, namespace):
        index = 0
        for feature in filter(predicate, self.map.features):
            marker = Marker(header=self.map.header,
                            ns=namespace,
                            id=index,
                            type=Marker.LINE_STRIP,
                            action=Marker.ADD,
                            scale=Vector3(x=2.),
                            color=color,
                            lifetime=self.marker_life)
            index += 1
            prev_point = None
            for mbr in feature.components:
                wu_point = self.map_points.get(str(mbr.uuid))
                if wu_point: 
                    p = wu_point.toPointXY()
                    if prev_point:
                        marker.points.append(prev_point)
                        marker.points.append(p)
                    prev_point = p
            self.msg.markers.append(marker)

    def mark_way_points(self, color):
        index = 0
        for wp in self.map_points:
            marker = Marker(header=self.map.header,
                            ns="waypoints_osm",
                            id=index,
                            type=Marker.CYLINDER,
                            action=Marker.ADD,
                            scale=Vector3(x=2., y=2., z=0.2),
                            color=color,
                            lifetime=self.marker_life)
            index += 1

            marker.pose.position = wp.toPointXY()
            marker.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)
            self.msg.markers.append(marker)

    def timer_callback(self):
       if self.msg is not None:
            now = self.get_clock().now().to_msg()
            for m in self.msg.markers:
                m.header.stamp = now
            self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)

    viznode = VizNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(viznode)
            if viznode.future.done():
                try:
                    result = viznode.future.result()
                except Exception as e:
                    viznode.get_logger().error(f"Service call failed: {e}")
                else:
                    if result.success:
                        viznode.get_markers(result.map)
                        viznode.pub.publish(viznode.msg)
                    else:
                        print('get_geographic_map failed, status:', str(result.status))
    except rclpy.exceptions.ROSInterruptException:
        pass

    viznode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())