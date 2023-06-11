import math
import sys
import threading

from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RoutePath
from geographic_msgs.srv import GetGeoPath
from geographic_msgs.srv import GetRoutePlan
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from route_network import planner
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import geodesy
import rclpy
from rclpy.node import Node
import tf2_ros

class RvizGoalListener(Node):
    def __init__(self):
        super().__init__("rviz_goal")

        self._lock = threading.RLock()
        self.graph = None
        self.get_logger().info("Waiting for 'get_geo_path' service...")

        self.cli = self.create_client(GetGeoPath, 'get_geo_path')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.viz_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 1)

        self._sub_fix = self.create_subscription(NavSatFix, 'fix', self.gps_fix_callback, 10)

        self._sub_init = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initpose_callback, 10)

        self._sub_goal = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)

        self._own_geo_point = None
        self._own_utmpoint = None
        tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tf2_buffer, self)
        self.gridZone = None

        self._sub_route = self.create_subscription(RouteNetwork, 'route_network', self.graph_callback, 10)

    def graph_callback(self, graph):
        if len(graph.points) > 0:
            utm = geodesy.utm.fromLatLong(graph.points[0].position.latitude,
                                          graph.points[0].position.longitude,
                                          graph.points[0].position.altitude)
            self.gridZone = utm.gridZone()
            self.points = geodesy.wu_point.WuPointSet(graph.points)
            self.segment_ids = {}
            for sid in xrange(len(graph.segments)):
                self.segment_ids[graph.segments[sid].id.uuid] = sid
            self.graph = graph

    def initpose_callback(self, msg):
        with self._lock:
            self.get_logger().info("Init pose received.")
            if not self.gridZone is None:
                (map_trans, map_rot) = self.listener.lookupTransform(msg.header.frame_id, "/world", rclpy.Time(0))
                self._own_utmpoint = geodesy.utm.UTMPoint(
                    easting=msg.pose.pose.position.x - map_trans[0],
                    northing=msg.pose.pose.position.y - map_trans[1],
                    altitude=msg.pose.pose.position.z - map_trans[2],
                    zone=self.gridZone[0],
                    band=self.gridZone[1])
                self._own_geo_point = self._own_utmpoint.toMsg()
            else:
                self.get_logger().error("Unknown UMT grid zone!")

    def gps_fix_callback(self, msg):
        with self._lock:
            self._own_utmpoint = geodesy.utm.fromLatLong(msg.latitude, msg.longitude, msg.altitude)
            self._own_geo_point = GeoPoint(msg.latitude, msg.longitude, msg.altitude)

    def goal_callback(self, msg):
        with self._lock:
            self.get_logger().info("Goal pose received.")
            if not self._own_geo_point is None:
                try:
                    map_trans, map_rot = self.listener.lookupTransform(msg.header.frame_id, "/world", rclpy.Time(0))
                    gridZone = self._own_utmpoint.gridZone()
                    goal_utm = geodesy.utm.UTMPoint(easting=msg.pose.position.x - map_trans[0],
                                                    northing=msg.pose.position.y - map_trans[1],
                                                    altitude=msg.pose.position.z - map_trans[2],
                                                    zone=gridZone[0],
                                                    band=gridZone[1])
                    geo_goal = goal_utm.toMsg()
                    path_plan = self.get_geo_path(self._own_geo_point, geo_goal)
                    self._pub_viz_path([pose.pose.position for pose in path_plan.plan.poses],
                                       self._own_geo_point, geo_goal, path_plan.start_seg,
                                       path_plan.goal_seg, path_plan.distance)
                except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                    import traceback
                    print(traceback.format_exc())
            else:
                print("no init pose")

    def _pub_viz_path(self, path, start, goal, start_seg, end_seg, distance):
        if self.viz_pub.get_num_connections() > 0 and self.graph is not None:
            life_time = rclpy.Duration(120)
            marks = MarkerArray()
            hdr = self.graph.header
            hdr.stamp = self.now()
            index = 0
            marker = Marker(header=hdr,
                            ns='start segment',
                            id=index,
                            type=Marker.LINE_STRIP,
                            action=Marker.ADD,
                            scale=Vector3(x=3.0),
                            color=ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.8),
                            lifetime=life_time)
            segment = self.graph.segments[self.segment_ids[start_seg.uuid]]
            marker.points.append(self.points[segment.start.uuid].toPointXY())
            marker.points.append(self.points[segment.end.uuid].toPointXY())
            marks.markers.append(marker)
            marker = Marker(header=hdr,
                            ns='goal segment',
                            id=index,
                            type=Marker.LINE_STRIP,
                            action=Marker.ADD,
                            scale=Vector3(x=3.0),
                            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
                            lifetime=life_time)
            segment = self.graph.segments[self.segment_ids[end_seg.uuid]]
            marker.points.append(self.points[segment.start.uuid].toPointXY())
            marker.points.append(self.points[segment.end.uuid].toPointXY())
            marks.markers.append(marker)

            marker = Marker(header=hdr,
                            ns='start',
                            id=index,
                            type=Marker.CYLINDER,
                            action=Marker.ADD,
                            scale=Vector3(x=6.0, y=4., z=10.0),
                            color=ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.8),
                            lifetime=life_time)
            utm_start = geodesy.utm.fromMsg(start)
            marker.pose.position.x = utm_start.easting
            marker.pose.position.y = utm_start.northing
            marks.markers.append(marker)
            index += 1
            path_marker = Marker(header=hdr,
                                 ns='path',
                                 id=index,
                                 type=Marker.LINE_STRIP,
                                 action=Marker.ADD,
                                 scale=Vector3(x=3.0),
                                 color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8),
                                 lifetime=life_time)
            for geo_point in path:
                utm_point = geodesy.utm.fromMsg(geo_point)
                path_marker.points.append(Point(utm_point.easting, utm_point.northing, 0.))
            marks.markers.append(path_marker)
            index += 1
            marker = Marker(header=hdr,
                            ns='goal',
                            id=index,
                            type=Marker.CYLINDER,
                            action=Marker.ADD,
                            scale=Vector3(x=6.0, y=4., z=10.0),
                            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
                            lifetime=life_time)
            utm_goal = geodesy.utm.fromMsg(goal)
            marker.pose.position.x = utm_goal.easting
            marker.pose.position.y = utm_goal.northing
            marks.markers.append(marker)

            marker = Marker(header=hdr,
                            ns='goal distance',
                            id=index,
                            text='{0:.2f}m'.format(distance),
                            type=Marker.TEXT_VIEW_FACING,
                            action=Marker.ADD,
                            scale=Vector3(x=6.0, y=4., z=10.0),
                            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
                            lifetime=life_time)
            utm_goal = geodesy.utm.fromMsg(goal)
            marker.pose.position.x = utm_goal.easting
            marker.pose.position.y = utm_goal.northing
            marker.pose.position.z = 15
            marks.markers.append(marker)
            self.viz_pub.publish(marks)


def main(args=None):
    rclpy.init(args=args)
    node_class = RvizGoalListener()

    try:
        rclpy.spin(node_class)
    except rclpy.ROSInterruptException:
        pass

    node_class.destroy_node()
    rclpy.shutdown()

    return 0

if __name__ == '__main__':
    sys.exit(main())