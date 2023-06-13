import sys

from route_network import planner

from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RoutePath

from geographic_msgs.srv import GetRoutePlan
from geographic_msgs.srv import GetGeoPath
import geodesy
import rclpy
from rclpy.node import Node


class RoutePlannerNode(Node):

    def __init__(self):
        super().__init__("route_planner")
        self.graph = None
        self.planner = None
        self._calc_idx = 0

        self.srv = self.create_service(GetRoutePlan, 'get_route_plan', self.route_planner)

        self.srv_geo = self.create_service(GetGeoPath, 'get_geo_path', self.geo_path_planner)

        self.resp = None

        self.sub = self.create_subscription(RouteNetwork, 'route_network', self.graph_callback, 10)

    def graph_callback(self, graph):
        self.graph = graph
        self.planner = planner.Planner(graph)

    def route_planner(self, req, resp):
        self.get_logger().info('GetRoutePlan: ' + str(req))
        if self.graph is None:
            resp.success = False
            resp.status = 'no RouteNetwork available for GetRoutePlan'
            self.get_logger().error(resp.status)
            return resp

        try:
            resp.plan = self.planner.planner(req)
        except (ValueError, planner.NoPathToGoalError) as e:
            resp.success = False
            resp.status = str(e)
            self.get_logger().error('route planner exception: ' + str(e))
        else:
            resp.success = True
            resp.plan.header.stamp = self.now()
            resp.plan.header.frame_id = self.graph.header.frame_id
        return resp

    def geo_path_planner(self, req, resp):
        """
        Processes the planning request.

        If the planning service fails, self.resp.success is set to False.

        :param req: The planning request (start and goal positions).
        :type req: geographic_msgs/GetGeoPath

        :return: The plan from start to goal position.
        :rtype: geographic_msgs/GetGeoPathResponse 
        """
        self.get_logger().info('GetGeoPath: ' + str(req))
        self._calc_idx += 1
        resp.plan.header.seq = self._calc_idx
        resp.plan.header.stamp = self.now()
        if self.graph is None:
            resp.success = False
            resp.status = 'No RouteNetwork available for GetGeoPath.'
            self.get_logger().error(self.resp.status)
            return resp

        try:
            planner_result = self.planner.geo_path(req)

            resp.plan.poses = [GeoPoseStamped(pose=GeoPose(position=point)) for point in
                               planner_result[0]]
            resp.network = planner_result[1]
            resp.start_seg = planner_result[2]
            resp.goal_seg = planner_result[3]
            resp.distance = planner_result[4]
            resp.success = not (resp.distance == -1)
            if not resp.success:
                resp.status = 'No route found from :\n' + str(req.start) + ' to:\n' + str(
                    req.goal)
            self.get_logger().info('GetGeoPath result: ' + str(self.resp))
        except ValueError as e:
            resp.success = False
            resp.status = str(e)
            resp.network = self.graph.id
            resp.distance = -1
            self.get_logger().error('Route planner exception: ' + str(e))

        return resp


def main(args=None):
    rclpy.init(args=args)
    node_class = RoutePlannerNode()

    try:
        rclpy.spin(node_class)  # wait for messages
    except rclpy.exceptions.ROSInterruptException:
        pass

    node_class.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
