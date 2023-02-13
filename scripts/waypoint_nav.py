import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.msg import Waypoint

class WaypointAdder(Node):

    def __init__(self):
        super().__init__('waypoint_adder')
        self.get_logger().info('Waypoint adder node started')
        self.waypoints_pub = self.create_publisher(Waypoint, 'waypoints', 10)
        self.follow_waypoints_client = self.create_client(FollowWaypoints, 'follow_waypoints')
        while not self.follow_waypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Follow Waypoints service not available, waiting again...')
        self.add_waypoints()

    def add_waypoints(self):
        waypoint = Waypoint()
        waypoint.pose.header.frame_id = 'map'
        waypoint.pose.pose.position.x = 1.0
        waypoint.pose.pose.position.y = 2.0
        waypoint.pose.pose.position.z = 0.0
        waypoint.pose.pose.orientation.w = 1.0
        self.waypoints_pub.publish(waypoint)
        self.get_logger().info(f'Waypoint added: {waypoint}')
        self.send_waypoints_to_navigator()

    def send_waypoints_to_navigator(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses.append(PoseStamped())
        goal_msg.poses[0].header.frame_id = 'map'
        goal_msg.poses[0].pose = Waypoint().pose.pose
        self.follow_waypoints_client.wait_for_server()
        self.follow_waypoints_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Waypoints sent to the navigator.')
        self.get_logger().info(f'Waiting for the navigator to finish the mission...')
        self.follow_waypoints_client.wait_for_result()
        self.get_logger().info(f'Navigator finished the mission')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_adder = WaypointAdder()
    rclpy.spin(waypoint_adder)

if __name__ == '__main__':
    main()

