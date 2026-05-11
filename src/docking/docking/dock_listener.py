import math
import sqlite3

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from std_msgs.msg import String


class DockingNode(Node):
    def __init__(self):
        super().__init__('dock_listener')

        self.declare_parameter('database_path', 'docks.db')
        self.database_path = self.get_parameter('database_path').value

        self.nav = BasicNavigator()
        self.current_pose = None
        self.con = sqlite3.connect(self.database_path)
        cur = self.con.cursor()
        cur.execute('CREATE TABLE IF NOT EXISTS docks (px, py, pz, qx, qy, qz, qw)')
        self.con.commit()

        self.create_subscription(String, '/dock_command', self.dock_command_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.current_pose_callback, 10)

        self.get_logger().info('Dock listener ready')

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def dock_command_callback(self, msg):
        self.get_logger().info('Received dock command: "%s"' % msg.data)
        dock_pose = self.get_nearest_dock_pose()
        if dock_pose is None:
            return

        self.get_logger().info('Navigating to nearest dock pose')
        self.nav.goToPose(dock_pose)

    def get_nearest_dock_pose(self):
        cur = self.con.cursor()
        docks = cur.execute('SELECT px, py, pz, qx, qy, qz, qw FROM docks').fetchall()

        if not docks:
            self.get_logger().warn('No dock poses found in %s' % self.database_path)
            return None

        if self.current_pose is None:
            self.get_logger().warn('No current pose received yet; using first dock pose')
            return self.row_to_pose(docks[0])

        nearest_dock = min(docks, key=self.distance_to_current_pose)
        return self.row_to_pose(nearest_dock)

    def distance_to_current_pose(self, dock):
        dx = self.current_pose.position.x - dock[0]
        dy = self.current_pose.position.y - dock[1]
        return math.hypot(dx, dy)

    def row_to_pose(self, row):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = row[0]
        pose.pose.position.y = row[1]
        pose.pose.position.z = row[2]
        pose.pose.orientation.x = row[3]
        pose.pose.orientation.y = row[4]
        pose.pose.orientation.z = row[5]
        pose.pose.orientation.w = row[6]
        return pose


def main(args=None):
    rclpy.init(args=args)
    docking_node = DockingNode()
    rclpy.spin(docking_node)
    docking_node.con.close()
    docking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
