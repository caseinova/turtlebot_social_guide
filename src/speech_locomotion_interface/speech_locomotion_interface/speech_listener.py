import rclpy
from rclpy.node import Node
from geometry_msgs.msg import  PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

LOCATION_DICT = {'pillar': [1.0,1.0]}

class NavigationSpeech(Node):
    def __init__(self):
        super().__init__('speech_listener')
        self.subscriber_ = self.create_subscription(String, '/speech/intent',self.callback_,10)
        self.nav = BasicNavigator()
        self.subscriber_
        self.current_goal = PoseStamped()
    def callback_(self,msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        x_goal,y_goal = LOCATION_DICT[msg.data][0],LOCATION_DICT[msg.data][1]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = x_goal
        goal.pose.position.y = y_goal
        self.get_logger().info('Trying to go to goal')
        self.nav.goToPose(goal)
        


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = NavigationSpeech()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

        