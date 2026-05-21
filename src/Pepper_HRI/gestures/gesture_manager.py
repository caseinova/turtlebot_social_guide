import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory

try:
    from . import default_pose
    from . import explain_gesture
    from . import listening_gesture
    from . import nodding_gesture
    from . import wave_hello_gesture
except ImportError:
    import default_pose
    import explain_gesture
    import listening_gesture
    import nodding_gesture
    import wave_hello_gesture


class GestureManager(Node):

    def __init__(self):
        super().__init__('gesture_manager')

        self.left_arm_client = ActionClient(self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')
        self.right_arm_client = ActionClient(self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')

        self.right_hand_client = ActionClient(self, FollowJointTrajectory, '/right_hand_controller/follow_joint_trajectory')
        self.left_hand_client = ActionClient(self, FollowJointTrajectory, '/left_hand_controller/follow_joint_trajectory')

        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        self.state_file = "/home/hayden/ros_ws/src/pepper_ign_moveit2/pepper_robot_description/launch/previous_gesture.txt"
        self.current_cmd = "default"

        self.gesture_timer = self.create_timer(4.5, self.gesture_loop_callback)

        self.subscription = self.create_subscription(
            String,
            '/pepper/gesture_command',
            self.command_callback,
            10)

        self.get_logger().info("Gesture Controller Node is ready.")

    def command_callback(self, msg):
        new_cmd = msg.data.lower().strip()
        self.get_logger().info(f"Received new command string: {new_cmd}")

        if new_cmd != self.current_cmd:
            self.current_cmd = new_cmd

            if self.current_cmd in ["nod", "wave_hello", "listen", "default"]:
                self.execute_single_gesture(self.current_cmd)

    def gesture_loop_callback(self):
        if self.current_cmd == "explain":
            self.get_logger().info("Looping explain gesture...")
            explain_gesture.execute(self)

    def execute_single_gesture(self, cmd):
        if cmd == "nod":
            nodding_gesture.execute(self)
        elif cmd == "listen":
            listening_gesture.execute(self)
        elif cmd == "wave_hello":
            wave_hello_gesture.execute(self)
        elif cmd == "default":
            default_pose.execute(self)
        else:
            self.get_logger().info(f"Unknown command state: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = GestureManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
