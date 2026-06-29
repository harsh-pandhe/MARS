import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RobotKiller(Node):
    def __init__(self):
        super().__init__('robot_killer')
        self.get_logger().info("Robot Killer Node initialized and active.")
        
        # Wait 15 seconds into the test run before injecting a failure
        self.timer = self.create_timer(15.0, self.inject_failure)
        self.hijack_timer = None
        self.failed_robot = None
        self.pub = None
        
    def inject_failure(self):
        self.timer.cancel()  # Disable startup timer so failure injection is single-trigger
        
        robots = ['tb1', 'tb2', 'tb3']
        self.failed_robot = random.choice(robots)
        self.get_logger().error(f"==================================================")
        self.get_logger().error(f"!!! CRITICAL FAILURE INJECTED: DIALING OUT {self.failed_robot.upper()} !!!")
        self.get_logger().error(f"==================================================")
        
        # Create publisher to override and hijack command velocities
        self.pub = self.create_publisher(Twist, f"/{self.failed_robot}/cmd_vel", 10)
        
        # Override velocity to zero at 10Hz frequency
        self.hijack_timer = self.create_timer(0.1, self.publish_zeros)
        
    def publish_zeros(self):
        msg = Twist()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotKiller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
