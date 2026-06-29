import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy

class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        
        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 100)
        self.pub_tf_static = self.create_publisher(TFMessage, '/tf_static', static_qos)
        
        self.subs = []
        for i in range(1, 4):
            namespace = f'tb{i}'
            # Subscribe to namespaced tf
            sub_tf = self.create_subscription(
                TFMessage,
                f'/{namespace}/tf',
                lambda msg, ns=namespace: self.tf_callback(msg, ns),
                100
            )
            # Subscribe to namespaced tf_static
            sub_tf_static = self.create_subscription(
                TFMessage,
                f'/{namespace}/tf_static',
                lambda msg, ns=namespace: self.tf_static_callback(msg, ns),
                static_qos
            )
            self.subs.extend([sub_tf, sub_tf_static])
            
        self.get_logger().info("TF Relay Node started, forwarding namespaced transforms to global /tf and /tf_static")
        
    def tf_callback(self, msg, ns):
        new_msg = TFMessage()
        for transform in msg.transforms:
            t = transform
            # Check and prefix frame_id if not already prefixed
            if not t.header.frame_id.startswith(f"{ns}/"):
                t.header.frame_id = f"{ns}/{t.header.frame_id}"
            # Check and prefix child_frame_id if not already prefixed
            if not t.child_frame_id.startswith(f"{ns}/"):
                t.child_frame_id = f"{ns}/{t.child_frame_id}"
            new_msg.transforms.append(t)
        self.pub_tf.publish(new_msg)
        
    def tf_static_callback(self, msg, ns):
        new_msg = TFMessage()
        for transform in msg.transforms:
            t = transform
            # Check and prefix frame_id if not already prefixed
            if not t.header.frame_id.startswith(f"{ns}/"):
                t.header.frame_id = f"{ns}/{t.header.frame_id}"
            # Check and prefix child_frame_id if not already prefixed
            if not t.child_frame_id.startswith(f"{ns}/"):
                t.child_frame_id = f"{ns}/{t.child_frame_id}"
            new_msg.transforms.append(t)
        self.pub_tf_static.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
