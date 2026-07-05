import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class SemanticVisionNode(Node):
    """
    Semantic Vision Node:
    Subscribes to namespaced camera images, performs real-time HSV color segmentation
    to detect coffee shop landmarks (e.g., red/yellow cups or safety zones),
    estimates their relative bearing and distance, and publishes their global positions.
    """
    def __init__(self):
        super().__init__('semantic_vision')
        
        # 1. Parameters
        self.declare_parameter('robot_name', 'tb1')
        self.robot_name = self.get_parameter('robot_name').value
        
        self.bridge = CvBridge()
        self.current_pose = (0.0, 0.0, 0.0) # (x, y, yaw)
        self.latest_scan = None
        
        # 2. Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            f'/{self.robot_name}/camera/image_raw',
            self.image_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10
        )
        
        # 3. Publishers
        self.landmark_pub = self.create_publisher(
            PointStamped,
            f'/{self.robot_name}/detected_landmarks',
            10
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            f'/{self.robot_name}/camera/debug_image',
            10
        )
        
        # 4. Define target color range (HSV for Red/Orange landmarks like coffee cups/cones)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.get_logger().info(f"[{self.robot_name}] Semantic Vision Node initialized.")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Simple quaternion to yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = (pos.x, pos.y, yaw)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        # Throttle processing to 5 Hz (every 0.2 seconds) to avoid CPU bottleneck
        now = self.get_clock().now()
        if hasattr(self, 'last_process_time'):
            dt = (now - self.last_process_time).nanoseconds / 1e9
            if dt < 0.2:
                return
        self.last_process_time = now

        try:
            # Convert ROS Image to OpenCV BGR image
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        h, w, _ = cv_img.shape
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        # Segment Red colors representing specific coffee shop landmarks
        mask1 = cv2.inRange(hsv_img, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv_img, self.lower_red2, self.upper_red2)
        mask = mask1 | mask2
        
        # Morphological operations to clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        debug_img = cv_img.copy()
        
        if contours:
            # Get largest contour representing the landmark
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 150: # Minimum size threshold
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Draw visual indicator on debug image
                    cv2.circle(debug_img, (cx, cy), 10, (0, 255, 0), -1)
                    cv2.drawContours(debug_img, [largest_contour], -1, (0, 0, 255), 2)
                    
                    # Estimate relative angle: mapping horizontal pixel coordinate to angle
                    # Camera Horizontal FOV is approx 80 degrees (1.396 rad)
                    fov_h = 80.0 * (math.pi / 180.0)
                    rel_angle = -((cx - (w / 2.0)) / (w / 2.0)) * (fov_h / 2.0)
                    
                    # Estimate distance based on physical height/width or Lidar mapping helper
                    distance = 2.0 # Default fallback distance (meters)
                    if self.latest_scan is not None:
                        # Match relative angle with corresponding Lidar sector distance
                        ranges = self.latest_scan.ranges
                        num_ranges = len(ranges)
                        scan_angle_min = self.latest_scan.angle_min
                        scan_angle_inc = self.latest_scan.angle_increment
                        
                        # Find closest Lidar beam matching target angle
                        target_index = int((rel_angle - scan_angle_min) / scan_angle_inc)
                        if 0 <= target_index < num_ranges:
                            beam_val = ranges[target_index]
                            if not np.isnan(beam_val) and not np.isinf(beam_val) and beam_val > 0.12:
                                distance = beam_val
                                
                    # Compute global landmark coordinates using robot's current pose
                    rx, ry, ryaw = self.current_pose
                    global_angle = ryaw + rel_angle
                    lx = rx + distance * math.cos(global_angle)
                    ly = ry + distance * math.sin(global_angle)
                    
                    # Publish the PointStamped landmark message
                    landmark_msg = PointStamped()
                    landmark_msg.header.stamp = self.get_clock().now().to_msg()
                    landmark_msg.header.frame_id = 'tb1/odom'
                    landmark_msg.point.x = lx
                    landmark_msg.point.y = ly
                    landmark_msg.point.z = 0.0
                    self.landmark_pub.publish(landmark_msg)
                    
                    cv2.putText(debug_img, f"Landmark: dist={distance:.2f}m, angle={rel_angle*180/math.pi:.1f}deg",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                
        # Publish debug image to view in RViz
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header.stamp = msg.header.stamp
            debug_msg.header.frame_id = f"{self.robot_name}/camera_link"
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
