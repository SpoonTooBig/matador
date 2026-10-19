import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ApriltagFollow(Node):

    def __init__(self):
        super().__init__('apriltag_follow')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
            )
        
        # Publisher creation: create_publisher(MsgType, 'topic_name', queue_size)
        self.apritag_subscriber = self.create_subscription(AprilTagDetectionArray, '/detections', self.apriltag_callback, qos_profile)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)

    def apriltag_callback(self, msg):
        self.get_logger().info(f'Received AprilTag detections: {len(msg.detections)}')
        
        if len(msg.detections) > 0:
            detection = msg.detections[0] # Get the first detection if there is one
            x = detection.centre.x
            y = detection.centre.y
            output = self.generate_twist(0.2, 0.0)
        else:
            output = self.generate_twist(0.0, 0.0)
            
        self.cmd_vel_publisher.publish(output)
      
    def generate_twist(self, x, theta):
        output = Twist()
        
        output.linear.x = x
        output.linear.y = 0.0
        output.linear.z = 0.0

        output.angular.x = 0.0
        output.angular.y = 0.0
        output.angular.z = theta
    
        return output
    
def main(args=None):
    rclpy.init(args=args)
    apriltag_follow = ApriltagFollow()

    try:
        rclpy.spin(apriltag_follow)
    except KeyboardInterrupt:
        pass
    finally:
        apriltag_follow.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()