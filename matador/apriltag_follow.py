import rclpy
import numpy as np
import math
from PIDController import PIDController as PID
from rclpy.node import Node
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration

class State(Enum):
    START = 1
    SEARCH = 2
    ALIGN = 3
    APPROACH = 4
    HOLD = 5
    SAFTEY_STOP = 6

class ApriltagFollow(Node):

    def __init__(self):
        super().__init__('apriltag_follow')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
            )
        self.deadzone_theta = 5 # center deadzone either way in degrees
        
        self.t0 = self.get_clock().now() # for hold timeout
        self.state = State.START
        self.rotation_controller = PID(2, .05, 0.0, 0, [-2.5, 2.5])
        self.position_controller = PID(.5, 0.01, 0.0, 0.5, [-0.22, 0.22])
        # Publisher creation: create_publisher(MsgType, 'topic_name', queue_size)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to the actual camera detection topic instead of /tf
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            qos_profile
        )

    def apriltag_callback(self, msg):
        if self.state == State.START:
            print("Press enter to begin searching")
            input()
            self.state = State.SEARCH
                          
        output = self.generate_twist(0.0, 0.0)
        x = 0.0
        z = 0.5

        facing_center = False
        distance_good = False
                        
        tag_found = len(msg.detections) > 0
        
        now = self.get_clock().now()

        if tag_found:
            try:
                transform = self.tf_buffer.lookup_transform('camera', "tag_0_frame", now)
                x = transform.transform.translation.x
                z = transform.transform.translation.z

                self.rotation_controller.update(math.atan(x/z))
                self.position_controller.update(z)
                
                facing_center = abs(self.rotation_controller.prev_error) < self.deadzone_theta*(math.pi/180)
                distance_good = 0.45 < self.position_controller.prev_error < 0.55  
                self.t0 = now              
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"Waiting for TF tree frames to initialize: {e}")
        
        if not tag_found:
            if (now - self.t0).nanoseconds / 1e9 < 1.0:
                self.state = State.HOLD
            else:
                self.state = State.SEARCH
        elif (not facing_center) and (not distance_good):
            self.state = State.ALIGN
        elif facing_center and (not distance_good):
            self.state = State.APPROACH
        elif facing_center and distance_good:
            self.state = State.HOLD
                
        if self.state == State.SEARCH:
            output = self.search_procedure()
        elif self.state == State.ALIGN:
            output = self.align_procedure()
        elif self.state == State.APPROACH:
            output = self.approach_procedure()
        elif self.state == State.HOLD:
            output = self.hold_procedure()
        elif self.state == State.SAFTEY_STOP:
            None
        self.cmd_vel_publisher.publish(output)
    
    def search_procedure(self):
        return self.generate_twist(0.0,0.8)
        
    def align_procedure(self):
        return self.generate_twist(0.0, self.rotation_controller.val)
    
    def approach_procedure(self):   
        output = self.generate_twist(-self.position_controller.val, 0.0)
        return output
    
    def hold_procedure(self):   
        output = self.generate_twist(0.0, 0.0)        
        return output
            
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