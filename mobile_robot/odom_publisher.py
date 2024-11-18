import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, TransformStamped, Vector3
import tf2_ros

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)
        self.timer_ = self.create_timer(1.0, self.publish_odom)
        self.odom_msg = Odometry()
        self.tf_msg = TransformStamped()

    def publish_odom(self):
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.odom_msg.twist.twist = Twist()

        self.odom_publisher_.publish(self.odom_msg)
        self.get_logger().info('Publishing odom message')

        # Publish transform from odom to base_link
        self.tf_msg.header.stamp = self.odom_msg.header.stamp
        self.tf_msg.header.frame_id = 'odom'
        self.tf_msg.child_frame_id = 'base_link'
        self.tf_msg.transform.translation = Vector3(x=self.odom_msg.pose.pose.position.x,
                                                    y=self.odom_msg.pose.pose.position.y,
                                                    z=self.odom_msg.pose.pose.position.z)
        self.tf_msg.transform.rotation = self.odom_msg.pose.pose.orientation

        self.tf_broadcaster_.sendTransform(self.tf_msg)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
