import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal_pose)
        self.get_logger().info('Goal pose publisher initialized')

    def publish_goal_pose(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.publisher.publish(goal_pose)
        self.get_logger().info('Goal pose sent: x={}, y={}'.format(
            goal_pose.pose.position.x, goal_pose.pose.position.y))

def main(args=None):
    rclpy.init(args=args)
    goal_pose_publisher = GoalPosePublisher()
    rclpy.spin(goal_pose_publisher)
    goal_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
