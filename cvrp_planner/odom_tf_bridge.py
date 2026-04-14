import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.declare_parameter('odom_topic', 'chassis/odom')
        self.declare_parameter('parent_frame_id', '')
        self.declare_parameter('child_frame_id', '')

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.parent_frame_override = str(self.get_parameter('parent_frame_id').value)
        self.child_frame_override = str(self.get_parameter('child_frame_id').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._handle_odom,
            50,
        )
        self.get_logger().info(
            f'将把里程计话题 {self.odom_topic} 转成 TF (odom->base_link)',
        )

    def _handle_odom(self, msg: Odometry):
        parent_frame = self.parent_frame_override or msg.header.frame_id or 'odom'
        child_frame = self.child_frame_override or msg.child_frame_id or 'base_link'

        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
