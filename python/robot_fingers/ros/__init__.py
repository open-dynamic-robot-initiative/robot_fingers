"""ROS-related classes/functions."""
import rclpy
import rclpy.node
import rclpy.qos
from std_msgs.msg import String
from std_srvs.srv import Empty


class NotificationNode(rclpy.node.Node):
    """Simple ROS node for communication with other processes."""

    _status_topic_name = "~/status"
    _shutdown_service_name = "~/shutdown"

    def __init__(self, name):
        super().__init__(name)

        # quality of service profile keep the last published message for late
        # subscribers (like "latched" in ROS 1).
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )

        self._status_publisher = self.create_publisher(
            String, self._status_topic_name, qos_profile
        )

        self.shutdown_requested = False
        self._shutdown_srv = self.create_service(
            Empty, self._shutdown_service_name, self.shutdown_callback
        )

    def shutdown_callback(self, request, response):
        self.get_logger().info(
            "Node {} received shutdown request.".format(self.get_name())
        )
        self.shutdown_requested = True
        return response

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_publisher.publish(msg)
