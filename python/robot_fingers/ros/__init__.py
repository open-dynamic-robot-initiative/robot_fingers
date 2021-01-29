"""ROS-related classes/functions."""
import rclpy
import rclpy.node
from std_msgs.msg import String
from std_srvs.srv import Empty


class NotificationNode(rclpy.node.Node):
    """Simple ROS node for communication with other processes."""

    def __init__(self, name):
        super().__init__(name)
        self._status_publisher = self.create_publisher(String, "~/status", 1)

        self.shutdown_requested = False
        self._shutdown_srv = self.create_service(
            Empty, "~/shutdown", self.shutdown_callback
        )

    def shutdown_callback(self, request, response):
        self.shutdown_requested = True
        return response

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_publisher.publish(msg)
