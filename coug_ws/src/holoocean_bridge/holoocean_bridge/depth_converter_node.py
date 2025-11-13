import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class DepthConverterNode(Node):
    """
    Converts depth data from HoloOcean to an Odometry message.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Nov 2025

    Subscribers:
    - auv0/DepthSensor (geometry_msgs/PoseWithCovarianceStamped)
    Publishers:
    - odometry/depth (nav_msgs/Odometry)
    """

    def __init__(self):
        super().__init__("depth_converter_node")

        self.declare_parameter("input_topic", "auv0/DepthSensor")
        self.declare_parameter("output_topic", "odometry/depth")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.1)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.noise_sigma = (
            self.get_parameter("noise_sigma").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(
            f"Depth converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: PoseWithCovarianceStamped):
        
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position.z = msg.pose.pose.position.z

        if self.override_covariance:
            odom_msg.pose.covariance[14] = (
                self.noise_sigma * self.noise_sigma
            )  # Z covariance
        else:
            odom_msg.pose.covariance = msg.pose.covariance

        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    depth_converter_node = DepthConverterNode()
    try:
        rclpy.spin(depth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
