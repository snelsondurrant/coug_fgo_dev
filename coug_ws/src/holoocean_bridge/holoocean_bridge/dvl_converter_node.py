import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped


class DvlConverterNode(Node):
    """
    Converts DVL data from HoloOcean to a TwistWithCovarianceStamped message.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Nov 2025

    Subscribers:
    - auv0/DVLSensorVelocity (geometry_msgs/TwistWithCovarianceStamped)
    Publishers:
    - dvl/data (geometry_msgs/TwistWithCovarianceStamped)
    """

    def __init__(self):
        super().__init__("dvl_converter_node")

        self.declare_parameter("input_topic", "auv0/DVLSensorVelocity")
        self.declare_parameter("output_topic", "dvl/data")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.1)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.noise_sigma = (
            self.get_parameter("noise_sigma").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            TwistWithCovarianceStamped, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped, output_topic, 10
        )

        self.get_logger().info(
            f"DVL converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: TwistWithCovarianceStamped):

        msg.header.frame_id = self.frame_id
        if self.override_covariance:
            covariance = self.noise_sigma * self.noise_sigma
            msg.twist.covariance[0] = covariance  # Vx
            msg.twist.covariance[7] = covariance  # Vy
            msg.twist.covariance[14] = covariance  # Vz

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    dvl_converter_node = DvlConverterNode()
    try:
        rclpy.spin(dvl_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
