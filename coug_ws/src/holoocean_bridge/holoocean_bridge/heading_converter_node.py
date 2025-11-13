import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class HeadingConverterNode(Node):
    """
    Converts heading data from HoloOcean to a standard Imu message.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Nov 2025

    Subscribers:
    - auv0/DynamicsSensorIMU (sensor_msgs/msg/Imu)
    Publishers:
    - imu/heading (sensor_msgs/msg/Imu)
    """

    def __init__(self):
        super().__init__("heading_converter_node")

        self.declare_parameter("input_topic", "auv0/DynamicsSensorIMU")
        self.declare_parameter("output_topic", "imu/heading")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.05)

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
            Imu, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Imu, output_topic, 10)

        self.get_logger().info(
            f"IMU converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: Imu):
        
        msg.header.frame_id = self.frame_id
        if self.override_covariance:
            covariance = self.noise_sigma * self.noise_sigma
            msg.orientation_covariance[8] = covariance  # Z axis

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    heading_converter_node = HeadingConverterNode()
    try:
        rclpy.spin(heading_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        heading_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
