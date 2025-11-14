import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import message_filters


class TruthConvertorNode(Node):
    """
    Converts ground truth data from HoloOcean to an Odometry message.

    IMPORTANT! This is only in the map frame when the vehicle starts at the origin (0,0,0) 
    with no rotation. Otherwise, the HoloOcean and map frames need to be aligned in post-processing.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Nov 2025

    Subscribers:
    - auv0/LocationSensor (geometry_msgs/msg/PoseWithCovarianceStamped)
    - auv0/DynamicsSensorIMU (sensor_msgs/msg/Imu)
    Publishers:
    - odometry/truth (nav_msgs/msg/Odometry)
    """

    def __init__(self):
        super().__init__("truth_converter_node")

        self.declare_parameter("input_topic", "auv0/LocationSensor")
        self.declare_parameter("imu_topic", "auv0/DynamicsSensorIMU")
        self.declare_parameter("output_topic", "odometry/truth")
        self.declare_parameter("child_frame_id", "base_link")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        # Set up message filters for synchronization
        location_sub = message_filters.Subscriber(
            self, PoseWithCovarianceStamped, input_topic
        )
        imu_sub = message_filters.Subscriber(self, Imu, imu_topic)

        ts = message_filters.TimeSynchronizer([location_sub, imu_sub], 10)
        ts.registerCallback(self.sync_callback)

        self.get_logger().info(
            f"Location converter started. Listening on {input_topic} and {imu_topic} and publishing on {output_topic}."
        )

    def sync_callback(self, location_msg: PoseWithCovarianceStamped, imu_msg: Imu):
        
        odom_msg = Odometry()
        odom_msg.header = location_msg.header
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position = location_msg.pose.pose.position
        odom_msg.pose.pose.orientation = imu_msg.orientation
        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    truth_converter_node = TruthConvertorNode()
    try:
        rclpy.spin(truth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        truth_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
