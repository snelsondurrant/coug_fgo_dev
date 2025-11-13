import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math


class GpsConverterNode(Node):
    """
    Converts GPS data from HoloOcean to a NavSatFix message.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Nov 2025

    Subscribers:
    - auv0/GPSSensor (nav_msgs/Odometry)
    Publishers:
    - gps/fix (sensor_msgs/NavSatFix)
    """

    def __init__(self):
        super().__init__("gps_converter_node")

        self.declare_parameter("input_topic", "auv0/GPSSensor")
        self.declare_parameter("output_topic", "gps/fix")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("origin_latitude", 40.23890)
        self.declare_parameter("origin_longitude", -111.74212)
        self.declare_parameter("origin_altitude", 1412.0)
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.5)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.origin_lat = (
            self.get_parameter("origin_latitude").get_parameter_value().double_value
        )
        self.origin_lon = (
            self.get_parameter("origin_longitude").get_parameter_value().double_value
        )
        self.origin_alt = (
            self.get_parameter("origin_altitude").get_parameter_value().double_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.noise_sigma = (
            self.get_parameter("noise_sigma").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)

        self.get_logger().info(
            f"GPS converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: Odometry):
        
        navsat_msg = NavSatFix()
        navsat_msg.header = msg.header
        navsat_msg.header.frame_id = self.frame_id
        navsat_msg.status.status = navsat_msg.status.STATUS_FIX
        navsat_msg.status.service = navsat_msg.status.SERVICE_GPS

        d_east = msg.pose.pose.position.x
        d_north = msg.pose.pose.position.y

        lat, lon = self.calculate_inverse_haversine(
            self.origin_lat, self.origin_lon, d_north, d_east
        )

        navsat_msg.latitude = lat
        navsat_msg.longitude = lon
        navsat_msg.altitude = self.origin_alt + msg.pose.pose.position.z

        if self.override_covariance:
            covariance = self.noise_sigma * self.noise_sigma
            navsat_msg.position_covariance[0] = covariance  # East
            navsat_msg.position_covariance[4] = covariance  # North
            navsat_msg.position_covariance[8] = covariance  # Up
            navsat_msg.position_covariance_type = (
                navsat_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN
            )
        else:
            navsat_msg.position_covariance = msg.pose.covariance
            navsat_msg.position_covariance_type = navsat_msg.COVARIANCE_TYPE_UNKNOWN

        self.publisher.publish(navsat_msg)

    def calculate_inverse_haversine(self, ref_lat, ref_lon, d_north, d_east):
        
        # Earth radius in meters
        earth_radius = 6378137.0

        # Convert reference point to radians
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        # Calculate distance and bearing
        d = math.sqrt(d_north**2 + d_east**2)
        theta = math.atan2(d_east, d_north)

        # Calculate new latitude and longitude
        lat_rad = math.asin(
            math.sin(ref_lat_rad) * math.cos(d / earth_radius)
            + math.cos(ref_lat_rad) * math.sin(d / earth_radius) * math.cos(theta)
        )
        lon_rad = ref_lon_rad + math.atan2(
            math.sin(theta) * math.sin(d / earth_radius) * math.cos(ref_lat_rad),
            math.cos(d / earth_radius) - math.sin(ref_lat_rad) * math.sin(lat_rad),
        )

        # Convert back to degrees
        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)

        return lat, lon


def main(args=None):
    rclpy.init(args=args)
    gps_converter_node = GpsConverterNode()
    try:
        rclpy.spin(gps_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
