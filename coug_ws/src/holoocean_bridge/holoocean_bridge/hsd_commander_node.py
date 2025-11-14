import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from holoocean_interfaces.msg import DesiredCommand, AgentCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math


class HsdCommanderNode(Node):
    """
    High-Level Mission Commander for HoloOcean.

    IMPORTANT! This only works for the BlueROV2 when it starts at the origin (0,0,0) with no rotation.
    Honestly tho, the BlueROV2 PID controller doesn't work so well anyway, so I'd just drive it or use the CougUV.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Nov 2025

    Subscribers:
    - waypoints (geometry_msgs/PoseArray)
    - odometry/global (nav_msgs/Odometry)
    Publishers:
    - heading (holoocean_interfaces/DesiredCommand)
    - speed (holoocean_interfaces/DesiredCommand)
    - depth (holoocean_interfaces/DesiredCommand)
    - command/agent (holoocean_interfaces/AgentCommand)
    """

    def __init__(self):
        super().__init__("hsd_commander_node")

        self.declare_parameter("waypoint_topic", "waypoints")
        self.declare_parameter("current_odom_topic", "odometry/global")
        self.declare_parameter("heading_topic", "heading")
        self.declare_parameter("speed_topic", "speed")
        self.declare_parameter("depth_topic", "depth")
        self.declare_parameter("agent_command_topic", "command/agent")
        self.declare_parameter("agent_name", "auv0")
        self.declare_parameter("capture_radius", 25.0)  # meters
        self.declare_parameter(
            "slip_radius", 50.0
        )  # meters, typically 2x capture_radius
        self.declare_parameter("desired_speed_rpm", 1500.0)  # RPM
        self.declare_parameter("odom_timeout_sec", 2.0)  # seconds

        waypoint_topic = (
            self.get_parameter("waypoint_topic").get_parameter_value().string_value
        )
        odom_topic = (
            self.get_parameter("current_odom_topic").get_parameter_value().string_value
        )
        heading_topic = (
            self.get_parameter("heading_topic").get_parameter_value().string_value
        )
        speed_topic = (
            self.get_parameter("speed_topic").get_parameter_value().string_value
        )
        depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        agent_command_topic = (
            self.get_parameter("agent_command_topic").get_parameter_value().string_value
        )
        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )
        self.capture_radius = (
            self.get_parameter("capture_radius").get_parameter_value().double_value
        )
        self.slip_radius = (
            self.get_parameter("slip_radius").get_parameter_value().double_value
        )
        self.desired_speed = (
            self.get_parameter("desired_speed_rpm").get_parameter_value().double_value
        )
        self.odom_timeout_sec = (
            self.get_parameter("odom_timeout_sec").get_parameter_value().double_value
        )

        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_active = False
        self.current_pose: Pose = None
        self.previous_distance = -1.0
        self.last_odom_time = None
        self.odom_timed_out = False
        self.published_for_waypoint = False

        self.heading_pub = self.create_publisher(DesiredCommand, heading_topic, 10)
        self.speed_pub = self.create_publisher(DesiredCommand, speed_topic, 10)
        self.depth_pub = self.create_publisher(DesiredCommand, depth_topic, 10)
        self.agent_command_pub = self.create_publisher(AgentCommand, agent_command_topic, 10)

        self.waypoint_sub = self.create_subscription(
            PoseArray, waypoint_topic, self.waypoint_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self.timeout_check_timer = self.create_timer(1.0, self.check_odom_timeout)

        self.get_logger().info(
            f"HSD Commander started. Listening for waypoints on '{waypoint_topic}'."
        )

    def check_odom_timeout(self):

        if not self.mission_active or self.last_odom_time is None:
            return

        time_since_last_odom = (
            self.get_clock().now() - self.last_odom_time
        ).nanoseconds / 1e9

        if time_since_last_odom > self.odom_timeout_sec:
            self.get_logger().warn(
                f"Odometry timeout! No message received for {time_since_last_odom:.1f}s. Pausing mission."
            )
            # Pause the mission; do not discard it.
            self.mission_active = False
            self.odom_timed_out = True
            self.publish_fossen_commands(heading=None, speed=0.0, depth=None)

    def waypoint_callback(self, msg: PoseArray):

        if not msg.poses:
            self.get_logger().warn("Received an empty waypoint list. Mission stopped.")
            self.mission_active = False
            self.waypoints = []
            self.publish_fossen_commands(heading=None, speed=0.0, depth=None)
            return

        self.waypoints = msg.poses
        self.current_waypoint_index = 0
        self.mission_active = True
        self.previous_distance = -1.0
        self.published_for_waypoint = False
        self.get_logger().info(
            f"Received new mission with {len(self.waypoints)} waypoints. Mission started."
        )

    def odom_callback(self, msg: Odometry):

        self.last_odom_time = self.get_clock().now()
        if self.odom_timed_out:
            self.get_logger().info("Odometry signal recovered.")
            self.odom_timed_out = False
            # Resume the mission if there are waypoints left to complete.
            if self.waypoints and self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(f"Resuming mission from waypoint {self.current_waypoint_index + 1}.")
                self.mission_active = True

        self.current_pose = msg.pose.pose

        if not self.mission_active:
            return

        # Check if the last waypoint has been completed.
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Mission complete! Holding position.")
            self.mission_active = False
            self.publish_fossen_commands(heading=None, speed=0.0, depth=None)
            return

        target_waypoint = self.waypoints[self.current_waypoint_index]

        # Calculate horizontal distance to the target waypoint.
        dx = target_waypoint.position.x - self.current_pose.position.x
        dy = target_waypoint.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # 1. Primary check: Has the AUV entered the target's capture radius?
        if distance < self.capture_radius:
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1} reached by capture radius."
            )
            self.current_waypoint_index += 1
            self.previous_distance = -1.0
            self.published_for_waypoint = False
            return

        # 2. Secondary check: Has the AUV overshot the target (slipped past)?
        # This triggers if the AUV is now moving away from the target.
        if (
            self.previous_distance != -1.0
            and distance > self.previous_distance
            and distance < self.slip_radius
        ):
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1} reached by slip radius."
            )
            self.current_waypoint_index += 1
            self.previous_distance = -1.0
            self.published_for_waypoint = False
            return

        self.previous_distance = distance

        # Calculate desired heading from atan2, then convert to HoloOcean's NED frame.
        desired_heading_rad = math.atan2(dx, dy)
        desired_heading_deg = math.degrees(desired_heading_rad)
        angle_in_360 = (450 - desired_heading_deg) % 360
        desired_heading_ned = (angle_in_360 + 180) % 360 - 180
        desired_depth = target_waypoint.position.z

        self.publish_fossen_commands(
            heading=desired_heading_ned, speed=self.desired_speed, depth=desired_depth
        )

        # NOTE! This is kind of hacky, and the built-in PID controller isn't very good.
        # This should also change in the future to make sure the waypoint completes even with estimate drift.
        if not self.published_for_waypoint:
            self.publish_agent_commands(
                x=target_waypoint.position.x, y=target_waypoint.position.y, z=-target_waypoint.position.z
            )
            self.published_for_waypoint = True

    def publish_fossen_commands(self, heading, speed, depth):

        common_header = Header()
        common_header.stamp = self.get_clock().now().to_msg()
        common_header.frame_id = self.agent_name

        if heading is not None:
            heading_msg = DesiredCommand(header=common_header, data=float(heading))
            self.heading_pub.publish(heading_msg)

        if speed is not None:
            speed_msg = DesiredCommand(header=common_header, data=float(speed))
            self.speed_pub.publish(speed_msg)

        if depth is not None:
            depth_msg = DesiredCommand(header=common_header, data=float(depth))
            self.depth_pub.publish(depth_msg)

    def publish_agent_commands(self, x, y, z):

        # NOTE! I don't really know how to stop this once started...
        agent_cmd_msg = AgentCommand()
        agent_cmd_msg.header = Header()
        agent_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        agent_cmd_msg.header.frame_id = self.agent_name
        agent_cmd_msg.command = [
            float(x),
            float(y),
            float(z),
            0.0,
            0.0,
            0.0,
        ]
        self.agent_command_pub.publish(agent_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    hsd_commander_node = HsdCommanderNode()
    try:
        rclpy.spin(hsd_commander_node)
    except KeyboardInterrupt:
        pass
    finally:
        hsd_commander_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
