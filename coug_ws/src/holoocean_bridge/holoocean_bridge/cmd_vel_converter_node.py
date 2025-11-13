import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from holoocean_interfaces.msg import ControlCommand, AgentCommand
import math


class CmdVelConverterNode(Node):
    """
    Converts ROS 2 cmd_vel commands to both HoloOcean ControlCommand and AgentCommand messages.

    Modified in part from Braden's HoloOcean-ROS example code.

    TODO: Adjust scalars so the velocities actually kind of match.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Nov 2025

    Subscribers:
    - cmd_vel (geometry_msgs/Twist)
    Publishers:
    - command/control (holoocean_interfaces/ControlCommand)
    - command/agent (holoocean_interfaces/AgentCommand)
    """

    def __init__(self):
        super().__init__("cmd_vel_converter_node")

        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "command/control")
        self.declare_parameter("agent_command_topic", "command/agent")
        self.declare_parameter("agent_name", "auv0")
        self.declare_parameter("max_fin_angle_deg", 45.0)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        agent_command_topic = (
            self.get_parameter("agent_command_topic").get_parameter_value().string_value
        )
        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )
        self.max_fin_angle = math.radians(
            self.get_parameter("max_fin_angle_deg").get_parameter_value().double_value
        )

        self.last_command_msg = None

        self.subscription = self.create_subscription(
            Twist, input_topic, self.cmd_vel_callback, 10
        )
        self.publisher = self.create_publisher(ControlCommand, output_topic, 10)
        self.agent_command_publisher = self.create_publisher(
            AgentCommand, agent_command_topic, 10
        )
        self.get_logger().info(
            f"Cmd_vel converter started. Listening on {input_topic} and publishing on {output_topic} and {agent_command_topic}."
        )

    def cmd_vel_callback(self, msg: Twist):

        current_time = self.get_clock().now().to_msg()

        # --- 1. Publish ControlCommand (Fins + Thruster) ---
        control_cmd = ControlCommand()
        control_cmd.header.stamp = current_time
        control_cmd.header.frame_id = self.agent_name

        linear_x = msg.linear.x * 2000
        if abs(msg.linear.z * 2000) > 0.01:
            if linear_x < abs(msg.linear.z * 2000):
                linear_x = abs(msg.linear.z * 2000)
        thruster_rpm = linear_x

        rudder_angle = msg.angular.z * 2000

        # Mix linear.z (heave) and angular.y (pitch) for the elevators
        elevator_angle_heave = msg.linear.z * 2000
        elevator_angle_pitch = msg.angular.y * 2000
        total_elevator_angle = elevator_angle_heave + elevator_angle_pitch

        # Clamp the angles to the max fin angle
        total_elevator_angle = max(
            min(total_elevator_angle, self.max_fin_angle), -self.max_fin_angle
        )
        rudder_angle = max(min(rudder_angle, self.max_fin_angle), -self.max_fin_angle)

        # [rudder, starboard_elevator, port_elevator, thruster]
        control_cmd.cs = [
            rudder_angle,
            total_elevator_angle,
            -total_elevator_angle,
            thruster_rpm,
        ]

        self.last_command_msg = control_cmd
        self.publisher.publish(self.last_command_msg)

        # --- 2. Publish AgentCommand (BlueROV Direct Thruster Control) ---
        agent_cmd = AgentCommand()
        agent_cmd.header.stamp = current_time
        agent_cmd.header.frame_id = self.agent_name

        # Map Twist 6-DOF to BlueROV variables
        forward = msg.linear.x * 10
        vertical = msg.linear.z * 10
        yaw = msg.angular.z * 0.1
        left = msg.linear.y * 10
        pitch = msg.angular.y * 0.1
        roll = msg.angular.x * 0.1

        agent_cmd.command = [
            (vertical + pitch + roll),
            (vertical + pitch - roll),
            (vertical - pitch - roll),
            (vertical - pitch + roll),
            (forward + yaw + left),
            (forward - yaw - left),
            (forward - yaw + left),
            (forward + yaw - left),
        ]

        self.agent_command_publisher.publish(agent_cmd)


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_converter = CmdVelConverterNode()
    try:
        rclpy.spin(cmd_vel_converter)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
