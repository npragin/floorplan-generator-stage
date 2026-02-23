"""Publishes each robot's initial pose as a PoseWithCovarianceStamped."""

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class InitialPosePublisher(Node):
    """Reads spawn positions from YAML and publishes latched initial poses per robot."""

    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        self.declare_parameter("spawn_positions_yaml", "")

        yaml_path = self.get_parameter("spawn_positions_yaml").get_parameter_value().string_value

        if not yaml_path:
            self.get_logger().error("No spawn_positions_yaml provided")
            return

        with open(yaml_path) as f:
            config = yaml.safe_load(f)

        spawn_positions: list[dict[str, float]] = config.get("robots", [])
        if not spawn_positions:
            self.get_logger().warning("No robot spawn positions found in YAML")
            return

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        now = self.get_clock().now().to_msg()

        for spawn in spawn_positions:
            robot_id = int(spawn["robot_id"])
            topic = f"/robot_{robot_id}/initialpose"

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = now
            msg.pose.pose.position.x = float(spawn["x"])
            msg.pose.pose.position.y = float(spawn["y"])
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance = [0.0] * 36

            pub = self.create_publisher(PoseWithCovarianceStamped, topic, latched_qos)
            pub.publish(msg)

            self.get_logger().info(f"Published initial pose on {topic}: ({spawn['x']}, {spawn['y']})")


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
