"""Publishes the generated floorplan PNG as a nav_msgs/OccupancyGrid."""

from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class GroundTruthMapPublisher(Node):
    """Reads a floorplan PNG and publishes it as a latched OccupancyGrid."""

    def __init__(self) -> None:
        super().__init__("ground_truth_map_publisher")

        self.declare_parameter("image_path", "")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("map_width", 0.0)
        self.declare_parameter("map_height", 0.0)
        self.declare_parameter("interior_x", 0.0)
        self.declare_parameter("interior_y", 0.0)

        image_path = self.get_parameter("image_path").get_parameter_value().string_value
        resolution = self.get_parameter("resolution").get_parameter_value().double_value
        map_width = self.get_parameter("map_width").get_parameter_value().double_value
        map_height = self.get_parameter("map_height").get_parameter_value().double_value
        interior_x = self.get_parameter("interior_x").get_parameter_value().double_value
        interior_y = self.get_parameter("interior_y").get_parameter_value().double_value

        if not image_path:
            self.get_logger().error("No image_path provided")
            return

        self._msg = self._build_occupancy_grid(image_path, resolution, map_width, map_height, interior_x, interior_y)

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(OccupancyGrid, "/ground_truth_map", latched_qos)

        self._publish()

        self.get_logger().info(
            f"Publishing ground truth map ({self._msg.info.width}x{self._msg.info.height}, resolution={resolution})"
        )

    def _publish(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)

    def _build_occupancy_grid(
        self,
        image_path: str,
        resolution: float,
        map_width: float,
        map_height: float,
        interior_x: float,
        interior_y: float,
    ) -> OccupancyGrid:
        image = Image.open(image_path).convert("L")

        grid_width = round(map_width / resolution)
        grid_height = round(map_height / resolution)
        image = image.resize((grid_width, grid_height), Image.Resampling.NEAREST)

        # PNG: 0=occupied (black), 255=free (white)
        # OccupancyGrid: 0=free, 100=occupied
        pixels = np.array(image, dtype=np.float64)
        # Flip vertically: image origin is top-left, OccupancyGrid origin is bottom-left
        pixels = np.flipud(pixels)
        occupancy = (100.0 * (1.0 - pixels / 255.0)).astype(np.int8)

        occupancy = self._mark_exterior_unseen(occupancy, interior_x, interior_y, resolution, map_width, map_height)

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width = grid_width
        msg.info.height = grid_height
        msg.info.origin.position.x = -map_width / 2.0
        msg.info.origin.position.y = -map_height / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy.flatten().tolist()

        return msg

    def _mark_exterior_unseen(
        self,
        occupancy: np.ndarray[tuple[int, int], np.dtype[np.int8]],
        interior_x: float,
        interior_y: float,
        resolution: float,
        map_width: float,
        map_height: float,
    ) -> np.ndarray[tuple[int, int], np.dtype[np.int8]]:
        """Mark free cells outside the floorplan interior as unseen (-1) via BFS."""
        rows, cols = occupancy.shape
        origin_x = -map_width / 2.0
        origin_y = -map_height / 2.0

        # Convert world coordinates to grid cell indices
        seed_col = int((interior_x - origin_x) / resolution)
        seed_row = int((interior_y - origin_y) / resolution)
        seed_col = max(0, min(cols - 1, seed_col))
        seed_row = max(0, min(rows - 1, seed_row))

        if occupancy[seed_row, seed_col] != 0:
            self.get_logger().warn(
                f"Interior seed ({interior_x}, {interior_y}) maps to an occupied cell; skipping exterior marking"
            )
            return occupancy

        # BFS flood-fill on free cells (value 0) from the interior seed point
        reachable = np.zeros_like(occupancy, dtype=bool)
        reachable[seed_row, seed_col] = True
        queue: deque[tuple[int, int]] = deque([(seed_row, seed_col)])

        while queue:
            r, c = queue.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and not reachable[nr, nc] and occupancy[nr, nc] == 0:
                    reachable[nr, nc] = True
                    queue.append((nr, nc))

        # Mark unreachable free cells as unseen; occupied cells stay as-is
        exterior_free = ~reachable & (occupancy == 0)
        occupancy[exterior_free] = -1
        return occupancy


def main() -> None:
    rclpy.init()
    node = GroundTruthMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
