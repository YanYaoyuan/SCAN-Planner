#!/usr/bin/env python3
"""Republish PointCloud2 with an equivalent frame name for local RViz."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudFrameAlias(Node):
    def __init__(self):
        super().__init__("rviz_pointcloud_frame_alias")

        input_topic = self.declare_parameter("input_topic", "/lidar_points").value
        output_topic = self.declare_parameter(
            "output_topic", "/lidar_points_rviz"
        ).value
        output_frame = self.declare_parameter("output_frame", "lidar_frame").value

        for name, value in (
            ("input_topic", input_topic),
            ("output_topic", output_topic),
            ("output_frame", output_frame),
        ):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._output_frame = output_frame
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, output_qos
        )
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._point_cloud_callback, input_qos
        )

        self.get_logger().info(
            f"RViz point cloud alias: {input_topic} -> {output_topic}, "
            f"frame={output_frame}"
        )

    def _point_cloud_callback(self, message):
        # The dog3 visualization profile treats these names as equivalent.
        # This changes only the RViz copy and never feeds planning.
        message.header.frame_id = self._output_frame
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameAlias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
