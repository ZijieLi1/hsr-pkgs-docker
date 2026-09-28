#!/usr/bin/env python3
"""Publish HSR head RGB-D camera pose as geometry_msgs/PoseStamped for
concept-graphs RosRgbDPoseSource (BEST_EFFORT, keep-last 1, T_wc optical).

TF map → camera_link includes AMCL plus neck joints. camera_link is x-forward;
this node converts to optical axes (x right, y down, z forward) unless --child
already contains "optical".

    python3 /ws/scripts/report_camera_pose.py
    # conceptgraph hydra: ros_pose_topic:=/hsr/cam_pose
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# camera_link (x fwd, y left, z up) → optical (x right, y down, z fwd).
_R_LINK_FROM_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def R_to_quat(R: np.ndarray):
    t = float(np.trace(R))
    if t > 0.0:
        s = 0.5 / math.sqrt(t + 1.0)
        return (
            (R[2, 1] - R[1, 2]) * s,
            (R[0, 2] - R[2, 0]) * s,
            (R[1, 0] - R[0, 1]) * s,
            0.25 / s,
        )
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return (
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[2, 1] - R[1, 2]) / s,
        )
    if R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return (
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
            (R[0, 2] - R[2, 0]) / s,
        )
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return (
        (R[0, 2] + R[2, 0]) / s,
        (R[1, 2] + R[2, 1]) / s,
        0.25 * s,
        (R[1, 0] - R[0, 1]) / s,
    )


def quat_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny_cosp, cosy_cosp)


def tf_to_optical_pose(tf, child: str):
    """Return (xyz, quat_xyzw) of T_wc in optical axes."""
    t = tf.transform.translation
    q = tf.transform.rotation
    if "optical" in child:
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = (t.x, t.y, t.z)
    T_link_opt = np.eye(4)
    T_link_opt[:3, :3] = _R_LINK_FROM_OPTICAL
    T_wc = T @ T_link_opt
    return tuple(T_wc[:3, 3].tolist()), R_to_quat(T_wc[:3, :3])


class CameraPoseReporter(Node):
    def __init__(self, parent: str, child: str, topic: str, rate_hz: float):
        super().__init__("camera_pose_report")
        self.parent = parent
        self.child = child
        self.buf = Buffer()
        self._tf_listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PoseStamped, topic, sensor_qos())
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"TF {parent} → {child} → PoseStamped {topic} at {rate_hz:.1f} Hz"
        )

    def _tick(self) -> None:
        try:
            tf = self.buf.lookup_transform(
                self.parent, self.child, Time(), timeout=Duration(seconds=0.2)
            )
        except TransformException as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return
        xyz, quat = tf_to_optical_pose(tf, self.child)
        msg = PoseStamped()
        msg.header = tf.header
        msg.header.frame_id = self.parent
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pub.publish(msg)
        roll, pitch, yaw = quat_to_rpy(*quat)
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        print(
            f"{stamp:12.3f}  xyz {xyz[0]:7.3f} {xyz[1]:7.3f} {xyz[2]:7.3f}  "
            f"rpy {roll:7.3f} {pitch:7.3f} {yaw:7.3f}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parent", default="map")
    parser.add_argument(
        "--child",
        default="head_rgbd_sensor_rgb_frame",
        help="camera_link; use *optical* in the name to skip the axes conversion",
    )
    parser.add_argument(
        "--topic",
        default="/hsr/cam_pose",
        help="PoseStamped topic RosRgbDPoseSource subscribes to",
    )
    parser.add_argument("--rate", type=float, default=20.0)
    args, _unknown = parser.parse_known_args()

    rclpy.init()
    node = CameraPoseReporter(args.parent, args.child, args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
