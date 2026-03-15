#!/usr/bin/env python3
import os, signal, argparse, time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


def write_ply_ascii(path, xyz, intensity=None):
    n = xyz.shape[0]
    has_i = intensity is not None
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if has_i:
            f.write("property float intensity\n")
        f.write("end_header\n")
        if has_i:
            for (x, y, z), i in zip(xyz, intensity):
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {float(i):.6f}\n")
        else:
            for x, y, z in xyz:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def write_pcd_ascii(path, xyz, intensity=None):
    n = xyz.shape[0]
    has_i = intensity is not None
    fields = "x y z intensity" if has_i else "x y z"
    size = "4 4 4 4" if has_i else "4 4 4"
    type_ = "F F F F" if has_i else "F F F"
    count = "1 1 1 1" if has_i else "1 1 1"
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write(f"FIELDS {fields}\n")
        f.write(f"SIZE {size}\n")
        f.write(f"TYPE {type_}\n")
        f.write(f"COUNT {count}\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        if has_i:
            for (x, y, z), i in zip(xyz, intensity):
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {float(i):.6f}\n")
        else:
            for x, y, z in xyz:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


class ScanToCloudSaver(Node):
    def __init__(
        self,
        scan_topic="/scan",
        height_m=0.40,
        duration_s=600.0,
        out_prefix="~/cloud/lidar_cloud",
        scan_skip=1,
        point_skip=1,
        use_intensity=False,
        countdown=0.0,
        flip_x=False,
        flip_y=False,
    ):
        super().__init__("scan_to_cloud_saver")

        self.scan_topic = scan_topic
        self.height_m = float(height_m)
        self.duration_s = float(duration_s)
        self.out_prefix = os.path.expanduser(out_prefix)
        self.scan_skip = max(1, int(scan_skip))
        self.point_skip = max(1, int(point_skip))
        self.use_intensity = bool(use_intensity)
        self.countdown = float(countdown)
        self.flip_x = bool(flip_x)
        self.flip_y = bool(flip_y)

        self.start_wall = time.time()
        self.start_time = None  # ROS time for alpha=0
        self.frame_count = 0
        self.total_points = 0
        self.finished = False

        self.xyz_chunks = []
        self.int_chunks = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, qos)

        self.get_logger().info(
            f"Subscribing {self.scan_topic} | height_m={self.height_m}, duration_s={self.duration_s}, "
            f"scan_skip={self.scan_skip}, point_skip={self.point_skip}, countdown={self.countdown}s, "
            f"flip_x={self.flip_x}, flip_y={self.flip_y}"
        )

    def cb_scan(self, msg: LaserScan):
        if self.finished:
            return

        # countdown 동안은 수집 안 함
        if self.countdown > 0 and (time.time() - self.start_wall) < self.countdown:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        self.frame_count += 1
        if (self.frame_count - 1) % self.scan_skip != 0:
            return

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        alpha = min(max(t / self.duration_s, 0.0), 1.0)
        z = alpha * self.height_m

        ranges = np.array(msg.ranges, dtype=np.float32)
        n = ranges.shape[0]
        if n == 0:
            return

        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        valid = np.isfinite(ranges)
        if msg.range_min > 0:
            valid &= (ranges >= msg.range_min)
        if msg.range_max > 0:
            valid &= (ranges <= msg.range_max)

        idx = np.nonzero(valid)[0]
        if idx.size == 0:
            if alpha >= 1.0:
                self.save_and_finish()
            return

        idx = idx[:: self.point_skip]

        r = ranges[idx].astype(np.float32)
        a = angles[idx].astype(np.float32)

        x = r * np.cos(a)
        y = r * np.sin(a)

        # ===== 여기서 미러(좌우 반전) 처리 =====
        if self.flip_x:
            x = -x
        if self.flip_y:
            y = -y
        # =====================================

        z_arr = np.full_like(x, z, dtype=np.float32)
        xyz = np.stack([x, y, z_arr], axis=1)

        self.xyz_chunks.append(xyz)
        self.total_points += xyz.shape[0]

        if self.use_intensity and hasattr(msg, "intensities") and msg.intensities:
            intens = np.array(msg.intensities, dtype=np.float32)
            if intens.shape[0] == n:
                self.int_chunks.append(intens[idx])

        self.get_logger().info(f"alpha={alpha:.3f}, z={z:.3f}m, points={self.total_points}")

        if alpha >= 1.0:
            self.save_and_finish()

    def save_and_finish(self):
        if self.finished:
            return
        self.finished = True

        if self.total_points == 0:
            self.get_logger().error("No points collected. Nothing to save.")
            rclpy.shutdown()
            return

        xyz = np.concatenate(self.xyz_chunks, axis=0)

        intensity = None
        if self.use_intensity and len(self.int_chunks) == len(self.xyz_chunks) and len(self.int_chunks) > 0:
            intensity = np.concatenate(self.int_chunks, axis=0)
            if intensity.shape[0] != xyz.shape[0]:
                intensity = None

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        ply_path = f"{self.out_prefix}_{ts}.ply"
        pcd_path = f"{self.out_prefix}_{ts}.pcd"

        os.makedirs(os.path.dirname(ply_path) or ".", exist_ok=True)

        write_ply_ascii(ply_path, xyz, intensity=intensity)
        write_pcd_ascii(pcd_path, xyz, intensity=intensity)

        self.get_logger().info(f"Saved PLY: {ply_path}")
        self.get_logger().info(f"Saved PCD: {pcd_path}")
        self.get_logger().info("Done.")
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan_topic", default="/scan")
    ap.add_argument("--height_m", type=float, default=0.40)
    ap.add_argument("--duration_s", type=float, default=600.0)
    ap.add_argument("--out_prefix", default=os.path.expanduser("~/cloud/lidar_cloud"))
    ap.add_argument("--scan_skip", type=int, default=1)
    ap.add_argument("--point_skip", type=int, default=1)
    ap.add_argument("--intensity", action="store_true")
    ap.add_argument("--countdown", type=float, default=0.0, help="ignore scans for N seconds after start (for sync)")
    ap.add_argument("--flip_x", action="store_true", help="mirror: x -> -x")
    ap.add_argument("--flip_y", action="store_true", help="mirror: y -> -y")
    args = ap.parse_args()

    rclpy.init()
    node = ScanToCloudSaver(
        scan_topic=args.scan_topic,
        height_m=args.height_m,
        duration_s=args.duration_s,
        out_prefix=os.path.expanduser(args.out_prefix),
        scan_skip=args.scan_skip,
        point_skip=args.point_skip,
        use_intensity=args.intensity,
        countdown=args.countdown,
        flip_x=args.flip_x,
        flip_y=args.flip_y,
    )

    def sigint_handler(sig, frame):
        # 이미 저장/종료 중이면 무시
        if getattr(node, "finished", False):
            return
        node.save_and_finish()

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        sigint_handler(None, None)

    node.destroy_node()


if __name__ == "__main__":
    main()
