#!/usr/bin/env python3
import argparse
import os
import signal
import threading
import time
from datetime import datetime

import numpy as np
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def write_ply_ascii(path, xyz):
    n = xyz.shape[0]
    with open(path, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for p in xyz:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


def write_pcd_ascii(path, xyz):
    n = xyz.shape[0]
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for p in xyz:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


class LayerCapture(Node):
    def __init__(self, scan_topic, height_m, segments, capture_s, settle_s,
                 out_prefix, scan_skip, point_skip, serial_port, baud):
        super().__init__("scan_layer_capture")

        self.height_m = float(height_m)
        self.segments = int(segments)
        self.capture_s = float(capture_s)
        self.settle_s = float(settle_s)

        self.out_prefix = out_prefix
        self.scan_skip = max(1, int(scan_skip))
        self.point_skip = max(1, int(point_skip))

        # /scan QoS: BEST_EFFORT
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)

        # 누적
        self.xyz_chunks = []
        self.total_points = 0
        self.scan_count = 0

        # 캡처 상태(REST에서만)
        self._lock = threading.Lock()
        self.capturing = False
        self.capture_until = 0.0
        self.current_z = 0.0
        self.last_rest_time = 0.0
        self.last_seg = 0

        # 시리얼 리더
        self.ser = serial.Serial(serial_port, baud, timeout=0.1)
        self.get_logger().info(f"Serial: {serial_port} @ {baud}")

        t = threading.Thread(target=self.serial_loop, daemon=True)
        t.start()

        self.get_logger().info(f"Subscribe: {scan_topic}")
        self.get_logger().info(f"Mode: capture ONLY during REST (capture_s={self.capture_s}s, settle_s={self.settle_s}s)")
        self.get_logger().info(f"segments={self.segments}, height={self.height_m}m -> dz≈{(self.height_m/max(1,self.segments)):.4f}m")
        self.get_logger().info("아두이노에서 'REST <seg>'가 오면 그 층에서 캡처를 시작합니다.")
        self.get_logger().info("Ctrl+C 하면 지금까지 저장하고 종료합니다.")

    def z_from_seg(self, seg: int) -> float:
        # seg: 1..segments (REST 1이면 첫 층)
        # 층 위치를 0~height로 균등 배치 (필요하면 seg-1 기준으로 바꿔도 됨)
        # 여기서는 REST 1을 z=0, REST segments를 z=height로 두고 싶으면:
        if self.segments <= 1:
            return 0.0
        a = (seg - 1) / (self.segments - 1)
        return self.height_m * a

    def start_capture(self, seg: int):
        now = time.time()
        z = self.z_from_seg(seg)
        with self._lock:
            self.capturing = True
            self.current_z = z
            self.last_rest_time = now
            self.capture_until = now + self.settle_s + self.capture_s
            self.last_seg = seg
        self.get_logger().info(f"REST {seg} -> z={z:.3f}m (settle {self.settle_s}s then capture {self.capture_s}s)")

    def serial_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue

            # 기대: "REST 3"
            if line.startswith("REST"):
                parts = line.split()
                if len(parts) >= 2 and parts[1].isdigit():
                    seg = int(parts[1])
                    self.start_capture(seg)
            elif line == "DONE":
                self.get_logger().info("Arduino DONE received -> saving...")
                self.save_and_exit()

    def on_scan(self, msg: LaserScan):
        self.scan_count += 1
        if (self.scan_count % self.scan_skip) != 0:
            return

        with self._lock:
            if not self.capturing:
                return
            now = time.time()
            # settle 시간 동안은 버림
            if now < (self.last_rest_time + self.settle_s):
                return
            if now > self.capture_until:
                self.capturing = False
                return
            z = self.current_z

        ranges = np.array(msg.ranges, dtype=np.float32)
        n = ranges.shape[0]
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        idx = np.where(valid)[0]
        if idx.size == 0:
            return

        idx = idx[::self.point_skip]
        r = ranges[idx]
        a = angles[idx]

        x = r * np.cos(a)
        y = r * np.sin(a)
        zz = np.full_like(x, z)

        xyz = np.stack([x, y, zz], axis=1).astype(np.float32)
        self.xyz_chunks.append(xyz)
        self.total_points += xyz.shape[0]

        if (self.scan_count % (self.scan_skip * 30)) == 0:
            self.get_logger().info(f"seg={self.last_seg}, z={z:.3f}m, points={self.total_points}")

    def save_and_exit(self):
        if self.total_points == 0:
            self.get_logger().error("No points collected. Nothing to save.")
            rclpy.shutdown()
            return

        xyz = np.concatenate(self.xyz_chunks, axis=0)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        ply_path = f"{self.out_prefix}_{ts}.ply"
        pcd_path = f"{self.out_prefix}_{ts}.pcd"
        os.makedirs(os.path.dirname(ply_path) or ".", exist_ok=True)

        write_ply_ascii(ply_path, xyz)
        write_pcd_ascii(pcd_path, xyz)

        self.get_logger().info(f"Saved PLY: {ply_path}")
        self.get_logger().info(f"Saved PCD: {pcd_path}")
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan_topic", default="/scan")
    ap.add_argument("--height_m", type=float, default=0.40)
    ap.add_argument("--segments", type=int, default=80)         # ✅ 층 개수
    ap.add_argument("--capture_s", type=float, default=2.0)     # ✅ REST에서 몇 초 캡처할지
    ap.add_argument("--settle_s", type=float, default=0.2)      # ✅ 멈춘 직후 진동 제거 시간
    ap.add_argument("--out_prefix", default=os.path.expanduser("~/cloud/lidar_layer"))
    ap.add_argument("--scan_skip", type=int, default=2)
    ap.add_argument("--point_skip", type=int, default=2)
    ap.add_argument("--serial_port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    rclpy.init()
    node = LayerCapture(
        scan_topic=args.scan_topic,
        height_m=args.height_m,
        segments=args.segments,
        capture_s=args.capture_s,
        settle_s=args.settle_s,
        out_prefix=os.path.expanduser(args.out_prefix),
        scan_skip=args.scan_skip,
        point_skip=args.point_skip,
        serial_port=args.serial_port,
        baud=args.baud,
    )

    def sigint_handler(sig, frame):
        node.get_logger().info("SIGINT -> saving...")
        node.save_and_exit()

    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
