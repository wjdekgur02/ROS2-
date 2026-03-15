#!/usr/bin/env python3
import argparse
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def voxel_downsample(xyz: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0:
        return xyz
    q = np.floor(xyz / voxel).astype(np.int32)
    q_view = np.ascontiguousarray(q).view(np.dtype((np.void, q.dtype.itemsize * q.shape[1])))
    _, idx = np.unique(q_view, return_index=True)
    return xyz[idx]


def save_ply_xyz_ascii(path: str, xyz: np.ndarray) -> None:
    xyz = np.asarray(xyz, dtype=np.float32)
    with open(path, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {xyz.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        np.savetxt(f, xyz, fmt="%.6f %.6f %.6f")


class PointsToPly(Node):
    def __init__(self, topic: str, duration_s: float, out_prefix: str,
                 voxel: float, target_points: int, max_points: int, no_msg_timeout_s: float):
        super().__init__("ouster_points_to_ply")

        self.duration_s = float(duration_s)
        self.out_prefix = os.path.expanduser(out_prefix)
        self.voxel = float(voxel)
        self.target_points = int(target_points)
        self.max_points = int(max_points)
        self.no_msg_timeout_s = float(no_msg_timeout_s)

        self.topic = topic if topic else self.auto_detect_points_topic()
        if not self.topic:
            self.get_logger().error("No PointCloud2 points topic found. Run: ros2 topic list | grep points")
            raise RuntimeError("No points topic")

        self.buf = []
        self.first_msg_t = None
        self.start_wall = time.time()

        # ✅ 핵심: Ouster points는 보통 BEST_EFFORT라서 sensor_data QoS로 구독해야 함
        self.sub = self.create_subscription(PointCloud2, self.topic, self.cb, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(f"Using topic: {self.topic}")
        self.get_logger().info(
            f"Duration(after first msg): {self.duration_s:.3f}s, voxel={self.voxel}, "
            f"target_points={self.target_points}, max_points={self.max_points}"
        )

    def auto_detect_points_topic(self) -> str:
        time.sleep(0.2)
        topics = self.get_topic_names_and_types()
        candidates = []
        for name, types in topics:
            if "sensor_msgs/msg/PointCloud2" in types and ("points" in name):
                candidates.append(name)
        if not candidates:
            return ""
        preferred = ["/ouster/points", "/os_driver/points"]
        for p in preferred:
            if p in candidates:
                return p
        candidates.sort(key=lambda s: (len(s), s))
        return candidates[0]

    def cb(self, msg: PointCloud2):
        if self.first_msg_t is None:
            self.first_msg_t = time.time()

        # ✅ 여기 수정 포인트:
        # read_points 결과가 "structured dtype"로 잡힐 수 있어서 x,y,z만 (N,3) float로 재구성
        raw = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )))
        if raw.size == 0:
            return

        if raw.dtype.names is not None:
            # structured array: raw['x'], raw['y'], raw['z']
            pts = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32, copy=False)
        else:
            # normal (N,3) array
            pts = np.asarray(raw, dtype=np.float32)

        self.buf.append(pts)

        if self.max_points > 0:
            total = sum(x.shape[0] for x in self.buf)
            if total > self.max_points:
                xyz = np.vstack(self.buf)
                idx = np.random.choice(xyz.shape[0], self.max_points, replace=False)
                self.buf = [xyz[idx]]

    def on_timer(self):
        now = time.time()

        if self.first_msg_t is None:
            if (now - self.start_wall) > self.no_msg_timeout_s:
                self.get_logger().error("No points received (timeout). Check topic + sensor running.")
                rclpy.shutdown()
            return

        if (now - self.first_msg_t) < self.duration_s:
            return

        xyz = np.vstack(self.buf) if self.buf else np.zeros((0, 3), dtype=np.float32)
        if xyz.shape[0] == 0:
            self.get_logger().error("Buffer empty even after receiving messages.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Collected raw points: {xyz.shape[0]}")

        if self.voxel > 0:
            xyz = voxel_downsample(xyz, self.voxel)
            self.get_logger().info(f"After voxel downsample: {xyz.shape[0]}")

        if self.target_points > 0:
            if xyz.shape[0] < self.target_points:
                self.get_logger().warn(
                    f"Not enough points ({xyz.shape[0]}) < target_points ({self.target_points}). "
                    f"Increase duration_s or lower target_points."
                )
            else:
                idx = np.random.choice(xyz.shape[0], self.target_points, replace=False)
                xyz = xyz[idx]
                self.get_logger().info(f"After target_points sampling: {xyz.shape[0]}")

        os.makedirs(os.path.dirname(self.out_prefix), exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = f"{self.out_prefix}_{ts}.ply"
        save_ply_xyz_ascii(out_path, xyz)
        self.get_logger().info(f"Saved PLY: {out_path}")
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="", help="PointCloud2 points topic (비우면 자동 탐색)")
    ap.add_argument("--duration_s", type=float, default=0.20, help="첫 메시지 받은 뒤 모을 시간(초)")
    ap.add_argument("--out_prefix", default="~/cloud/ouster_samepoints", help="출력 prefix")
    ap.add_argument("--voxel", type=float, default=0.0, help="0이면 다운샘플 끔 (예: 0.01)")
    ap.add_argument("--target_points", type=int, default=346335, help="고정2D와 점수 맞추기")
    ap.add_argument("--max_points", type=int, default=3000000)
    ap.add_argument("--no_msg_timeout_s", type=float, default=5.0, help="메시지 0개면 종료(초)")
    args = ap.parse_args()

    rclpy.init()
    node = PointsToPly(args.topic, args.duration_s, args.out_prefix,
                       args.voxel, args.target_points, args.max_points, args.no_msg_timeout_s)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
