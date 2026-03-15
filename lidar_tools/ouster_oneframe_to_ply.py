#!/usr/bin/env python3
import os
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

def save_ply_xyz_ascii(path, xyz):
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {xyz.shape[0]}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        np.savetxt(f, xyz.astype(np.float32), fmt="%.6f %.6f %.6f")

class OneFrame(Node):
    def __init__(self, topic, out_prefix):
        super().__init__("ouster_oneframe_to_ply")
        self.topic = topic
        self.out_prefix = os.path.expanduser(out_prefix)
        self.sub = self.create_subscription(PointCloud2, self.topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"Waiting 1 frame from: {self.topic}")

    def cb(self, msg):
        raw = np.array(list(pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)))
        if raw.size == 0:
            self.get_logger().warn("Empty frame.")
            return
        if raw.dtype.names is not None:
            xyz = np.column_stack((raw["x"], raw["y"], raw["z"]))
        else:
            xyz = raw[:, :3]
        os.makedirs(os.path.dirname(self.out_prefix), exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = f"{self.out_prefix}_{ts}.ply"
        save_ply_xyz_ascii(out, xyz)
        self.get_logger().info(f"Saved ONE-FRAME PLY: {out} (points={xyz.shape[0]})")
        rclpy.shutdown()

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/ouster/points")
    ap.add_argument("--out_prefix", default="~/cloud/ouster_oneframe")
    args = ap.parse_args()
    rclpy.init()
    rclpy.spin(OneFrame(args.topic, args.out_prefix))

if __name__ == "__main__":
    main()
