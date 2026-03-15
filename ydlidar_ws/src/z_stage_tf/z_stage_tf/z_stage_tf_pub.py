#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def rpy_to_quat(roll: float, pitch: float, yaw: float):
    # roll, pitch, yaw (rad) -> quaternion (x,y,z,w)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

class ZStageTFPub(Node):
    """
    Publish:
      - /z_stage/position (Float32, meters)
      - TF: base_link -> laser_frame with z = z_offset + z_stage
    Baseline(고정형): z=0.0 고정
    Z-scan: z 값을 바꿔가며 TF/토픽에 기록
    """
    def __init__(self):
        super().__init__('z_stage_tf_pub')

        # frames
        self.declare_parameter('frame_parent', 'base_link')
        self.declare_parameter('frame_child', 'laser_frame')

        # mount offsets (m)
        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('z_offset', 0.02)  # 라이다 장착 높이 기본값

        # fixed orientation (rad)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        # stage z position (m)
        self.declare_parameter('z', 0.0)  # baseline은 0.0

        # publish rate (Hz)
        self.declare_parameter('rate', 30.0)

        self.pub = self.create_publisher(Float32, '/z_stage/position', 10)
        self.br = TransformBroadcaster(self)

        rate = float(self.get_parameter('rate').value)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        # 파라미터는 런타임에 바꿀 수 있게 매 tick마다 읽음
        parent = self.get_parameter('frame_parent').value
        child  = self.get_parameter('frame_child').value

        x_off = float(self.get_parameter('x_offset').value)
        y_off = float(self.get_parameter('y_offset').value)
        z_off = float(self.get_parameter('z_offset').value)

        roll  = float(self.get_parameter('roll').value)
        pitch = float(self.get_parameter('pitch').value)
        yaw   = float(self.get_parameter('yaw').value)

        z_stage = float(self.get_parameter('z').value)

        # 1) publish z topic
        msg = Float32()
        msg.data = z_stage
        self.pub.publish(msg)

        # 2) publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x_off
        t.transform.translation.y = y_off
        t.transform.translation.z = z_off + z_stage

        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = ZStageTFPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
