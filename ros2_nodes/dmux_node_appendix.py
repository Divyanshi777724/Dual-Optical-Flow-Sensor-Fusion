#!/usr/bin/env python3
"""
dmux_node.py — Demultiplexer for dual H-Flow sensor stream.

/fmu/out/sensor_optical_flow carries messages from both sensors interleaved.
This node separates them by device_id and republishes to two clean topics.
fusion_node.py subscribes to those two topics.

Architecture:
  /fmu/out/sensor_optical_flow (mixed, ~150 Hz)
      ↓
  [this node]
      ├─→ /optical_flow/sensor1 (~75 Hz)
      └─→ /optical_flow/sensor2 (~75 Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import SensorOpticalFlow

# ══════════════════════════════════════════════════════════════════════════════
#  TUNABLE PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════

# Physical sensor device IDs — verify with:
#   ros2 topic echo /fmu/out/sensor_optical_flow --field device_id
# These MUST match what PX4 assigns (changes if firmware is reflashed).
DEVICE_ID_S1 = 0x847c03   # = 8682499  H-Flow sensor 1
DEVICE_ID_S2 = 0x847d03   # = 8682755  H-Flow sensor 2

# ══════════════════════════════════════════════════════════════════════════════
#  END OF TUNABLE PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)


class DmuxNode(Node):
    def __init__(self):
        super().__init__('dmux_node')

        self._seen = set()   # device_ids seen so far (for startup diagnostics)

        self.create_subscription(
            SensorOpticalFlow,
            '/fmu/out/sensor_optical_flow',
            self._cb, QOS)

        self.pub1 = self.create_publisher(
            SensorOpticalFlow, '/optical_flow/sensor1', QOS)
        self.pub2 = self.create_publisher(
            SensorOpticalFlow, '/optical_flow/sensor2', QOS)

        self.get_logger().info(
            f'Dmux node ready  '
            f'S1=0x{DEVICE_ID_S1:06x}({DEVICE_ID_S1})  '
            f'S2=0x{DEVICE_ID_S2:06x}({DEVICE_ID_S2})')

    def _cb(self, msg: SensorOpticalFlow):
        did = msg.device_id

        # First-time detection: log any new device_id (helps verify after reflash)
        if did not in self._seen:
            self._seen.add(did)
            self.get_logger().info(
                f'New device_id detected: 0x{did:06x} ({did})')
            if did not in (DEVICE_ID_S1, DEVICE_ID_S2):
                self.get_logger().warn(
                    f'device_id 0x{did:06x} not in config — '
                    f'update DEVICE_ID_S1/S2 in dmux_node.py',
                    throttle_duration_sec=30.0)

        if   did == DEVICE_ID_S1:
            self.pub1.publish(msg)
        elif did == DEVICE_ID_S2:
            self.pub2.publish(msg)
        # unknown IDs are silently dropped (warning already issued above)


def main(args=None):
    rclpy.init(args=args)
    node = DmuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
