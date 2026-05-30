"""
dmux_node.py :- Demultiplexer for dual H-Flow optical flow sensor stream.
Splits /fmu/out/sensor_optical_flow by device_id into two clean topics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import SensorOpticalFlow

# ══════════════════════════════════════════════════════════════════════════════
#  TUNABLE PARAMETERS :- update these if device IDs change after firmware reflash
# ══════════════════════════════════════════════════════════════════════════════

DEVICE_ID_S1 = 0x847c03   # = 8682499  H-Flow unit 1
DEVICE_ID_S2 = 0x847d03   # = 8682755  H-Flow unit 2

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
