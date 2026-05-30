"""
offboard_manager.py :- PX4 offboard mode heartbeat and safety gate.
Publishes OffboardControlMode at 10 Hz. Signals /offboard/active only
when PX4 is in offboard mode and EKF2 position estimates are valid.
"""

import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, VehicleStatus, VehicleLocalPosition

NAV_STATE_OFFBOARD = 14

QOS_PX4 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

QOS_REL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class OffboardManager(Node):

    def __init__(self):
        super().__init__('offboard_manager')

        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._cb_status, QOS_PX4)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._cb_local_pos, QOS_PX4)

        self.pub_ocm    = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', QOS_PX4)
        self.pub_active = self.create_publisher(
            Bool, '/offboard/active', QOS_REL)

        self.nav_state   = -1
        self.is_offboard = False
        self.xy_valid    = False
        self.z_valid     = False
        self.lp_time     = 0.0

        self.create_timer(0.10, self._heartbeat)
        self.get_logger().info('Offboard manager started')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_status(self, msg):
        was              = self.is_offboard
        self.nav_state   = msg.nav_state
        self.is_offboard = (msg.nav_state == NAV_STATE_OFFBOARD)
        if self.is_offboard and not was:
            self.get_logger().info('PX4 entered offboard mode')
        if not self.is_offboard and was:
            self.get_logger().info('PX4 exited offboard mode')

    def _cb_local_pos(self, msg):
        self.xy_valid = msg.xy_valid
        self.z_valid  = msg.z_valid
        self.lp_time  = time.time()

    # ── Heartbeat ─────────────────────────────────────────────────────────────

    def _heartbeat(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)

        # Heartbeat always runs regardless of mode or estimator state
        ocm              = OffboardControlMode()
        ocm.timestamp    = ts
        ocm.position     = False
        ocm.velocity     = False
        ocm.acceleration = False
        ocm.attitude     = True
        ocm.body_rate    = False
        self.pub_ocm.publish(ocm)

        # Safety gate: signal controller active only when position is valid
        lp_fresh          = time.time() - self.lp_time < 0.5
        pos_ready         = lp_fresh and self.xy_valid and self.z_valid
        controller_active = self.is_offboard and pos_ready

        if self.is_offboard and not pos_ready:
            self.get_logger().warn(
                f'Offboard active but position NOT ready — '
                f'xy_valid={self.xy_valid} z_valid={self.z_valid} '
                f'lp_fresh={lp_fresh} — holding safe hover',
                throttle_duration_sec=2.0)

        active      = Bool()
        active.data = controller_active
        self.pub_active.publish(active)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardManager()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
