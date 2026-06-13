"""
fusion_node.py - Quality-weighted fusion of two H-Flow sensors.

Reads from dmux output topics, publishes FusedOpticalFlow back to PX4.
PX4's VehicleOpticalFlow module reads this topic and feeds the result
into EKF2. No velocity computation here - that is EKF2's job.

Architecture:
  /optical_flow/sensor1 ──┐
                           ├─→ [this node] ─→ /fmu/in/fused_optical_flow ─→ PX4
  /optical_flow/sensor2 ──┘
  /fmu/out/distance_sensor ──────────────────────────────────────────────────┘

Author: Divyanshi
"""

import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import SensorOpticalFlow, FusedOpticalFlow, DistanceSensor

# ══════════════════════════════════════════════════════════════════════════════
#  TUNABLE PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════

#   ros2 topic echo /fmu/out/sensor_optical_flow --field device_id
DEVICE_ID_S1    = 0x847c03   # = 8682499  H-Flow sensor 1 (body-frame aligned)
DEVICE_ID_S2    = 0x847d03   # = 8682755  H-Flow sensor 2

# Physical mounting of sensor 2 relative to sensor 1 (degrees, yaw only).
# 0.0 means both sensors are body-frame aligned.
SENSOR2_YAW_DEG = 0.0

# Virtual device ID written into FusedOpticalFlow - unique, non-zero.
FUSED_DEVICE_ID = 999999

# Quality gate: sensors below this are excluded from fusion entirely.
# Range: 0-255.
MIN_QUALITY     = 50

# Quality penalty applied when only one sensor is available.
SINGLE_SENSOR_QUALITY_PENALTY = 20

# Sensor stale threshold (s). 2.0 s tolerates serial hiccups observed in logs.
FLOW_TIMEOUT_S  = 2.0

# Distance sensor stale threshold (s).
DIST_TIMEOUT_S  = 0.5

# Fusion output rate (Hz).
LOOP_HZ         = 50

# Sanity bounds - pixel_flow values outside this range are rejected (rad).
MAX_FLOW_RAD    = 0.5

# Passed through to FusedOpticalFlow for EKF2 range gating.
MIN_GROUND_DIST = 0.08   # m
MAX_GROUND_DIST = 30.0   # m

# ══════════════════════════════════════════════════════════════════════════════
#  END OF TUNABLE PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=5)

QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class SensorBuf:
    """Holds the latest message from one sensor."""
    def __init__(self, name: str):
        self.name = name
        self.msg: SensorOpticalFlow | None = None
        self.t = 0.0

    def update(self, msg: SensorOpticalFlow):
        self.msg = msg
        self.t   = time.time()

    def fresh(self) -> bool:
        return (self.msg is not None) and (time.time() - self.t < FLOW_TIMEOUT_S)

    def usable(self) -> bool:
        if not self.fresh():
            return False
        m = self.msg
        if m.quality < MIN_QUALITY:
            return False
        if abs(m.pixel_flow[0]) > MAX_FLOW_RAD or abs(m.pixel_flow[1]) > MAX_FLOW_RAD:
            return False
        return True

    def pixel_flow_body(self, yaw_deg: float):
        """Return pixel_flow rotated from sensor frame into body frame."""
        pf = self.msg.pixel_flow
        if yaw_deg == 0.0:
            return float(pf[0]), float(pf[1])
        rad = math.radians(yaw_deg)
        c, s = math.cos(rad), math.sin(rad)
        return (c * pf[0] - s * pf[1],
                s * pf[0] + c * pf[1])

    def delta_angle_body(self, yaw_deg: float):
        """Return delta_angle rotated into body frame (yaw axis unchanged)."""
        if self.msg is None or not self.msg.delta_angle_available:
            return None
        da = self.msg.delta_angle
        if yaw_deg == 0.0:
            return [float(da[0]), float(da[1]), float(da[2])]
        rad = math.radians(yaw_deg)
        c, s = math.cos(rad), math.sin(rad)
        return [c * da[0] - s * da[1],
                s * da[0] + c * da[1],
                float(da[2])]


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.s1 = SensorBuf('S1')
        self.s2 = SensorBuf('S2')

        self.dist_m = 0.0
        self.dist_t = 0.0

        self._n            = 0
        self._cnt_both     = 0
        self._cnt_single   = 0
        self._cnt_none     = 0
        self._cnt_total    = 0

        self.create_subscription(SensorOpticalFlow,
            '/optical_flow/sensor1', self._cb_s1, QOS)
        self.create_subscription(SensorOpticalFlow,
            '/optical_flow/sensor2', self._cb_s2, QOS)
        self.create_subscription(DistanceSensor,
            '/fmu/out/distance_sensor', self._cb_dist, QOS)

        self.pub = self.create_publisher(
            FusedOpticalFlow, '/fmu/in/fused_optical_flow', QOS_PUB)

        self.create_timer(1.0 / LOOP_HZ, self._fuse)
        self.get_logger().info(
            f'Fusion node ready  '
            f'S1=0x{DEVICE_ID_S1:06x}  S2=0x{DEVICE_ID_S2:06x}  '
            f'rate={LOOP_HZ}Hz  flow_timeout={FLOW_TIMEOUT_S}s')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_s1(self, msg: SensorOpticalFlow):
        self.s1.update(msg)

    def _cb_s2(self, msg: SensorOpticalFlow):
        self.s2.update(msg)

    def _cb_dist(self, msg: DistanceSensor):
        if msg.current_distance > 0.0:
            self.dist_m = msg.current_distance
            self.dist_t = time.time()

    def _dist_fresh(self) -> bool:
        return (self.dist_m > 0.0) and (time.time() - self.dist_t < DIST_TIMEOUT_S)

    # ── Fusion loop ───────────────────────────────────────────────────────────

    def _fuse(self):
        self._n         += 1
        self._cnt_total += 1

        s1_ok = self.s1.usable()
        s2_ok = self.s2.usable()

        out = FusedOpticalFlow()
        out.timestamp             = int(time.time() * 1e6)
        out.device_id             = FUSED_DEVICE_ID
        out.max_flow_rate         = MAX_FLOW_RAD / (16e-3)
        out.min_ground_distance   = MIN_GROUND_DIST
        out.max_ground_distance   = MAX_GROUND_DIST
        out.mode                  = 0

        if s1_ok and s2_ok:
            # ── Both sensors: quality-weighted average ─────────────────────
            q1 = float(self.s1.msg.quality)
            q2 = float(self.s2.msg.quality)
            total = q1 + q2
            w1, w2 = q1 / total, q2 / total

            pf1x, pf1y = self.s1.pixel_flow_body(0.0)
            pf2x, pf2y = self.s2.pixel_flow_body(SENSOR2_YAW_DEG)
            out.pixel_flow = [
                float(w1 * pf1x + w2 * pf2x),
                float(w1 * pf1y + w2 * pf2y)]

            da1 = self.s1.delta_angle_body(0.0)
            da2 = self.s2.delta_angle_body(SENSOR2_YAW_DEG)
            if da1 and da2:
                out.delta_angle = [
                    float(w1 * da1[i] + w2 * da2[i]) for i in range(3)]
                out.delta_angle_available = True
            elif da1:
                out.delta_angle = da1
                out.delta_angle_available = True
            elif da2:
                out.delta_angle = da2
                out.delta_angle_available = True

            out.quality = int(w1 * q1 + w2 * q2)
            out.timestamp_sample = min(
                self.s1.msg.timestamp_sample,
                self.s2.msg.timestamp_sample)
            out.integration_timespan_us = int(
                (self.s1.msg.integration_timespan_us +
                 self.s2.msg.integration_timespan_us) / 2)
            out.error_count = (self.s1.msg.error_count +
                               self.s2.msg.error_count)
            self._cnt_both += 1

        elif s1_ok or s2_ok:
            # ── Single sensor fallback ─────────────────────────────────────
            src, yaw = (self.s1, 0.0) if s1_ok else (self.s2, SENSOR2_YAW_DEG)
            pfx, pfy = src.pixel_flow_body(yaw)
            out.pixel_flow = [float(pfx), float(pfy)]

            da = src.delta_angle_body(yaw)
            if da:
                out.delta_angle           = da
                out.delta_angle_available = True

            out.quality = max(0, src.msg.quality - SINGLE_SENSOR_QUALITY_PENALTY)
            out.timestamp_sample        = src.msg.timestamp_sample
            out.integration_timespan_us = src.msg.integration_timespan_us
            out.error_count             = src.msg.error_count
            self._cnt_single += 1

        else:
            out.pixel_flow  = [0.0, 0.0]
            out.delta_angle = [0.0, 0.0, 0.0]
            out.quality     = 0
            out.integration_timespan_us = int(1e6 / LOOP_HZ)
            self._cnt_none += 1

        # Distance from dedicated rangefinder topic
        if self._dist_fresh():
            out.distance_m         = self.dist_m
            out.distance_available = True
        else:
            out.distance_m         = 0.0
            out.distance_available = False

        self.pub.publish(out)

        # Periodic status log (every 5 s)
        if self._n % (LOOP_HZ * 5) == 0:
            pct_both   = 100.0 * self._cnt_both   / max(1, self._cnt_total)
            pct_single = 100.0 * self._cnt_single / max(1, self._cnt_total)
            pct_none   = 100.0 * self._cnt_none   / max(1, self._cnt_total)
            q1 = self.s1.msg.quality if self.s1.msg else 0
            q2 = self.s2.msg.quality if self.s2.msg else 0
            self.get_logger().info(
                f'Both:{pct_both:.0f}%  Single:{pct_single:.0f}%  '
                f'None:{pct_none:.0f}%  '
                f'Q1={q1} Q2={q2}  dist={self.dist_m:.2f}m  '
                f'out_q={out.quality}  '
                f'pf=[{out.pixel_flow[0]:.4f},{out.pixel_flow[1]:.4f}]')


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
