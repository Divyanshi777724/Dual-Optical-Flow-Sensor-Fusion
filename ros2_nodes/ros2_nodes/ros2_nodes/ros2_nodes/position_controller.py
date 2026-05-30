"""
position_controller.py:- GPS-denied indoor position hold controller.
Attitude-mode offboard: publishes VehicleAttitudeSetpoint at 50 Hz.
Feedforward (stick active) and position hold PD (stick centred) paths
are mutually exclusive, selected by deadband gate.
"""

import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import (
    VehicleLocalPosition, VehicleAttitude, VehicleStatus,
    ManualControlSetpoint, VehicleAttitudeSetpoint,
)

# ── Tunable parameters ────────────────────────────────────────────────────────
STICK_DEADBAND  = 0.10
MAX_VEL_XY      = 1.5           # [m/s]
MAX_VEL_Z       = 0.8           # [m/s]
MAX_YAW_RATE    = 0.8           # [rad/s]

KP_VEL_XY      = 0.15           # [rad/(m/s)]
MAX_TILT        = math.radians(20)

KP_POS          = 0.4
KD_POS          = 0.3
MAX_POS_VEL     = 1.0           # [m/s]

HOVER_THRUST    = 0.55
KP_VEL_Z        = 0.20
KD_ALT          = 0.15
MAX_THRUST      = 0.80
MIN_THRUST      = 0.10
ENTRY_RAMP_S    = 1.5           # [s]

KP_YAW          = 1.2
DATA_TIMEOUT    = 0.5           # [s]
# ─────────────────────────────────────────────────────────────────────────────

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

QOS_REL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


def clamp(v, lo, hi): return max(lo, min(hi, v))

def deadband(v, b):
    if abs(v) < b: return 0.0
    return math.copysign((abs(v) - b) / (1.0 - b), v)

def quat_to_yaw(q):
    return math.atan2(2.0*(q[0]*q[3] + q[1]*q[2]),
                      1.0 - 2.0*(q[2]**2 + q[3]**2))

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    return [cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy]


class PositionController(Node):

    def __init__(self):
        super().__init__('position_controller')

        self.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',  self._cb_pos,    QOS)
        self.create_subscription(VehicleAttitude,
            '/fmu/out/vehicle_attitude',         self._cb_att,    QOS)
        self.create_subscription(ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',  self._cb_rc,     QOS)
        self.create_subscription(TwistStamped,
            '/fused_velocity',                   self._cb_fused,  QOS)
        self.create_subscription(Bool,
            '/offboard/active',                  self._cb_active, QOS_REL)

        self.pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint', QOS)

        self.lp = None;  self.lp_t  = 0.0
        self.att = None; self.att_t = 0.0
        self.rc = None;  self.rc_t  = 0.0
        self.fused_vx = 0.0; self.fused_vy = 0.0; self.fused_t = 0.0

        self.offboard_active = False
        self.entry_time      = None

        self.hold_x = None; self.hold_y = None
        self.hold_z = None; self.hold_yaw = 0.0
        self.yaw_sp = 0.0
        self.holding_xy  = False
        self.holding_z   = False
        self.holding_yaw = False

        self.create_timer(0.02, self._loop)
        self.get_logger().info('Position controller ready')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_pos(self, msg):   self.lp  = msg; self.lp_t  = time.time()
    def _cb_att(self, msg):   self.att = msg; self.att_t = time.time()
    def _cb_rc(self,  msg):   self.rc  = msg; self.rc_t  = time.time()

    def _cb_fused(self, msg):
        self.fused_vx = msg.twist.linear.x
        self.fused_vy = msg.twist.linear.y
        self.fused_t  = time.time()

    def _cb_active(self, msg):
        was = self.offboard_active
        self.offboard_active = msg.data
        if msg.data and not was:
            self.entry_time = time.time()
            self._on_enter()
        if not msg.data and was:
            self.get_logger().info('Offboard exited')
            self.hold_x = None

    # ── Mode entry ────────────────────────────────────────────────────────────

    def _on_enter(self):
        self.hold_yaw = self._yaw() if self._att_ok() else 0.0
        self.yaw_sp   = self.hold_yaw

        if self._pos_ok():
            lp = self.lp
            self.hold_x = lp.x; self.hold_y = lp.y; self.hold_z = lp.z
            self.holding_xy = True; self.holding_z = True; self.holding_yaw = True
            self.get_logger().info(
                f'Offboard entered — hold x={self.hold_x:.2f} '
                f'y={self.hold_y:.2f} z={self.hold_z:.2f} '
                f'yaw={math.degrees(self.hold_yaw):.1f}deg')
        else:
            self.hold_x = None
            self.get_logger().warn('Offboard entered — position NOT valid, level hover only')

    # ── Validity helpers ──────────────────────────────────────────────────────

    def _att_ok(self):
        return self.att is not None and time.time() - self.att_t < DATA_TIMEOUT

    def _pos_ok(self):
        return (self.lp is not None and time.time() - self.lp_t < DATA_TIMEOUT
                and self.lp.xy_valid and self.lp.v_xy_valid)

    def _alt_ok(self):
        return (self.lp is not None and time.time() - self.lp_t < DATA_TIMEOUT
                and self.lp.z_valid and self.lp.v_z_valid)

    def _rc_ok(self):
        return (self.rc is not None and time.time() - self.rc_t < DATA_TIMEOUT
                and self.rc.valid)

    def _fused_ok(self):
        return time.time() - self.fused_t < DATA_TIMEOUT

    def _yaw(self):
        return quat_to_yaw(self.att.q) if self._att_ok() else self.hold_yaw

    def _ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # ── Safe hover ────────────────────────────────────────────────────────────

    def _safe_hover(self, thrust_override=None):
        yaw    = self._yaw() if self._att_ok() else self.yaw_sp
        q      = euler_to_quat(0.0, 0.0, yaw)
        thrust = thrust_override if thrust_override is not None else HOVER_THRUST
        msg = VehicleAttitudeSetpoint()
        msg.timestamp        = self._ts()
        msg.roll_body        = 0.0
        msg.pitch_body       = 0.0
        msg.yaw_body         = yaw
        msg.yaw_sp_move_rate = 0.0
        msg.q_d              = q
        msg.thrust_body      = [0.0, 0.0, -thrust]
        msg.reset_integral   = False
        self.pub.publish(msg)

    def _ramp_thrust(self):
        """Ramp thrust 0 → HOVER_THRUST over ENTRY_RAMP_S on mode entry."""
        if self.entry_time is None:
            return None
        elapsed = time.time() - self.entry_time
        if elapsed >= ENTRY_RAMP_S:
            return None
        return HOVER_THRUST * (elapsed / ENTRY_RAMP_S)

    # ── Thrust computation (NED sign-corrected) ───────────────────────────────

    def _compute_thrust(self, vel_cmd_z):
        """T = HOVER_THRUST - KP_VEL_Z * (vel_cmd_z - vz). NED: minus sign."""
        if not self._alt_ok():
            return HOVER_THRUST
        return clamp(HOVER_THRUST - KP_VEL_Z * (vel_cmd_z - self.lp.vz),
                     MIN_THRUST, MAX_THRUST)

    # ── Main control loop (50 Hz) ─────────────────────────────────────────────

    def _loop(self):
        if not self._att_ok():
            return

        # Always publish so PX4 has a valid setpoint before offboard entry
        if not self.offboard_active:
            self._safe_hover()
            return

        ramp = self._ramp_thrust()
        if ramp is not None:
            self._safe_hover(thrust_override=ramp)
            return

        if not self._rc_ok() or not self._alt_ok():
            self.get_logger().warn('RC or altitude data stale — safe hover',
                                   throttle_duration_sec=2.0)
            self._safe_hover()
            return

        rc = self.rc; lp = self.lp; dt = 0.02

        sx   = deadband(rc.pitch,    STICK_DEADBAND)
        sy   = deadband(rc.roll,     STICK_DEADBAND)
        sz   = deadband(rc.throttle, STICK_DEADBAND)
        syaw = deadband(rc.yaw,      STICK_DEADBAND)

        # ── XY: feedforward (stick active) or position hold (stick centred) ──
        if abs(rc.pitch) > STICK_DEADBAND or abs(rc.roll) > STICK_DEADBAND:
            yaw_now = self._yaw()
            c, s = math.cos(yaw_now), math.sin(yaw_now)
            vx_cmd = c * sx * MAX_VEL_XY - s * sy * MAX_VEL_XY
            vy_cmd = s * sx * MAX_VEL_XY + c * sy * MAX_VEL_XY
            if self._pos_ok():
                self.hold_x = lp.x; self.hold_y = lp.y
            self.holding_xy = False
        else:
            if not self.holding_xy:
                if self._pos_ok():
                    self.hold_x = lp.x; self.hold_y = lp.y
                self.holding_xy = True
            if self._pos_ok() and self.hold_x is not None:
                fvx = self.fused_vx if self._fused_ok() else lp.vx
                fvy = self.fused_vy if self._fused_ok() else lp.vy
                vx_cmd = clamp(KP_POS*(self.hold_x - lp.x) - KD_POS*fvx,
                               -MAX_POS_VEL, MAX_POS_VEL)
                vy_cmd = clamp(KP_POS*(self.hold_y - lp.y) - KD_POS*fvy,
                               -MAX_POS_VEL, MAX_POS_VEL)
            else:
                vx_cmd = vy_cmd = 0.0

        fvx = self.fused_vx if self._fused_ok() else (lp.vx if self._pos_ok() else 0.0)
        fvy = self.fused_vy if self._fused_ok() else (lp.vy if self._pos_ok() else 0.0)
        pitch_cmd = clamp(-KP_VEL_XY * (vx_cmd - fvx), -MAX_TILT, MAX_TILT)
        roll_cmd  = clamp( KP_VEL_XY * (vy_cmd - fvy), -MAX_TILT, MAX_TILT)

        # ── Z: feedforward or altitude hold ──────────────────────────────────
        if abs(rc.throttle) > STICK_DEADBAND:
            vel_cmd_z = -sz * MAX_VEL_Z   # NED: stick up → negative z
            if self._alt_ok(): self.hold_z = lp.z
            self.holding_z = False
        else:
            if not self.holding_z:
                if self._alt_ok(): self.hold_z = lp.z
                self.holding_z = True
            if self._alt_ok() and self.hold_z is not None:
                alt_err   = self.hold_z - lp.z
                vel_cmd_z = clamp(KD_ALT*alt_err - KD_POS*lp.vz,
                                  -MAX_VEL_Z, MAX_VEL_Z)
            else:
                vel_cmd_z = 0.0

        thrust = self._compute_thrust(vel_cmd_z)

        # ── Yaw ──────────────────────────────────────────────────────────────
        if abs(rc.yaw) > STICK_DEADBAND:
            self.yaw_sp = math.atan2(
                math.sin(self.yaw_sp + syaw*MAX_YAW_RATE*dt),
                math.cos(self.yaw_sp + syaw*MAX_YAW_RATE*dt))
            self.holding_yaw = False
        else:
            if not self.holding_yaw:
                self.hold_yaw    = self._yaw()
                self.yaw_sp      = self.hold_yaw
                self.holding_yaw = True
            yaw_err = math.atan2(
                math.sin(self.hold_yaw - self._yaw()),
                math.cos(self.hold_yaw - self._yaw()))
            self.yaw_sp += clamp(KP_YAW*yaw_err*dt, -0.1, 0.1)

        # ── Publish ───────────────────────────────────────────────────────────
        q   = euler_to_quat(roll_cmd, pitch_cmd, self.yaw_sp)
        msg = VehicleAttitudeSetpoint()
        msg.timestamp        = self._ts()
        msg.roll_body        = roll_cmd
        msg.pitch_body       = pitch_cmd
        msg.yaw_body         = self.yaw_sp
        msg.yaw_sp_move_rate = syaw * MAX_YAW_RATE
        msg.q_d              = q
        msg.thrust_body      = [0.0, 0.0, -thrust]
        msg.reset_integral   = False
        self.pub.publish(msg)

        self.get_logger().info(
            f'r={math.degrees(roll_cmd):+5.1f}  p={math.degrees(pitch_cmd):+5.1f}  '
            f'thrust={thrust:.3f}  vz_cmd={vel_cmd_z:+.2f}  vz={lp.vz:+.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
