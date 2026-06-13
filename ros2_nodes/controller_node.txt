"""
controller_node.py - Attitude-based position hold controller.

Publishes VehicleAttitudeSetpoint (attitude offboard mode).
PX4 attitude controller executes roll/pitch/thrust directly.

XY control - two mutually exclusive modes (no mixing):
  Sticks outside deadband → feedforward ONLY → direct tilt, no velocity loop
  Sticks inside deadband  → position hold PD feedback ONLY → no feedforward
  This eliminates the conflict between FF and feedback that caused axis mixing.

Z control - same separation:
  Stick outside deadband → feedforward thrust offset + velocity correction
  Stick inside deadband  → altitude hold feedback only

Sign convention (verified Futaba T14SG through PX4 manual_control_setpoint):
  rc.pitch:    push forward = POSITIVE → we want nose-down = NEGATIVE pitch_body
               so pitch_cmd += -Kp_ff * rc.pitch  (Kp_ff positive)
  rc.roll:     push right   = POSITIVE → we want right-roll = POSITIVE roll_body
               so roll_cmd  += +Kp_ff * rc.roll   (Kp_ff positive)
  rc.throttle: push up      = POSITIVE → we want climb = MORE thrust
               NED: thrust_body[2] is negative upward force
               so thrust += +Kp_ff_z * rc.throttle (Kp_ff_z positive)

Author: Divyanshi
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    VehicleLocalPosition, VehicleAttitude, VehicleStatus,
    ManualControlSetpoint, VehicleAttitudeSetpoint, OffboardControlMode,
)
from std_msgs.msg import String

# ══════════════════════════════════════════════════════════════════════════════
#  TUNABLE PARAMETERS 
# ══════════════════════════════════════════════════════════════════════════════

# ── Stick feel ────────────────────────────────────────────────────────────────
STICK_DEADBAND  = 0.10

# ── XY feedforward (sticks active) ───────────────────────────────────────────
# Direct stick → tilt angle. No velocity loop when sticks are moving.
# Increase until stick feel is direct and proportional.
# 0.10 rad at full stick = ~5.7deg immediate tilt.
Kp_ff           = 0.20          # rad per normalized stick unit (POSITIVE)

# ── XY position hold (sticks centered) ───────────────────────────────────────
# PD controller. Runs ONLY when sticks are inside deadband.
Kp_pos          = 0.4           # m/s per m of position error
Kd_pos          = 0.3           # damping  m/s per m/s of velocity
MAX_POS_VEL     = 1.0           # m/s  clamp on position hold velocity output

# Velocity error → tilt angle (used in position hold only)
Kp_vel_xy       = 0.15          # rad per m/s of velocity error
MAX_TILT        = math.radians(10)   # hard clamp on pitch/roll command

# ── Z control ─────────────────────────────────────────────────────────────────
# Hover thrust: fraction of max thrust to hover (0.0–1.0).
# Measure: in stabilized hover check actuator_controls_0.control[3] in QGC.
# Tune: if drone climbs with stick centered → lower; if descends → raise.
HOVER_THRUST    = 0.50

# Z feedforward (stick active): direct stick → thrust offset.
# Added ON TOP of velocity correction when stick is outside deadband.
# Increase until throttle stick feels immediate and proportional.
Kp_ff_z         = 0.20          # thrust units per normalized stick (POSITIVE)

# Z velocity → thrust correction (active in both stick modes)
Kp_vel_z        = 0.35          # thrust per m/s of vertical velocity error

# Altitude hold (stick centered): how hard to correct altitude error
Kd_alt          = 0.25          # m/s of correction per m of altitude error
MAX_VEL_Z_HOLD  = 0.5           # m/s  clamp on altitude hold correction

# Max Z velocity from stick
MAX_VEL_Z       = 0.8           # m/s  full stick → this rate

MAX_THRUST      = 0.85
MIN_THRUST      = 0.10

# ── Yaw ───────────────────────────────────────────────────────────────────────
MAX_YAW_RATE    = 0.8           # rad/s
Kp_yaw          = 1.2

# ── Entry ramp ────────────────────────────────────────────────────────────────
# Ramps thrust 0 → HOVER_THRUST over this many seconds on offboard entry.
# Prevents instant rocket on mode switch. Set to 0.0 to disable.
ENTRY_RAMP_S    = 0.8           # s  reduced from 1.5 for more responsiveness

# ── Data staleness ────────────────────────────────────────────────────────────
DATA_TIMEOUT    = 0.5           # s

# PX4 nav_state for offboard  do not change
NAV_STATE_OFFBOARD = 14

# ══════════════════════════════════════════════════════════════════════════════
#  END OF TUNABLE PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

QOS_REL = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=10)

QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def deadband(v, b):
    if abs(v) < b:
        return 0.0
    return math.copysign((abs(v) - b) / (1.0 - b), v)

def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q[0]*q[3] + q[1]*q[2]),
        1.0 - 2.0 * (q[2]**2 + q[3]**2))

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    return [cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy]


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.lp    = None;  self.lp_t  = 0.0
        self.att   = None;  self.att_t = 0.0
        self.rc    = None;  self.rc_t  = 0.0
        self.nav_state = -1

        self.in_offboard      = False
        self.prev_in_offboard = False
        self.entry_time       = None

        self.hold_x   = None;  self.hold_y   = None
        self.hold_z   = None;  self.hold_yaw = 0.0
        self.yaw_sp   = 0.0

        self.holding_xy  = False
        self.holding_z   = False
        self.holding_yaw = False

        self._n = 0

        self.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',  self._cb_pos,    QOS)
        self.create_subscription(VehicleAttitude,
            '/fmu/out/vehicle_attitude',         self._cb_att,    QOS)
        self.create_subscription(ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',  self._cb_rc,     QOS)
        self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status',           self._cb_status, QOS)

        self.pub_att = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint', QOS_REL)
        self.pub_ocm = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode', QOS_REL)
        self.pub_status = self.create_publisher(
            String, '/controller_status', QOS_STATUS)

        self.create_timer(0.02, self._loop)   # 50 Hz

        self.get_logger().info(
            f'Controller ready  HOVER={HOVER_THRUST}  '
            f'Kp_ff={Kp_ff}  Kp_ff_z={Kp_ff_z}  '
            f'Kp_pos={Kp_pos}  MAX_TILT={math.degrees(MAX_TILT):.0f}deg')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_pos(self, msg):
        self.lp   = msg
        self.lp_t = time.time()

    def _cb_att(self, msg):
        self.att   = msg
        self.att_t = time.time()

    def _cb_rc(self, msg):
        if not msg.valid:
            return
        self.rc   = msg
        self.rc_t = time.time()

    def _cb_status(self, msg):
        self.nav_state = msg.nav_state

    # ── Validity ──────────────────────────────────────────────────────────────

    def _att_ok(self):
        return self.att is not None and (time.time() - self.att_t) < DATA_TIMEOUT

    def _pos_ok(self):
        return (self.lp is not None
                and (time.time() - self.lp_t) < DATA_TIMEOUT
                and self.lp.xy_valid and self.lp.v_xy_valid)

    def _alt_ok(self):
        return (self.lp is not None
                and (time.time() - self.lp_t) < DATA_TIMEOUT
                and self.lp.z_valid)

    def _rc_ok(self):
        return self.rc is not None and (time.time() - self.rc_t) < DATA_TIMEOUT

    def _yaw(self):
        return quat_to_yaw(self.att.q) if self._att_ok() else self.yaw_sp

    def _ts(self):
        return self.get_clock().now().nanoseconds // 1000

    # ── Offboard entry ────────────────────────────────────────────────────────

    def _on_enter(self):
        self.entry_time  = time.time()
        self.hold_yaw    = self._yaw() if self._att_ok() else 0.0
        self.yaw_sp      = self.hold_yaw
        self.holding_xy  = False
        self.holding_z   = False
        self.holding_yaw = True

        if self._pos_ok():
            self.hold_x = self.lp.x
            self.hold_y = self.lp.y
            self.hold_z = self.lp.z
            self.get_logger().info(
                f'Offboard — locked x={self.hold_x:.2f} '
                f'y={self.hold_y:.2f} z={self.hold_z:.2f} '
                f'yaw={math.degrees(self.hold_yaw):.1f}deg')
        else:
            self.hold_x = None
            self.hold_z = None
            self.get_logger().warn('Offboard — position invalid, hover only')

    def _ramp_thrust(self):
        if self.entry_time is None or ENTRY_RAMP_S <= 0.0:
            return None
        elapsed = time.time() - self.entry_time
        if elapsed >= ENTRY_RAMP_S:
            return None
        return HOVER_THRUST * (elapsed / ENTRY_RAMP_S)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_ocm(self):
        ocm = OffboardControlMode()
        ocm.timestamp    = self._ts()
        ocm.position     = False
        ocm.velocity     = False
        ocm.acceleration = False
        ocm.attitude     = True
        ocm.body_rate    = False
        self.pub_ocm.publish(ocm)

    def _publish_hover(self, thrust=None):
        yaw = self._yaw() if self._att_ok() else self.yaw_sp
        t   = thrust if thrust is not None else HOVER_THRUST
        q   = euler_to_quat(0.0, 0.0, yaw)
        msg = VehicleAttitudeSetpoint()
        msg.timestamp        = self._ts()
        msg.roll_body        = 0.0
        msg.pitch_body       = 0.0
        msg.yaw_body         = yaw
        msg.yaw_sp_move_rate = 0.0
        msg.q_d              = q
        msg.thrust_body      = [0.0, 0.0, -t]
        msg.reset_integral   = False
        self.pub_att.publish(msg)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        self._n        += 1
        self.in_offboard = (self.nav_state == NAV_STATE_OFFBOARD)

        if self.in_offboard and not self.prev_in_offboard:
            self._on_enter()
        if not self.in_offboard and self.prev_in_offboard:
            self.get_logger().info('Left offboard — lock reset.')
            self.hold_x     = None
            self.hold_z     = None
            self.entry_time = None
        self.prev_in_offboard = self.in_offboard

        self._publish_ocm()

        if not self._att_ok():
            return

        if not self.in_offboard:
            self._publish_hover()
            self._pub_status(0, 0, 0, HOVER_THRUST, False)
            return

        ramp = self._ramp_thrust()
        if ramp is not None:
            self._publish_hover(thrust=ramp)
            return

        if not self._rc_ok():
            self.get_logger().warn('RC stale — safe hover', throttle_duration_sec=2.0)
            self._publish_hover()
            return

        rc = self.rc
        dt = 0.02

        # ── XY: strictly separated - FF when moving, PD when holding ─────────
        #
        # Sign reasoning (Futaba T14SG verified):
        #   rc.pitch > 0 (push forward) → want nose-down → pitch_body negative
        #   rc.roll  > 0 (push right)   → want right-roll → roll_body positive
        #
        # Feedforward path (stick outside deadband):
        #   pitch_cmd = -Kp_ff * rc.pitch   (positive Kp_ff)
        #   roll_cmd  = +Kp_ff * rc.roll    (positive Kp_ff)
        #   NO velocity feedback — eliminates axis mixing from feedback conflict

        pitch_stick = deadband(rc.pitch, STICK_DEADBAND)
        roll_stick  = deadband(rc.roll,  STICK_DEADBAND)
        sticks_xy_active = (abs(rc.pitch) > STICK_DEADBAND or
                            abs(rc.roll)  > STICK_DEADBAND)

        if sticks_xy_active:
            # ── Feedforward only — direct tilt, no velocity loop ─────────────
            pitch_cmd = clamp(-Kp_ff * pitch_stick, -MAX_TILT, MAX_TILT)
            roll_cmd  = clamp( Kp_ff * roll_stick,  -MAX_TILT, MAX_TILT)

            # Update hold position continuously while moving
            if self._pos_ok():
                self.hold_x = self.lp.x
                self.hold_y = self.lp.y
            self.holding_xy = False

        else:
            # ── Position hold — PD feedback only, no feedforward ─────────────
            if not self.holding_xy:
                if self._pos_ok():
                    self.hold_x = self.lp.x
                    self.hold_y = self.lp.y
                self.holding_xy = True

            if self._pos_ok() and self.hold_x is not None:
                # Velocity command from position error
                vx_cmd = clamp(
                    Kp_pos * (self.hold_x - self.lp.x) - Kd_pos * self.lp.vx,
                    -MAX_POS_VEL, MAX_POS_VEL)
                vy_cmd = clamp(
                    Kp_pos * (self.hold_y - self.lp.y) - Kd_pos * self.lp.vy,
                    -MAX_POS_VEL, MAX_POS_VEL)
                # Velocity error → tilt angle
                pitch_cmd = clamp( Kp_vel_xy * (vx_cmd - self.lp.vx),
                                  -MAX_TILT, MAX_TILT)
                roll_cmd  = clamp(-Kp_vel_xy * (vy_cmd - self.lp.vy),
                                  -MAX_TILT, MAX_TILT)
            else:
                pitch_cmd = 0.0
                roll_cmd  = 0.0

        # ── Z: same separation - FF when stick active, hold when centered ─────
        #
        # Sign reasoning:
        #   rc.throttle > 0 (stick up) → want climb → need MORE thrust
        #   NED: thrust_body[2] negative = upward force → higher magnitude = more thrust
        #   vel_cmd_z: climb = negative NED Z velocity
        #
        # Feedforward:
        #   thrust_ff = +Kp_ff_z * rc.throttle   (positive Kp_ff_z, stick up = more thrust)

        sz = deadband(rc.throttle, STICK_DEADBAND)
        stick_z_active = abs(rc.throttle) > STICK_DEADBAND

        if stick_z_active:
            # Feedforward: immediate thrust response to stick
            thrust_ff = Kp_ff_z * sz   # sz > 0 = stick up = more thrust

            # Velocity correction on top (keeps it from over/undershooting)
            vel_cmd_z = -sz * MAX_VEL_Z   # NED: up = negative Z
            if self._alt_ok():
                vel_err_z = vel_cmd_z - self.lp.vz
                thrust_correction = -Kp_vel_z * vel_err_z
                self.hold_z = self.lp.z
            else:
                thrust_correction = 0.0

            self.holding_z = False
            thrust = clamp(HOVER_THRUST + thrust_ff + thrust_correction,
                           MIN_THRUST, MAX_THRUST)

        else:
            # Altitude hold: feedback only
            if not self.holding_z:
                if self._alt_ok():
                    self.hold_z = self.lp.z
                self.holding_z = True

            if self._alt_ok() and self.hold_z is not None:
                alt_err   = self.hold_z - self.lp.z
                vel_cmd_z = clamp(Kd_alt * alt_err - Kd_pos * self.lp.vz,
                                  -MAX_VEL_Z_HOLD, MAX_VEL_Z_HOLD)
                vel_err_z = vel_cmd_z - self.lp.vz
                thrust    = clamp(HOVER_THRUST - Kp_vel_z * vel_err_z,
                                  MIN_THRUST, MAX_THRUST)
            else:
                thrust    = HOVER_THRUST
                vel_cmd_z = 0.0

        # ── Yaw ──────────────────────────────────────────────────────────────
        syaw = deadband(rc.yaw, STICK_DEADBAND)

        if abs(rc.yaw) > STICK_DEADBAND:
            self.yaw_sp = math.atan2(
                math.sin(self.yaw_sp + syaw * MAX_YAW_RATE * dt),
                math.cos(self.yaw_sp + syaw * MAX_YAW_RATE * dt))
            self.holding_yaw = False
        else:
            if not self.holding_yaw:
                self.hold_yaw    = self._yaw()
                self.yaw_sp      = self.hold_yaw
                self.holding_yaw = True
            yaw_err = math.atan2(
                math.sin(self.hold_yaw - self._yaw()),
                math.cos(self.hold_yaw - self._yaw()))
            self.yaw_sp += clamp(Kp_yaw * yaw_err * dt, -0.1, 0.1)

        # ── Publish ───────────────────────────────────────────────────────────
        q = euler_to_quat(roll_cmd, pitch_cmd, self.yaw_sp)
        msg = VehicleAttitudeSetpoint()
        msg.timestamp        = self._ts()
        msg.roll_body        = roll_cmd
        msg.pitch_body       = pitch_cmd
        msg.yaw_body         = self.yaw_sp
        msg.yaw_sp_move_rate = syaw * MAX_YAW_RATE
        msg.q_d              = q
        msg.thrust_body      = [0.0, 0.0, -thrust]
        msg.reset_integral   = False
        self.pub_att.publish(msg)

        self._pub_status(pitch_cmd, roll_cmd, vel_cmd_z if 'vel_cmd_z' in dir() else 0.0,
                         thrust, True)

        if self._n % 50 == 0:
            mode = 'FF' if sticks_xy_active else 'HOLD'
            self.get_logger().info(
                f'[{mode}]  '
                f'r={math.degrees(roll_cmd):+5.1f}deg  '
                f'p={math.degrees(pitch_cmd):+5.1f}deg  '
                f'thrust={thrust:.3f}  '
                f'holding_xy={self.holding_xy}  '
                f'holding_z={self.holding_z}')

    def _pub_status(self, pitch_cmd, roll_cmd, vz_cmd, thrust, in_ob):
        s = {
            'phase':       'FF+hold',
            'in_offboard': in_ob,
            'rc_fresh':    self._rc_ok(),
            'rc_thr':      round(self.rc.throttle, 4) if self.rc else 0.0,
            'vx_cmd':      round(pitch_cmd, 4),
            'vy_cmd':      round(roll_cmd,  4),
            'vz_cmd':      round(vz_cmd,    4),
            'thrust':      round(thrust,    4),
            'x_target':    round(self.hold_x, 4) if self.hold_x is not None else None,
            'y_target':    round(self.hold_y, 4) if self.hold_y is not None else None,
            'xy_valid':    self.lp.xy_valid if self.lp else False,
            'pos_x':       round(self.lp.x, 4) if self.lp else 0.0,
            'pos_y':       round(self.lp.y, 4) if self.lp else 0.0,
            'pos_z':       round(self.lp.z, 4) if self.lp else 0.0,
            'holding_xy':  self.holding_xy,
            'holding_z':   self.holding_z,
        }
        smsg = String()
        smsg.data = json.dumps(s)
        self.pub_status.publish(smsg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


