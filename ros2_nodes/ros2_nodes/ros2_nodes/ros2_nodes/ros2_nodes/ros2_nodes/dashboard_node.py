"""
dashboard_node.py:- Live UAV system health monitor using rich.
Four panels: Infrastructure, Sensor pipeline, EKF2 state, Controller.
"""

import rclpy, time, subprocess, json, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    FusedOpticalFlow, SensorOpticalFlow, VehicleOpticalFlowVel,
    VehicleLocalPosition, VehicleStatus, ManualControlSetpoint,
    EstimatorStatus, DistanceSensor,
)
from std_msgs.msg import String
from rich.console import Console, Group as RGroup
from rich.table import Table
from rich.live import Live
from rich.panel import Panel
from rich.columns import Columns
from rich.text import Text
from rich import box

# ══════════════════════════════════════════════════════════════════════════════
#  TUNABLE PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════
REFRESH_HZ    = 4
RC_STALE_S    = 0.5
TOPIC_STALE_S = 1.0
# ══════════════════════════════════════════════════════════════════════════════

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)

NAV_NAMES = {0:'MANUAL', 1:'ALTCTL', 2:'POSCTL', 3:'MISSION', 4:'LOITER',
             5:'RTL', 14:'OFFBOARD', 15:'STABILIZED', 17:'TAKEOFF', 18:'LAND'}
ARM_NAMES = {1:'DISARMED', 2:'ARMED'}


def _age(t):   return None if t is None else time.time() - t
def _ok(t, lim=TOPIC_STALE_S):
    a = _age(t); return a is not None and a < lim

def _age_str(t):
    a = _age(t)
    if a is None:   return '[grey50]never[/]'
    if a < 0.5:     return f'[green]{a*1000:.0f}ms[/]'
    if a < 2.0:     return f'[yellow]{a:.1f}s[/]'
    return                 f'[red]{a:.1f}s[/]'

def _tick(ok):   return Text('● OK  ', style='green') if ok else Text('● FAIL', style='red')
def _vtick(v):
    if v is None: return Text('—', style='grey50')
    return Text('✓ True ', style='green') if v else Text('✗ False', style='red')

def _proc(name):
    try:
        out = subprocess.check_output(['pgrep', '-f', name], stderr=subprocess.DEVNULL)
        return len(out.strip()) > 0
    except subprocess.CalledProcessError:
        return False


class DashboardNode(Node):

    def __init__(self):
        super().__init__('dashboard_node')

        self.t = {k: None for k in
                  ['fused', 'raw', 'of_vel', 'local', 'status', 'rc', 'ekf', 'dist', 'ctrl']}

        self.fused_q = None; self.fused_pf = [None, None]
        self.fused_dist = None; self.fused_dok = None
        self.raw_q    = {}

        self.of_body  = [None, None]; self.of_ne = [None, None]

        self.pos      = [None, None, None]; self.vel = [None, None, None]
        self.xy_valid = None; self.v_xy_valid = None; self.z_valid = None
        self.eph = None; self.evh = None; self.dead_reck = None; self.vxy_max = None

        self.nav_state = None; self.arm_state = None

        self.rc_thr = None; self.rc_roll = None
        self.rc_pitch = None; self.rc_yaw = None; self.rc_valid = None

        self.ekf_of = None; self.ekf_vel = None; self.ekf_innov = None
        self.dist_m = None; self.dist_q  = None
        self.ctrl   = {}

        self.create_subscription(FusedOpticalFlow,
            '/fmu/in/fused_optical_flow',        self._cb_fused,  QOS)
        self.create_subscription(SensorOpticalFlow,
            '/fmu/out/sensor_optical_flow',      self._cb_raw,    QOS)
        self.create_subscription(VehicleOpticalFlowVel,
            '/fmu/out/vehicle_optical_flow_vel', self._cb_of_vel, QOS)
        self.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',   self._cb_local,  QOS)
        self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status',           self._cb_status, QOS)
        self.create_subscription(ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',  self._cb_rc,     QOS)
        self.create_subscription(EstimatorStatus,
            '/fmu/out/estimator_status',         self._cb_ekf,    QOS)
        self.create_subscription(DistanceSensor,
            '/fmu/out/distance_sensor',          self._cb_dist,   QOS)
        self.create_subscription(String,
            '/controller_status',                self._cb_ctrl,   QOS)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_fused(self, m):
        self.t['fused'] = time.time()
        self.fused_q = m.quality; self.fused_pf = [m.pixel_flow[0], m.pixel_flow[1]]
        self.fused_dist = m.distance_m; self.fused_dok = m.distance_available

    def _cb_raw(self, m):
        self.t['raw'] = time.time(); self.raw_q[m.device_id] = m.quality

    def _cb_of_vel(self, m):
        self.t['of_vel'] = time.time()
        self.of_body = [m.vel_body[0], m.vel_body[1]]
        self.of_ne   = [m.vel_ne[0],   m.vel_ne[1]]

    def _cb_local(self, m):
        self.t['local'] = time.time()
        self.pos = [m.x, m.y, m.z]; self.vel = [m.vx, m.vy, m.vz]
        self.xy_valid = m.xy_valid; self.v_xy_valid = m.v_xy_valid
        self.z_valid  = m.z_valid;  self.eph = m.eph; self.evh = m.evh
        self.dead_reck = m.dead_reckoning; self.vxy_max = m.vxy_max

    def _cb_status(self, m):
        self.t['status'] = time.time()
        self.nav_state = m.nav_state; self.arm_state = m.arming_state

    def _cb_rc(self, m):
        self.t['rc'] = time.time()
        self.rc_thr = m.throttle; self.rc_roll = m.roll
        self.rc_pitch = m.pitch;  self.rc_yaw  = m.yaw; self.rc_valid = m.valid

    def _cb_ekf(self, m):
        self.t['ekf'] = time.time()
        self.ekf_of    = bool(m.control_mode_flags & (1 << 10))
        self.ekf_vel   = bool(m.control_mode_flags & (1 << 6))
        self.ekf_innov = m.innovation_check_flags

    def _cb_dist(self, m):
        self.t['dist'] = time.time()
        self.dist_m = m.current_distance; self.dist_q = m.signal_quality

    def _cb_ctrl(self, m):
        self.t['ctrl'] = time.time()
        try: self.ctrl = json.loads(m.data)
        except Exception: pass

    # ── Render ────────────────────────────────────────────────────────────────

    def render(self):
        # Group 1: Infrastructure
        g1 = Table(box=box.SIMPLE, show_header=True, header_style='bold cyan',
                   title='[bold cyan]Infrastructure[/]')
        g1.add_column('Component', min_width=22)
        g1.add_column('Status',    min_width=8)
        g1.add_column('Detail',    min_width=28)

        dds_ok = _ok(self.t['local']) or _ok(self.t['fused'])
        g1.add_row('uXRCE-DDS bridge', _tick(dds_ok),
                   f'last data {_age_str(self.t["local"])}')
        for proc, label in [
            ('dmux_node', 'dmux_node'), ('fusion_node', 'fusion_node'),
            ('controller_node', 'controller_node'), ('logger_node', 'logger_node'),
            ('dashboard_node', 'dashboard_node'),
        ]:
            alive = _proc(proc)
            g1.add_row(label, _tick(alive),
                       '[green]running[/]' if alive else '[red]not running[/]')
        rc_age   = _age(self.t['rc'])
        rc_fresh = rc_age is not None and rc_age < RC_STALE_S
        rc_ok    = rc_fresh and self.rc_valid is True
        g1.add_row('RC signal', _tick(rc_ok),
                   f'valid={self.rc_valid}  age={_age_str(self.t["rc"])}')

        # Group 2: Sensor pipeline
        g2 = Table(box=box.SIMPLE, show_header=True, header_style='bold magenta',
                   title='[bold magenta]Sensor pipeline[/]')
        g2.add_column('Signal',  min_width=24)
        g2.add_column('Status',  min_width=8)
        g2.add_column('Value',   min_width=30)
        raw_ok = _ok(self.t['raw'])
        for i, did in enumerate(sorted(self.raw_q.keys())[:2]):
            q = self.raw_q[did]
            qc = 'green' if q >= 70 else ('yellow' if q >= 50 else 'red')
            g2.add_row(f'sensor {i+1} (0x{did:06x})', _tick(raw_ok),
                       f'Q=[{qc}]{q}[/]')
        fused_ok = _ok(self.t['fused'])
        fq = self.fused_q or 0
        fqc = 'green' if fq >= 70 else ('yellow' if fq >= 50 else 'red')
        pf_str = (f'pf=[{self.fused_pf[0]:.4f},{self.fused_pf[1]:.4f}]'
                  if self.fused_pf[0] is not None else 'pf=[—,—]')
        g2.add_row('fused → PX4 (device 999999)', _tick(fused_ok),
                   f'Q=[{fqc}]{fq}[/]  {pf_str}  {_age_str(self.t["fused"])}')
        dist_ok = _ok(self.t['dist'])
        g2.add_row('distance_sensor', _tick(dist_ok),
                   f'{self.dist_m:.3f}m  sig={self.dist_q}'
                   if self.dist_m is not None else '—')
        of_ok = _ok(self.t['of_vel'])
        g2.add_row('vehicle_optical_flow_vel', _tick(of_ok),
                   f'body=[{self.of_body[0]:+.3f},{self.of_body[1]:+.3f}]'
                   if self.of_body[0] is not None else '—')

        # Group 3: EKF2 state
        g3 = Table(box=box.SIMPLE, show_header=True, header_style='bold yellow',
                   title='[bold yellow]EKF2 state[/]')
        g3.add_column('Flag',   min_width=20)
        g3.add_column('Value',  min_width=8)
        g3.add_column('Detail', min_width=24)
        g3.add_row('xy_valid',      _vtick(self.xy_valid),
                   f'eph={self.eph:.3f}m' if self.eph else '—')
        g3.add_row('v_xy_valid',    _vtick(self.v_xy_valid),
                   f'evh={self.evh:.3f}m/s' if self.evh else '—')
        g3.add_row('z_valid',       _vtick(self.z_valid),
                   f'dist={self.fused_dist:.3f}m' if self.fused_dist else '—')
        g3.add_row('EKF2 OF fused', _vtick(self.ekf_of),
                   f'innov=0x{self.ekf_innov:04x}' if self.ekf_innov is not None else '—')
        g3.add_row('dead_reckoning',
                   Text('⚠ YES', style='red bold') if self.dead_reck else Text('No', style='green'), '')
        if self.pos[0] is not None:
            local_ok = _ok(self.t['local'])
            g3.add_row('position (NED)', _tick(local_ok),
                       f'x={self.pos[0]:+.3f}  y={self.pos[1]:+.3f}  z={self.pos[2]:+.3f}')
            g3.add_row('velocity (NED)', _tick(local_ok),
                       f'vx={self.vel[0]:+.3f}  vy={self.vel[1]:+.3f}  vz={self.vel[2]:+.3f}')

        # Group 4: Controller
        g4 = Table(box=box.SIMPLE, show_header=True, header_style='bold green',
                   title='[bold green]Controller[/]')
        g4.add_column('Parameter', min_width=22)
        g4.add_column('Value',     min_width=32)
        nav_color = 'green' if self.nav_state == 14 else 'yellow'
        arm_color = 'green' if self.arm_state == 2 else 'grey50'
        g4.add_row('Flight mode',
                   f'[{nav_color}]{NAV_NAMES.get(self.nav_state, self.nav_state)}[/]  '
                   f'[{arm_color}]{ARM_NAMES.get(self.arm_state, self.arm_state)}[/]')
        if rc_ok and self.rc_thr is not None:
            tc = 'yellow' if abs(self.rc_thr) > 0.08 else 'green'
            g4.add_row('RC sticks',
                       f'thr=[{tc}]{self.rc_thr:+.3f}[/]  '
                       f'roll={self.rc_roll:+.3f}  pitch={self.rc_pitch:+.3f}  yaw={self.rc_yaw:+.3f}')
        else:
            g4.add_row('RC sticks', f'[red]NO SIGNAL[/]  age={_age_str(self.t["rc"])}')

        ts = time.strftime('%H:%M:%S')
        return Panel(RGroup(Columns([g1, g2]), Columns([g3, g4])),
                     title=f'[bold white]UAV Dashboard[/]  [grey50]{ts}[/]',
                     border_style='white', padding=(0, 1))


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    console = Console()
    with Live(console=console, refresh_per_second=REFRESH_HZ, screen=True) as live:
        try:
            while rclpy.ok():
                live.update(node.render())
                time.sleep(1.0 / REFRESH_HZ)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
