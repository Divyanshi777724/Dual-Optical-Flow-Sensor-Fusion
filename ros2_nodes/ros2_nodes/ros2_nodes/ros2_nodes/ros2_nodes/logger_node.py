"""
logger_node.py:- UAV telemetry logger.
Writes timestamped CSV at 10 Hz to ~/uav_logs/.
"""

import rclpy, csv, os, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from datetime import datetime
from px4_msgs.msg import (
    SensorOpticalFlow, FusedOpticalFlow,
    VehicleOpticalFlowVel, VehicleLocalPosition,
    VehicleAttitude, SensorCombined, EstimatorStatus,
    DistanceSensor, ManualControlSetpoint,
    VehicleAttitudeSetpoint, VehicleStatus,
)

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class UavLogger(Node):

    def __init__(self):
        super().__init__('uav_logger')

        self.state = {
            'rc_roll': None, 'rc_pitch': None,
            'rc_throttle': None, 'rc_yaw': None, 'rc_valid': None,

            'cmd_roll_deg': None, 'cmd_pitch_deg': None,
            'cmd_yaw_deg': None, 'cmd_thrust': None,

            'fused_pixel_flow_x': None, 'fused_pixel_flow_y': None,
            'fused_quality': None, 'fused_distance_m': None,
            'fused_dist_avail': None,

            'raw_device_id': None, 'raw_quality': None,
            'raw_pixel_flow_x': None, 'raw_pixel_flow_y': None,
            'raw_distance_m': None,

            'of_vel_body_x': None, 'of_vel_body_y': None,
            'of_vel_ne_x': None, 'of_vel_ne_y': None,

            'local_x': None, 'local_y': None, 'local_z': None,
            'local_vx': None, 'local_vy': None, 'local_vz': None,
            'local_xy_valid': None, 'local_v_xy_valid': None,
            'local_z_valid': None, 'local_dead_reck': None,
            'local_eph': None, 'local_evh': None,

            'q0': None, 'q1': None, 'q2': None, 'q3': None,

            'acc_x': None, 'acc_y': None, 'acc_z': None,
            'gyro_x': None, 'gyro_y': None, 'gyro_z': None,

            'ekf_vel_horiz_fused': None,
            'ekf_opt_flow_fused': None,
            'ekf_innovation_check_flags': None,

            'dist_m': None, 'dist_sig_quality': None,

            'nav_state': None, 'arming_state': None,
        }

        self.create_subscription(ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',  self.cb_rc,        QOS)
        self.create_subscription(VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint', self.cb_att_sp,    QOS)
        self.create_subscription(FusedOpticalFlow,
            '/fmu/in/fused_optical_flow',        self.cb_fused,     QOS)
        self.create_subscription(SensorOpticalFlow,
            '/fmu/out/sensor_optical_flow',      self.cb_raw,       QOS)
        self.create_subscription(VehicleOpticalFlowVel,
            '/fmu/out/vehicle_optical_flow_vel', self.cb_of_vel,    QOS)
        self.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',   self.cb_local_pos, QOS)
        self.create_subscription(VehicleAttitude,
            '/fmu/out/vehicle_attitude',         self.cb_attitude,  QOS)
        self.create_subscription(SensorCombined,
            '/fmu/out/sensor_combined',          self.cb_imu,       QOS)
        self.create_subscription(EstimatorStatus,
            '/fmu/out/estimator_status',         self.cb_ekf,       QOS)
        self.create_subscription(DistanceSensor,
            '/fmu/out/distance_sensor',          self.cb_dist,      QOS)
        self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status',           self.cb_status,    QOS)

        log_dir = os.path.expanduser('~/uav_logs')
        os.makedirs(log_dir, exist_ok=True)
        fname = os.path.join(
            log_dir,
            f"uav_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.csv_file = open(fname, 'w', newline='')
        self.writer   = csv.DictWriter(
            self.csv_file,
            fieldnames=['ros_time'] + list(self.state.keys()))
        self.writer.writeheader()
        self.get_logger().info(f'Logging to: {fname}')

        self.row_count = 0
        self.create_timer(0.1, self.log_row)
        self.create_timer(5.0, self.print_summary)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cb_rc(self, msg: ManualControlSetpoint):
        self.state.update({
            'rc_roll': msg.roll, 'rc_pitch': msg.pitch,
            'rc_throttle': msg.throttle, 'rc_yaw': msg.yaw,
            'rc_valid': msg.valid,
        })

    def cb_att_sp(self, msg: VehicleAttitudeSetpoint):
        self.state.update({
            'cmd_roll_deg':  math.degrees(msg.roll_body),
            'cmd_pitch_deg': math.degrees(msg.pitch_body),
            'cmd_yaw_deg':   math.degrees(msg.yaw_body),
            'cmd_thrust':    -msg.thrust_body[2],
        })

    def cb_fused(self, msg: FusedOpticalFlow):
        self.state.update({
            'fused_pixel_flow_x': msg.pixel_flow[0],
            'fused_pixel_flow_y': msg.pixel_flow[1],
            'fused_quality':      msg.quality,
            'fused_distance_m':   msg.distance_m,
            'fused_dist_avail':   msg.distance_available,
        })

    def cb_raw(self, msg: SensorOpticalFlow):
        self.state.update({
            'raw_device_id':    msg.device_id,
            'raw_quality':      msg.quality,
            'raw_pixel_flow_x': msg.pixel_flow[0],
            'raw_pixel_flow_y': msg.pixel_flow[1],
            'raw_distance_m':   msg.distance_m,
        })

    def cb_of_vel(self, msg: VehicleOpticalFlowVel):
        self.state.update({
            'of_vel_body_x': msg.vel_body[0],
            'of_vel_body_y': msg.vel_body[1],
            'of_vel_ne_x':   msg.vel_ne[0],
            'of_vel_ne_y':   msg.vel_ne[1],
        })

    def cb_local_pos(self, msg: VehicleLocalPosition):
        self.state.update({
            'local_x': msg.x, 'local_y': msg.y, 'local_z': msg.z,
            'local_vx': msg.vx, 'local_vy': msg.vy, 'local_vz': msg.vz,
            'local_xy_valid':   msg.xy_valid,
            'local_v_xy_valid': msg.v_xy_valid,
            'local_z_valid':    msg.z_valid,
            'local_dead_reck':  msg.dead_reckoning,
            'local_eph':        msg.eph,
            'local_evh':        msg.evh,
        })

    def cb_attitude(self, msg: VehicleAttitude):
        self.state.update({
            'q0': msg.q[0], 'q1': msg.q[1],
            'q2': msg.q[2], 'q3': msg.q[3],
        })

    def cb_imu(self, msg: SensorCombined):
        self.state.update({
            'acc_x': msg.accelerometer_m_s2[0],
            'acc_y': msg.accelerometer_m_s2[1],
            'acc_z': msg.accelerometer_m_s2[2],
            'gyro_x': msg.gyro_rad[0],
            'gyro_y': msg.gyro_rad[1],
            'gyro_z': msg.gyro_rad[2],
        })

    def cb_ekf(self, msg: EstimatorStatus):
        self.state.update({
            'ekf_vel_horiz_fused':        bool(msg.control_mode_flags & (1 << 6)),
            'ekf_opt_flow_fused':         bool(msg.control_mode_flags & (1 << 10)),
            'ekf_innovation_check_flags': msg.innovation_check_flags,
        })

    def cb_dist(self, msg: DistanceSensor):
        self.state.update({
            'dist_m':           msg.current_distance,
            'dist_sig_quality': msg.signal_quality,
        })

    def cb_status(self, msg: VehicleStatus):
        self.state.update({
            'nav_state':    msg.nav_state,
            'arming_state': msg.arming_state,
        })

    # ── Logging ───────────────────────────────────────────────────────────────

    def log_row(self):
        row = {'ros_time': self.get_clock().now().nanoseconds / 1e9}
        row.update(self.state)
        self.writer.writerow(row)
        self.csv_file.flush()
        self.row_count += 1

    def print_summary(self):
        self.get_logger().info(
            f"rows={self.row_count} | "
            f"nav={self.state['nav_state']} | "
            f"fused_q={self.state['fused_quality']} | "
            f"vz={self.state['local_vz']} | "
            f"ekf_of={self.state['ekf_opt_flow_fused']}")

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'Log closed — {self.row_count} rows written')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UavLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
