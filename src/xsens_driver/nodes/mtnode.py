#!/usr/bin/env python3
import select
import time
import datetime
from math import radians, sqrt, atan2

import rclpy
from rclpy.node import Node

import mtdevice
import mtdef

from std_msgs.msg import Header, String, UInt16
from sensor_msgs.msg import (Imu, NavSatFix, NavSatStatus, MagneticField,
                              FluidPressure, Temperature, TimeReference)
from geometry_msgs.msg import TwistStamped, PointStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

try:
    from tf_transformations import quaternion_from_matrix, quaternion_from_euler, identity_matrix
except ImportError:
    from math import cos, sin
    def quaternion_from_euler(roll, pitch, yaw):
        cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
        cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
        cr, sr = cos(roll * 0.5), sin(roll * 0.5)
        return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]
    def identity_matrix():
        import numpy as np
        return np.eye(4)
    def quaternion_from_matrix(m):
        import numpy as np
        t = m[0, 0] + m[1, 1] + m[2, 2]
        if t > 0:
            r = sqrt(t + 1.0)
            s = 0.5 / r
            return [s*(m[2,1]-m[1,2]), s*(m[0,2]-m[2,0]),
                    s*(m[1,0]-m[0,1]), 0.5*r]
        return [0., 0., 0., 1.]


class XSensDriver(Node):

    def __init__(self):
        super().__init__('xsens_driver')

        self.declare_parameter('device', 'auto')
        self.declare_parameter('baudrate', 0)
        self.declare_parameter('timeout', 0.002)
        self.declare_parameter('no_rotation_duration', 0)
        self.declare_parameter('frame_id', '/base_imu')
        self.declare_parameter('frame_local', 'ENU')
        self.declare_parameter('filter_scenario', 50)

        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value

        if device == 'auto':
            devs = mtdevice.find_devices()
            if devs:
                device, baudrate = devs[0]
                self.get_logger().info(
                    'Detected MT device on port %s @ %d bps' % (device, baudrate))
            else:
                self.get_logger().error('Fatal: could not find proper MT device.')
                raise RuntimeError('Could not find proper MT device.')
        if not baudrate:
            baudrate = mtdevice.find_baudrate(device)
        if not baudrate:
            self.get_logger().error('Fatal: could not find proper baudrate.')
            raise RuntimeError('Could not find proper baudrate.')

        self.get_logger().info('MT node interface: %s at %d bd.' % (device, baudrate))
        self.mt = mtdevice.MTDevice(device, baudrate, timeout)

        no_rotation_duration = self.get_parameter('no_rotation_duration').value
        if no_rotation_duration:
            self.get_logger().info(
                'Starting no-rotation procedure for %d s.' % no_rotation_duration)
            self.mt.SetNoRotation(no_rotation_duration)

        self.frame_id = self.get_parameter('frame_id').value
        self.frame_local = self.get_parameter('frame_local').value
        filter_scenario = self.get_parameter('filter_scenario').value
        self.mt.SetCurrentScenario(filter_scenario)

        self.diag_msg = DiagnosticArray()
        self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1,
                                           message='No status information')
        self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1,
                                         message='No status information')
        self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1,
                                         message='No status information')
        self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]

        # Create all publishers up front (ROS 2 best practice)
        self.str_pub = self.create_publisher(String, 'imu_data_str', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'velocity', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.press_pub = self.create_publisher(FluidPressure, 'pressure', 10)
        self.analog_in1_pub = self.create_publisher(UInt16, 'analog_in1', 10)
        self.analog_in2_pub = self.create_publisher(UInt16, 'analog_in2', 10)
        self.ecef_pub = self.create_publisher(PointStamped, 'ecef', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.old_bGPS = 256
        self.last_delta_q_time = None
        self.delta_q_rate = None

        # Timer to read device at ~100 Hz
        self.create_timer(0.01, self._spin_once_timer)

    def reset_vars(self):
        self.imu_msg = Imu()
        self.imu_msg.orientation_covariance = (-1.,) * 9
        self.imu_msg.angular_velocity_covariance = (-1.,) * 9
        self.imu_msg.linear_acceleration_covariance = (-1.,) * 9
        self.pub_imu = False
        self.gps_msg = NavSatFix()
        self.pub_gps = False
        self.vel_msg = TwistStamped()
        self.pub_vel = False
        self.mag_msg = MagneticField()
        self.mag_msg.magnetic_field_covariance = (0,) * 9
        self.pub_mag = False
        self.temp_msg = Temperature()
        self.temp_msg.variance = 0.
        self.pub_temp = False
        self.press_msg = FluidPressure()
        self.press_msg.variance = 0.
        self.pub_press = False
        self.anin1_msg = UInt16()
        self.pub_anin1 = False
        self.anin2_msg = UInt16()
        self.pub_anin2 = False
        self.ecef_msg = PointStamped()
        self.pub_ecef = False
        self.pub_diag = False

    def _spin_once_timer(self):
        try:
            self.spin_once()
            self.reset_vars()
        except select.error:
            pass

    def spin_once(self):
        """Read data from device and publish ROS 2 messages."""
        frame_local = self.frame_local

        def convert_coords(x, y, z, source, dest=frame_local):
            if source == dest:
                return x, y, z
            if source == 'NED':
                x, y, z = y, x, -z
            elif source == 'NWU':
                x, y, z = -y, x, z
            if dest == 'NED':
                x, y, z = y, x, -z
            elif dest == 'NWU':
                x, y, z = y, -x, z
            return x, y, z

        def convert_quat(q, source, dest=frame_local):
            def q_mult(q0, q1):
                w0, x0, y0, z0 = q0
                w1, x1, y1, z1 = q1
                w = w0*w1 - x0*x1 - y0*y1 - z0*z1
                x = w0*x1 + x0*w1 + y0*z1 - z0*y1
                y = w0*y1 - x0*z1 + y0*w1 + z0*x1
                z = w0*z1 + x0*y1 - y0*x1 + z0*w1
                return (w, x, y, z)
            q_enu_ned = (0, 1./sqrt(2), 1./sqrt(2), 0)
            q_enu_nwu = (1./sqrt(2), 0, 0, -1./sqrt(2))
            q_ned_nwu = (0, -1, 0, 0)
            q_ned_enu = (0, -1./sqrt(2), -1./sqrt(2), 0)
            q_nwu_enu = (1./sqrt(2), 0, 0, 1./sqrt(2))
            q_nwu_ned = (0, 1, 0, 0)
            if source == dest:
                return q
            if source == 'ENU':
                return q_mult(q_enu_ned, q) if dest == 'NED' else q_mult(q_enu_nwu, q)
            elif source == 'NED':
                return q_mult(q_ned_enu, q) if dest == 'ENU' else q_mult(q_ned_nwu, q)
            elif source == 'NWU':
                return q_mult(q_nwu_enu, q) if dest == 'ENU' else q_mult(q_nwu_ned, q)
            return q

        def publish_time_ref(secs, nsecs, source):
            time_ref_msg = TimeReference()
            time_ref_msg.header = self.h
            time_ref_msg.time_ref.sec = int(secs)
            time_ref_msg.time_ref.nanosec = int(nsecs)
            time_ref_msg.source = source
            self.time_ref_pub.publish(time_ref_msg)

        def stamp_from_itow(itow, y=None, m=None, d=None, ns=0, week=None):
            if y is not None:
                stamp_day = datetime.datetime(y, m, d)
            elif week is not None:
                epoch = datetime.datetime(1980, 1, 6)
                stamp_day = epoch + datetime.timedelta(weeks=week)
            else:
                today = datetime.date.today()
                stamp_day = datetime.datetime(today.year, today.month, today.day)
            iso_day = stamp_day.isoweekday()
            start_of_week = stamp_day - datetime.timedelta(days=iso_day)
            stamp_ms = start_of_week + datetime.timedelta(milliseconds=itow)
            secs = time.mktime((stamp_ms.year, stamp_ms.month, stamp_ms.day,
                                stamp_ms.hour, stamp_ms.minute, stamp_ms.second,
                                0, 0, -1))
            nsecs = stamp_ms.microsecond * 1000 + ns
            if nsecs < 0:
                secs -= 1
                nsecs += 1e9
            return (secs, nsecs)

        def fill_from_RAW(raw_data):
            self.get_logger().info("Got MTi data packet: 'RAW', ignored!")

        def fill_from_RAWGPS(rawgps_data):
            if rawgps_data['bGPS'] < self.old_bGPS:
                self.pub_gps = True
                self.gps_msg.latitude = rawgps_data['LAT'] * 1e-7
                self.gps_msg.longitude = rawgps_data['LON'] * 1e-7
                self.gps_msg.altitude = rawgps_data['ALT'] * 1e-3
            self.old_bGPS = rawgps_data['bGPS']

        def fill_from_Temp(temp):
            self.pub_temp = True
            self.temp_msg.temperature = temp

        def fill_from_Calib(imu_data):
            try:
                self.pub_imu = True
                x, y, z = convert_coords(imu_data['gyrX'], imu_data['gyrY'],
                                         imu_data['gyrZ'], o['frame'])
                self.imu_msg.angular_velocity.x = x
                self.imu_msg.angular_velocity.y = y
                self.imu_msg.angular_velocity.z = z
                self.imu_msg.angular_velocity_covariance = (
                    radians(0.025), 0., 0., 0., radians(0.025), 0.,
                    0., 0., radians(0.025))
                self.pub_vel = True
                self.vel_msg.twist.angular.x = x
                self.vel_msg.twist.angular.y = y
                self.vel_msg.twist.angular.z = z
            except KeyError:
                pass
            try:
                self.pub_imu = True
                x, y, z = convert_coords(imu_data['accX'], imu_data['accY'],
                                         imu_data['accZ'], o['frame'])
                self.imu_msg.linear_acceleration.x = x
                self.imu_msg.linear_acceleration.y = y
                self.imu_msg.linear_acceleration.z = z
                self.imu_msg.linear_acceleration_covariance = (
                    0.0004, 0., 0., 0., 0.0004, 0., 0., 0., 0.0004)
            except KeyError:
                pass
            try:
                self.pub_mag = True
                x, y, z = convert_coords(imu_data['magX'], imu_data['magY'],
                                         imu_data['magZ'], o['frame'])
                self.mag_msg.magnetic_field.x = x
                self.mag_msg.magnetic_field.y = y
                self.mag_msg.magnetic_field.z = z
            except KeyError:
                pass

        def fill_from_Orient(orient_data):
            self.pub_imu = True
            if 'quaternion' in orient_data:
                w, x, y, z = orient_data['quaternion']
            elif 'roll' in orient_data:
                x, y, z, w = quaternion_from_euler(
                    radians(orient_data['roll']), radians(orient_data['pitch']),
                    radians(orient_data['yaw']))
            elif 'matrix' in orient_data:
                import numpy as np
                m = identity_matrix()
                m[:3, :3] = orient_data['matrix']
                x, y, z, w = quaternion_from_matrix(m)
            else:
                return
            self.imu_msg.orientation.x = x
            self.imu_msg.orientation.y = y
            self.imu_msg.orientation.z = z
            self.imu_msg.orientation.w = w
            self.imu_msg.orientation_covariance = (
                radians(1.), 0., 0., 0., radians(1.), 0., 0., 0., radians(9.))

        def fill_from_Auxiliary(aux_data):
            try:
                self.anin1_msg.data = o['Ain_1']
                self.pub_anin1 = True
            except KeyError:
                pass
            try:
                self.anin2_msg.data = o['Ain_2']
                self.pub_anin2 = True
            except KeyError:
                pass

        def fill_from_Pos(position_data):
            self.pub_gps = True
            self.gps_msg.latitude = position_data['Lat']
            self.gps_msg.longitude = position_data['Lon']
            self.gps_msg.altitude = position_data['Alt']

        def fill_from_Vel(velocity_data):
            self.pub_vel = True
            x, y, z = convert_coords(
                velocity_data['Vel_X'], velocity_data['Vel_Y'],
                velocity_data['Vel_Z'], o['frame'])
            self.vel_msg.twist.linear.x = x
            self.vel_msg.twist.linear.y = y
            self.vel_msg.twist.linear.z = z

        def fill_from_Stat(status):
            self.pub_diag = True
            self.stest_stat.level = (DiagnosticStatus.OK if status & 0b0001
                                     else DiagnosticStatus.ERROR)
            self.stest_stat.message = 'Ok' if status & 0b0001 else 'Failed'
            self.xkf_stat.level = (DiagnosticStatus.OK if status & 0b0010
                                    else DiagnosticStatus.WARN)
            self.xkf_stat.message = 'Valid' if status & 0b0010 else 'Invalid'
            if status & 0b0100:
                self.gps_stat.level = DiagnosticStatus.OK
                self.gps_stat.message = 'Ok'
                self.gps_msg.status.status = NavSatStatus.STATUS_FIX
                self.gps_msg.status.service = NavSatStatus.SERVICE_GPS
            else:
                self.gps_stat.level = DiagnosticStatus.WARN
                self.gps_stat.message = 'No fix'
                self.gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
                self.gps_msg.status.service = 0

        def fill_from_Sample(ts):
            pass  # seq was removed from Header in ROS 2

        def fill_from_Temperature(o):
            self.pub_temp = True
            self.temp_msg.temperature = o['Temp']

        def fill_from_Timestamp(o):
            try:
                y, m, d, hr, mi, s, ns, f = (o['Year'], o['Month'], o['Day'],
                                               o['Hour'], o['Minute'], o['Second'],
                                               o['ns'], o['Flags'])
                if f & 0x4:
                    secs = time.mktime((y, m, d, hr, mi, s, 0, 0, 0))
                    publish_time_ref(secs, ns, 'UTC time')
            except KeyError:
                pass
            try:
                itow = o['TimeOfWeek']
                secs, nsecs = stamp_from_itow(itow)
                publish_time_ref(secs, nsecs, 'integer time of week')
            except KeyError:
                pass
            try:
                sample_time_fine = o['SampleTimeFine']
                secs = int(sample_time_fine / 1000)
                nsecs = 1e6 * (sample_time_fine % 1000)
                publish_time_ref(secs, nsecs, 'sample time fine')
            except KeyError:
                pass
            try:
                sample_time_coarse = o['SampleTimeCoarse']
                publish_time_ref(sample_time_coarse, 0, 'sample time coarse')
            except KeyError:
                pass

        def fill_from_Orientation_Data(o):
            self.pub_imu = True
            x = y = z = w = 0.
            try:
                x, y, z, w = o['Q1'], o['Q2'], o['Q3'], o['Q0']
            except KeyError:
                pass
            try:
                x, y, z, w = quaternion_from_euler(
                    radians(o['Roll']), radians(o['Pitch']), radians(o['Yaw']))
            except KeyError:
                pass
            try:
                import numpy as np
                a, b, c = o['a'], o['b'], o['c']
                d, e, f = o['d'], o['e'], o['f']
                g, h, i = o['g'], o['h'], o['i']
                mat = identity_matrix()
                mat[:3, :3] = ((a, b, c), (d, e, f), (g, h, i))
                x, y, z, w = quaternion_from_matrix(mat)
            except KeyError:
                pass
            w, x, y, z = convert_quat((w, x, y, z), o['frame'])
            self.imu_msg.orientation.x = x
            self.imu_msg.orientation.y = y
            self.imu_msg.orientation.z = z
            self.imu_msg.orientation.w = w
            self.imu_msg.orientation_covariance = (
                radians(1.), 0., 0., 0., radians(1.), 0., 0., 0., radians(9.))

        def fill_from_Pressure(o):
            self.press_msg.fluid_pressure = o['Pressure']
            self.pub_press = True

        def fill_from_Acceleration(o):
            self.pub_imu = True
            x = y = z = 0.
            try:
                x, y, z = o['Delta v.x'], o['Delta v.y'], o['Delta v.z']
            except KeyError:
                pass
            try:
                x, y, z = o['freeAccX'], o['freeAccY'], o['freeAccZ']
            except KeyError:
                pass
            try:
                x, y, z = o['accX'], o['accY'], o['accZ']
            except KeyError:
                pass
            x, y, z = convert_coords(x, y, z, o['frame'])
            self.imu_msg.linear_acceleration.x = x
            self.imu_msg.linear_acceleration.y = y
            self.imu_msg.linear_acceleration.z = z
            self.imu_msg.linear_acceleration_covariance = (
                0.0004, 0., 0., 0., 0.0004, 0., 0., 0., 0.0004)

        def fill_from_Position(o):
            try:
                self.gps_msg.latitude = o['lat']
                self.gps_msg.longitude = o['lon']
                self.pub_gps = True
                self.gps_msg.altitude = o.get('altEllipsoid', o.get('altMsl', 0))
            except KeyError:
                pass
            try:
                self.ecef_msg.point.x = o['ecefX']
                self.ecef_msg.point.y = o['ecefY']
                self.ecef_msg.point.z = o['ecefZ']
                self.pub_ecef = True
            except KeyError:
                pass

        def fill_from_GNSS(o):
            try:
                itow, y, m, d, ns, f = (o['itow'], o['year'], o['month'],
                                         o['day'], o['nano'], o['valid'])
                if f & 0x4:
                    secs, nsecs = stamp_from_itow(itow, y, m, d, ns)
                    publish_time_ref(secs, nsecs, 'GNSS time UTC')
                fixtype = o['fixtype']
                if fixtype == 0x00:
                    self.gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
                    self.gps_msg.status.service = 0
                else:
                    self.gps_msg.status.status = NavSatStatus.STATUS_FIX
                    self.gps_msg.status.service = NavSatStatus.SERVICE_GPS
                self.gps_msg.latitude = o['lat']
                self.gps_msg.longitude = o['lon']
                self.gps_msg.altitude = o['height'] / 1e3
                self.pub_gps = True
            except KeyError:
                pass

        def fill_from_Angular_Velocity(o):
            try:
                dqw, dqx, dqy, dqz = convert_quat(
                    (o['Delta q0'], o['Delta q1'], o['Delta q2'], o['Delta q3']),
                    o['frame'])
                now_ns = self.get_clock().now().nanoseconds
                if self.last_delta_q_time is None:
                    self.last_delta_q_time = now_ns
                else:
                    delta_t = (now_ns - self.last_delta_q_time) * 1e-9
                    if self.delta_q_rate is None:
                        self.delta_q_rate = 1. / delta_t
                    delta_t_filtered = .95 / self.delta_q_rate + .05 * delta_t
                    self.delta_q_rate = round(1. / delta_t_filtered)
                    self.last_delta_q_time = now_ns
                    ca_2, sa_2 = dqw, sqrt(dqx**2 + dqy**2 + dqz**2)
                    ca = ca_2**2 - sa_2**2
                    sa = 2 * ca_2 * sa_2
                    rotation_angle = atan2(sa, ca)
                    rotation_speed = rotation_angle * self.delta_q_rate
                    if sa_2 != 0:
                        f = rotation_speed / sa_2
                        x, y, z = f*dqx, f*dqy, f*dqz
                        self.imu_msg.angular_velocity.x = x
                        self.imu_msg.angular_velocity.y = y
                        self.imu_msg.angular_velocity.z = z
                        self.imu_msg.angular_velocity_covariance = (
                            radians(0.025), 0., 0., 0., radians(0.025), 0.,
                            0., 0., radians(0.025))
                        self.pub_imu = True
                        self.vel_msg.twist.angular.x = x
                        self.vel_msg.twist.angular.y = y
                        self.vel_msg.twist.angular.z = z
                        self.pub_vel = True
            except KeyError:
                pass
            try:
                x, y, z = convert_coords(o['gyrX'], o['gyrY'], o['gyrZ'],
                                         o['frame'])
                self.imu_msg.angular_velocity.x = x
                self.imu_msg.angular_velocity.y = y
                self.imu_msg.angular_velocity.z = z
                self.imu_msg.angular_velocity_covariance = (
                    radians(0.025), 0., 0., 0., radians(0.025), 0.,
                    0., 0., radians(0.025))
                self.pub_imu = True
                self.vel_msg.twist.angular.x = x
                self.vel_msg.twist.angular.y = y
                self.vel_msg.twist.angular.z = z
                self.pub_vel = True
            except KeyError:
                pass

        def fill_from_GPS(o):
            try:
                self.ecef_msg.point.x = o['ecefX'] * 0.01
                self.ecef_msg.point.y = o['ecefY'] * 0.01
                self.ecef_msg.point.z = o['ecefZ'] * 0.01
                self.pub_ecef = True
                self.vel_msg.twist.linear.x = o['ecefVX'] * 0.01
                self.vel_msg.twist.linear.y = o['ecefVY'] * 0.01
                self.vel_msg.twist.linear.z = o['ecefVZ'] * 0.01
                self.pub_vel = True
                itow, ns, week, f = o['iTOW'], o['fTOW'], o['Week'], o['Flags']
                if (f & 0x0C) == 0xC:
                    secs, nsecs = stamp_from_itow(itow, ns=ns, week=week)
                    publish_time_ref(secs, nsecs, 'GPS Time')
            except KeyError:
                pass
            try:
                itow, y, m, d, ns, f = (o['iTOW'], o['year'], o['month'],
                                         o['day'], o['nano'], o['valid'])
                if f & 0x4:
                    secs, nsecs = stamp_from_itow(itow, y, m, d, ns)
                    publish_time_ref(secs, nsecs, 'GPS Time UTC')
            except KeyError:
                pass

        def fill_from_SCR(o):
            pass

        def fill_from_Analog_In(o):
            try:
                self.anin1_msg.data = o['analogIn1']
                self.pub_anin1 = True
            except KeyError:
                pass
            try:
                self.anin2_msg.data = o['analogIn2']
                self.pub_anin2 = True
            except KeyError:
                pass

        def fill_from_Magnetic(o):
            x, y, z = convert_coords(o['magX'], o['magY'], o['magZ'],
                                     o['frame'])
            self.mag_msg.magnetic_field.x = x
            self.mag_msg.magnetic_field.y = y
            self.mag_msg.magnetic_field.z = z
            self.pub_mag = True

        def fill_from_Velocity(o):
            x, y, z = convert_coords(o['velX'], o['velY'], o['velZ'],
                                     o['frame'])
            self.vel_msg.twist.linear.x = x
            self.vel_msg.twist.linear.y = y
            self.vel_msg.twist.linear.z = z
            self.pub_vel = True

        def fill_from_Status(o):
            try:
                fill_from_Stat(o['StatusByte'])
            except KeyError:
                pass
            try:
                fill_from_Stat(o['StatusWord'])
            except KeyError:
                pass

        def find_handler_name(name):
            return 'fill_from_%s' % name.replace(' ', '_')

        try:
            data = self.mt.read_measurement()
        except mtdef.MTTimeoutException:
            time.sleep(0.1)
            return

        self.h = Header()
        self.h.stamp = self.get_clock().now().to_msg()
        self.h.frame_id = self.frame_id

        self.reset_vars()

        for n, o in data.items():
            try:
                locals()[find_handler_name(n)](o)
            except KeyError:
                self.get_logger().warn(
                    "Unknown MTi data packet: '%s', ignoring." % n)

        if self.pub_imu:
            self.imu_msg.header = self.h
            self.imu_pub.publish(self.imu_msg)
        if self.pub_gps:
            self.gps_msg.header = self.h
            self.gps_pub.publish(self.gps_msg)
        if self.pub_vel:
            self.vel_msg.header = self.h
            self.vel_pub.publish(self.vel_msg)
        if self.pub_mag:
            self.mag_msg.header = self.h
            self.mag_pub.publish(self.mag_msg)
        if self.pub_temp:
            self.temp_msg.header = self.h
            self.temp_pub.publish(self.temp_msg)
        if self.pub_press:
            self.press_msg.header = self.h
            self.press_pub.publish(self.press_msg)
        if self.pub_anin1:
            self.analog_in1_pub.publish(self.anin1_msg)
        if self.pub_anin2:
            self.analog_in2_pub.publish(self.anin2_msg)
        if self.pub_ecef:
            self.ecef_msg.header = self.h
            self.ecef_pub.publish(self.ecef_msg)
        if self.pub_diag:
            self.diag_msg.header = self.h
            self.diag_pub.publish(self.diag_msg)

        str_msg = String()
        str_msg.data = str(data)
        self.str_pub.publish(str_msg)


def main(args=None):
    """Create a ROS 2 node and start the XSens driver."""
    rclpy.init(args=args)
    node = XSensDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

