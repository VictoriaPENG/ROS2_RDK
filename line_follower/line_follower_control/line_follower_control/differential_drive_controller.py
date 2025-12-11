import math
import struct
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

try:
  import serial  # type: ignore
except ImportError:  # pragma: no cover
  serial = None


class PID:
  def __init__(self, kp: float, ki: float, kd: float, limit: Optional[float] = None):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.limit = limit
    self.integral = 0.0
    self.prev_error = 0.0

  def reset(self):
    self.integral = 0.0
    self.prev_error = 0.0

  def compute(self, target: float, measurement: float, dt: float) -> float:
    error = target - measurement
    self.integral += error * dt
    derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
    output = self.kp * error + self.ki * self.integral + self.kd * derivative
    if self.limit is not None:
      output = max(-self.limit, min(self.limit, output))
    self.prev_error = error
    return output


class DifferentialDriveController(Node):
  """Convert perception outputs into wheel RPMs and push them over RS-485."""

  def __init__(self):
    super().__init__('line_follower_control')

    self.declare_parameter('wheel_diameter', 0.14)
    self.declare_parameter('wheel_base', 0.5)
    self.declare_parameter('max_linear_speed', 1.11)
    self.declare_parameter('rated_motor_rpm', 2750.0)
    self.declare_parameter('default_linear_speed', 0.2)
    self.declare_parameter('camera_fov_deg', 60.0)
    self.declare_parameter('image_width', 960)
    self.declare_parameter('control_rate', 50.0)
    self.declare_parameter('port', '/dev/ttyUSB0')
    self.declare_parameter('baudrate', 115200)
    self.declare_parameter('kp', 0.8)
    self.declare_parameter('ki', 0.02)
    self.declare_parameter('kd', 0.05)

    self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
    self.wheel_base = float(self.get_parameter('wheel_base').value)
    self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
    self.rated_motor_rpm = float(self.get_parameter('rated_motor_rpm').value)
    self.target_linear = float(self.get_parameter('default_linear_speed').value)
    self.camera_fov_deg = float(self.get_parameter('camera_fov_deg').value)
    self.image_width = float(self.get_parameter('image_width').value)
    self.control_rate = float(self.get_parameter('control_rate').value)
    self.port = str(self.get_parameter('port').value)
    self.baudrate = int(self.get_parameter('baudrate').value)

    self.angular_target = 0.0
    self.left_feedback = 0.0
    self.right_feedback = 0.0

    max_rpm = self.rated_motor_rpm
    self.left_pid = PID(self.get_parameter('kp').value,
                        self.get_parameter('ki').value,
                        self.get_parameter('kd').value,
                        max_rpm)
    self.right_pid = PID(self.get_parameter('kp').value,
                         self.get_parameter('ki').value,
                         self.get_parameter('kd').value,
                         max_rpm)

    self.cmd_sub = self.create_subscription(
        Twist, 'cmd_vel', self.cmd_callback, 10)
    self.center_sub = self.create_subscription(
        Point, 'line_center', self.center_callback, 10)
    self.feedback_sub = self.create_subscription(
        Int16MultiArray, 'wheel_feedback', self.feedback_callback, 10)

    self.serial_handle = self._open_serial()
    self.last_time = time.time()
    self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
    self.get_logger().info('DifferentialDriveController ready. Using port %s', self.port)

  def _open_serial(self):
    if serial is None:
      self.get_logger().warn('pyserial is not available; RS-485 messages will be skipped.')
      return None
    try:
      return serial.Serial(self.port, self.baudrate, timeout=0.02)
    except Exception as exc:  # pragma: no cover
      self.get_logger().warn('Unable to open serial port %s: %s', self.port, exc)
      return None

  def cmd_callback(self, msg: Twist):
    self.target_linear = max(-self.max_linear_speed,
                             min(self.max_linear_speed, msg.linear.x))
    self.angular_target = msg.angular.z

  def center_callback(self, msg: Point):
    normalized = (msg.x - self.image_width / 2.0) / (self.image_width / 2.0)
    half_fov_rad = math.radians(self.camera_fov_deg / 2.0)
    heading_error = normalized * half_fov_rad
    self.angular_target = heading_error
    self.target_linear = min(self.target_linear, self.max_linear_speed)
    self.get_logger().debug('Center offset %.2f px -> heading %.3f rad',
                            msg.x - self.image_width / 2.0, heading_error)

  def feedback_callback(self, msg: Int16MultiArray):
    if len(msg.data) >= 2:
      self.left_feedback = float(msg.data[0])
      self.right_feedback = float(msg.data[1])

  def control_loop(self):
    now = time.time()
    dt = now - self.last_time
    self.last_time = now

    left_rpm_target, right_rpm_target = self._compute_wheel_rpm(
        self.target_linear, self.angular_target)

    left_cmd = self.left_pid.compute(left_rpm_target, self.left_feedback, dt)
    right_cmd = self.right_pid.compute(right_rpm_target, self.right_feedback, dt)

    self._send_rs485_frame(int(left_cmd), int(right_cmd))

  def _compute_wheel_rpm(self, linear: float, angular: float):
    linear = max(-self.max_linear_speed, min(self.max_linear_speed, linear))
    v_left = linear - angular * self.wheel_base / 2.0
    v_right = linear + angular * self.wheel_base / 2.0
    meters_per_rev = math.pi * self.wheel_diameter
    left_rpm = (v_left / meters_per_rev) * 60.0
    right_rpm = (v_right / meters_per_rev) * 60.0
    limit = self.rated_motor_rpm
    return (max(-limit, min(limit, left_rpm)),
            max(-limit, min(limit, right_rpm)))

  def _send_rs485_frame(self, left_rpm: int, right_rpm: int):
    if self.serial_handle is None:
      return
    left_rpm = max(-32768, min(32767, left_rpm))
    right_rpm = max(-32768, min(32767, right_rpm))
    frame = struct.pack('>BBhh', 0xAA, 0x55, left_rpm, right_rpm)
    checksum = sum(frame) & 0xFF
    frame += struct.pack('B', checksum)
    try:
      self.serial_handle.write(frame)
    except Exception as exc:  # pragma: no cover
      self.get_logger().warn('Failed to send RS-485 frame: %s', exc)


def main(args=None):
  rclpy.init(args=args)
  node = DifferentialDriveController()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
