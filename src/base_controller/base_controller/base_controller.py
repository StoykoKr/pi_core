#!/usr/bin/env python3
"""
base_controller — wheel telemetry to odometry (contract v4 §3b).

Subscribes /wheel_telemetry (sensor_msgs/JointState from esp_drive:
  name     = [wheel_left, wheel_right]
  position = cumulative travelled distance per wheel [m]  (signed, forward +)
  velocity = wheel angular velocity [rad/s])
Publishes /odom (nav_msgs/Odometry) and the odom->base_link TF.

Deliberately on the Pi, not the MCU: odometry drift gets debugged with
`ros2 topic echo`, not firmware reflashes, and robot_localization fuses the
IMU here later. The ESP reports honest wheel facts; this node does the math.

Stamps: the ESP has no synced clock, so incoming headers are zero — this node
stamps with its own clock at arrival. At 5 Hz telemetry the few-ms transport
delay is far below odometry noise.

NOTE wheel_track default is a PLACEHOLDER until measured on the robot
(center-to-center of the wheel contact patches). It must match the firmware's
WHEEL_TRACK_M define. A wrong value = systematic rotation error (robot turns
more/less than odometry believes), which SLAM partially absorbs but should be
measured, not tuned.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

LEFT = 'wheel_left'
RIGHT = 'wheel_right'


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')

        self.declare_parameter('wheel_track', 0.50)   # TODO MEASURE — placeholder!
        self.declare_parameter('wheel_radius', 0.0825)  # 165 mm hoverboard wheel / 2
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.track = self.get_parameter('wheel_track').value
        self.radius = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        if abs(self.track - 0.50) < 1e-9:
            self.get_logger().warn(
                'wheel_track is the UNMEASURED placeholder (0.50 m) — rotation '
                'odometry will be systematically wrong until measured.')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._prev = None  # (pos_left, pos_right)

        # Odometry goes out RELIABLE (default QoS): Nav2's OdomSmoother
        # subscribes reliable, and a reliable sub cannot match a best-effort
        # pub — publishing odom best-effort = Nav2 silently receives nothing.
        # (The /wheel_telemetry SUBSCRIPTION below stays best-effort to match
        # the firmware's best-effort publisher.)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, 'wheel_telemetry',
                                 self._on_telemetry, qos_profile_sensor_data)
        self.get_logger().info(
            f'base_controller up (track={self.track} m, radius={self.radius} m)')

    # ------------------------------------------------------------------
    def _on_telemetry(self, msg: JointState):
        try:
            i_l = msg.name.index(LEFT)
            i_r = msg.name.index(RIGHT)
        except ValueError:
            self.get_logger().warn(
                f'wheel_telemetry without {LEFT}/{RIGHT} names: {list(msg.name)}',
                throttle_duration_sec=5.0)
            return
        if len(msg.position) <= max(i_l, i_r):
            return

        pos_l = msg.position[i_l]
        pos_r = msg.position[i_r]

        if self._prev is None:
            self._prev = (pos_l, pos_r)
            return

        d_l = pos_l - self._prev[0]
        d_r = pos_r - self._prev[1]
        self._prev = (pos_l, pos_r)

        # A VESC reconnect / tacho reset shows up as a huge jump — don't
        # teleport the robot, just resync silently.
        if abs(d_l) > 0.5 or abs(d_r) > 0.5:
            self.get_logger().warn(
                f'wheel distance jump (dl={d_l:.2f}, dr={d_r:.2f}) — resyncing')
            return

        d = (d_l + d_r) / 2.0
        d_yaw = (d_r - d_l) / self.track

        # Midpoint integration
        mid_yaw = self.yaw + d_yaw / 2.0
        self.x += d * math.cos(mid_yaw)
        self.y += d * math.sin(mid_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + d_yaw),
                              math.cos(self.yaw + d_yaw))

        # Velocities from the ESP's wheel angular velocities
        v_l = v_r = 0.0
        if len(msg.velocity) > max(i_l, i_r):
            v_l = msg.velocity[i_l] * self.radius
            v_r = msg.velocity[i_r] * self.radius
        v = (v_l + v_r) / 2.0
        w = (v_r - v_l) / self.track

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Rough, honest covariances: position grows with travel, trust twist more.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05
        self.pub_odom.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quat(self.yaw)
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
