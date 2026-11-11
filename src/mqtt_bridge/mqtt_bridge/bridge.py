#!/usr/bin/env python3
"""
mqtt_bridge — mobile app (MQTT over mTLS) <-> ROS2, per interface contract v4
sections 7-9. The app lives ENTIRELY in percent-of-map-image coordinates
(u, v in [0,1] from the image's top-left); this bridge owns every conversion.

Threading model: paho-mqtt runs its network loop in its own thread; incoming
MQTT messages are queued and drained by a ROS timer, so ALL ROS work happens
on the rclpy spin thread. Outgoing MQTT publishes (paho) are thread-safe.

Bridge state (contract: exactly these, more = design smell):
  1. current nav goal handle            2. map_id -> metadata cache
  3. current SLAM mode (stub "unknown") 4. teleop held-twist/active flag
  5. sensor freshness timestamps (camera_ok / lidar_ok / battery / ranges)

Graceful degradation is deliberate: with no Nav2 running, goals answer
"planning_failed: nav server unavailable"; with no map yet, no map is
published and goals answer "planning_failed: no map"; a missing
robot_interfaces build only disables actuator passthrough. The bridge itself
always comes up, so the app always gets status.

NOT implemented (recorded in the plan, Stage 2 top-risk item): the
slam_toolbox mapping/localization mode RESTART — load_map / start_mapping
reply ok:false until the supervisor mechanism exists. save_map works when
slam_toolbox is up (serialize service).
"""

import io
import json
import math
import os
import queue
import re
import time
from collections import OrderedDict

import numpy as np
from PIL import Image

import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       qos_profile_sensor_data)
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Range, LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty as EmptySrv
from nav2_msgs.action import NavigateToPose

import tf2_ros

try:
    from robot_interfaces.msg import ActuatorCommand
    HAVE_INTERFACES = True
except ImportError:          # workspace built without robot_interfaces
    HAVE_INTERFACES = False

try:
    from slam_toolbox.srv import SerializePoseGraph
    HAVE_SLAM_SRV = True
except ImportError:
    HAVE_SLAM_SRV = False

NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,64}$')


def _get(d, *keys, default=None):
    """Tolerant JSON field lookup: tries several key spellings, case-insensitive."""
    lower = {str(k).lower(): v for k, v in d.items()}
    for k in keys:
        if k.lower() in lower:
            return lower[k.lower()]
    return default


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        # ── Parameters ──────────────────────────────────────────────────
        p = self.declare_parameter
        p('broker_host', '127.0.0.1')
        p('broker_port', 8883)
        p('use_tls', True)
        p('ca_path', '/robot_data/certs/root_ca.crt')
        p('cert_path', '/robot_data/certs/bridge_client.crt')
        p('key_path', '/robot_data/certs/bridge_client.key')
        p('wheel_track', 0.50)          # MUST match firmware / base_controller
        p('wheel_diameter', 0.165)
        p('right_rpm_sign', 1.0)        # legacy app negated right; flip here if it still does
        p('max_lin', 0.40)              # teleop caps (teleop bypasses velocity_smoother)
        p('max_ang', 0.80)
        p('deadman_s', 0.40)
        p('wasd_lin', 0.25)
        p('wasd_ang', 0.60)
        p('map_min_interval_s', 2.0)
        p('maps_dir', '/robot_data/maps')
        gp = lambda n: self.get_parameter(n).value
        self.track = float(gp('wheel_track'))
        self.wheel_d = float(gp('wheel_diameter'))
        self.right_sign = float(gp('right_rpm_sign'))
        self.max_lin = float(gp('max_lin'))
        self.max_ang = float(gp('max_ang'))
        self.deadman = float(gp('deadman_s'))
        self.wasd_lin = float(gp('wasd_lin'))
        self.wasd_ang = float(gp('wasd_ang'))
        self.map_min_dt = float(gp('map_min_interval_s'))
        self.maps_dir = gp('maps_dir')

        # ── Bridge state ────────────────────────────────────────────────
        self.goal_handle = None
        self.cancel_requested = False
        self.map_metas = OrderedDict()   # map_id -> meta dict (last 10)
        self.map_id = 0
        self.last_map_pub = 0.0
        self.slam_mode = 'unknown'
        self.held_twist = (0.0, 0.0)
        self.teleop_active = False
        self.last_teleop = 0.0
        self.e_stop = False
        self.fresh = {}                  # name -> monotonic ts of last msg
        self.battery_v = None
        self.ranges = {}                 # left/mid/right/hand -> m
        self.provisioning_ok = None

        # ── ROS interfaces ──────────────────────────────────────────────
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_teleop = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.pub_estop = self.create_publisher(Bool, '/e_stop', latched)
        self.pub_actuator = (self.create_publisher(ActuatorCommand, '/actuator_cmd', 10)
                             if HAVE_INTERFACES else None)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_mem_cli = self.create_client(EmptySrv, '/camera_memory/clear')
        self.serialize_cli = (self.create_client(SerializePoseGraph,
                                                 '/slam_toolbox/serialize_map')
                              if HAVE_SLAM_SRV else None)

        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Float32, '/battery_voltage', self.on_battery,
                                 qos_profile_sensor_data)
        for name in ('left', 'mid', 'right', 'hand'):
            self.create_subscription(
                Range, f'/ultrasonic/{name}',
                lambda m, n=name: self.on_range(n, m), qos_profile_sensor_data)
        self.create_subscription(String, '/ui/provisioning_status',
                                 self.on_prov_status, 10)
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.touch('lidar'),
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/obstacle_cloud',
                                 lambda m: self.touch('camera'),
                                 qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── MQTT ────────────────────────────────────────────────────────
        self.inbox = queue.Queue(maxsize=200)
        self.mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                              client_id='robot_bridge', protocol=mqtt.MQTTv311)
        if gp('use_tls'):
            self.mq.tls_set(ca_certs=gp('ca_path'),
                            certfile=gp('cert_path'), keyfile=gp('key_path'))
        self.mq.on_connect = self._mq_connected
        self.mq.on_message = self._mq_message
        # Failures must be LOUD: paho retries silently forever otherwise (a
        # wrong CA / clock skew would look like a healthy-but-mute bridge).
        self.mq.on_connect_fail = lambda c, u: self.get_logger().warn(
            'MQTT connect failed — retrying (broker down? certs?)')
        self.mq.on_disconnect = lambda c, u, flags, rc, props=None: \
            self.get_logger().warn(f'MQTT disconnected (rc={rc}) — reconnecting')
        self.mq.connect_async(gp('broker_host'), int(gp('broker_port')),
                              keepalive=30)
        self.mq.loop_start()             # own thread; retries until broker up

        # ── Timers (all ROS-thread) ─────────────────────────────────────
        self.create_timer(0.05, self.drain_inbox)
        self.create_timer(0.10, self.teleop_tick)       # deadman + >=10 Hz republish
        self.create_timer(0.50, self.publish_pose)      # 2 Hz
        self.create_timer(0.50, self.publish_distances) # 2 Hz
        self.create_timer(2.00, self.publish_system)    # 0.5 Hz

        self.get_logger().info(
            f'mqtt_bridge up (broker {gp("broker_host")}:{gp("broker_port")}, '
            f'tls={gp("use_tls")}, interfaces={HAVE_INTERFACES}, '
            f'slam_srv={HAVE_SLAM_SRV})')

    # ================= MQTT plumbing =================
    APP_TOPICS = [
        'robot/commands/goal', 'robot/commands/stop_autonomy',
        'robot/commands/motor_manual', 'Movement', 'location/commands',
        'robot/commands/emergency_stop', 'hand/control',
        'robot/commands/actuator', 'robot/commands/save_map',
        'robot/commands/load_map', 'robot/commands/start_mapping',
        'robot/commands/list_maps', 'robot/commands/clear_camera_memory',
        'robot/requests/request_pose',
    ]

    def _mq_connected(self, client, userdata, flags, reason_code, properties):
        self.get_logger().info(f'MQTT connected (rc={reason_code})')
        for t in self.APP_TOPICS:
            client.subscribe(t, qos=1)

    def _mq_message(self, client, userdata, msg):
        try:
            self.inbox.put_nowait((msg.topic, msg.payload))
        except queue.Full:
            pass                         # drop oldest-style behavior not needed

    def mq_pub(self, topic, payload, retain=False, qos=0):
        try:
            if isinstance(payload, (dict, list)):
                payload = json.dumps(payload)
            self.mq.publish(topic, payload, qos=qos, retain=retain)
        except Exception as e:           # noqa: BLE001
            self.get_logger().warn(f'MQTT publish failed ({topic}): {e}')

    def drain_inbox(self):
        while True:
            try:
                topic, payload = self.inbox.get_nowait()
            except queue.Empty:
                return
            try:
                self.dispatch(topic, payload)
            except Exception as e:       # noqa: BLE001 — one bad msg never kills the bridge
                self.get_logger().error(f'dispatch error on {topic}: {e}')

    def dispatch(self, topic, payload):
        try:
            data = json.loads(payload) if payload else {}
            if not isinstance(data, dict):
                data = {}
        except (ValueError, UnicodeDecodeError):
            data = {}

        if topic == 'robot/commands/goal':
            self.handle_goal(data)
        elif topic == 'robot/commands/stop_autonomy':
            self.cancel_nav('stopped')
            self.zero_teleop()
        elif topic in ('robot/commands/motor_manual', 'Movement',
                       'location/commands'):
            self.handle_teleop_msg(data)
        elif topic == 'robot/commands/emergency_stop':
            self.handle_estop(data)
        elif topic == 'hand/control':
            self.mq_pub('robot/status/arm',
                        {'status': 'arm_disabled',
                         'detail': 'arm subsystem deferred'})
        elif topic == 'robot/commands/actuator':
            self.handle_actuator(data)
        elif topic == 'robot/commands/save_map':
            self.handle_save_map(data)
        elif topic in ('robot/commands/load_map', 'robot/commands/start_mapping'):
            self.mq_pub('robot/status/ops',
                        {'op': topic.rsplit('/', 1)[-1], 'ok': False,
                         'detail': 'slam mode switching not implemented yet (Stage 2 TODO)'})
        elif topic == 'robot/commands/list_maps':
            self.handle_list_maps()
        elif topic == 'robot/commands/clear_camera_memory':
            self.handle_clear_memory()
        elif topic == 'robot/requests/request_pose':
            self.publish_pose()

    # ================= Map pipeline (contract section 7) =================
    def on_map(self, msg: OccupancyGrid):
        now = time.monotonic()
        if now - self.last_map_pub < self.map_min_dt:
            return
        self.last_map_pub = now

        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        self.map_id += 1
        meta = {
            'map_id': self.map_id,
            'width': int(w), 'height': int(h),
            'resolution': float(msg.info.resolution),
            'origin_x': float(msg.info.origin.position.x),
            'origin_y': float(msg.info.origin.position.y),
        }
        self.map_metas[self.map_id] = meta
        while len(self.map_metas) > 10:
            self.map_metas.popitem(last=False)

        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        img = np.full((h, w), 128, dtype=np.uint8)   # unknown = gray
        img[(grid >= 0) & (grid < 50)] = 255         # free = white
        img[grid >= 50] = 0                          # occupied = black
        img = np.flipud(img)                         # grid row0=bottom -> PNG row0=top

        buf = io.BytesIO()
        Image.fromarray(img, mode='L').save(buf, format='PNG')
        # Image FIRST, meta second: a client reacting to the meta update then
        # always finds the matching (or newer) retained image, never a stale one.
        self.mq_pub('robot/map/image', buf.getvalue(), retain=True)
        self.mq_pub('robot/map/meta', meta, retain=True)

    def latest_meta(self):
        return next(reversed(self.map_metas.values())) if self.map_metas else None

    def uv_to_xy(self, u, v, meta):
        x = meta['origin_x'] + u * meta['width'] * meta['resolution']
        y = meta['origin_y'] + (1.0 - v) * meta['height'] * meta['resolution']
        return x, y

    def xy_to_uv(self, x, y, meta):
        u = (x - meta['origin_x']) / (meta['width'] * meta['resolution'])
        v = 1.0 - (y - meta['origin_y']) / (meta['height'] * meta['resolution'])
        return u, v

    # ================= Navigation =================
    def handle_goal(self, data):
        u = _get(data, 'u', 'x')
        v = _get(data, 'v', 'y')
        if u is None or v is None:
            return self.status('planning_failed', 'goal needs {u, v}')
        try:
            req_id = int(_get(data, 'map_id'))
        except (TypeError, ValueError):
            req_id = None
        meta = self.map_metas.get(req_id) or self.latest_meta()
        if meta is None:
            return self.status('planning_failed', 'no map yet')
        if not self.nav_client.server_is_ready():
            return self.status('planning_failed', 'nav server unavailable')

        x, y = self.uv_to_xy(float(u), float(v), meta)
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        goal.pose = ps

        self.status('executing', f'goal ({x:.2f}, {y:.2f}) m sent')
        self.cancel_requested = False
        fut = self.nav_client.send_goal_async(goal, feedback_callback=self.on_nav_feedback)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        try:
            gh = fut.result()
        except Exception as e:           # noqa: BLE001
            self.goal_handle = None
            return self.status('planning_failed', f'send failed: {e}')
        if gh is None or not gh.accepted:
            self.goal_handle = None
            return self.status('planning_failed', 'goal rejected')
        self.goal_handle = gh
        # A stop that raced the accept window (before the handle existed) is
        # honored now instead of being lost.
        if self.cancel_requested:
            self.cancel_requested = False
            return self.cancel_nav('stopped')
        # Bind THIS handle into the result callback: with back-to-back goals
        # (Nav2 preempts the old one), the old goal's late result must not
        # clobber the new goal's handle or publish a bogus terminal status.
        gh.get_result_async().add_done_callback(
            lambda f, gh=gh: self.on_nav_result(f, gh))

    def on_nav_feedback(self, fb):
        try:
            d = fb.feedback.distance_remaining
            self.status('executing', f'distance_remaining={d:.2f}')
        except AttributeError:
            pass

    def on_nav_result(self, fut, gh):
        if self.goal_handle is not None and self.goal_handle is not gh:
            return                       # stale result of a preempted goal
        self.goal_handle = None
        try:
            st = fut.result().status
        except Exception:                # noqa: BLE001
            return self.status('execution_error', 'result lost')
        vocab = {GoalStatus.STATUS_SUCCEEDED: ('finished', 'reached_goal'),
                 GoalStatus.STATUS_CANCELED: ('stopped', 'canceled'),
                 GoalStatus.STATUS_ABORTED: ('execution_error', 'aborted')}
        state, detail = vocab.get(st, ('execution_error', f'status={st}'))
        self.status(state, detail)

    def cancel_nav(self, state=None):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:            # noqa: BLE001
                pass
            self.goal_handle = None
            if state:
                self.status(state, 'canceled by user')
        else:
            # Goal may be mid-accept (handle not delivered yet): remember the
            # intent; on_goal_response honors it the moment the handle exists.
            self.cancel_requested = True

    def status(self, state, detail=''):
        self.mq_pub('robot/status/autonomy', {'state': state, 'detail': detail})

    # ================= Teleop (contract: deadman, caps, cancel-on-teleop) ==
    def handle_teleop_msg(self, data):
        cmd = _get(data, 'Command', 'command', 'cmd')
        if isinstance(cmd, str):
            c = cmd.strip().upper()
            presets = {'W': (self.wasd_lin, 0.0), 'S': (-self.wasd_lin, 0.0),
                       'A': (0.0, self.wasd_ang), 'D': (0.0, -self.wasd_ang),
                       'N': (0.0, 0.0)}
            if c in presets:
                return self.apply_teleop(*presets[c])

        lv = _get(data, 'v', 'linear', 'linear_x')
        av = _get(data, 'w', 'omega', 'angular', 'angular_z')
        if lv is not None or av is not None:
            return self.apply_teleop(float(lv or 0.0), float(av or 0.0))

        lr = _get(data, 'left_rpm', 'left', 'leftRpm', 'LeftRpm')
        rr = _get(data, 'right_rpm', 'right', 'rightRpm', 'RightRpm')
        if lr is not None and rr is not None:
            rpm2ms = math.pi * self.wheel_d / 60.0
            vl = float(lr) * rpm2ms
            vr = float(rr) * rpm2ms * self.right_sign
            v = (vl + vr) / 2.0
            w = (vr - vl) / self.track
            return self.apply_teleop(v, w)

    def apply_teleop(self, v, w):
        if self.e_stop:
            return
        v = max(-self.max_lin, min(self.max_lin, v))
        w = max(-self.max_ang, min(self.max_ang, w))
        # Contract: first manual input cancels an active nav goal — the human
        # took over; the robot must NOT resume the old goal on release.
        # ONLY non-zero input counts: a stop preset ('N') or zero keepalive
        # from the app must not kill autonomy.
        if (v != 0.0 or w != 0.0) and self.goal_handle is not None:
            self.cancel_nav('stopped')
        self.held_twist = (v, w)
        self.teleop_active = not (v == 0.0 and w == 0.0)
        self.last_teleop = time.monotonic()
        self.pub_twist(v, w)

    def pub_twist(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub_teleop.publish(t)

    def zero_teleop(self):
        self.held_twist = (0.0, 0.0)
        self.teleop_active = False
        self.pub_twist(0.0, 0.0)

    def teleop_tick(self):
        if not self.teleop_active:
            return
        if time.monotonic() - self.last_teleop > self.deadman:
            # Deadman: phone/app/network went quiet -> one zero, go inactive.
            self.zero_teleop()
        else:
            # Republish held twist at 10 Hz (> twist_mux 0.5 s input timeout).
            self.pub_twist(*self.held_twist)

    # ================= E-stop =================
    def handle_estop(self, data):
        value = _get(data, 'value', 'engaged', default=True)
        if isinstance(value, str):       # tolerate "false"/"0"/"off" strings
            value = value.strip().lower() not in ('false', '0', 'off', 'no')
        self.e_stop = bool(value)
        m = Bool()
        m.data = self.e_stop
        self.pub_estop.publish(m)
        if self.e_stop:
            self.cancel_nav()
            self.zero_teleop()
            self.status('stopped', 'emergency_stop')
        else:
            self.status('stopped', 'e_stop released — idle, awaiting commands')

    # ================= Misc commands =================
    def handle_actuator(self, data):
        if self.pub_actuator is None:
            return self.mq_pub('robot/status/ops',
                               {'op': 'actuator', 'ok': False,
                                'detail': 'robot_interfaces not built'})
        m = ActuatorCommand()
        m.id = int(_get(data, 'id', default=0))
        m.action = int(_get(data, 'action', default=0))
        self.pub_actuator.publish(m)

    def handle_save_map(self, data):
        name = str(_get(data, 'name', 'MapName', default='')).strip()
        if not NAME_RE.match(name):
            return self.mq_pub('robot/status/ops',
                               {'op': 'save_map', 'ok': False, 'detail': 'bad name'})
        if self.serialize_cli is None or not self.serialize_cli.service_is_ready():
            return self.mq_pub('robot/status/ops',
                               {'op': 'save_map', 'ok': False,
                                'detail': 'slam_toolbox serialize service unavailable'})
        req = SerializePoseGraph.Request()
        req.filename = os.path.join(self.maps_dir, name)
        fut = self.serialize_cli.call_async(req)
        fut.add_done_callback(
            lambda f: self.mq_pub('robot/status/ops',
                                  {'op': 'save_map', 'ok': f.exception() is None,
                                   'name': name}))

    def handle_list_maps(self):
        names = []
        try:
            for f in os.listdir(self.maps_dir):
                if f.endswith('.posegraph'):
                    names.append(f[:-len('.posegraph')])
        except OSError:
            pass
        self.mq_pub('robot/status/maps', {'maps': sorted(names)}, retain=True)

    def handle_clear_memory(self):
        if not self.clear_mem_cli.service_is_ready():
            return self.mq_pub('robot/status/ops',
                               {'op': 'clear_camera_memory', 'ok': False,
                                'detail': 'service unavailable'})
        fut = self.clear_mem_cli.call_async(EmptySrv.Request())
        fut.add_done_callback(
            lambda f: self.mq_pub('robot/status/ops',
                                  {'op': 'clear_camera_memory',
                                   'ok': f.exception() is None}))

    # ================= Robot -> app telemetry =================
    def on_plan(self, msg: Path):
        meta = self.latest_meta()
        if meta is None or not msg.poses:
            return
        step = max(1, len(msg.poses) // 50)
        pts = []
        for ps in msg.poses[::step]:
            u, v = self.xy_to_uv(ps.pose.position.x, ps.pose.position.y, meta)
            pts.append([round(u, 4), round(v, 4)])
        self.mq_pub('robot/path', {'points': pts, 'map_id': meta['map_id']})

    def publish_pose(self):
        meta = self.latest_meta()
        if meta is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link',
                                                 rclpy.time.Time())
        except Exception:                # noqa: BLE001 — TF not up yet
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        u, v = self.xy_to_uv(x, y, meta)
        self.mq_pub('robot/sensors/pose',
                    {'u': round(u, 4), 'v': round(v, 4),
                     'theta': round(theta, 4), 'map_id': meta['map_id']})

    def publish_distances(self):
        if not self.ranges:
            return
        out = {k: round(val, 3) for k, val in self.ranges.items()
               if time.monotonic() - self.fresh.get(f'us_{k}', 0) < 2.0}
        if out:
            self.mq_pub('robot/sensors/distances', out)

    def publish_system(self):
        now = time.monotonic()
        self.mq_pub('robot/status/system', {
            'battery_v': (round(self.battery_v, 2)
                          if self.battery_v is not None and
                          now - self.fresh.get('battery', 0) < 5.0 else None),
            'nav_ready': self.nav_client.server_is_ready(),
            'slam_mode': self.slam_mode,
            'provisioning_ok': self.provisioning_ok,
            'camera_ok': now - self.fresh.get('camera', 0) < 3.0,
            'lidar_ok': now - self.fresh.get('lidar', 0) < 3.0,
            'e_stop': self.e_stop,
            'teleop_active': self.teleop_active,
        })

    # ── small sensor handlers ───────────────────────────────────────────
    def touch(self, name):
        self.fresh[name] = time.monotonic()

    def on_battery(self, msg: Float32):
        self.battery_v = float(msg.data)
        self.touch('battery')

    def on_range(self, name, msg: Range):
        self.ranges[name] = float(msg.range)
        self.touch(f'us_{name}')

    def on_prov_status(self, msg: String):
        self.provisioning_ok = not msg.data.startswith('error')

    # ── shutdown ────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self.mq.loop_stop()
            self.mq.disconnect()
        except Exception:                # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
