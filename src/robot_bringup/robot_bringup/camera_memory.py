import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       QoSDurabilityPolicy, QoSReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Empty
import tf2_ros


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float32)


def make_xyz_cloud(header, xyz):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.data = np.ascontiguousarray(xyz, dtype=np.float32).tobytes()
    msg.is_dense = True
    return msg


class CameraMemory(Node):

    def __init__(self):
        super().__init__('camera_memory')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('optical_frame', 'camera_optical_frame')
        # --- NEW: 3D Voxel Sizes ---
        self.declare_parameter('voxel_size_xy', 0.10)
        self.declare_parameter('voxel_size_z', 0.15)
        
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('min_hit_separation', 0.4)
        
        # Spatial Confirmation Parameters
        self.declare_parameter('min_hit_distance', 0.05)
        self.declare_parameter('min_hit_angle', 0.08)
        
        # FOV Clearing Parameters
        self.declare_parameter('enable_fov_clearing', True)
        self.declare_parameter('clear_max_range', 3.0)
        self.declare_parameter('max_misses', 3)
        # Miss decay is applied at most once per this interval, NOT every
        # camera frame. At 10-15 fps, per-frame misses (-1 each) outran the
        # separated-hit bonus (+2 per >=0.4 s) so badly that a real obstacle
        # whose detection flickered for a few frames was deleted before its
        # second confirming hit could ever arrive.
        self.declare_parameter('fov_clear_interval', 0.4)

        self.declare_parameter('unconfirmed_timeout', 3.0) # this will be an issue, right? if its not seen at all is it unconfirmed?
        self.declare_parameter('decay_seconds', 600.0)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('max_points', 200000)

        self.declare_parameter('max_health', 15)
        self.declare_parameter('hit_bonus', 2)
        self.declare_parameter('miss_penalty', 1)




        # Voxel storage format: 
        # (ix, iy, iz) -> [x, y, z, last_seen_sec, hits, last_hit_time, last_rx, last_ry, last_rth, misses]
        self.voxels = {}
        self.last_fov_clear = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointCloud2, '/camera/obstacle_memory', 10)
        self.create_subscription(PointCloud2, '/camera/obstacle_cloud',
                                 self.cloud_callback, qos_profile_sensor_data)
        self.create_service(Empty, '~/clear', self.clear_callback)

        # ---- OccupancyGrid mirror of the confirmed memory -----------------
        # Consumed by a StaticLayer in the costmaps. Unlike an ObstacleLayer
        # (which only ever ADDS lethal cost), a StaticLayer REPLACES its
        # content on every map message, so a voxel this node deletes
        # genuinely disappears from the costmap on the next publish. This is
        # what finally lets the hit/miss/decay logic here also UN-mark.
        self.declare_parameter('publish_grid', True)
        self.declare_parameter('slam_map_topic', '/map')
        self.declare_parameter('grid_topic', '/camera/memory_grid')

        map_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('grid_topic').value, map_qos)
        # The grid MUST share the SLAM map's exact geometry: in a non-rolling
        # costmap every StaticLayer resizes the master costmap to match ITS
        # map, so two static layers with different geometry would fight.
        self.slam_map_info = None
        self.create_subscription(
            OccupancyGrid, self.get_parameter('slam_map_topic').value,
            self.map_info_callback, map_qos)

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self.publish_memory)

        self.get_logger().info('Camera obstacle memory up: Strict 3D Voxel & FOV clearing enabled.')

    # ------------------------------------------------------------------
    def clear_callback(self, request, response):
        n = len(self.voxels)
        self.voxels.clear()
        self.get_logger().info(f'Cleared {n} remembered obstacle voxels.')
        return response

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------
    def cloud_callback(self, msg):
        # 1. DO NOT RETURN ON EMPTY! The code must continue to clear the FOV.
        
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        opt_frame = self.get_parameter('optical_frame').value
        
        try:
            # TF for the points (usually base_link) -> Map
            tr_msg = self.tf_buffer.lookup_transform(
                map_frame, msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
            # TF for Robot Pose -> Map
            tr_base = self.tf_buffer.lookup_transform(
                map_frame, base_frame, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
            # TF for Camera Optical -> Map (CRUCIAL for FOV math)
            tr_opt = self.tf_buffer.lookup_transform(
                map_frame, opt_frame, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception as e:
            # LOUD, not silent: a wrong frame name here disconnects the
            # entire memory pipeline while everything else looks healthy.
            self.get_logger().warn(
                f'TF lookup failed ({e}); cloud DROPPED. Check the '
                f'map_frame/base_frame/optical_frame parameters '
                f'(currently "{map_frame}" / "{base_frame}" / "{opt_frame}").',
                throttle_duration_sec=5.0)
            return

        # 2. Safely parse points or create empty array
        if msg.width > 0:
            n_floats = msg.point_step // 4
            pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, n_floats)[:, :3]
        else:
            pts = np.empty((0, 3), dtype=np.float32)

        rx = tr_base.transform.translation.x
        ry = tr_base.transform.translation.y
        q = tr_base.transform.rotation
        rth = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        # Matrices for projecting incoming points
        R_msg = quat_to_rot(tr_msg.transform.rotation.x, tr_msg.transform.rotation.y,
                            tr_msg.transform.rotation.z, tr_msg.transform.rotation.w)
        t_msg = np.array([[tr_msg.transform.translation.x],
                          [tr_msg.transform.translation.y],
                          [tr_msg.transform.translation.z]], dtype=np.float32)
        
        # Matrices for FOV clearing (Optical Frame)
        R_opt = quat_to_rot(tr_opt.transform.rotation.x, tr_opt.transform.rotation.y,
                            tr_opt.transform.rotation.z, tr_opt.transform.rotation.w)
        t_opt = np.array([[tr_opt.transform.translation.x],
                          [tr_opt.transform.translation.y],
                          [tr_opt.transform.translation.z]], dtype=np.float32)

        bins = np.linspace(-np.pi, np.pi, 360)
        live_depths = np.full(361, np.inf)
        
        # 1. Project live cloud to map frame
        if pts.shape[0] > 0:
            m = (R_msg @ pts.T + t_msg).T
            vs_xy = float(self.get_parameter('voxel_size_xy').value)
            vs_z = float(self.get_parameter('voxel_size_z').value)
            
            ix = np.floor(m[:, 0] / vs_xy).astype(np.int64)
            iy = np.floor(m[:, 1] / vs_xy).astype(np.int64)
            iz = np.floor(m[:, 2] / vs_z).astype(np.int64)
            
            keys = np.stack([ix, iy, iz], axis=1)
            uniq, first_idx = np.unique(keys, axis=0, return_index=True)
            live_keys = set((int(kx), int(ky), int(kz)) for kx, ky, kz in uniq)
            
            angles = np.arctan2(m[:, 1] - ry, m[:, 0] - rx)
            dists = np.hypot(m[:, 0] - rx, m[:, 1] - ry)
            indices = np.digitize(angles, bins)
            
            for idx, d in zip(indices, dists):
                if d < live_depths[idx]:
                    live_depths[idx] = d
        else:
            # Both arrays must exist and be iterable: leaving first_idx
            # unassigned here crashed the node with UnboundLocalError on
            # the FIRST empty cloud (open floor -> nothing in the band ->
            # publish_empty), killing the whole memory pipeline.
            uniq = np.empty((0, 3), dtype=np.int64)
            first_idx = np.empty(0, dtype=np.int64)
            live_keys = set()

        now = self.now_sec()
        sep = float(self.get_parameter('min_hit_separation').value)
        min_dist = float(self.get_parameter('min_hit_distance').value)
        min_angle = float(self.get_parameter('min_hit_angle').value)
        max_health = float(self.get_parameter('max_health').value)
        hit_bonus = float(self.get_parameter('hit_bonus').value)

        # 2. UPDATE VOXELS
        for (kx, ky, kz), i in zip(uniq, first_idx):
            key = (int(kx), int(ky), int(kz))
            prev = self.voxels.get(key)
            if prev is None:
                # First sighting counts like a hit (health = hit_bonus, not
                # 1): at health 1 a single miss event deleted the candidate,
                # so anything not re-detected in the very next clear cycle
                # -- i.e. most real obstacles under a jittery fit -- never
                # survived to its second confirming hit.
                self.voxels[key] = [float(m[i, 0]), float(m[i, 1]), float(m[i, 2]),
                                    now, hit_bonus, now, rx, ry, rth, 0]
            else:
                health, last_inc = prev[4], prev[5]
                last_rx, last_ry, last_rth = prev[6], prev[7], prev[8]
                dist = np.hypot(rx - last_rx, ry - last_ry)
                ang_diff = abs((rth - last_rth + np.pi) % (2 * np.pi) - np.pi)
                
                if (now - last_inc >= sep) and (dist >= min_dist or ang_diff >= min_angle):
                    health = min(max_health, health + hit_bonus)
                    last_inc = now
                    last_rx, last_ry, last_rth = rx, ry, rth

                self.voxels[key] = [float(m[i, 0]), float(m[i, 1]), float(m[i, 2]),
                                    now, health, last_inc, last_rx, last_ry, last_rth, 0]

        # 3. STRICT 3D FOV CLEARING
        if self.get_parameter('enable_fov_clearing').value:
            clear_max = float(self.get_parameter('clear_max_range').value)
            miss_penalty = int(self.get_parameter('miss_penalty').value)
            interval = float(self.get_parameter('fov_clear_interval').value)
            # Misses are time-throttled; support bonuses are not. This makes
            # the decay rate independent of camera frame rate and roughly
            # symmetric with the separated-hit confirmation cadence.
            apply_misses = (now - self.last_fov_clear) >= interval

            # USE OPTICAL FRAME INVERSE! Z is now truly forward out of the lens.
            R_inv = R_opt.T
            t_inv = -R_inv @ t_opt.flatten()

            for key, v in list(self.voxels.items()):
                if v[3] == now:
                    continue 
                
                pt_map = np.array([v[0], v[1], v[2]], dtype=np.float32)
                pt_cam = R_inv @ pt_map + t_inv
                Z = pt_cam[2] 
                
                if 0.3 < Z < clear_max:
                    X, Y = pt_cam[0], pt_cam[1]
                    
                    if abs(X / Z) < 0.7 and abs(Y / Z) < 0.7:
                        v_ang = np.arctan2(v[1] - ry, v[0] - rx)
                        v_dist = np.hypot(v[0] - rx, v[1] - ry)
                        v_idx = np.digitize(v_ang, bins)
                        
                        if live_depths[v_idx] < (v_dist - 0.3):
                            continue 
                        
                        kx, ky, kz = key
                        found = False
                        for dz in (-1, 0, 1):
                            if (kx, ky, kz + dz) in live_keys:
                                found = True
                                break
                        
                        if not found:
                            if apply_misses:
                                v[4] -= miss_penalty
                                v[9] += 1
                                if v[4] <= 0:
                                    del self.voxels[key]
                        else:
                            v[4] = min(max_health, v[4] + 1)

            if apply_misses:
                self.last_fov_clear = now

    # ------------------------------------------------------------------
    def publish_memory(self):
        now = self.now_sec()
        decay = float(self.get_parameter('decay_seconds').value)
        min_hits = int(self.get_parameter('min_hits').value)
        unconf_timeout = float(self.get_parameter('unconfirmed_timeout').value)

        def keep(v):
            confirmed = v[4] >= min_hits
            age = now - v[3]
            if confirmed:
                return decay <= 0.0 or age <= decay
            return age <= unconf_timeout

        self.voxels = {k: v for k, v in self.voxels.items() if keep(v)}

        max_points = int(self.get_parameter('max_points').value)
        if len(self.voxels) > max_points:
            keep = sorted(self.voxels.items(), key=lambda kv: kv[1][3],
                          reverse=True)[:max_points]
            self.voxels = dict(keep)
            self.get_logger().warn('Camera memory hit max_points; oldest marks dropped.')

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('map_frame').value

        confirmed = [v for v in self.voxels.values() if v[4] >= min_hits]
        # Pipeline health at a glance: total candidates vs confirmed. If
        # 'total' stays 0 while the projector reports PUB, ingestion is
        # broken (TF/QoS); if total grows but confirmed stays 0, the
        # confirmation gates (min_hits / separation / movement) never pass.
        self.get_logger().info(
            f'memory: {len(self.voxels)} voxels, {len(confirmed)} confirmed',
            throttle_duration_sec=10.0)
        if confirmed:
            xyz = np.array([[v[0], v[1], v[2]] for v in confirmed], dtype=np.float32)
        else:
            xyz = np.empty((0, 3), dtype=np.float32)
            
        self.pub.publish(make_xyz_cloud(header, xyz))

        if self.get_parameter('publish_grid').value:
            self.publish_grid(xyz, header.stamp)

    # ------------------------------------------------------------------
    def map_info_callback(self, msg):
        self.slam_map_info = msg.info

    def publish_grid(self, xyz, stamp):
        """Rasterize the confirmed voxels into an OccupancyGrid with EXACTLY
        the geometry of the latest SLAM map (see __init__ for why)."""
        info = self.slam_map_info
        if info is None:
            self.get_logger().warn(
                'No SLAM map received yet; memory grid not published.',
                throttle_duration_sec=10.0)
            return

        h, w = int(info.height), int(info.width)
        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)

        occ = np.zeros((h, w), dtype=bool)
        if xyz.shape[0] > 0:
            ix = np.floor((xyz[:, 0] - ox) / res).astype(np.int64)
            iy = np.floor((xyz[:, 1] - oy) / res).astype(np.int64)
            inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            occ[iy[inb], ix[inb]] = True
            # Dilate by one cell (8-neighborhood) so a 0.10 m voxel covers
            # its full footprint on a 0.05 m grid instead of a single cell.
            d = occ.copy()
            d[1:, :] |= occ[:-1, :]
            d[:-1, :] |= occ[1:, :]
            d[:, 1:] |= d[:, :-1].copy()
            d[:, :-1] |= d[:, 1:].copy()
            occ = d

        grid = np.zeros((h, w), dtype=np.int8)
        grid[occ] = 100

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter('map_frame').value
        msg.info = info
        msg.data = grid.reshape(-1).tolist()
        self.grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraMemory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()