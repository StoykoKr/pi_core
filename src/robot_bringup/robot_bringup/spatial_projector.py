import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import onnxruntime as ort
import tf2_ros

MODEL_W, MODEL_H = 320, 256


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float32)


def make_xyz_cloud(header, xyz):
    """Fast manual PointCloud2 construction from an (N,3) float32 array."""
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


def robust_affine(x, y, min_inlier_frac=0.5):
    """Two-pass robust least squares of y = a*x + b.

    Returns (a, b, inlier_mask, ok). Rejects the fit when the slope is
    non-positive (physically impossible: more inverse-prediction must mean
    more inverse-depth) or when too few points survive outlier rejection.
    """
    A = np.stack([x, np.ones_like(x)], axis=1)
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    a, b = float(sol[0]), float(sol[1])
    res = np.abs(a * x + b - y)
    mad = float(np.median(res))
    inl = res < max(3.0 * mad, 0.02)
    if np.count_nonzero(inl) < max(8, min_inlier_frac * x.size):
        return a, b, inl, False
    sol, *_ = np.linalg.lstsq(A[inl], y[inl], rcond=None)
    a, b = float(sol[0]), float(sol[1])
    if a <= 0.0:
        return a, b, inl, False
    return a, b, inl, True


class SpatialProjector(Node):

    def __init__(self):
        super().__init__('spatial_projector')
        self.bridge = CvBridge()

        # ---------------- Parameters ----------------
        self.declare_parameter('model_path',
                               '/robot_ws/src/robot_bringup/robot_bringup/scdepthv3.onnx')
        self.declare_parameter('optical_frame', 'camera_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('camera_vfov_deg', 90.0)


        self.declare_parameter("ignore_bottom_rows_percent", 20)
        self.declare_parameter("ignore_side_cols_percent", 12)

        
        self.declare_parameter('ground_rej_rel', 0.20)

        self.declare_parameter('camera_height', 1.3)

        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('max_range', 3.0)

        self.declare_parameter('clearing_stride', 8)
        # Clearing rays must stop SHORT of the first believed surface so a
        # slightly-noisy depth estimate can never raytrace straight through
        # a real obstacle and erase its mark.
        self.declare_parameter('clearing_endpoint_margin', 0.15)
        self.clear_pub = self.create_publisher(PointCloud2, '/camera/clearing_cloud', 10)

        # Vertical-structure rescue for ground rejection. A pixel whose depth
        # is nearly identical to the pixel `vert_check_rows` above or below it
        # lies on a near-vertical surface (wall, leg, box face, table edge).
        # Floor -- even affine-warped floor -- changes depth along a column by
        # roughly N/dv relative per N rows, so it can never qualify. Such
        # pixels are exempt from ground rejection no matter how ground-like
        # their absolute depth looks.
        self.declare_parameter('vert_check_rows', 12)
        self.declare_parameter('vert_rel_tol', 0.05)

        self.declare_parameter('fit_alpha', 0.3)         # smoothing of (a, b)
        self.declare_parameter('min_lidar_pairs', 30)    # gate: enough beams in view
        self.declare_parameter('min_x_spread_rel', 0.15) # gate: identifiability of b
        self.declare_parameter('fit_range_min', 0.5)     # lidar ranges usable for fit
        self.declare_parameter('fit_range_max', 8.0)
        
        self.declare_parameter('residual_gate', 0.20)
        # Looser gate for CONTINUING to publish with the already-smoothed
        # model when the per-frame candidate fit is degenerate or gated.
        self.declare_parameter('publish_residual_gate', 0.35)
        # EMA step when the running model already fits this frame's pairs
        # (damped), vs fit_alpha when it has genuinely stopped fitting.
        self.declare_parameter('fit_alpha_stable', 0.10)
        self.declare_parameter('max_fit_age_frames', 15)

        self.declare_parameter('near_lockout_range', 0.6)

        self.declare_parameter('enable_floor_fallback', True)

        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('cloud_stride', 2)
        self.declare_parameter('band_z_min', 0.20)
        self.declare_parameter('band_z_max', 1.70)

        self.optical_frame = self.get_parameter('optical_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        # ---------------- Alignment state ----------------
        self.a = None                 # slope  in 1/Z = a*(1/pred) + b
        self.b = None                 # shift
        self.fit_initialized = False
        self.fit_source = 'NONE'
        self.frames_since_fit = 10 ** 6
        self.last_residual = float('nan')

        self.latest_scan = None
        self.camera_info_in = None
        self.height_from_tf_done = False
        self.node_launch_time = self.get_clock().now()

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- I/O ----------------
        self.depth_pub = self.create_publisher(Image, '/camera/depth', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/obstacle_cloud', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug_overlay', 10)

        self.create_subscription(Image, '/image_raw', self.image_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera_info_in', self.info_in_callback,
                                 qos_profile_sensor_data)

        # ---------------- Model ----------------
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        model_path = self.get_parameter('model_path').value
        self.session = ort.InferenceSession(model_path, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info('SC-DepthV3 ONNX model loaded (affine alignment mode).')

    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        self.latest_scan = msg

    def info_in_callback(self, msg):
        self.camera_info_in = msg

    # ------------------------------------------------------------------
    def model_intrinsics(self):
        """Intrinsics at MODEL resolution: from real CameraInfo if available,
        otherwise derived from the vfov parameter."""
        if self.camera_info_in is not None:
            ci = self.camera_info_in
            sx = MODEL_W / float(ci.width)
            sy = MODEL_H / float(ci.height)
            fx = ci.k[0] * sx
            fy = ci.k[4] * sy
            cx = ci.k[2] * sx
            cy = ci.k[5] * sy
        else:
            vfov = np.radians(float(self.get_parameter('camera_vfov_deg').value))
            fy = MODEL_H / (2.0 * np.tan(vfov / 2.0))
            fx = fy
            cx = MODEL_W / 2.0
            cy = MODEL_H / 2.0
        return float(fx), float(fy), float(cx), float(cy)

    def update_height_from_tf(self):
        if self.height_from_tf_done:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            z = float(t.transform.translation.z)
            if z > 0.2:
                if abs(z - self.camera_height) > 0.02:
                    self.get_logger().warn(
                        f'camera_height param {self.camera_height:.2f} m disagrees with '
                        f'TF ({z:.2f} m). Using TF value.')
                self.camera_height = z
                self.height_from_tf_done = True
        except Exception:
            self.get_logger().warn(
                f'No TF {self.base_frame} -> {self.optical_frame} yet; the camera '
                f'contributes nothing until the chain exists.',
                throttle_duration_sec=10.0)

    # ------------------------------------------------------------------
    def project_lidar(self, pred, fx, fy, cx, cy):
        """Project the latest scan into the model-resolution image.
        Returns (u, v, Z, pred_at_uv) arrays for in-view points, or None."""
        scan = self.latest_scan
        if scan is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform(
                self.optical_frame,              # target
                scan.header.frame_id,            # source
                scan.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.optical_frame, scan.header.frame_id, rclpy.time.Time())
            except Exception:
                self.get_logger().warn(
                    f'TF {scan.header.frame_id} -> {self.optical_frame} unavailable: '
                    f'alignment is DISABLED until this resolves.',
                    throttle_duration_sec=10.0)
                return None

        R = quat_to_rot(tr.transform.rotation.x, tr.transform.rotation.y,
                        tr.transform.rotation.z, tr.transform.rotation.w)
        t = np.array([[tr.transform.translation.x],
                      [tr.transform.translation.y],
                      [tr.transform.translation.z]], dtype=np.float32)

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        ok = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        if not np.any(ok):
            return None
        r, a = ranges[ok], angles[ok]

        pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=0)
        cam = R @ pts + t
        X, Y, Z = cam[0], cam[1], cam[2]

        front = Z > 0.05
        X, Y, Z = X[front], Y[front], Z[front]
        if Z.size == 0:
            return None

        u = np.round(fx * X / Z + cx).astype(np.int32)
        v = np.round(fy * Y / Z + cy).astype(np.int32)
        inview = (u >= 0) & (u < MODEL_W) & (v >= 0) & (v < MODEL_H)
        if not np.any(inview):
            return None
        u, v, Z = u[inview], v[inview], Z[inview]
        return u, v, Z, pred[v, u]

    # ------------------------------------------------------------------
    def model_residual(self, a, b, x, y):
        """Median relative depth error of model (a, b) over pairs (x, y),
        or None if the model puts too many pairs at non-positive depth."""
        inv_hat = a * x + b
        pos = inv_hat > 1e-6
        if np.count_nonzero(pos) < 0.5 * x.size:
            return None
        z_hat = 1.0 / inv_hat[pos]
        z_true = 1.0 / y[pos]
        return float(np.median(np.abs(z_hat - z_true) / z_true))

    # ------------------------------------------------------------------
    def estimate_alignment(self, proj, pred, fy, cy):
        """Per-frame scale-and-shift estimation with two-tier gating.

        Tier 1 (residual_gate, strict): a candidate fit passing this may
        UPDATE the smoothed model.
        Tier 2 (publish_residual_gate, loose): if the candidate fails but
        the current smoothed model still explains this frame's pairs to
        within the loose gate, we keep PUBLISHING with the smoothed model.

        Rationale: gate failures cluster exactly on the hard scenes (wire
        racks, overhangs) where marking matters most. A strict
        publish-or-nothing gate therefore starved the memory of the very
        obstacles it exists for. The loose publish tier keeps depth flowing
        there, and the memory node's multi-view confirmation absorbs the
        extra noise. False negatives cause crashes; false positives cost a
        detour.

        Returns a dict: {published_ok, n_pairs, n_inl, med_rel, inlier_mask}.
        Side effects: updates (self.a, self.b) only on tier-1 frames.
        """
        info = {'published_ok': False, 'n_pairs': 0, 'n_inl': 0,
                'med_rel': float('nan'), 'inlier_mask': None}
        gate = float(self.get_parameter('residual_gate').value)
        pub_gate = float(self.get_parameter('publish_residual_gate').value)

        # ---------------- pair collection --------------------------------
        x = y = None
        src = 'NONE'
        if proj is not None:
            u, v, Z, p = proj
            fit_min = float(self.get_parameter('fit_range_min').value)
            fit_max = float(self.get_parameter('fit_range_max').value)
            valid = (p > 1e-4) & np.isfinite(p) & (Z >= fit_min) & (Z <= fit_max)
            info['n_pairs'] = int(np.count_nonzero(valid))
            if info['n_pairs'] >= int(self.get_parameter('min_lidar_pairs').value):
                x = 1.0 / p[valid].astype(np.float64)   # inverse prediction
                y = 1.0 / Z[valid].astype(np.float64)   # inverse metric (lidar)
                src = 'LIDAR'

        # Floor strip: previously only a fallback, now ALSO an augmentation.
        # Lidar pairs come from one height slice of whatever happens to be
        # in view; against wire racks they are sparse and self-contradictory
        # (beams pass between wires, depth net smooths across them), which
        # is what made (a, b) swing 0.5 <-> 23 between frames. The floor
        # strip is a scene-independent metric anchor (known camera height)
        # with large x-spread, so joining it stabilizes b enormously. Its
        # own coherence check already refuses when an obstacle sits in the
        # ROI.
        if self.get_parameter('enable_floor_fallback').value:
            xy = self.floor_pairs(pred, fy, cy)
            if xy is not None:
                fx_p, fy_p = xy
                if x is None:
                    x, y, src = fx_p, fy_p, 'FLOOR'
                else:
                    # Cap floor pairs at 2x the lidar count so the (dense)
                    # strip anchors the fit without drowning the lidar.
                    k = max(50, 2 * x.size)
                    if fx_p.size > k:
                        sel = np.random.default_rng(0).choice(
                            fx_p.size, k, replace=False)
                        fx_p, fy_p = fx_p[sel], fy_p[sel]
                    x = np.concatenate([x, fx_p])
                    y = np.concatenate([y, fy_p])
                    src = 'LIDAR+FLR'

        if x is None:
            self.fit_source = 'STALE' if self.fit_initialized else 'NONE'
            self.frames_since_fit += 1
            return info

        # ---------------- candidate fit ----------------------------------
        # Identifiability of the shift: need spread in x. A single frontal
        # wall face gives near-constant x -> fit only the slope, keep b.
        cand = None   # (a_fit, b_fit, inlier_mask, tag)
        spread = np.percentile(x, 80) - np.percentile(x, 20)
        if spread < float(self.get_parameter('min_x_spread_rel').value) * np.median(x):
            if self.fit_initialized:
                ratios = (y - self.b) / x
                a_new = float(np.median(ratios))
                if a_new > 0:
                    cand = (a_new, self.b, np.ones(x.shape, bool), src + '-SLOPE')
        else:
            a_fit, b_fit, inl, ok = robust_affine(x, y)
            if ok:
                cand = (a_fit, b_fit, inl, src)

        # ---------------- residuals of candidate AND running model -------
        # b is deliberately never clamped: SC-DepthV3 output is affine-
        # invariant and a negative shift is legitimate; the gates validate
        # exactly the model that will be published.
        res_cand = None
        if cand is not None:
            res_cand = self.model_residual(cand[0], cand[1], x, y)
            info['n_inl'] = int(np.count_nonzero(cand[2]))
            info['inlier_mask'] = cand[2]
        res_ema = None
        if self.fit_initialized:
            res_ema = self.model_residual(self.a, self.b, x, y)

        # ---------------- tier 1: update the smoothed model --------------
        if cand is not None and res_cand is not None and res_cand <= gate:
            a_fit, b_fit, _, tag = cand
            if not self.fit_initialized:
                self.a, self.b = a_fit, b_fit
                self.fit_initialized = True
                self.get_logger().info(
                    f'Alignment initialized: a={self.a:.4g} b={self.b:.4g} ({tag})')
            else:
                # Damping: if the RUNNING model also explains this frame,
                # there is no evidence the world changed -- move only
                # gently toward the (possibly degenerate) candidate. Full
                # alpha is reserved for frames where the running model has
                # genuinely stopped fitting. This kills the a=0.5 <-> a=23
                # lurching that scattered marks across voxels and defeated
                # the memory's hit confirmation.
                if res_ema is not None and res_ema <= gate:
                    alpha = float(self.get_parameter('fit_alpha_stable').value)
                else:
                    alpha = float(self.get_parameter('fit_alpha').value)
                self.a = alpha * a_fit + (1 - alpha) * self.a
                self.b = alpha * b_fit + (1 - alpha) * self.b
            self.fit_source = tag
            self.frames_since_fit = 0
            self.last_residual = res_cand
            info['med_rel'] = res_cand
            info['published_ok'] = True
            return info

        # ---------------- tier 2: publish with the running model ---------
        if res_ema is not None and res_ema <= pub_gate:
            self.fit_source = src + '-EMA'
            self.frames_since_fit = 0        # model re-validated this frame
            self.last_residual = res_ema
            info['med_rel'] = res_ema
            info['published_ok'] = True
            return info

        # ---------------- hold -------------------------------------------
        info['med_rel'] = res_cand if res_cand is not None else (
            res_ema if res_ema is not None else float('nan'))
        self.fit_source = (f'{src}-GATED' if cand is not None
                           else ('STALE' if self.fit_initialized else 'NONE'))
        self.frames_since_fit += 1
        return info

    # ------------------------------------------------------------------
    def crop_limits(self):
        """Row/column limits of the trusted central region of the frame."""
        ig_rows = float(self.get_parameter('ignore_bottom_rows_percent').value)
        ig_cols = float(self.get_parameter('ignore_side_cols_percent').value)
        row_cut = MODEL_H * (1.0 - ig_rows / 100.0)
        col_lo = MODEL_W * (ig_cols / 100.0)
        col_hi = MODEL_W * (1.0 - ig_cols / 100.0)
        return row_cut, col_lo, col_hi

    def floor_roi(self):
        """Floor-strip ROI for the alignment fallback: a 20-row band placed
        just ABOVE the distorted bottom crop. The old hardcoded rows
        (225-245) sat INSIDE the distorted zone, so the fallback was fitting
        the metric alignment on exactly the pixels we cannot trust."""
        row_cut, col_lo, col_hi = self.crop_limits()
        row1 = int(min(row_cut, MODEL_H - 5))
        row0 = max(row1 - 20, 0)
        col0 = int(max(80, col_lo))
        col1 = int(min(240, col_hi))
        return row0, row1, col0, col1

    def floor_pairs(self, pred, fy, cy):
        """Fallback pair source when no lidar is in view: the floor strip.
        Returns (x, y) in inverse space, or None if the strip does not
        behave like a ground plane at the known camera height."""
        row0, row1, col0, col1 = self.floor_roi()
        rows = np.arange(row0, row1)
        dv = rows - cy
        if np.any(dv < 5):
            return None
        true_z = (self.camera_height * fy) / dv               # (rows,)
        block = pred[row0:row1, col0:col1]
        valid = block > 1e-4
        if np.count_nonzero(valid) < 30:
            return None
        tz = np.broadcast_to(true_z[:, None], block.shape)
        x = (1.0 / block[valid]).astype(np.float64)
        y = (1.0 / tz[valid]).astype(np.float64)
        # Consistency: candidates true/pred must be coherent, else an obstacle
        # occupies the ROI and the "floor" assumption is false.
        cand = (tz[valid] / block[valid])
        med = float(np.median(cand))
        if float(np.median(np.abs(cand - med))) / max(med, 1e-9) > 0.25:
            return None
        return x, y

    # ------------------------------------------------------------------
    def image_callback(self, rgb_msg):
        if (self.get_clock().now() - self.node_launch_time).nanoseconds / 1e9 < 3.0:
            return

        self.update_height_from_tf()

        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        native_h, native_w = cv_image.shape[:2]

        # ---------- inference at model resolution ----------
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        inp = cv2.resize(rgb, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        inp = inp.astype(np.float32) / 255.0
        inp = (inp - np.array([0.485, 0.456, 0.406], np.float32)) \
              / np.array([0.229, 0.224, 0.225], np.float32)
        inp = np.expand_dims(inp.transpose(2, 0, 1), 0)
        pred = np.squeeze(self.session.run(None, {self.input_name: inp})[0]).astype(np.float32)

        fx, fy, cx, cy = self.model_intrinsics()

        # ---------- alignment ----------
        proj = self.project_lidar(pred, fx, fy, cx, cy)
        fit_info = self.estimate_alignment(proj, pred, fy, cy)

        # ---------- near-field lockout ----------
        locked_out = False
        med_range = None
        if proj is not None and proj[2].size >= 5:
            med_range = float(np.median(proj[2]))
            if med_range < float(self.get_parameter('near_lockout_range').value):
                locked_out = True

        # ---------- publish decision ----------
        fit_usable = (self.fit_initialized and
                      self.frames_since_fit <=
                      int(self.get_parameter('max_fit_age_frames').value))
        do_publish = fit_usable and fit_info['published_ok'] and not locked_out
        # A stale-but-recent fit without fresh confirmation is still usable
        # as long as this frame was not explicitly gated as inconsistent:
        if (fit_usable and not locked_out and not fit_info['published_ok']
                and 'GATED' not in self.fit_source and fit_info['n_pairs'] == 0):
            do_publish = True

        # ---------- status log ----------
        self.get_logger().info(
            f"align: a={self.a if self.a is None else round(self.a, 4)} "
            f"b={self.b if self.b is None else round(self.b, 4)} "
            f"src={self.fit_source} pairs={fit_info['n_pairs']} "
            f"inl={fit_info['n_inl']} res={fit_info['med_rel']:.3f} "
            f"age={self.frames_since_fit} lockout={locked_out} publish={do_publish}",
            throttle_duration_sec=5.0)

        # ---------- debug overlay (always published) ----------
        debug = cv_image.copy()
        sw, sh = native_w / MODEL_W, native_h / MODEL_H
        if proj is not None:
            u, v = proj[0], proj[1]
            inl = fit_info['inlier_mask']
            for i in range(u.size):
                color = (0, 0, 255)
                if inl is not None and i < inl.size and inl[i]:
                    color = (0, 255, 0)
                cv2.circle(debug, (int(u[i] * sw), int(v[i] * sh)), 2, color, -1)
        fr0, fr1, fc0, fc1 = self.floor_roi()
        cv2.rectangle(debug, (int(fc0 * sw), int(fr0 * sh)),
                      (int(fc1 * sw), int(fr1 * sh)), (0, 255, 0), 2)
   
        row_cut, col_lo, col_hi = self.crop_limits()
        cv2.rectangle(debug, (int(col_lo * sw), 0),
                      (int(col_hi * sw), int(row_cut * sh)), (255, 0, 255), 1)
        a_txt = '-' if self.a is None else f'{self.a:.3f}'
        b_txt = '-' if self.b is None else f'{self.b:.3f}'
        status = (f'a={a_txt} b={b_txt} src={self.fit_source} '
                  f'res={fit_info["med_rel"]:.2f} '
                  f'{"LOCKOUT " if locked_out else ""}'
                  f'{"PUB" if do_publish else "HOLD"}')
        cv2.putText(debug, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
        dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        dbg_msg.header = rgb_msg.header
        dbg_msg.header.frame_id = self.optical_frame
        self.debug_pub.publish(dbg_msg)

        if not do_publish:
            # Publish NOTHING on hold. camera_memory treats an empty cloud
            # as "the camera looked and saw nothing here" and decays its
            # voxels accordingly -- an empty heartbeat during a fit dropout
            # would erode real memory. Empty clouds are only sent when the
            # depth was trustworthy and the view genuinely had no obstacles
            # (see publish_obstacle_cloud). No costmap layer consumes this
            # topic directly anymore, so no heartbeat is required.
            return

        # ---------- metric reconstruction: 1 / (a/pred + b) ----------
        # Use self.b UNCLAMPED: the residual gate validated this exact model.
        inv = self.a / np.maximum(pred, 1e-6) + self.b

        min_inv = 1.0 / self.max_range
        # far_mask: the model itself says "beyond max_range". These pixels
        # must become NaN in the MARKING depth (never snap them onto a
        # phantom shell at exactly max_range -- that shell is what produced
        # the curved point curtains) but they are exactly the pixels that
        # justify clearing all the way out to max_range.
        far_mask = inv <= min_inv
        with np.errstate(divide='ignore', invalid='ignore'):
            depth = np.where(inv > min_inv, 1.0 / inv, np.nan).astype(np.float32)
        depth[depth < self.min_range] = np.nan

        # ---------- clearing depth (separate product, separate rules) ----
        # finite surface  -> ray ends `margin` short of the surface
        # model says far  -> ray runs the full max_range
        # anything else (too close, invalid prediction) -> NO clearing ray:
        # "I don't know" must never erase a mark.
        margin = float(self.get_parameter('clearing_endpoint_margin').value)
        clear_depth = np.full(depth.shape, np.nan, dtype=np.float32)
        clear_depth[far_mask] = self.max_range
        finite = np.isfinite(depth)
        clear_depth[finite] = np.maximum(depth[finite] - margin, 0.05)

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header.stamp = rgb_msg.header.stamp
        depth_msg.header.frame_id = self.optical_frame
        self.depth_pub.publish(depth_msg)

        info = CameraInfo()
        info.header.stamp = rgb_msg.header.stamp
        info.header.frame_id = self.optical_frame
        info.width, info.height = MODEL_W, MODEL_H
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

        if self.get_parameter('publish_cloud').value:
            self.publish_obstacle_cloud(depth, clear_depth, fx, fy, cx, cy,
                                        rgb_msg.header.stamp)

    # ------------------------------------------------------------------
    def vertical_structure_mask(self, depth):
        """Boolean (H, W): True where the pixel lies on a near-vertical
        surface.

        Test: the depth `n` rows above or below is nearly identical
        (relative difference < vert_rel_tol). Any horizontal plane -- floor
        at height 0 or a warped version of it -- changes depth along an
        image column by ~n/dv relative over n rows (>= ~8% for n=12 inside
        the trusted crop), so it can never pass a 5% tolerance. Walls,
        table legs/edges and box faces pass trivially. The test is purely
        LOCAL and therefore invariant to affine error and to slow warp,
        which is exactly the mono-depth failure mode.
        """
        n = int(self.get_parameter('vert_check_rows').value)
        tol = float(self.get_parameter('vert_rel_tol').value)
        m = np.zeros(depth.shape, dtype=bool)
        # NaN comparisons are False, so invalid pixels never qualify.
        m[n:, :] = np.abs(depth[n:, :] - depth[:-n, :]) < tol * depth[n:, :]
        m[:-n, :] |= np.abs(depth[:-n, :] - depth[n:, :]) < tol * depth[:-n, :]
        return m

    # ------------------------------------------------------------------
    def publish_obstacle_cloud(self, depth, clear_depth, fx, fy, cx, cy, stamp):
        """Metric depth -> points -> base frame -> height-band crop."""
        header = Header()
        header.stamp = stamp
        header.frame_id = self.base_frame

        def publish_empty():
            self.cloud_pub.publish(make_xyz_cloud(header, np.empty((0, 3), dtype=np.float32)))

        try:
            tr = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.base_frame, self.optical_frame, rclpy.time.Time())
            except Exception:
                # TF hiccup = we cannot vouch for the view. Stay silent
                # rather than telling camera_memory the space is empty.
                return

        s = int(self.get_parameter('cloud_stride').value)
        Zs = depth[::s, ::s]
        vs, us = np.mgrid[0:MODEL_H:s, 0:MODEL_W:s]
        row_cut, col_lo, col_hi = self.crop_limits()

        # ==================== CLEARING CLOUD GENERATOR ====================
        # clear_depth semantics (built in image_callback):
        #   finite surface -> endpoint short of the surface
        #   model-far      -> endpoint at max_range
        #   unknown/close  -> NaN, DROPPED here: no ray, no clearing.
        # Restricted to the same trusted crop as marking: distorted border
        # pixels must not raytrace through valid marks either.
        cs = int(self.get_parameter('clearing_stride').value)
        Z_c = clear_depth[::cs, ::cs]
        v_c, u_c = np.mgrid[0:MODEL_H:cs, 0:MODEL_W:cs]
        keep_c = (np.isfinite(Z_c)
                  & (v_c < row_cut)
                  & (u_c >= col_lo) & (u_c < col_hi))
        if np.any(keep_c):
            Z_cf = Z_c[keep_c].astype(np.float32)
            u_cf = u_c[keep_c].astype(np.float32)
            v_cf = v_c[keep_c].astype(np.float32)
            X_cf = (u_cf - cx) * Z_cf / fx
            Y_cf = (v_cf - cy) * Z_cf / fy
            pts_opt_clear = np.stack([X_cf, Y_cf, Z_cf], axis=1)
        else:
            pts_opt_clear = np.empty((0, 3), dtype=np.float32)
        self.clear_pub.publish(make_xyz_cloud(
            Header(stamp=stamp, frame_id=self.optical_frame), pts_opt_clear))
        # =================================================================

        good = (np.isfinite(Zs)
                & (vs < row_cut)
                & (us >= col_lo) & (us < col_hi))

        ground_rel = float(self.get_parameter('ground_rej_rel').value)
        if ground_rel > 0.0:
            dv = vs - cy
            below_horizon = dv > 2.0
            with np.errstate(divide='ignore', invalid='ignore'):
                z_ground = (self.camera_height * fy) / np.where(below_horizon, dv, np.nan)

                max_dv = MODEL_H - cy
                # Multiplier softened 2.0 -> 1.0. Rationale: a tabletop at
                # ~0.75 m under a 1.3 m camera sits at ~0.42x the floor
                # depth, i.e. 58% away from z_ground. The old multiplier
                # reached 3x near the bottom, so ground_rel >= 0.20 could
                # swallow real overhang surfaces there. 1.0 caps the
                # tolerance at 2x ground_rel inside the trusted crop.
                dynamic_multiplier = 1.0 + 1.0 * (dv / max_dv)

                dynamic_tolerance = ground_rel * dynamic_multiplier
                ground_like = below_horizon & \
                    (np.abs(Zs - z_ground) < dynamic_tolerance * z_ground)
            # RESCUE: pixels on near-vertical structures are never "ground",
            # regardless of how ground-like their absolute depth looks. This
            # protects walls, legs and table edges near the floor line while
            # the tolerance above stays wide enough to eat residual warp.
            vert = self.vertical_structure_mask(depth)[::s, ::s]
            ground_like &= ~vert
            good &= ~ground_like
        if not np.any(good):
            publish_empty()  # view is entirely out of valid range
            return
        Z = Zs[good].astype(np.float32)
        u = us[good].astype(np.float32)
        v = vs[good].astype(np.float32)

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        pts_opt = np.stack([X, Y, Z], axis=0)

        R = quat_to_rot(tr.transform.rotation.x, tr.transform.rotation.y,
                        tr.transform.rotation.z, tr.transform.rotation.w)
        t = np.array([[tr.transform.translation.x],
                      [tr.transform.translation.y],
                      [tr.transform.translation.z]], dtype=np.float32)
        pts_base = (R @ pts_opt + t).T

        z = pts_base[:, 2]
        horiz = np.hypot(pts_base[:, 0], pts_base[:, 1])
        
        band = (z >= float(self.get_parameter('band_z_min').value)) & \
               (z <= float(self.get_parameter('band_z_max').value)) & \
               (horiz <= self.max_range)
        pts_base = pts_base[band]

        if pts_base.shape[0] == 0:
            publish_empty()  # no points survived the height/distance crop
            return

        self.cloud_pub.publish(make_xyz_cloud(header, pts_base))


def main(args=None):
    rclpy.init(args=args)
    node = SpatialProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()