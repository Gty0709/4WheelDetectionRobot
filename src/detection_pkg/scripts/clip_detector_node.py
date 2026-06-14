#!/usr/bin/env python3
"""Real-time YOLO clip detection with stereo back-projection to map waypoints."""

from __future__ import annotations

import os
import sys

# NumPy 1.x 隔离层（由 clip_detector_wrapper.sh 设置），必须在其它 import 之前。
_target = os.environ.get('DETECTION_PYTHON_TARGET', '')
if _target and _target not in sys.path:
    sys.path.insert(0, _target)

import math
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from detection_pkg.backprojection import (
    CAMERA_OPTICAL_FIX,
    DetectionBox,
    MapWaypoint,
    camera_matrix_from_info,
    deduplicate_waypoints,
    max_disparity_px,
    nms_detection_boxes,
    make_transform,
    match_stereo_detections,
    pixel_to_ray_optical,
    ray_ground_intersection,
    triangulate_stereo,
    validate_map_point,
    validate_stereo_point,
)
from detection_pkg.embedding_reid import WaypointReID, crop_detection_patch
from detection_pkg.motion_compensation import (
    OdomRingBuffer,
    compose_map_camera_tf,
    stamp_to_ns,
)
from detection_pkg.waypoint_track import (
    Observation,
    WaypointTrack,
    observation_from_detection,
)

# #region agent log
_AGENT_DEBUG_LOG = '/home/gty/ros2ws/robothomework20260613/.cursor/debug-373f49.log'
_AGENT_DEBUG_SESSION = '373f49'


def _agent_log(hypothesis_id: str, location: str, message: str, data: dict | None = None,
               run_id: str = 'pre-fix') -> None:
    import json
    try:
        with open(_AGENT_DEBUG_LOG, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps({
                'sessionId': _AGENT_DEBUG_SESSION,
                'runId': run_id,
                'hypothesisId': hypothesis_id,
                'location': location,
                'message': message,
                'data': data or {},
                'timestamp': int(time.time() * 1000),
            }, ensure_ascii=False) + '\n')
    except Exception:
        pass
# #endregion


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Parse sensor_msgs/Image without cv_bridge (NumPy 2.x safe)."""
    height, width = msg.height, msg.width
    encoding = msg.encoding.lower()
    if encoding == 'rgb8':
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if encoding == 'bgr8':
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3).copy()
    if encoding == 'rgba8':
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    if encoding == 'mono8':
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'Unsupported image encoding: {msg.encoding}')


def bgr_to_image_msg(bgr: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = msg.width * 3
    msg.data = bgr.tobytes()
    return msg


def _load_yolo(weights_path: str):
    """Import ultralytics after ROS stacks; append ~/.local only for YOLO/torch."""
    import site
    user_site = site.getusersitepackages()
    if user_site and user_site not in sys.path:
        sys.path.append(user_site)
    from ultralytics import YOLO
    return YOLO(weights_path)


def _resolve_weights(param: str) -> Path:
    if param:
        p = Path(param).expanduser()
        if p.is_file():
            return p.resolve()
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('detection_pkg'))
        for candidate in (share / 'simmodel' / 'best.pt', share / 'yolo26m.pt'):
            if candidate.is_file():
                return candidate.resolve()
    except Exception:
        pass
    local = Path(__file__).resolve().parents[1] / 'simmodel' / 'best.pt'
    if local.is_file():
        return local.resolve()
    raise FileNotFoundError('YOLO weights not found; set parameter weights')


def _tf_to_matrix(translation, rotation) -> np.ndarray:
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    rot = Rotation.from_quat(quat).as_matrix()
    trans = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
    return make_transform(rot, trans)


class ClipDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('clip_detector')

        self.declare_parameter('session_dir', '')
        self.declare_parameter('weights', '')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('max_fps', 8.0)
        self.declare_parameter('ground_z', 0.002)
        self.declare_parameter('dedup_radius_m', 0.45)
        self.declare_parameter('save_dedup_radius_m', 0.65)
        self.declare_parameter('bearing_merge_deg', 6.0)
        self.declare_parameter('merge_max_blend', 0.2)
        self.declare_parameter('outlier_reject_m', 0.5)
        self.declare_parameter('max_angular_for_wp', 0.12)
        self.declare_parameter('min_linear_for_wp', 0.04)
        self.declare_parameter('max_linear_for_wp', 0.20)
        self.declare_parameter('motion_settle_sec', 0.4)
        self.declare_parameter('enable_motion_gate', False)
        self.declare_parameter('use_odom_motion_gate', False)
        self.declare_parameter('max_odom_angular_for_wp', 0.15)
        self.declare_parameter('tf_prefer_image_stamp', True)
        self.declare_parameter('stereo_sync_slop_sec', 0.12)
        self.declare_parameter('allow_ground_fallback', True)
        self.declare_parameter('stereo_ground_crosscheck_m', 0.6)
        self.declare_parameter('max_tf_lag_ms', 800)
        self.declare_parameter('enable_motion_compensation', True)
        self.declare_parameter('odom_buffer_duration_sec', 2.0)
        self.declare_parameter('odom_interp_max_gap_ms', 80)
        self.declare_parameter('max_tf_extrapolate_ms', 120)
        self.declare_parameter('waypoint_save_interval_sec', 1.0)
        self.declare_parameter('marker_scale_m', 0.08)
        self.declare_parameter('enable_reid', True)
        self.declare_parameter('reid_backend', 'osnet')
        self.declare_parameter('reid_cosine_min', 0.42)
        self.declare_parameter('reid_vocab_size', 48)
        self.declare_parameter('reid_match_score_min', 0.42)
        self.declare_parameter('track_buffer_size', 30)
        self.declare_parameter('sigma_floor_m', 0.08)
        self.declare_parameter('pixel_merge_u_px', 32.0)
        self.declare_parameter('pixel_merge_v_px', 24.0)
        self.declare_parameter('track_uv_ema', 0.15)
        self.declare_parameter('track_map_ema', 1.0)
        self.declare_parameter('stereo_baseline_m', 0.0755)
        self.declare_parameter('min_depth_m', 0.6)
        self.declare_parameter('max_depth_m', 7.0)
        self.declare_parameter('min_disparity_px', 2.0)
        self.declare_parameter('max_ground_z_error', 0.15)
        self.declare_parameter('max_map_radius_m', 15.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        session = self.get_parameter('session_dir').get_parameter_value().string_value.strip()
        if not session:
            session = os.environ.get('PERCEPTION_MAP_SESSION', '')
        self._session_dir = Path(session).expanduser().resolve() if session else None

        self._conf = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self._max_fps = self.get_parameter('max_fps').get_parameter_value().double_value
        self._ground_z = self.get_parameter('ground_z').get_parameter_value().double_value
        self._dedup_radius = self.get_parameter('dedup_radius_m').get_parameter_value().double_value
        self._save_dedup_radius = max(
            self.get_parameter('save_dedup_radius_m').get_parameter_value().double_value,
            self._dedup_radius,
        )
        self._bearing_merge_deg = max(
            self.get_parameter('bearing_merge_deg').get_parameter_value().double_value, 1.0)
        self._merge_max_blend = min(
            self.get_parameter('merge_max_blend').get_parameter_value().double_value, 1.0)
        self._outlier_reject_m = max(
            self.get_parameter('outlier_reject_m').get_parameter_value().double_value, 0.1)
        self._max_angular_for_wp = abs(
            self.get_parameter('max_angular_for_wp').get_parameter_value().double_value)
        self._min_linear_for_wp = max(
            self.get_parameter('min_linear_for_wp').get_parameter_value().double_value, 0.0)
        self._max_linear_for_wp = max(
            self.get_parameter('max_linear_for_wp').get_parameter_value().double_value, 0.05)
        self._motion_settle_sec = max(
            self.get_parameter('motion_settle_sec').get_parameter_value().double_value, 0.0)
        self._enable_motion_gate = self.get_parameter(
            'enable_motion_gate').get_parameter_value().bool_value
        self._use_odom_motion_gate = self.get_parameter(
            'use_odom_motion_gate').get_parameter_value().bool_value
        self._max_odom_angular_for_wp = max(
            self.get_parameter('max_odom_angular_for_wp').get_parameter_value().double_value, 0.05)
        self._tf_prefer_image_stamp = self.get_parameter(
            'tf_prefer_image_stamp').get_parameter_value().bool_value
        self._stereo_sync_slop = max(
            self.get_parameter('stereo_sync_slop_sec').get_parameter_value().double_value, 0.01)
        self._allow_ground_fallback = self.get_parameter(
            'allow_ground_fallback').get_parameter_value().bool_value
        self._stereo_ground_crosscheck_m = max(
            self.get_parameter('stereo_ground_crosscheck_m').get_parameter_value().double_value,
            0.1,
        )
        self._max_tf_lag_ms = max(
            int(self.get_parameter('max_tf_lag_ms').get_parameter_value().integer_value), 100)
        self._enable_motion_compensation = self.get_parameter(
            'enable_motion_compensation').get_parameter_value().bool_value
        odom_buffer_sec = max(
            self.get_parameter('odom_buffer_duration_sec').get_parameter_value().double_value,
            0.5,
        )
        odom_interp_gap_ms = max(
            int(self.get_parameter('odom_interp_max_gap_ms').get_parameter_value().integer_value),
            10,
        )
        max_extrapolate_ms = max(
            int(self.get_parameter('max_tf_extrapolate_ms').get_parameter_value().integer_value),
            0,
        )
        self._odom_buffer: Optional[OdomRingBuffer] = None
        if self._enable_motion_compensation:
            self._odom_buffer = OdomRingBuffer(
                duration_sec=odom_buffer_sec,
                interp_max_gap_ns=odom_interp_gap_ms * 1_000_000,
                max_extrapolate_ns=max_extrapolate_ms * 1_000_000,
            )
        self._t_base_cam_cache: dict[str, np.ndarray] = {}
        self._last_pipeline_delay_ms = 0
        self._last_odom_extrapolate_ms = 0
        self._last_tf_lag_ms = 0
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._teleop_linear = 0.0
        self._teleop_angular = 0.0
        self._odom_omega = 0.0
        self._odom_linear = 0.0
        self._last_fast_motion_mono = 0.0
        self._waypoint_save_interval = max(
            self.get_parameter('waypoint_save_interval_sec').get_parameter_value().double_value,
            0.5,
        )
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._marker_scale = self.get_parameter('marker_scale_m').get_parameter_value().double_value
        self._enable_reid = self.get_parameter('enable_reid').get_parameter_value().bool_value
        reid_backend = self.get_parameter('reid_backend').get_parameter_value().string_value
        reid_cosine = self.get_parameter('reid_cosine_min').get_parameter_value().double_value
        reid_vocab = int(self.get_parameter('reid_vocab_size').get_parameter_value().integer_value)
        reid_score = self.get_parameter('reid_match_score_min').get_parameter_value().double_value
        self._reid_match_score_min = reid_score
        self._track_buffer_size = max(
            int(self.get_parameter('track_buffer_size').get_parameter_value().integer_value), 4)
        self._sigma_floor_m = max(
            self.get_parameter('sigma_floor_m').get_parameter_value().double_value, 0.01)
        self._pixel_merge_u = max(
            self.get_parameter('pixel_merge_u_px').get_parameter_value().double_value, 4.0)
        self._pixel_merge_v = max(
            self.get_parameter('pixel_merge_v_px').get_parameter_value().double_value, 4.0)
        self._track_uv_ema = min(max(
            self.get_parameter('track_uv_ema').get_parameter_value().double_value, 0.01), 1.0)
        self._track_map_ema = min(max(
            self.get_parameter('track_map_ema').get_parameter_value().double_value, 0.05), 1.0)
        self._stereo_baseline_m = max(
            self.get_parameter('stereo_baseline_m').get_parameter_value().double_value, 0.01)
        self._min_depth_m = max(
            self.get_parameter('min_depth_m').get_parameter_value().double_value, 0.1)
        self._max_depth_m = max(
            self.get_parameter('max_depth_m').get_parameter_value().double_value, self._min_depth_m + 0.1)
        self._min_disparity_px = max(
            self.get_parameter('min_disparity_px').get_parameter_value().double_value, 0.5)
        self._max_ground_z_error = max(
            self.get_parameter('max_ground_z_error').get_parameter_value().double_value, 0.01)
        self._max_map_radius_m = max(
            self.get_parameter('max_map_radius_m').get_parameter_value().double_value, 1.0)
        self._reid: Optional[WaypointReID] = None
        if self._enable_reid:
            self._reid = WaypointReID(
                backend=reid_backend,
                cosine_min=reid_cosine,
                orb_vocab_size=max(reid_vocab, 8),
                orb_match_score_min=reid_score,
            )
            msg = f're-ID enabled (backend={self._reid.active_backend}'
            if self._reid.fallback_reason:
                msg += f', orb fallback: {self._reid.fallback_reason}'
            msg += f', cosine>={reid_cosine}, orb>={reid_score})'
            self.get_logger().info(msg)
        if self.has_parameter('use_sim_time'):
            use_sim = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            use_sim = True

        weights = _resolve_weights(self.get_parameter('weights').get_parameter_value().string_value)
        self._model = _load_yolo(str(weights))
        self.get_logger().info(f'Loaded weights: {weights}')

        self._k_left: Optional[np.ndarray] = None
        self._k_right: Optional[np.ndarray] = None
        self._waypoints: List[MapWaypoint] = []
        self._tracks: List[WaypointTrack] = []
        self._last_infer_time = 0.0
        self._tf_ready = False
        self._warned_missing_info = False
        self._warned_tf = False
        self._warned_no_frames = False
        self._last_preview_time = 0.0
        self._last_waypoint_flush_time = 0.0
        self._finalize_overlay_done = False
        self._node_start_mono = time.monotonic()
        self._warned_tf_fallback = False
        self._stereo_frame_count = 0
        self._left_preview_count = 0
        self._marker_slot_count = 0
        self._motion_blocked_frames = 0
        self._stereo_reject_frames = 0
        self._tf_stamp_ok = 0
        self._tf_split_ok = 0
        self._tf_latest_ok = 0
        self._tf_composed_ok = 0
        self._tf_debug_samples = 0
        self._merge_pixel = 0
        self._merge_reid = 0
        self._merge_spatial = 0
        self._merge_cluster = 0
        self._merge_ambiguous_skip = 0
        self._merge_bearing = 0
        self._merge_new = 0
        self._merge_outlier_blocked = 0
        self._wp_obs_none = 0
        self._wp_depth_skip = 0
        self._ground_new_skip = 0
        self._spatial_pixel_skip = 0
        self._tf_lag_high = 0
        self._warned_tf_split = False
        self._use_sim_time = use_sim

        # Gazebo Classic 相机为 RELIABLE；勿用 sensor_data(BEST_EFFORT)，否则收不到图。
        self._camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=60.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._wp_pub = self.create_publisher(PoseArray, '/detection/waypoints', 10)
        self._img_pub = self.create_publisher(Image, '/detection/annotated_image', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/detection/markers', 10)

        self._finalize_sub = self.create_subscription(
            Empty, '/perception/mapping_finalized', self._on_finalize, 10)

        self.create_subscription(
            Image, '/camera/left/image_raw', self._on_left_preview, self._camera_qos)

        left_sub = Subscriber(self, Image, '/camera/left/image_raw', qos_profile=self._camera_qos)
        right_sub = Subscriber(self, Image, '/camera/right/image_raw', qos_profile=self._camera_qos)
        self._sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=30, slop=self._stereo_sync_slop)
        self._sync.registerCallback(self._on_stereo_images)

        self.create_subscription(
            CameraInfo, '/camera/left/camera_info', self._on_left_info, self._camera_qos)
        self.create_subscription(
            CameraInfo, '/camera/right/camera_info', self._on_right_info, self._camera_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self._on_cmd_vel_teleop, 10)

        if self._waypoint_save_interval > 0.0:
            self.create_timer(self._waypoint_save_interval, self._periodic_waypoint_flush)
        self.create_timer(1.0, self._republish_markers)

        self._load_waypoints_from_disk()
        if self._waypoints:
            self._publish_waypoints(live=True)

        if self._session_dir:
            self.get_logger().info(f'Session dir: {self._session_dir}')
        clock_kind = 'sim' if self.get_clock().ros_time_is_active else 'system'
        self.get_logger().info(
            f'clip_detector ready (use_sim_time={use_sim}, clock={clock_kind}, '
            f'motion_gate={"on" if self._enable_motion_gate else "off"}, '
            f'motion_comp={"on" if self._enable_motion_compensation else "off"})')

    def _on_left_info(self, msg: CameraInfo) -> None:
        self._k_left = camera_matrix_from_info(msg.k)
        if not self._warned_missing_info and self._k_right is not None:
            self._warned_missing_info = True
            self.get_logger().info('camera_info 就绪（左+右）')

    def _on_right_info(self, msg: CameraInfo) -> None:
        self._k_right = camera_matrix_from_info(msg.k)
        if not self._warned_missing_info and self._k_left is not None:
            self._warned_missing_info = True
            self.get_logger().info('camera_info 就绪（左+右）')

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_omega = float(msg.twist.twist.angular.z)
        self._odom_linear = float(msg.twist.twist.linear.x)
        if self._odom_buffer is not None:
            self._odom_buffer.add_from_odometry(msg)
        self._mark_fast_motion_if_needed()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_linear = float(msg.linear.x)
        self._cmd_angular = float(msg.angular.z)
        self._mark_fast_motion_if_needed()

    def _on_cmd_vel_teleop(self, msg: Twist) -> None:
        self._teleop_linear = float(msg.linear.x)
        self._teleop_angular = float(msg.angular.z)
        self._mark_fast_motion_if_needed()

    def _effective_motion(self) -> tuple[float, float]:
        v = max(abs(self._cmd_linear), abs(self._teleop_linear))
        w = max(abs(self._cmd_angular), abs(self._teleop_angular))
        return v, w

    def _mark_fast_motion_if_needed(self) -> None:
        v, w = self._effective_motion()
        if v > self._max_linear_for_wp or w > self._max_angular_for_wp:
            self._last_fast_motion_mono = time.monotonic()
            return
        if self._use_odom_motion_gate and abs(self._odom_omega) > self._max_odom_angular_for_wp:
            self._last_fast_motion_mono = time.monotonic()

    def _motion_allows_waypoint(self) -> bool:
        """Optional speed/settle gate only; TF sync already clamps to available transforms."""
        if not self._enable_motion_gate:
            return True
        v, w = self._effective_motion()
        if w > self._max_angular_for_wp:
            return False
        if v > self._max_linear_for_wp:
            return False
        if self._use_odom_motion_gate and abs(self._odom_omega) > self._max_odom_angular_for_wp:
            return False
        if self._motion_settle_sec > 0.0:
            if time.monotonic() - self._last_fast_motion_mono < self._motion_settle_sec:
                return False
        return True

    def _query_time_from_stamp(self, stamp) -> rclpy.time.Time:
        """Clamp image stamp to now — tf2 cannot extrapolate into the future."""
        stamp_time = rclpy.time.Time.from_msg(stamp)
        now = self.get_clock().now()
        return stamp_time if stamp_time <= now else now

    def _sync_query_time_for(
        self,
        target: str,
        source: str,
        stamp,
    ) -> tuple[rclpy.time.Time, int]:
        """Clamp to latest common TF time; return (query_time, lag_ms vs image stamp)."""
        stamp_time = self._query_time_from_stamp(stamp)
        stamp_ns = stamp_time.nanoseconds
        try:
            common = self._tf_buffer.get_latest_common_time(target, source)
            common_ns = common.nanoseconds
            if common_ns > 0 and stamp_ns > common_ns:
                return rclpy.time.Time(nanoseconds=common_ns), (stamp_ns - common_ns) // 1_000_000
        except Exception:
            pass
        return stamp_time, 0

    def _tf_matrix_at(
        self,
        target: str,
        source: str,
        query_time: rclpy.time.Time,
    ) -> np.ndarray:
        tf = self._tf_buffer.lookup_transform(
            target, source, query_time,
            timeout=rclpy.duration.Duration(seconds=0.25))
        return _tf_to_matrix(tf.transform.translation, tf.transform.rotation)

    def _static_base_to_camera_tf(self, camera_link: str) -> Optional[np.ndarray]:
        cached = self._t_base_cam_cache.get(camera_link)
        if cached is not None:
            return cached
        try:
            mat = self._tf_matrix_at(self._base_frame, camera_link, rclpy.time.Time())
            self._t_base_cam_cache[camera_link] = mat
            return mat
        except Exception:
            return None

    def _lookup_camera_tf_composed(self, stamp, camera_link: str) -> Optional[np.ndarray]:
        if self._odom_buffer is None or len(self._odom_buffer) == 0:
            return None
        t_base_cam = self._static_base_to_camera_tf(camera_link)
        if t_base_cam is None:
            return None
        stamp_time = self._query_time_from_stamp(stamp)
        stamp_ns = stamp_time.nanoseconds
        t_odom_base = self._odom_buffer.pose_at(stamp_ns)
        if t_odom_base is None:
            return None
        self._last_odom_extrapolate_ms = self._odom_buffer.last_extrapolate_ms
        q_mo, lag_map_odom = self._sync_query_time_for(self._map_frame, 'odom', stamp)
        try:
            t_map_odom = self._tf_matrix_at(self._map_frame, 'odom', q_mo)
        except Exception:
            return None
        self._last_tf_lag_ms = max(self._last_tf_lag_ms, lag_map_odom)
        return compose_map_camera_tf(t_map_odom, t_odom_base, t_base_cam)

    def _lookup_camera_tf_legacy(
        self,
        stamp,
        camera_link: str,
    ) -> tuple[Optional[np.ndarray], str, str, int, int, int]:
        """Fallback TF strategies; returns (matrix, strategy, err, lag_map_cam, lag_odom_cam, lag_map_odom)."""
        stamp_err = ''
        result: Optional[np.ndarray] = None
        lag_map_cam = 0
        lag_odom_cam = 0
        lag_map_odom = 0
        strategy = ''

        if self._tf_prefer_image_stamp:
            q_full, lag_map_cam = self._sync_query_time_for(
                self._map_frame, camera_link, stamp)
            try:
                result = self._tf_matrix_at(self._map_frame, camera_link, q_full)
                strategy = 'sync_full'
                self._tf_stamp_ok += 1
            except Exception as exc:
                stamp_err = f'{type(exc).__name__}:{exc}'

        if result is None and self._tf_prefer_image_stamp:
            try:
                q_oc, lag_odom_cam = self._sync_query_time_for('odom', camera_link, stamp)
                q_mo, lag_map_odom = self._sync_query_time_for(self._map_frame, 'odom', stamp)
                t_odom_cam = self._tf_matrix_at('odom', camera_link, q_oc)
                t_map_odom = self._tf_matrix_at(self._map_frame, 'odom', q_mo)
                result = t_map_odom @ t_odom_cam
                strategy = 'split_per_link'
                self._tf_split_ok += 1
                if not self._warned_tf_split:
                    self._warned_tf_split = True
                    self.get_logger().info(
                        '使用分链 TF：odom→camera@图像同步 + map→odom@可用时刻')
            except Exception as exc:
                if not stamp_err:
                    stamp_err = f'{type(exc).__name__}:{exc}'

        if result is None:
            try:
                result = self._tf_matrix_at(self._map_frame, camera_link, rclpy.time.Time())
                strategy = 'latest_full'
                self._tf_latest_ok += 1
                if self._tf_prefer_image_stamp and not self._warned_tf_fallback:
                    self._warned_tf_fallback = True
                    self.get_logger().warn(
                        f'{self._map_frame}→{camera_link} 分链失败，回退 latest 全链')
            except Exception as exc:
                stamp_err = stamp_err or f'{type(exc).__name__}:{exc}'

        return result, strategy, stamp_err, lag_map_cam, lag_odom_cam, lag_map_odom

    def _lookup_camera_tf(self, stamp, camera_link: str) -> Optional[np.ndarray]:
        """map→camera at image stamp; motion compensation composes chain at unified time."""
        stamp_ns = stamp_to_ns(stamp)
        clock_ns = self.get_clock().now().nanoseconds
        self._last_pipeline_delay_ms = max(0, (clock_ns - stamp_ns) // 1_000_000)
        self._last_odom_extrapolate_ms = 0
        self._last_tf_lag_ms = 0
        log_detail = (
            self._tf_debug_samples < 5
            or self._stereo_frame_count in (1, 2, 3)
            or self._stereo_frame_count % 40 == 0
        )
        if log_detail:
            self._tf_debug_samples += 1

        strategy = ''
        stamp_err = ''
        result: Optional[np.ndarray] = None
        lag_map_cam = 0
        lag_odom_cam = 0
        lag_map_odom = 0

        if self._enable_motion_compensation:
            result = self._lookup_camera_tf_composed(stamp, camera_link)
            if result is not None:
                strategy = 'composed'
                self._tf_composed_ok += 1

        if result is None:
            result, strategy, stamp_err, lag_map_cam, lag_odom_cam, lag_map_odom = (
                self._lookup_camera_tf_legacy(stamp, camera_link)
            )

        self._last_tf_lag_ms = max(
            self._last_tf_lag_ms, lag_map_cam, lag_odom_cam, lag_map_odom)

        if log_detail:
            # #region agent log
            _agent_log(
                'B' if strategy == 'latest_full' else 'A',
                'clip_detector_node.py:_lookup_camera_tf',
                'tf_lookup_ok' if result is not None else 'tf_lookup_failed',
                {
                    'frame': self._stereo_frame_count,
                    'camera_link': camera_link,
                    'strategy': strategy,
                    'stamp_ns': stamp_ns,
                    'clock_ns': clock_ns,
                    'delta_ms': (stamp_ns - clock_ns) / 1e6,
                    'lag_map_cam_ms': lag_map_cam,
                    'lag_odom_cam_ms': lag_odom_cam,
                    'lag_map_odom_ms': lag_map_odom,
                    'last_tf_lag_ms': self._last_tf_lag_ms,
                    'pipeline_delay_ms': self._last_pipeline_delay_ms,
                    'odom_extrapolate_ms': self._last_odom_extrapolate_ms,
                    'stamp_err': stamp_err,
                },
                run_id='post-fix-v2',
            )
            # #endregion

        return result

    def _draw_boxes(self, bgr: np.ndarray, boxes: List[DetectionBox]) -> np.ndarray:
        annotated = bgr.copy()
        for box in boxes:
            cv2.rectangle(
                annotated,
                (int(box.x1), int(box.y1)), (int(box.x2), int(box.y2)),
                (0, 255, 0), 2,
            )
            label = f'{box.confidence:.2f}'
            cv2.putText(
                annotated, label,
                (int(box.x1), max(0, int(box.y1) - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA,
            )
        return annotated

    def _publish_annotated(self, bgr: np.ndarray, header) -> None:
        try:
            self._img_pub.publish(bgr_to_image_msg(bgr, header))
        except Exception as exc:
            self.get_logger().warn(f'annotated image publish failed: {exc}')

    def _run_yolo(self, bgr: np.ndarray) -> List[DetectionBox]:
        results = self._model.predict(
            bgr, conf=self._conf, verbose=False, device='0' if self._has_cuda() else 'cpu')
        boxes: List[DetectionBox] = []
        if not results:
            return boxes
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                boxes.append(DetectionBox(
                    x1=float(xyxy[0]), y1=float(xyxy[1]),
                    x2=float(xyxy[2]), y2=float(xyxy[3]),
                    confidence=conf, class_id=cls_id,
                ))
        return boxes

    @staticmethod
    def _has_cuda() -> bool:
        try:
            import torch
            return torch.cuda.is_available()
        except Exception:
            return False

    def _health_check(self) -> None:
        if self._stereo_frame_count > 0 or self._warned_no_frames:
            return
        if time.monotonic() - self._node_start_mono < 10.0:
            return
        self._warned_no_frames = True
        self.get_logger().error(
            '10s 内未收到双目同步帧（RViz 会显示 No Image / 无航点）。\n'
            '  根因通常是 Gazebo 未发布 /camera/left|right/image_raw。\n'
            '  请在终端1仍运行时执行:\n'
            '    bash scripts/check_camera_topics.sh\n'
            '    ros2 topic list | grep camera\n'
            '  若 bag/metadata.yaml 也无 camera 话题，需 kill_sim 后重启仿真。\n'
            '  ros2 param get /clip_detector use_sim_time  (仿真应为 true)')

    def _republish_markers(self) -> None:
        self._health_check()
        if self._waypoints:
            self._publish_waypoints(live=True)

    def _load_waypoints_from_disk(self) -> None:
        if not self._session_dir:
            return
        wp_path = self._session_dir / 'waypoints.yaml'
        if not wp_path.is_file():
            return
        try:
            data = yaml.safe_load(wp_path.read_text(encoding='utf-8')) or {}
            for item in data.get('waypoints', []):
                self._waypoints.append(MapWaypoint(
                    x=float(item['x']), y=float(item['y']),
                    confidence=float(item.get('confidence', 0.5)),
                    source=str(item.get('source', 'file')),
                ))
                self._tracks.append(WaypointTrack(buffer_size=self._track_buffer_size))
            if self._waypoints:
                merged = deduplicate_waypoints(self._waypoints, radius_m=self._dedup_radius)
                if len(merged) < len(self._waypoints):
                    self._waypoints = merged
                    self._tracks = self._tracks[:len(merged)]
                self.get_logger().info(f'已从磁盘加载 {len(self._waypoints)} 个航点')
        except Exception as exc:
            self.get_logger().warn(f'加载 waypoints.yaml 失败: {exc}')

    def _on_left_preview(self, msg: Image) -> None:
        """左目预览：仅向 RViz 推图，不做单目航点。"""
        self._left_preview_count += 1
        if self._left_preview_count == 1:
            self.get_logger().info('已收到左目图像，/detection/annotated_image 将发布')
        now = time.monotonic()
        if now - self._last_preview_time < 0.4:
            return
        self._last_preview_time = now
        try:
            bgr = image_msg_to_bgr(msg)
        except Exception:
            return
        self._publish_annotated(bgr, msg.header)

    def _nms_boxes(self, boxes: List[DetectionBox]) -> List[DetectionBox]:
        """Frame NMS only; no cap on detection count."""
        return nms_detection_boxes(boxes)

    def _new_track(self) -> WaypointTrack:
        return WaypointTrack(
            buffer_size=self._track_buffer_size,
            uv_ema=self._track_uv_ema,
            map_ema=self._track_map_ema,
        )

    def _ensure_track(self, index: int) -> WaypointTrack:
        while len(self._tracks) <= index:
            self._tracks.append(self._new_track())
        return self._tracks[index]

    def _find_track_by_pixel(self, u: float, v: float) -> Optional[int]:
        for i, track in enumerate(self._tracks):
            if track.matches_pixel(u, v, self._pixel_merge_u, self._pixel_merge_v):
                return i
        return None

    def _nearest_waypoint_index(self, map_x: float, map_y: float) -> tuple[Optional[int], float]:
        if not self._waypoints:
            return None, float('inf')
        best_idx = 0
        best_dist = math.hypot(map_x - self._waypoints[0].x, map_y - self._waypoints[0].y)
        for i, wp in enumerate(self._waypoints[1:], start=1):
            dist = math.hypot(map_x - wp.x, map_y - wp.y)
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        return best_idx, best_dist

    def _apply_fused_track(self, index: int, obs: Observation) -> bool:
        track = self._ensure_track(index)
        prev = self._waypoints[index] if 0 <= index < len(self._waypoints) else None
        if prev is not None:
            jump = math.hypot(obs.map_x - prev.x, obs.map_y - prev.y)
            if jump > self._outlier_reject_m:
                self._merge_outlier_blocked += 1
                return False
        self._waypoints[index] = track.fuse_map_1sigma(
            obs, self._sigma_floor_m, prev=prev)
        return True

    def _add_waypoint(
        self,
        u: float,
        v: float,
        range_m: float,
        confidence: float,
        source: str,
        crop_bgr: Optional[np.ndarray],
        t_map_camera: np.ndarray,
    ) -> None:
        """Map-space nearest-neighbor association; no pixel/ReID relaxed merges."""
        if self._last_tf_lag_ms > self._max_tf_lag_ms:
            self._tf_lag_high += 1
        if not self._motion_allows_waypoint():
            self._motion_blocked_frames += 1
            return
        if self._k_left is None:
            return

        obs = observation_from_detection(
            u, v, range_m, confidence, source, self._k_left, t_map_camera,
        )
        if obs is None:
            self._wp_obs_none += 1
            return

        best_idx, best_dist = self._nearest_waypoint_index(obs.map_x, obs.map_y)
        if best_idx is not None and best_dist <= self._dedup_radius:
            if self._apply_fused_track(best_idx, obs):
                self._merge_spatial += 1
                if self._reid is not None and crop_bgr is not None:
                    self._reid.register(best_idx, crop_bgr)
                return

        if best_idx is not None and best_dist < self._save_dedup_radius:
            self._merge_ambiguous_skip += 1
            return

        track = self._new_track()
        wp = track.fuse_map_1sigma(obs, self._sigma_floor_m, prev=None)
        self._waypoints.append(wp)
        self._tracks.append(track)
        self._merge_new += 1
        new_idx = len(self._waypoints) - 1
        if self._reid is not None and crop_bgr is not None:
            self._reid.register(new_idx, crop_bgr)
        if self._merge_new <= 8 or self._merge_new % 5 == 0:
            # #region agent log
            _agent_log(
                'E',
                'clip_detector_node.py:_add_waypoint',
                'wp_new_track',
                {
                    'wp_count': len(self._waypoints),
                    'u': round(u, 1),
                    'v': round(v, 1),
                    'range_m': round(range_m, 2),
                    'map_x': round(obs.map_x, 2),
                    'map_y': round(obs.map_y, 2),
                    'source': source,
                },
                run_id='scatter-fix-v3',
            )
            # #endregion

    def _ground_hit_map(
        self,
        u: float,
        v: float,
        t_map_camera: np.ndarray,
    ) -> Optional[tuple[float, float, float, str]]:
        ray_optical = pixel_to_ray_optical(u, v, self._k_left)
        t_map_optical = t_map_camera @ make_transform(CAMERA_OPTICAL_FIX, np.zeros(3))
        origin = t_map_optical[:3, 3]
        direction = t_map_optical[:3, :3] @ ray_optical
        hit = ray_ground_intersection(origin, direction, self._ground_z)
        if hit is None:
            return None
        range_m = validate_map_point(
            hit, t_map_camera,
            min_depth_m=self._min_depth_m,
            max_depth_m=self._max_depth_m,
            max_ground_z_error=self._max_ground_z_error,
            ground_z=self._ground_z,
            max_map_radius_m=self._max_map_radius_m,
        )
        if range_m is None:
            return None
        return float(hit[0]), float(hit[1]), range_m, 'ground'

    def _stereo_range_crosscheck(
        self,
        lb: DetectionBox,
        rb: DetectionBox,
        t_left: np.ndarray,
        t_right: np.ndarray,
        ground_range_m: float,
    ) -> bool:
        """Optional sanity check: reject when stereo depth disagrees with ground plane."""
        fx = float(self._k_left[0, 0])
        point = triangulate_stereo(
            lb.center_u, lb.center_v, rb.center_u, rb.center_v,
            self._k_left, self._k_right, t_left, t_right,
        )
        if point is None:
            return True
        stereo_range = validate_stereo_point(
            point, lb.center_u, rb.center_u, fx, self._stereo_baseline_m, t_left,
            min_depth_m=self._min_depth_m,
            max_depth_m=self._max_depth_m,
            max_ground_z_error=self._max_ground_z_error,
            min_disparity_px=self._min_disparity_px,
            ground_z=self._ground_z,
            max_map_radius_m=self._max_map_radius_m,
        )
        if stereo_range is None:
            return True
        if abs(stereo_range - ground_range_m) > self._stereo_ground_crosscheck_m:
            self._stereo_reject_frames += 1
            return False
        return True

    def _process_stereo_detection(
        self,
        lb: DetectionBox,
        rb: DetectionBox,
        left_bgr: np.ndarray,
        t_left: np.ndarray,
        t_right: np.ndarray,
    ) -> None:
        patch = crop_detection_patch(left_bgr, lb.x1, lb.y1, lb.x2, lb.y2)
        if not self._allow_ground_fallback:
            self._wp_depth_skip += 1
            return
        mapped = self._ground_hit_map(lb.center_u, lb.center_v, t_left)
        if not mapped:
            self._wp_depth_skip += 1
            return
        _x, _y, range_m, src = mapped
        if not self._stereo_range_crosscheck(lb, rb, t_left, t_right, range_m):
            return
        self._add_waypoint(
            lb.center_u, lb.center_v, range_m,
            0.5 * (lb.confidence + rb.confidence), src, patch, t_left,
        )

    def _process_mono_detection(
        self,
        lb: DetectionBox,
        left_bgr: np.ndarray,
        t_left: np.ndarray,
    ) -> None:
        if not self._allow_ground_fallback:
            self._wp_depth_skip += 1
            return
        patch = crop_detection_patch(left_bgr, lb.x1, lb.y1, lb.x2, lb.y2)
        mapped = self._ground_hit_map(lb.center_u, lb.center_v, t_left)
        if mapped:
            _x, _y, range_m, src = mapped
            self._add_waypoint(
                lb.center_u, lb.center_v, range_m,
                lb.confidence, src, patch, t_left,
            )
        else:
            self._wp_depth_skip += 1

    def _on_stereo_images(self, left_msg: Image, right_msg: Image) -> None:
        if self._k_left is None or self._k_right is None:
            if not self._warned_missing_info:
                self.get_logger().warn(
                    '等待 /camera/left|right/camera_info；请确认终端1仿真已启动且 record_bag 含相机')
                self._warned_missing_info = True
            return
        self._stereo_frame_count += 1
        now = time.monotonic()
        min_interval = 1.0 / max(self._max_fps, 0.1)
        if now - self._last_infer_time < min_interval:
            return
        self._last_infer_time = now

        try:
            left_bgr = image_msg_to_bgr(left_msg)
            right_bgr = image_msg_to_bgr(right_msg)
        except Exception as exc:
            self.get_logger().warn(f'image decode failed: {exc}')
            return

        left_boxes = self._nms_boxes(self._run_yolo(left_bgr))
        right_boxes = self._nms_boxes(self._run_yolo(right_bgr))
        self._publish_annotated(self._draw_boxes(left_bgr, left_boxes), left_msg.header)

        stamp = left_msg.header.stamp
        t_left = self._lookup_camera_tf(stamp, 'camera_left_link')
        t_right = self._lookup_camera_tf(stamp, 'camera_right_link')
        if t_left is None or t_right is None:
            if not self._warned_tf:
                self.get_logger().warn(
                    f'无法查询 {self._map_frame}→camera_*_link TF；'
                    '检测框可见但航点需等 SLAM 发布 map 帧（通常启动后数秒）')
                self._warned_tf = True
            elif self._stereo_frame_count % 80 == 0 and left_boxes:
                self.get_logger().info(
                    f'仍等待 {self._map_frame} TF（frame {self._stereo_frame_count}，'
                    f'检测 {len(left_boxes)} 个）；SLAM 就绪后自动记点')
            self._publish_waypoints(live=True)
            return

        fx = float(self._k_left[0, 0])
        disp_max = max_disparity_px(fx, self._stereo_baseline_m, self._min_depth_m)
        pairs = match_stereo_detections(
            left_boxes, right_boxes,
            min_disparity_px=self._min_disparity_px,
            max_disparity_px=disp_max,
        )

        if self._stereo_frame_count == 1 or self._stereo_frame_count % 40 == 0:
            v, w = self._effective_motion()
            motion_ok = self._motion_allows_waypoint()
            self.get_logger().info(
                f'stereo frame {self._stereo_frame_count}: L={len(left_boxes)} R={len(right_boxes)} '
                f'pairs={len(pairs)} wps={len(self._waypoints)} motion='
                f'{"OK" if motion_ok else "BLOCK"} v={v:.2f} w={w:.2f} '
                f'motion_blk={self._motion_blocked_frames} stereo_rej={self._stereo_reject_frames}')
            motion_blk_snapshot = self._motion_blocked_frames
            depth_skip_snapshot = self._wp_depth_skip
            obs_none_snapshot = self._wp_obs_none
            tf_lag_high_snapshot = self._tf_lag_high
            ground_new_skip_snapshot = self._ground_new_skip
            ambiguous_skip_snapshot = self._merge_ambiguous_skip
            spatial_pixel_skip_snapshot = self._spatial_pixel_skip
            self._motion_blocked_frames = 0
            self._stereo_reject_frames = 0
            # #region agent log
            total_tf = (
                self._tf_composed_ok + self._tf_stamp_ok
                + self._tf_split_ok + self._tf_latest_ok
            )
            compensated = self._tf_composed_ok + self._tf_stamp_ok + self._tf_split_ok
            _agent_log(
                'D',
                'clip_detector_node.py:_on_stereo_images',
                'tf_stats_window',
                {
                    'frame': self._stereo_frame_count,
                    'tf_composed_ok': self._tf_composed_ok,
                    'tf_stamp_ok': self._tf_stamp_ok,
                    'tf_split_ok': self._tf_split_ok,
                    'tf_latest_ok': self._tf_latest_ok,
                    'compensated_rate': (compensated / total_tf) if total_tf else 0.0,
                    'pipeline_delay_ms': self._last_pipeline_delay_ms,
                    'odom_extrapolate_ms': self._last_odom_extrapolate_ms,
                },
                run_id='post-fix-v2',
            )
            _agent_log(
                'E',
                'clip_detector_node.py:_on_stereo_images',
                'merge_stats_window',
                {
                    'frame': self._stereo_frame_count,
                    'wps': len(self._waypoints),
                    'left_boxes': len(left_boxes),
                    'pairs': len(pairs),
                    'motion_blk': motion_blk_snapshot,
                    'tf_lag_high': tf_lag_high_snapshot,
                    'ground_new_skip': ground_new_skip_snapshot,
                    'ambiguous_skip': ambiguous_skip_snapshot,
                    'spatial_pixel_skip': spatial_pixel_skip_snapshot,
                    'last_tf_lag_ms': self._last_tf_lag_ms,
                    'depth_skip': depth_skip_snapshot,
                    'obs_none': obs_none_snapshot,
                    'merge_pixel': self._merge_pixel,
                    'merge_reid': self._merge_reid,
                    'merge_spatial': self._merge_spatial,
                    'merge_cluster': self._merge_cluster,
                    'merge_bearing': self._merge_bearing,
                    'merge_new': self._merge_new,
                    'outlier_blocked': self._merge_outlier_blocked,
                },
                run_id='merge-fix-v2',
            )
            self._tf_composed_ok = 0
            self._tf_stamp_ok = 0
            self._tf_split_ok = 0
            self._tf_latest_ok = 0
            self._merge_pixel = 0
            self._merge_reid = 0
            self._merge_spatial = 0
            self._merge_cluster = 0
            self._merge_bearing = 0
            self._merge_new = 0
            self._merge_outlier_blocked = 0
            self._wp_obs_none = 0
            self._wp_depth_skip = 0
            self._ground_new_skip = 0
            self._merge_ambiguous_skip = 0
            self._spatial_pixel_skip = 0
            self._tf_lag_high = 0
            # #endregion

        paired_left: set[int] = set()
        for lb, rb in pairs:
            paired_left.add(id(lb))
            self._process_stereo_detection(lb, rb, left_bgr, t_left, t_right)

        for lb in left_boxes:
            if id(lb) in paired_left:
                continue
            self._process_mono_detection(lb, left_bgr, t_left)

        self._publish_waypoints(live=True)

    def _publish_waypoints(self, live: bool = True) -> None:
        # 内存列表已在 _add_waypoint 合并；发布时不再二次聚类，避免 marker 数量闪烁
        wps = self._waypoints
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        for wp in wps:
            pose = Pose()
            pose.position.x = wp.x
            pose.position.y = wp.y
            pose.position.z = self._ground_z
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._wp_pub.publish(msg)
        self._publish_markers(wps)

    def _publish_markers(self, wps: List[MapWaypoint]) -> None:
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, wp in enumerate(wps):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'clip_waypoints'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.x
            m.pose.position.y = wp.y
            m.pose.position.z = self._ground_z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self._marker_scale
            m.color.r = 0.1
            m.color.g = 0.95
            m.color.b = 0.1
            m.color.a = 0.95
            arr.markers.append(m)
        for idx in range(len(wps), self._marker_slot_count):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'clip_waypoints'
            m.id = idx
            m.action = Marker.DELETE
            arr.markers.append(m)
        self._marker_slot_count = len(wps)
        if arr.markers:
            self._marker_pub.publish(arr)

    def _on_finalize(self, _msg: Empty) -> None:
        self.get_logger().info('Mapping finalized — saving waypoints')
        self._finalize_waypoints(publish=True)

    def _waypoints_yaml_payload(self, wps: List[MapWaypoint]) -> dict:
        return {
            'frame_id': self._map_frame,
            'ground_z': self._ground_z,
            'dedup_radius_m': self._dedup_radius,
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'waypoints': [
                {
                    'id': i + 1,
                    'x': round(wp.x, 4),
                    'y': round(wp.y, 4),
                    'confidence': round(wp.confidence, 4),
                    'source': wp.source,
                }
                for i, wp in enumerate(wps)
            ],
        }

    def _write_waypoints_to_disk(self) -> int:
        if not self._session_dir:
            return 0
        final = deduplicate_waypoints(self._waypoints, radius_m=self._save_dedup_radius)
        self._session_dir.mkdir(parents=True, exist_ok=True)
        out = self._session_dir / 'waypoints.yaml'
        out.write_text(
            yaml.safe_dump(self._waypoints_yaml_payload(final), allow_unicode=True, sort_keys=False),
            encoding='utf-8',
        )
        return len(final)

    def _periodic_waypoint_flush(self) -> None:
        if not self._session_dir:
            return
        now = time.monotonic()
        if now - self._last_waypoint_flush_time < self._waypoint_save_interval - 0.05:
            return
        self._last_waypoint_flush_time = now
        self._write_waypoints_to_disk()

    def _save_map_odom_offset(self) -> None:
        if not self._session_dir:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, 'odom', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            out = self._session_dir / 'map_odom_offset.yaml'
            payload = {
                'map_frame': self._map_frame,
                'odom_frame': 'odom',
                'x': round(float(t.x), 4),
                'y': round(float(t.y), 4),
                'yaw': round(float(yaw), 4),
                'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
                'note': 'GT in odom frame → map: p_map = R(yaw)*p_odom + [x,y]',
            }
            out.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
        except Exception as exc:
            self.get_logger().warn(f'无法保存 map_odom_offset: {exc}')

    def _render_waypoints_overlay(self) -> None:
        if self._finalize_overlay_done or not self._session_dir:
            return
        try:
            self._save_map_odom_offset()
            from detection_pkg.map_overlay import render_waypoints_overlay
            overlay = render_waypoints_overlay(self._session_dir)
            if overlay:
                self.get_logger().info(f'Waypoints map: {overlay}')
        except Exception as exc:
            self.get_logger().warn(f'waypoints map overlay failed: {exc}')
        finally:
            self._finalize_overlay_done = True

    def _finalize_waypoints(self, publish: bool = True, force: bool = False) -> None:
        count = self._write_waypoints_to_disk()
        if count == 0 and not self._waypoints and not force:
            return
        if self._session_dir:
            self.get_logger().info(
                f'Waypoints saved: {self._session_dir / "waypoints.yaml"} ({count} points)')
        self._render_waypoints_overlay()
        if publish:
            try:
                self._publish_waypoints(live=False)
            except Exception as exc:
                self.get_logger().warn(f'waypoint publish skipped on shutdown: {exc}')

    def destroy_node(self) -> None:
        try:
            self._finalize_waypoints(publish=False, force=True)
        except Exception as exc:
            self.get_logger().warn(f'exit waypoint save failed: {exc}')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClipDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
