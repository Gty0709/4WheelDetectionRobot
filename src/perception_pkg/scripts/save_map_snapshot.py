#!/usr/bin/env python3
"""
save_map_snapshot.py — 订阅 /map，保存地图到会话目录 map_<时间戳>/，并写重定位先验。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

try:
    from PIL import Image
except ImportError:
    Image = None

FREE_THRESH = 0.196
OCC_THRESH = 0.65
_NODE: 'MapSnapshotSaver | None' = None


def resolve_maps_dir(param_value: str) -> Path:
    if param_value:
        return Path(param_value).expanduser().resolve()
    env_dir = os.environ.get('PERCEPTION_MAPS_DIR', '')
    if env_dir:
        return Path(env_dir).expanduser().resolve()
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = Path(get_package_share_directory('perception_pkg'))
        ws_root = pkg_share.parents[3]
        src_maps = ws_root / 'src' / 'perception_pkg' / 'maps'
        if src_maps.is_dir():
            return src_maps.resolve()
        return (pkg_share / 'maps').resolve()
    except Exception:
        return Path.cwd() / 'maps'


def resolve_session_dir(param_value: str, maps_dir: Path) -> Path | None:
    if param_value:
        return Path(param_value).expanduser().resolve()
    env = os.environ.get('PERCEPTION_MAP_SESSION', '').strip()
    if env:
        return Path(env).expanduser().resolve()
    return None


def _occ_to_pgm_byte(occ: int) -> int:
    if occ < 0:
        return 205
    if occ == 0:
        return 254
    return 0


def _write_pgm(path: Path, msg: OccupancyGrid) -> None:
    w, h = msg.info.width, msg.info.height
    rows = bytearray(w * h)
    data = msg.data
    for y in range(h):
        for x in range(w):
            src = y * w + x
            dst = (h - y - 1) * w + x
            rows[dst] = _occ_to_pgm_byte(int(data[src]))
    with path.open('wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(rows)


def _write_yaml(path: Path, pgm_name: str, msg: OccupancyGrid) -> None:
    o = msg.info.origin.position
    text = (
        f'image: {pgm_name}\n'
        f'mode: trinary\n'
        f'resolution: {msg.info.resolution:.3f}\n'
        f'origin: [{o.x:.3f}, {o.y:.3f}, {o.z:.3f}]\n'
        f'negate: 0\n'
        f'occupied_thresh: {OCC_THRESH}\n'
        f'free_thresh: {FREE_THRESH}\n'
    )
    path.write_text(text, encoding='utf-8')


def _write_png_preview(pgm_path: Path, png_path: Path) -> bool:
    if Image is None:
        return False
    gray = Image.open(pgm_path)
    rgb = Image.new('RGB', gray.size)
    pixels = []
    for v in gray.get_flattened_data():
        if v >= 250:
            pixels.append((245, 245, 245))
        elif v > 100:
            pixels.append((180, 180, 180))
        else:
            pixels.append((30, 30, 30))
    rgb.putdata(pixels)
    rgb.save(png_path)
    return True


def _update_latest_links(maps_dir: Path, stem: str) -> None:
    for ext in ('yaml', 'pgm', 'png'):
        src = maps_dir / f'{stem}.{ext}'
        if not src.exists():
            continue
        link = maps_dir / f'slam_map.{ext}'
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(src.name)


def _update_map_latest_session(maps_root: Path, session_dir: Path) -> None:
    link = maps_root / 'map_latest'
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(session_dir.name, target_is_directory=True)


def write_session_meta(session_dir: Path, maps_root: Path) -> None:
    meta = {
        'session_id': session_dir.name,
        'created_at': datetime.now().isoformat(),
        'maps_dir': str(session_dir),
        'bag_dir': str(session_dir / 'bag'),
        'map_yaml': 'slam_map.yaml',
        'map_png': 'slam_map.png',
        'waypoints_yaml': 'waypoints.yaml',
        'map_waypoints_png': 'slam_map_waypoints.png',
        'initial_pose_yaml': 'initial_pose.yaml',
    }
    (session_dir / 'session_meta.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False), encoding='utf-8')
    _update_map_latest_session(maps_root, session_dir)


def save_occupancy_grid(
    msg: OccupancyGrid,
    maps_dir: Path,
    filename_prefix: str = 'slam_map',
    timestamp: str | None = None,
    update_latest_symlink: bool = True,
    session_dir: Path | None = None,
) -> Path | None:
    if msg.info.width == 0 or msg.info.height == 0:
        return None

    if session_dir is not None:
        out_dir = session_dir
        stem = 'slam_map'
        pgm_name = 'slam_map.pgm'
    else:
        out_dir = maps_dir
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = timestamp or datetime.now().strftime('%Y%m%d_%H%M%S')
        stem = f'{filename_prefix}_{ts}'
        pgm_name = f'{stem}.pgm'

    out_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = out_dir / f'{stem}.yaml'
    pgm_path = out_dir / pgm_name
    png_path = out_dir / f'{stem}.png'

    _write_pgm(pgm_path, msg)
    _write_yaml(yaml_path, pgm_name, msg)
    _write_png_preview(pgm_path, png_path)

    if session_dir is not None:
        write_session_meta(session_dir, maps_dir)
        refresh_waypoints_overlay(session_dir)
    elif update_latest_symlink:
        _update_latest_links(maps_dir, stem)

    return yaml_path


def refresh_waypoints_overlay(session_dir: Path) -> None:
    """Regenerate slam_map_waypoints.png after SLAM map updates."""
    try:
        ws_root = Path(__file__).resolve().parents[3]
        det_src = ws_root / 'src' / 'detection_pkg'
        if det_src.is_dir():
            import sys
            src = str(det_src)
            if src not in sys.path:
                sys.path.insert(0, src)
        from detection_pkg.map_overlay import render_waypoints_overlay
        out = render_waypoints_overlay(session_dir)
        if out is not None:
            print(f'[save_map] waypoints overlay: {out}')
    except Exception as exc:
        print(f'[save_map] waypoints overlay skipped: {exc}')


def write_cache(cache_dir: Path, msg: OccupancyGrid) -> None:
    cache_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = cache_dir / 'latest.pgm'
    yaml_path = cache_dir / 'latest.yaml'
    meta_path = cache_dir / 'latest_meta.json'
    _write_pgm(pgm_path, msg)
    _write_yaml(yaml_path, 'latest.pgm', msg)
    _write_png_preview(pgm_path, cache_dir / 'latest.png')
    meta_path.write_text(json.dumps({
        'saved_at': datetime.now().isoformat(),
        'width': msg.info.width,
        'height': msg.info.height,
        'resolution': msg.info.resolution,
    }), encoding='utf-8')


def promote_cache_to_session(
    session_dir: Path,
    maps_root: Path,
    filename_prefix: str = 'slam_map',
) -> Path | None:
    cache_dir = session_dir / '_cache'
    pgm_path = cache_dir / 'latest.pgm'
    yaml_path = cache_dir / 'latest.yaml'
    if not pgm_path.exists() or not yaml_path.exists():
        return None

    session_dir.mkdir(parents=True, exist_ok=True)
    out_pgm = session_dir / 'slam_map.pgm'
    out_yaml = session_dir / 'slam_map.yaml'
    out_png = session_dir / 'slam_map.png'

    out_pgm.write_bytes(pgm_path.read_bytes())
    text = yaml_path.read_text(encoding='utf-8').replace('latest.pgm', 'slam_map.pgm')
    out_yaml.write_text(text, encoding='utf-8')
    cache_png = cache_dir / 'latest.png'
    if cache_png.exists():
        out_png.write_bytes(cache_png.read_bytes())
    else:
        _write_png_preview(out_pgm, out_png)

    write_session_meta(session_dir, maps_root)
    return out_yaml


def promote_cache_to_timestamped(maps_dir: Path, filename_prefix: str = 'slam_map') -> Path | None:
    """Legacy: flat maps dir without session."""
    cache_dir = Path(maps_dir) / '_cache'
    pgm_path = cache_dir / 'latest.pgm'
    yaml_path = cache_dir / 'latest.yaml'
    if not pgm_path.exists() or not yaml_path.exists():
        return None

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    stem = f'{filename_prefix}_{ts}'
    maps_dir = Path(maps_dir)
    maps_dir.mkdir(parents=True, exist_ok=True)

    out_pgm = maps_dir / f'{stem}.pgm'
    out_yaml = maps_dir / f'{stem}.yaml'
    out_png = maps_dir / f'{stem}.png'

    out_pgm.write_bytes(pgm_path.read_bytes())
    text = yaml_path.read_text(encoding='utf-8').replace('latest.pgm', f'{stem}.pgm')
    out_yaml.write_text(text, encoding='utf-8')
    cache_png = cache_dir / 'latest.png'
    if cache_png.exists():
        out_png.write_bytes(cache_png.read_bytes())
    else:
        _write_png_preview(out_pgm, out_png)

    _update_latest_links(maps_dir, stem)
    return out_yaml


def write_map_odom_offset_yaml(path: Path, x: float, y: float, yaw: float) -> None:
    data = {
        'map_frame': 'map',
        'odom_frame': 'odom',
        'x': round(x, 4),
        'y': round(y, 4),
        'yaw': round(yaw, 4),
        'saved_at': datetime.now().isoformat(),
        'note': 'GT in odom frame → map: p_map = R(yaw)*p_odom + [x,y]',
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


def write_initial_pose_yaml(
    path: Path,
    x: float,
    y: float,
    yaw: float,
    cov_xx: float = 0.05,
    cov_yy: float = 0.05,
    cov_aa: float = 0.02,
) -> None:
    data = {
        'frame_id': 'map',
        'x': round(x, 4),
        'y': round(y, 4),
        'yaw': round(yaw, 4),
        'cov_xx': cov_xx,
        'cov_yy': cov_yy,
        'cov_aa': cov_aa,
        'saved_at': datetime.now().isoformat(),
        'source': 'tf_map_base_footprint',
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


class MapSnapshotSaver(Node):
    def __init__(self):
        super().__init__('map_snapshot_saver')
        self.declare_parameter('maps_dir', '')
        self.declare_parameter('session_dir', '')
        self.declare_parameter('filename_prefix', 'slam_map')
        self.declare_parameter('update_latest_symlink', True)
        self.declare_parameter('save_on_shutdown', True)
        self.declare_parameter('save_interval_sec', 45.0)
        self.declare_parameter('cache_interval_sec', 5.0)
        self.declare_parameter('save_settle_sec', 2.5)
        self.declare_parameter('stop_robot_before_save', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self._maps_root = resolve_maps_dir(
            self.get_parameter('maps_dir').get_parameter_value().string_value
        )
        self._session_dir = resolve_session_dir(
            self.get_parameter('session_dir').get_parameter_value().string_value,
            self._maps_root,
        )
        self._prefix = self.get_parameter('filename_prefix').get_parameter_value().string_value
        self._update_latest = self.get_parameter('update_latest_symlink').get_parameter_value().bool_value
        self._save_on_shutdown = self.get_parameter('save_on_shutdown').get_parameter_value().bool_value
        self._save_interval = self.get_parameter('save_interval_sec').get_parameter_value().double_value
        self._cache_interval = self.get_parameter('cache_interval_sec').get_parameter_value().double_value
        self._save_settle_sec = max(
            self.get_parameter('save_settle_sec').get_parameter_value().double_value, 0.0)
        self._stop_robot_before_save = self.get_parameter(
            'stop_robot_before_save').get_parameter_value().bool_value
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self._last_map: OccupancyGrid | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._shutdown_save_done = False
        self._saving = False
        self._last_cache_time = 0.0

        if self._session_dir:
            self._session_dir.mkdir(parents=True, exist_ok=True)
            write_session_meta(self._session_dir, self._maps_root)
            self._cache_dir = self._session_dir / '_cache'
            os.environ['PERCEPTION_MAP_SESSION'] = str(self._session_dir)
        else:
            self._cache_dir = self._maps_root / '_cache'

        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._finalize_pub = self.create_publisher(Empty, '/perception/mapping_finalized', 10)
        stop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_vel_teleop_pub = self.create_publisher(Twist, '/cmd_vel_teleop', stop_qos)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', stop_qos)

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        default = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, default)
        self.create_service(Trigger, 'save_map', self._handle_save_map)

        if self._save_interval > 0.0:
            self.create_timer(self._save_interval, self._periodic_save)
        self.create_timer(1.0, self._update_pose_cache)

        if self._session_dir:
            self.get_logger().info(f'会话目录: {self._session_dir}')
        else:
            self.get_logger().info(f'地图保存目录: {self._maps_root}')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last_map = msg
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_cache_time >= self._cache_interval:
            self._last_cache_time = now
            try:
                write_cache(self._cache_dir, msg)
            except Exception as exc:
                self.get_logger().warn(f'写缓存失败: {exc}')

    def _update_pose_cache(self) -> None:
        if not self._session_dir:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self._last_pose = (float(t.x), float(t.y), float(yaw))
        except Exception:
            return

        out = self._session_dir / 'initial_pose.yaml'
        write_initial_pose_yaml(out, *self._last_pose)
        self._save_map_odom_offset()

    def _stop_robot_and_settle(self) -> None:
        """Publish zero velocity and wait for SLAM to integrate last scans."""
        if not self._stop_robot_before_save:
            return
        stop = Twist()
        for _ in range(12):
            self._cmd_vel_teleop_pub.publish(stop)
            self._cmd_vel_pub.publish(stop)
            time.sleep(0.05)
        if self._save_settle_sec <= 0.0:
            return
        deadline = time.monotonic() + self._save_settle_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

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
            write_map_odom_offset_yaml(out, float(t.x), float(t.y), float(yaw))
        except Exception as exc:
            self.get_logger().warn(f'无法保存 map_odom_offset: {exc}')

    def _periodic_save(self) -> None:
        if self._last_map is not None:
            self.get_logger().info('定期备份地图...')
            self.save_now(write_pose=False, finalize=False)

    def _handle_save_map(self, _req, response):
        ok = self.save_now(write_pose=True, finalize=True)
        response.success = ok
        if ok and self._session_dir:
            response.message = f'saved to {self._session_dir}'
        else:
            response.message = 'saved' if ok else 'no /map yet'
        return response

    def _save_initial_pose(self) -> None:
        if not self._session_dir:
            return
        if self._last_pose is not None:
            out = self._session_dir / 'initial_pose.yaml'
            write_initial_pose_yaml(out, *self._last_pose)
            self.get_logger().info(f'重定位先验已保存: {out}')
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self._last_pose = (float(t.x), float(t.y), float(yaw))
            out = self._session_dir / 'initial_pose.yaml'
            write_initial_pose_yaml(out, *self._last_pose)
            self.get_logger().info(f'重定位先验已保存: {out}')
        except Exception as exc:
            self.get_logger().warn(f'无法保存 initial_pose: {exc}')

    def save_now(self, write_pose: bool = False, finalize: bool = False) -> bool:
        if finalize:
            self._stop_robot_and_settle()
        if self._last_map is None:
            self.get_logger().warn('尚未收到 /map，跳过保存')
            return False
        out = save_occupancy_grid(
            self._last_map, self._maps_root,
            filename_prefix=self._prefix,
            update_latest_symlink=self._update_latest and self._session_dir is None,
            session_dir=self._session_dir,
        )
        if out is None:
            return False
        self.get_logger().info(f'地图已保存: {out}')
        self._save_map_odom_offset()
        if write_pose:
            self._save_initial_pose()
        if finalize:
            self._signal_finalize()
        return True

    def _signal_finalize(self) -> None:
        if self._session_dir:
            flag = self._session_dir / '.mapping_finalized'
            try:
                flag.write_text(datetime.now().isoformat(), encoding='utf-8')
            except OSError as exc:
                self.get_logger().warn(f'无法写 finalize 标记: {exc}')
        try:
            self._finalize_pub.publish(Empty())
        except Exception as exc:
            self.get_logger().warn(f'finalize 话题发布失败（已写标记文件）: {exc}')

    def save_on_exit(self) -> None:
        if self._shutdown_save_done or not self._save_on_shutdown or self._saving:
            return
        self._saving = True
        self._shutdown_save_done = True
        try:
            self.get_logger().info('退出保存地图...')
            self._stop_robot_and_settle()
            if not self.save_now(write_pose=True, finalize=True):
                if self._session_dir:
                    promoted = promote_cache_to_session(
                        self._session_dir, self._maps_root, self._prefix)
                    if promoted:
                        self._save_initial_pose()
                        self._signal_finalize()
                        self.get_logger().info(f'已从缓存保存: {promoted}')
                else:
                    promoted = promote_cache_to_timestamped(self._maps_root, self._prefix)
                    if promoted:
                        self.get_logger().info(f'已从缓存保存: {promoted.name}')
        except Exception as exc:
            self.get_logger().error(f'退出保存异常: {exc}')
        finally:
            self._saving = False

    def destroy_node(self) -> None:
        if self._save_on_shutdown and not self._shutdown_save_done:
            self.save_on_exit()
        super().destroy_node()


def _parse_cli(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Save /map snapshot')
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--from-cache', action='store_true', help='从 _cache 提升地图，无需 ROS')
    parser.add_argument('--maps-dir', default='')
    parser.add_argument('--session-dir', default='')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    global _NODE
    cli = _parse_cli(remove_ros_args(args=argv if argv is not None else sys.argv)[1:])

    if cli.from_cache:
        maps_dir = resolve_maps_dir(cli.maps_dir)
        session_dir = resolve_session_dir(cli.session_dir, maps_dir)
        if session_dir:
            out = promote_cache_to_session(session_dir, maps_dir)
        else:
            out = promote_cache_to_timestamped(maps_dir)
        if out:
            print(f'[save_map_snapshot] 已从缓存保存: {out}')
            sys.exit(0)
        print('[save_map_snapshot] 无缓存可保存', file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=argv)
    _NODE = MapSnapshotSaver()

    try:
        if cli.once:
            timeout = 90.0
            elapsed = 0.0
            while rclpy.ok() and _NODE._last_map is None and elapsed < timeout:
                rclpy.spin_once(_NODE, timeout_sec=0.5)
                elapsed += 0.5
            _NODE.save_now(write_pose=True, finalize=True)
        else:
            rclpy.spin(_NODE)
    except KeyboardInterrupt:
        pass
    finally:
        if _NODE is not None:
            _NODE.destroy_node()
            _NODE = None
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
