#!/usr/bin/env python3
"""
RANSAC wall detector — lidar hadap belakang, 180°.

Koordinat lidar (0° = lurus belakang robot):
  +x = belakang robot
  +y = kiri robot
  -y = kanan robot

Sektor:
  BELAKANG : -45° s/d +45°
  KIRI     : +45° s/d +90°
  KANAN    : -90° s/d -45°

Publishes:
  /walls        (Float32MultiArray) [d_belakang, d_kiri, d_kanan] meter
  /wall_markers (MarkerArray)       garis + label untuk RViz2
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Float32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ── RANSAC ───────────────────────────────────────────────────────────────────

def ransac_line(pts, n_iter=80, dist_thr=0.05, min_inliers=8):
    """
    Fit garis ax+by+c=0 ke pts (N,2) pakai RANSAC.
    Return (a, b, c, inlier_mask) atau None.
    """
    if len(pts) < min_inliers:
        return None

    best_mask = np.zeros(len(pts), dtype=bool)
    best_abc  = None

    for _ in range(n_iter):
        i, j = np.random.choice(len(pts), 2, replace=False)
        dx = pts[j, 0] - pts[i, 0]
        dy = pts[j, 1] - pts[i, 1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            continue
        a, b = dy / norm, -dx / norm
        c = -(a * pts[i, 0] + b * pts[i, 1])

        dists = np.abs(a * pts[:, 0] + b * pts[:, 1] + c)
        mask  = dists < dist_thr
        if mask.sum() > best_mask.sum():
            best_mask = mask
            best_abc  = (a, b, c)

    if best_abc is None or best_mask.sum() < min_inliers:
        return None

    # refit dengan semua inlier
    inliers = pts[best_mask]
    cx, cy  = inliers.mean(axis=0)
    _, vecs = np.linalg.eigh(np.cov(inliers.T))
    dx, dy  = vecs[:, 1]
    a, b    = dy, -dx
    c       = -(a * cx + b * cy)
    norm    = math.hypot(a, b)
    a, b, c = a / norm, b / norm, c / norm

    return a, b, c, best_mask


def line_endpoints(pts, mask, half_len=1.5):
    """
    Dua titik ujung garis untuk visualisasi,
    sepanjang ±half_len dari centroid inlier.
    """
    inliers = pts[mask]
    cx, cy  = inliers.mean(axis=0)
    _, vecs = np.linalg.eigh(np.cov(inliers.T))
    dx, dy  = vecs[:, 1]          # arah garis
    p1 = (cx - dx * half_len, cy - dy * half_len)
    p2 = (cx + dx * half_len, cy + dy * half_len)
    return p1, p2


def wall_distance(a, b, c):
    return abs(c)


# ── node ─────────────────────────────────────────────────────────────────────

# warna per dinding
_COLOR = {
    'belakang': (1.0, 0.2, 0.2),   # merah
    'kiri':     (0.2, 1.0, 0.2),   # hijau
    'kanan':    (0.2, 0.5, 1.0),   # biru
}
_IDS   = {'belakang': 0, 'kiri': 1, 'kanan': 2}


class WallNode(Node):
    def __init__(self):
        super().__init__('wall_node')

        self.declare_parameter('scan_topic',   '/scan_filtered')
        self.declare_parameter('walls_topic',  '/walls')
        self.declare_parameter('marker_topic', '/wall_markers')
        self.declare_parameter('laser_frame',  'laser')
        self.declare_parameter('ransac_iter',    80)
        self.declare_parameter('dist_thr',       0.05)
        self.declare_parameter('min_inliers',    4)
        # sudut toleransi orientasi dinding (derajat)
        # kiri/kanan harus sejajar robot (normal ≈ y), belakang tegak lurus (normal ≈ x)
        self.declare_parameter('orient_thr_deg', 45.0)
        self.declare_parameter('filter_alpha',    0.15)   # EMA: 0=freeze, 1=raw

        scan_topic   = self.get_parameter('scan_topic').value
        walls_topic  = self.get_parameter('walls_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self._frame      = self.get_parameter('laser_frame').value
        self._n_iter     = self.get_parameter('ransac_iter').value
        self._dist_thr   = self.get_parameter('dist_thr').value
        self._min_inlier = self.get_parameter('min_inliers').value
        orient_deg       = self.get_parameter('orient_thr_deg').value
        # cos threshold: normal harus dalam ±orient_thr_deg dari sumbu yang diharapkan
        self._orient_thr  = math.cos(math.radians(orient_deg))
        self._alpha       = float(self.get_parameter('filter_alpha').value)

        # EMA state (mm), -1 = belum ada data
        self._filt = {'belakang': -1.0, 'kiri': -1.0, 'kanan': -1.0}

        self._walls_pub    = self.create_publisher(Float32MultiArray, walls_topic,  10)
        self._marker_pub   = self.create_publisher(MarkerArray,       marker_topic, 10)
        self._pub_belakang = self.create_publisher(Float32, '/wall/belakang', 10)
        self._pub_kiri     = self.create_publisher(Float32, '/wall/kiri',     10)
        self._pub_kanan    = self.create_publisher(Float32, '/wall/kanan',    10)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, scan_topic, self._cb, sensor_qos)
        self.get_logger().info(
            f'wall_node ready  {scan_topic} → /wall/belakang  /wall/kiri  /wall/kanan'
        )

    # --------------------------------------------------------------------- #

    def _cb(self, msg: LaserScan):
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        r = np.array(msg.ranges, dtype=np.float32)

        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        r, a  = r[valid], angles[valid]
        pts   = np.column_stack([r * np.cos(a), r * np.sin(a)])

        DEG45 = math.pi / 4
        stamp      = msg.header.stamp
        marker_arr = MarkerArray()

        sektors = {
            'belakang': pts[(a >= -DEG45) & (a <= DEG45)],
            'kanan':    pts[a >  DEG45],
            'kiri':     pts[a < -DEG45],
        }

        self.get_logger().info(
            f'sektor pts: belakang={len(sektors["belakang"])}  '
            f'kanan={len(sektors["kanan"])}  kiri={len(sektors["kiri"])}',
            throttle_duration_sec=2.0,
        )

        raw = {}
        for name, sektor_pts in sektors.items():
            result = ransac_line(sektor_pts, self._n_iter, self._dist_thr, self._min_inlier)
            if result is not None:
                a_c, b_c, c_c, mask = result
                if name == 'belakang' and abs(a_c) < self._orient_thr:
                    result = None
                elif name in ('kiri', 'kanan') and abs(b_c) < self._orient_thr:
                    result = None
            if result is not None:
                a_c, b_c, c_c, mask = result
                raw[name] = wall_distance(a_c, b_c, c_c) * 1000.0  # meter → mm
                p1, p2 = line_endpoints(sektor_pts, mask)
                marker_arr.markers += self._wall_markers(name, p1, p2, raw[name] / 1000.0, stamp)
            else:
                raw[name] = -1.0
                marker_arr.markers.append(self._delete_marker(_IDS[name], stamp))

        # ── EMA filter: hanya update saat ada data valid ──────────────────────
        for name, val in raw.items():
            if val > 0.0:
                if self._filt[name] < 0.0:
                    self._filt[name] = val          # inisialisasi pertama
                else:
                    self._filt[name] = self._alpha * val + (1.0 - self._alpha) * self._filt[name]
            # kalau tidak terdeteksi: biarkan nilai filter tetap (tidak reset)

        fb = self._filt['belakang']
        fk = self._filt['kanan']
        fi = self._filt['kiri']

        self._pub_belakang.publish(Float32(data=fb))
        self._pub_kanan.publish(   Float32(data=fk))
        self._pub_kiri.publish(    Float32(data=fi))

        out      = Float32MultiArray()
        out.data = [fb, fk, fi]
        self._walls_pub.publish(out)

        # ── tambah label filtered mm di tengah setiap dinding ────────────────
        filt_vals = {'belakang': fb, 'kanan': fk, 'kiri': fi}
        for name, fval in filt_vals.items():
            if fval > 0.0:
                marker_arr.markers.append(
                    self._filt_label(name, fval, stamp))

        self._marker_pub.publish(marker_arr)

        self.get_logger().info(
            f'walls(mm)  belakang={fb:.0f}  kanan={fk:.0f}  kiri={fi:.0f}',
            throttle_duration_sec=0.5,
        )

    # -- marker builders -------------------------------------------------- #

    def _wall_markers(self, name, p1, p2, dist, stamp):
        r, g, b = _COLOR[name]
        mid_id  = _IDS[name]

        # garis dinding
        line          = Marker()
        line.header.stamp    = stamp
        line.header.frame_id = self._frame
        line.ns       = 'walls'
        line.id       = mid_id
        line.type     = Marker.LINE_STRIP
        line.action   = Marker.ADD
        line.scale.x  = 0.04
        line.color    = ColorRGBA(r=r, g=g, b=b, a=1.0)
        line.points   = [
            Point(x=float(p1[0]), y=float(p1[1]), z=0.0),
            Point(x=float(p2[0]), y=float(p2[1]), z=0.0),
        ]

        # label jarak
        text          = Marker()
        text.header.stamp    = stamp
        text.header.frame_id = self._frame
        text.ns       = 'wall_labels'
        text.id       = mid_id + 10
        text.type     = Marker.TEXT_VIEW_FACING
        text.action   = Marker.ADD
        mx            = (p1[0] + p2[0]) / 2
        my            = (p1[1] + p2[1]) / 2
        text.pose.position = Point(x=float(mx), y=float(my), z=0.15)
        text.scale.z  = 0.18
        text.color    = ColorRGBA(r=r, g=g, b=b, a=1.0)
        text.text     = f'{name}\n{dist*1000.0:.0f} mm'

        return [line, text]

    def _filt_label(self, name, filt_mm, stamp):
        r, g, b = _COLOR[name]
        t = Marker()
        t.header.stamp    = stamp
        t.header.frame_id = self._frame
        t.ns    = 'wall_filt'
        t.id    = _IDS[name] + 20
        t.type  = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        # posisi label: di origin, offset per dinding agar tidak tumpang tindih
        offsets = {'belakang': (1.5, 0.0), 'kiri': (0.5, 1.2), 'kanan': (0.5, -1.2)}
        ox, oy  = offsets.get(name, (0.0, 0.0))
        t.pose.position = Point(x=ox, y=oy, z=0.3)
        t.scale.z = 0.22
        t.color   = ColorRGBA(r=r, g=g, b=b, a=1.0)
        t.text    = f'[{name}]\n{filt_mm:.0f} mm'
        return t

    def _delete_marker(self, mid, stamp):
        m             = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = self._frame
        m.ns          = 'walls'
        m.id          = mid
        m.action      = Marker.DELETE
        return m


# ── entry ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WallNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
