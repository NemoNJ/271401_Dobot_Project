#!/usr/bin/env python3
import argparse
import csv
import json
import os
import queue
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


try:
    import detect_object as _do
except ImportError:
    _do = None


def _local_imgmsg_to_bgr(msg):
    enc = msg.encoding.lower()
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8uc1': 1, '8uc3': 3}.get(enc)
    if ch is None:
        raise ValueError(f'ไม่รองรับ encoding ภาพสี: {msg.encoding}')
    buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = buf[:, :msg.width * ch].reshape(msg.height, msg.width, ch) if ch > 1 else buf[:, :msg.width]
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if ch == 1:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img.copy()


def _local_imgmsg_to_array(msg):
    """depth -> uint16 mm"""
    enc = msg.encoding.lower()
    if enc in ('16uc1', 'mono16'):
        dt, scale = np.uint16, None
    elif enc == '32fc1':
        dt, scale = np.float32, 1000.0           # เมตร -> mm
    else:
        raise ValueError(f'ไม่รองรับ encoding depth: {msg.encoding}')
    dt = np.dtype(dt).newbyteorder('>' if msg.is_bigendian else '<')
    row = msg.step // dt.itemsize
    arr = np.frombuffer(msg.data, dt).reshape(msg.height, row)[:, :msg.width]
    if scale is None:
        return arr.astype(np.uint16)
    arr = np.nan_to_num(arr.astype(np.float32) * scale, nan=0.0, posinf=0.0, neginf=0.0)
    return np.clip(arr, 0, 65535).astype(np.uint16)


def _local_compute_crop_rect(iw, ih, x, y, w, h):
    if w <= 0 or h <= 0:
        return 0, 0, iw, ih
    x0 = int(min(max(x, 0), iw - 1))
    y0 = int(min(max(y, 0), ih - 1))
    return x0, y0, int(min(w, iw - x0)), int(min(h, ih - y0))


imgmsg_to_bgr = getattr(_do, 'imgmsg_to_bgr', None) or _local_imgmsg_to_bgr
imgmsg_to_array = getattr(_do, 'imgmsg_to_array', None) or _local_imgmsg_to_array
compute_crop_rect = getattr(_do, 'compute_crop_rect', None) or _local_compute_crop_rect
# depth flat-field: ใช้ได้เฉพาะเมื่อ detect_object.py มีครบ 3 ฟังก์ชัน
load_flat_calibration = getattr(_do, 'load_flat_calibration', None)
flat_slice = getattr(_do, 'flat_slice', None)
apply_flat_correction = getattr(_do, 'apply_flat_correction', None)
HAVE_FLAT = None not in (load_flat_calibration, flat_slice, apply_flat_correction)


# ============================================================================
#  ส่วนที่ 1: ตำแหน่งวัตถุสีเหลืองเทียบเฟรมกล้อง
# ============================================================================
def detect_yellow_objects(bgr, depth_mm, hsv_lo=(20, 100, 120), hsv_hi=(35, 255, 255), min_area_px=300):
    """
    หา blob สีเหลืองด้วย HSV คืน (list ของ dict เรียงใหญ่ -> เล็ก, mask รวม)
    dict: bbox, area_px, touches_edge, z_mm (median depth ของผิวบน), centroid (u, v) บนภาพ crop
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lo, np.uint8), np.array(hsv_hi, np.uint8))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    H, W = mask.shape[:2]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(c)
        obj = np.zeros((H, W), np.uint8)
        cv2.drawContours(obj, [c], -1, 255, -1)
        m = cv2.moments(obj, binaryImage=True)
        if m['m00'] <= 0:
            continue
        # depth: ใช้ส่วนในของวัตถุ (erode ตัดขอบที่ depth มักเพี้ยน)
        inner = cv2.erode(obj, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
        if cv2.countNonZero(inner) < 20:
            inner = obj
        d = depth_mm[inner > 0]
        d = d[d > 0]
        z = float(np.median(d)) if d.size >= 10 else None
        results.append({
            'bbox': (x, y, w, h), 'area_px': float(area), 'z_mm': z,
            'centroid': (m['m10'] / m['m00'], m['m01'] / m['m00']),
            'touches_edge': x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1,
        })
    results.sort(key=lambda o: o['area_px'], reverse=True)
    return results, mask


def find_yellow_positions(bgr, depth_mm, intr, crop_offset=(0, 0),
                          hsv_lo=(20, 100, 120), hsv_hi=(35, 255, 255), min_area_px=300):
    """
    หาวัตถุสีเหลืองทุกชิ้น (ใหญ่ -> เล็ก) แล้วคืนตำแหน่ง 3 มิติเทียบเฟรมกล้อง (optical, mm)
      bgr, depth_mm : ภาพสี / depth (uint16 mm) ที่ crop แล้ว ขนาดเท่ากัน (aligned)
      intr          : (fx, fy, cx, cy) ของภาพเต็ม (จาก camera_info)
      crop_offset   : (crop_x0, crop_y0) มุมซ้ายบนของกรอบ crop บนภาพเต็ม
    คืน list ของ dict: xyz_mm (np.array [X, Y, Z]), uv_crop (centroid บนภาพ crop),
                       bbox, area_px, touches_edge
    """
    if intr is None:
        return []
    fx, fy, cx, cy = intr
    ox, oy = crop_offset
    results, _ = detect_yellow_objects(bgr, depth_mm, hsv_lo, hsv_hi, min_area_px)
    out = []
    for o in results:
        if o['z_mm'] is None:
            continue
        u, v = o['centroid']
        z = o['z_mm']
        out.append({
            'xyz_mm': np.array([(u + ox - cx) * z / fx, (v + oy - cy) * z / fy, z]),
            'uv_crop': (u, v), 'bbox': o['bbox'], 'area_px': o['area_px'],
            'touches_edge': o['touches_edge'],
        })
    return out


# ============================================================================
#  ส่วนที่ 2: fit transform กล้อง -> หุ่น  (numpy ล้วน)
# ============================================================================
def fit_rigid(P, Q):
    """Kabsch/SVD: หา R, t ที่ทำให้ R @ p + t ใกล้ q ที่สุด คืนเมทริกซ์ 3x4 [R | t]"""
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    d = 1.0 if np.linalg.det(Vt.T @ U.T) >= 0 else -1.0     # กันได้ reflection
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return np.hstack([R, (cq - R @ cp)[:, None]])


def fit_affine(P, Q):
    """least squares แบบ affine 12 พารามิเตอร์ คืนเมทริกซ์ 3x4"""
    X, *_ = np.linalg.lstsq(np.hstack([P, np.ones((len(P), 1))]), Q, rcond=None)
    return X.T


def apply_transform(M, P):
    P = np.atleast_2d(np.asarray(P, np.float64))
    return P @ M[:, :3].T + M[:, 3]


def planarity(P):
    """สัดส่วน singular value เล็กสุด/ใหญ่สุด ของจุด (ใกล้ 0 = จุดอยู่ระนาบเดียวกัน)"""
    sv = np.linalg.svd(P - P.mean(0), compute_uv=False)
    return float(sv[2] / max(sv[0], 1e-9))


def calibrate(P, Q, model='rigid'):
    """
    fit transform จากจุดคู่ P (กล้อง) -> Q (หุ่น)  Nx3 mm
    คืน dict: M (3x4), model, residuals (mm ต่อจุด), rmse, loo (leave-one-out error ต่อจุด), notes
    """
    P, Q = np.asarray(P, np.float64), np.asarray(Q, np.float64)
    n = len(P)
    if n < 4:
        raise ValueError('ต้องมีอย่างน้อย 4 จุด')
    notes = []
    flat = planarity(P)
    if flat < 0.02:
        notes.append('จุดแทบอยู่ระนาบเดียวกัน (ความสูงไม่หลากหลาย) ควรเพิ่มจุดที่ความสูงต่างกัน')
    if model == 'affine' and (n < 6 or flat < 0.02):
        notes.append('affine ต้องมี >= 6 จุดและไม่อยู่ระนาบเดียวกัน จึงใช้ rigid แทน')
        model = 'rigid'
    fit = fit_affine if model == 'affine' else fit_rigid
    M = fit(P, Q)
    res = np.linalg.norm(apply_transform(M, P) - Q, axis=1)
    loo = None
    if n >= 5:
        loo = np.zeros(n)
        for i in range(n):
            keep = np.arange(n) != i
            loo[i] = np.linalg.norm(apply_transform(fit(P[keep], Q[keep]), P[i]) - Q[i])
    return {'M': M, 'model': model, 'residuals': res, 'rmse': float(np.sqrt(np.mean(res ** 2))),
            'loo': loo, 'planarity': flat, 'notes': notes}


def format_report(P, Q, r):
    lines = [f"calibrate ({r['model']}) จาก {len(P)} จุด",
             ' #   cam X,Y,Z (mm)               robot X,Y,Z (mm)             residual   LOO']
    for i in range(len(P)):
        loo = f"{r['loo'][i]:6.2f}" if r['loo'] is not None else '   -  '
        lines.append(
            f"{i + 1:2d}   ({P[i, 0]:7.1f},{P[i, 1]:7.1f},{P[i, 2]:7.1f})   "
            f"({Q[i, 0]:7.1f},{Q[i, 1]:7.1f},{Q[i, 2]:7.1f})   {r['residuals'][i]:6.2f}   {loo}")
    s = f"RMSE = {r['rmse']:.2f} mm   max residual = {r['residuals'].max():.2f} mm"
    if r['loo'] is not None:
        s += f"   LOO mean = {r['loo'].mean():.2f} mm  max = {r['loo'].max():.2f} mm"
    lines.append(s)
    lines += [f'หมายเหตุ: {t}' for t in r['notes']]
    if r['rmse'] > 3.0:
        lines.append('หมายเหตุ: RMSE > 3 mm  ตรวจว่าอ่านพิกัดหุ่นที่จุดเดียวกับที่กล้องวัด (กึ่งกลางผิวบน), '
                     'crop/depth flat ตรงกับตอนใช้งาน, และไม่มีจุดที่อ่านค่าผิด (ดูจุดที่ residual/LOO สูง)')
    return '\n'.join(lines)


class CameraToRobot:
    """transform พิกัดกล้อง (optical, mm) -> พิกัดหุ่น (mm)  p_robot = M[:, :3] @ p_cam + M[:, 3]"""

    def __init__(self, M, meta=None):
        self.M = np.asarray(M, np.float64).reshape(3, 4)
        self.meta = meta or {}

    def cam_to_robot(self, p_cam):
        p = apply_transform(self.M, p_cam)
        return tuple(float(v) for v in p[0]) if np.ndim(p_cam) == 1 else p

    def save(self, path, P, Q, r, extra=None):
        data = {
            'created': datetime.now().isoformat(timespec='seconds'),
            'camera_frame': 'RealSense optical frame (X right, Y down, Z forward), mm',
            'model': r['model'], 'M': self.M.tolist(),
            'R': self.M[:, :3].tolist(), 't': self.M[:, 3].tolist(),
            'n_points': int(len(P)), 'rmse_mm': r['rmse'],
            'max_residual_mm': float(r['residuals'].max()),
            'loo_mean_mm': None if r['loo'] is None else float(r['loo'].mean()),
            'loo_max_mm': None if r['loo'] is None else float(r['loo'].max()),
            'points': [{'cam': P[i].tolist(), 'robot': Q[i].tolist(),
                        'residual_mm': float(r['residuals'][i])} for i in range(len(P))],
        }
        data.update(extra or {})
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        return cls(d['M'], meta=d)


_CSV_HEADER = ['cam_x', 'cam_y', 'cam_z', 'std_x', 'std_y', 'std_z', 'rob_x', 'rob_y', 'rob_z']


def parse_xyz(text):
    """'x y z' หรือ 'x,y,z' (หรือ x y z r จาก GetPose -> ใช้ 3 ค่าแรก) คืน np.array หรือ None"""
    parts = text.replace(',', ' ').split()
    if len(parts) not in (3, 4):
        return None
    try:
        return np.array([float(v) for v in parts[:3]])
    except ValueError:
        return None


def load_pairs_csv(path):
    P, Q = [], []
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            P.append([float(row['cam_x']), float(row['cam_y']), float(row['cam_z'])])
            Q.append([float(row['rob_x']), float(row['rob_y']), float(row['rob_z'])])
    return np.array(P), np.array(Q)


def load_robot_rows(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as f:
        for line in f:
            v = parse_xyz(line.strip())
            if v is not None:
                rows.append(v)
    return rows


# ============================================================================
#  ROS node: เก็บจุดคู่ -> fit -> บันทึก
# ============================================================================
class YellowCalibNode(Node):
    def __init__(self):
        super().__init__('yellow_robot_calibration')
        d = self.declare_parameter
        d('color_topic', '/camera/camera/color/image_raw')
        d('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        d('info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        d('cam_topic', '/yellow/position_cam')
        d('display_scale', 3.0)
        d('crop_x', 460)
        d('crop_y', 310)
        d('crop_width', 305)
        d('crop_height', 208)
        d('depth_flat_file', '')
        for k, v in (('h_low', 20), ('h_high', 35), ('s_low', 100), ('s_high', 255),
                     ('v_low', 120), ('v_high', 255)):
            d('yellow_' + k, v)
        d('yellow_min_area_px', 300)
        d('num_points', 12)
        d('samples', 30)
        d('max_std_mm', 3.0)
        d('model', 'rigid')
        d('robot_csv', '')
        d('output_dir', '.')

        g = lambda n: self.get_parameter(n).get_parameter_value()  # noqa: E731
        self.scale = g('display_scale').double_value
        self.crop = (g('crop_x').integer_value, g('crop_y').integer_value,
                     g('crop_width').integer_value, g('crop_height').integer_value)
        self.hsv_lo = (g('yellow_h_low').integer_value, g('yellow_s_low').integer_value,
                       g('yellow_v_low').integer_value)
        self.hsv_hi = (g('yellow_h_high').integer_value, g('yellow_s_high').integer_value,
                       g('yellow_v_high').integer_value)
        self.min_area = g('yellow_min_area_px').integer_value
        self.num_points = g('num_points').integer_value
        self.n_samples = g('samples').integer_value
        self.max_std = g('max_std_mm').double_value
        self.model = g('model').string_value.lower()
        self.out_dir = os.path.abspath(g('output_dir').string_value)
        os.makedirs(self.out_dir, exist_ok=True)
        self.csv_path = os.path.join(self.out_dir, 'calib_points.csv')
        self.json_path = os.path.join(self.out_dir, 'handeye_calibration.json')

        self.robot_rows = []
        rcsv = g('robot_csv').string_value
        if rcsv:
            self.robot_rows = load_robot_rows(rcsv)
            self.get_logger().info(f'อ่านพิกัดหุ่นจาก {rcsv}: {len(self.robot_rows)} แถว')

        self.flat = None
        self._flat_warned = False
        fpath = g('depth_flat_file').string_value
        if fpath:
            if HAVE_FLAT:
                self.flat = load_flat_calibration(fpath)
            else:
                self.get_logger().warn('detect_object.py ไม่มีฟังก์ชัน depth flat จึงไม่ใช้ depth_flat_file')

        self.intrinsics = None
        self.lock = threading.Lock()
        self.latest = None            # (color, depth, header, crop_x0, crop_y0)
        self.pairs = []               # list ของ (cam xyz, std xyz, robot xyz)
        self.T = None                 # CameraToRobot หลัง fit
        self.state = 'await_robot'    # await_robot -> ready_scan -> capturing -> (จุดถัดไป) ... -> done
        self.robot_pending = None     # พิกัดหุ่นของจุดปัจจุบัน (ใส่ก่อนสแกน)
        self.target = None            # วัตถุเหลืองที่เห็นในเฟรมล่าสุด
        self.samples = []
        self.cap_t0 = 0.0
        self.last_stamp = None
        self.msg = ''
        self.msg_until = 0.0

        # อ่าน stdin ใน thread แยก เพื่อให้หน้าต่างภาพไม่ค้างตอนรอพิมพ์พิกัดหุ่น
        self.lines = queue.Queue()
        threading.Thread(target=self._stdin_loop, daemon=True).start()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.cam_pub = self.create_publisher(PointStamped, g('cam_topic').string_value, 10)
        self.create_subscription(CameraInfo, g('info_topic').string_value, self.on_info, qos)
        cs = Subscriber(self, Image, g('color_topic').string_value, qos_profile=qos)
        ds = Subscriber(self, Image, g('depth_topic').string_value, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer([cs, ds], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.on_frames)

        self.win = 'Yellow calibration'
        cv2.namedWindow(self.win)
        self.create_timer(1.0 / 30.0, self.on_display)
        self.get_logger().info(
            f'พร้อม calibrate {self.num_points} จุด (model={self.model}) | '
            'ขั้นตอน: วางวัตถุเหลือง -> jog หุ่นไปแตะกึ่งกลางผิวบน -> พิมพ์ x y z (mm) ในเทอร์มินัล '
            '-> ยกหุ่นออก -> กด SPACE ที่หน้าต่างภาพเพื่อสแกน')
        self.get_logger().info(f'จุดที่เก็บจะถูกบันทึกที่ {self.csv_path}')
        self.prompt_robot()

    # ---------------------------------------------------------------- input
    def _stdin_loop(self):
        for line in sys.stdin:
            self.lines.put(line.strip())

    def _drain(self):
        while not self.lines.empty():
            self.lines.get_nowait()

    def on_info(self, msg):
        if self.intrinsics is None and msg.k[0] > 0 and msg.k[4] > 0:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.get_logger().info(
                f'intrinsics: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')

    def on_frames(self, color_msg, depth_msg):
        try:
            color, depth = imgmsg_to_bgr(color_msg), imgmsg_to_array(depth_msg)
        except Exception as e:
            self.get_logger().error(f'แปลงภาพไม่สำเร็จ: {e}')
            return
        ih, iw = color.shape[:2]
        x0, y0, cw, ch = compute_crop_rect(iw, ih, *self.crop)
        color = np.ascontiguousarray(color[y0:y0 + ch, x0:x0 + cw])
        depth = np.ascontiguousarray(depth[y0:y0 + ch, x0:x0 + cw])
        if self.flat is not None:
            delta = flat_slice(self.flat, (x0, y0, cw, ch), (iw, ih))
            if delta is not None:
                depth = apply_flat_correction(depth, delta)
            elif not self._flat_warned:
                self._flat_warned = True
                self.get_logger().warn('depth flat: กรอบ crop/ขนาดภาพไม่ตรงกับตอน calibrate จึงไม่แก้ค่า')
        with self.lock:
            self.latest = (color, depth, depth_msg.header, x0, y0)

    # ---------------------------------------------------------------- state machine
    def say(self, text, warn=False, en=''):
        """text = ข้อความเต็มในเทอร์มินัล, en = ข้อความสั้นภาษาอังกฤษบนหน้าต่างภาพ (putText แสดงภาษาไทยไม่ได้)"""
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self.msg, self.msg_until = en, time.monotonic() + 4.0

    def prompt_robot(self):
        """เริ่มจุดใหม่: ขอพิกัดหุ่นก่อน (หรือดึงจาก robot_csv ถ้ามี)"""
        self.robot_pending = None
        idx = len(self.pairs)
        if idx >= self.num_points:
            return
        if idx < len(self.robot_rows):
            self.set_robot(self.robot_rows[idx], from_csv=True)
            return
        self._drain()
        self.state = 'await_robot'
        print(f'\n[จุด {idx + 1}/{self.num_points}] พิมพ์ตำแหน่งหุ่น x y z (mm) แล้วกด Enter: ', flush=True)

    def set_robot(self, xyz, from_csv=False):
        self.robot_pending = np.asarray(xyz, np.float64)
        self.state = 'ready_scan'
        src = ' (จาก robot_csv)' if from_csv else ''
        self.say(f'จุด {len(self.pairs) + 1}: หุ่น ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}){src} '
                 '-> ยกหุ่นออกจากมุมกล้อง แล้วกด SPACE เพื่อสแกน', en='robot xyz received - press SPACE to scan')

    def poll_terminal(self):
        """อ่านบรรทัดจากเทอร์มินัล (ใช้ทั้งตอนรอพิกัดหุ่น และตอนรอสแกน)"""
        while not self.lines.empty():
            line = self.lines.get_nowait()
            if self.state == 'ready_scan' and line == '':
                self.start_capture()                      # Enter เปล่า = สแกน (เหมือน SPACE)
                return
            if line == '':
                continue
            xyz = parse_xyz(line)
            if xyz is None:
                self.get_logger().warn("รูปแบบไม่ถูก: พิมพ์ 'x y z' (mm) เช่น 312.5 -41.2 -150.3")
                continue
            if self.state == 'ready_scan':
                self.get_logger().info('แทนที่พิกัดหุ่นของจุดนี้ด้วยค่าใหม่')
            self.set_robot(xyz)
            return

    def start_capture(self):
        if self.state == 'await_robot':
            self.say('ใส่ตำแหน่งหุ่น x y z ในเทอร์มินัลก่อน แล้วค่อยกด SPACE', True, en='enter robot x y z in the terminal first')
            return
        if self.state != 'ready_scan':
            return
        if self.intrinsics is None:
            self.say('ยังไม่ได้รับ camera_info รอสักครู่', True, en='waiting for camera_info')
            return
        if self.target is None:
            self.say('ไม่เห็นวัตถุเหลืองเต็มชิ้น (อย่าให้ชิดขอบภาพ/หุ่นบัง) แล้วกด SPACE ใหม่', True,
                     en='no full yellow object visible - press SPACE again')
            return
        self.samples, self.cap_t0, self.last_stamp = [], time.monotonic(), None
        self.state = 'capturing'
        self.get_logger().info(f'กำลังสแกน {self.n_samples} เฟรม... (อย่าขยับวัตถุ)')

    def feed_capture(self, target, header):
        stamp = (header.stamp.sec, header.stamp.nanosec)
        if stamp != self.last_stamp and target is not None:       # นับเฉพาะเฟรมใหม่ที่เห็นวัตถุ
            self.last_stamp = stamp
            self.samples.append(target['xyz_mm'])
        if len(self.samples) >= self.n_samples:
            arr = np.array(self.samples)
            cam, std = np.median(arr, axis=0), arr.std(axis=0)
            if std.max() > self.max_std:
                self.state = 'ready_scan'
                self.say(f'ค่าแกว่งเกินไป (std {std.max():.1f} mm > {self.max_std}) วัตถุขยับ/depth noise: '
                         'กด SPACE สแกนใหม่', True, en=f'unstable (std {std.max():.1f} mm) - press SPACE again')
                return
            self.get_logger().info(
                f'จุดที่ {len(self.pairs) + 1}: กล้อง X,Y,Z = ({cam[0]:.1f}, {cam[1]:.1f}, {cam[2]:.1f}) mm '
                f'(std {std.max():.2f} mm)')
            self.finish_point(cam, std)
        elif time.monotonic() - self.cap_t0 > max(10.0, self.n_samples / 5.0):
            self.state = 'ready_scan'
            self.say('สแกนไม่สำเร็จ: ไม่เห็นวัตถุเหลืองเต็มชิ้น/ไม่มีเฟรมใหม่ กด SPACE ลองใหม่', True,
                     en='scan failed - press SPACE again')

    def finish_point(self, cam, std):
        rob = self.robot_pending
        self.pairs.append((cam, std, rob))
        self.robot_pending = None
        self.write_csv()
        self.say(f'บันทึกจุดที่ {len(self.pairs)}/{self.num_points}: '
                 f'กล้อง ({cam[0]:.1f}, {cam[1]:.1f}, {cam[2]:.1f}) -> '
                 f'หุ่น ({rob[0]:.1f}, {rob[1]:.1f}, {rob[2]:.1f})', en=f'point {len(self.pairs)} saved')
        if len(self.pairs) >= self.num_points:
            self.do_fit()
        else:
            self.prompt_robot()

    def write_csv(self):
        with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(_CSV_HEADER)
            for cam, std, rob in self.pairs:
                w.writerow([f'{v:.3f}' for v in (*cam, *std, *rob)])

    def do_fit(self):
        if len(self.pairs) < 4:
            self.say('ต้องมีอย่างน้อย 4 จุดถึงจะ fit ได้', True, en='need >= 4 points to fit')
            return
        P = np.array([p[0] for p in self.pairs])
        Q = np.array([p[2] for p in self.pairs])
        r = calibrate(P, Q, self.model)
        self.get_logger().info('\n' + format_report(P, Q, r))
        self.T = CameraToRobot(r['M'])
        extra = {'crop_xywh': list(self.crop), 'intrinsics_fx_fy_cx_cy': self.intrinsics,
                 'yellow_hsv_lo': list(self.hsv_lo), 'yellow_hsv_hi': list(self.hsv_hi)}
        self.T.save(self.json_path, P, Q, r, extra)
        self.say(f'calibrate เสร็จ: RMSE {r["rmse"]:.2f} mm  -> {self.json_path}', en=f'calibrated: RMSE {r["rmse"]:.2f} mm')
        if len(self.pairs) >= self.num_points:
            self.state = 'done'

    def undo(self):
        if self.state == 'capturing':
            self.state = 'ready_scan'
            self.say('ยกเลิกการสแกน (พิกัดหุ่นยังอยู่ กด SPACE สแกนใหม่ได้)', en='scan cancelled')
        elif self.state == 'ready_scan' and len(self.pairs) >= len(self.robot_rows):
            self.say('ทิ้งพิกัดหุ่นของจุดนี้ ใส่ใหม่ในเทอร์มินัล', en='robot xyz cleared - re-enter in terminal')
            self.prompt_robot()
        elif self.pairs:
            self.pairs.pop()
            self.write_csv()
            self.T = None
            self.say(f'ลบจุดล่าสุดแล้ว เหลือ {len(self.pairs)} จุด', en=f'last point removed ({len(self.pairs)} left)')
            self.prompt_robot()

    def reset(self):
        self.pairs, self.T = [], None
        self.write_csv()
        self.say('เริ่มใหม่ทั้งหมด', en='reset')
        self.prompt_robot()

    # ---------------------------------------------------------------- display loop
    def on_display(self):
        if self.state in ('await_robot', 'ready_scan'):
            self.poll_terminal()                     # รับพิกัดหุ่นได้แม้ยังไม่มีภาพ
        else:
            self._drain()
        with self.lock:
            fr = self.latest
        if fr is None:
            cv2.waitKey(1)
            return
        color, depth, header, ox, oy = fr
        dets = find_yellow_positions(color, depth, self.intrinsics, (ox, oy),
                                     self.hsv_lo, self.hsv_hi, self.min_area)
        full = [dd for dd in dets if not dd['touches_edge']]     # ชิ้นที่เห็นเต็ม (ไม่ชิดขอบ)
        target = full[0] if full else None
        self.target = target

        if target is not None:
            p = PointStamped()
            p.header = header
            p.point.x, p.point.y, p.point.z = (float(v) * 0.001 for v in target['xyz_mm'])
            self.cam_pub.publish(p)

        if self.state == 'capturing':
            self.feed_capture(target, header)

        self.draw(color, dets, target)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            self.start_capture()
        elif key == ord('u'):
            self.undo()
        elif key == ord('f'):
            self.do_fit()
        elif key == ord('r'):
            self.reset()
        elif key == ord('q'):
            rclpy.shutdown()

    def draw(self, color, dets, target):
        sc = self.scale if self.scale > 0 else 1.0
        img = cv2.resize(color, None, fx=sc, fy=sc,
                         interpolation=cv2.INTER_CUBIC if sc > 1 else cv2.INTER_AREA)

        def text(t, org, col=(0, 255, 255), size=0.6):
            cv2.putText(img, t, org, cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, t, org, cv2.FONT_HERSHEY_SIMPLEX, size, col, 1, cv2.LINE_AA)

        for dd in dets:
            x, y, w, h = (int(v * sc) for v in dd['bbox'])
            good = dd is target
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255) if good else (0, 0, 255), 2)
            u, v = int(dd['uv_crop'][0] * sc), int(dd['uv_crop'][1] * sc)
            cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
            X, Y, Z = dd['xyz_mm']
            lines = [f'cam {X:.1f} {Y:.1f} {Z:.1f} mm']
            if dd['touches_edge']:
                lines.append('touches edge: not used')
            if good and self.T is not None:
                rx, ry, rz = self.T.cam_to_robot(dd['xyz_mm'])
                lines.append(f'robot {rx:.1f} {ry:.1f} {rz:.1f} mm')
            for i, t in enumerate(lines):
                text(t, (x, max(y - 8 - 20 * (len(lines) - 1 - i), 18)))

        status = {'await_robot': 'type robot x y z (mm) in the terminal',
                  'ready_scan': 'press SPACE to scan',
                  'capturing': f'scanning {len(self.samples)}/{self.n_samples}',
                  'done': 'calibrated - verify with a new position'}[self.state]
        if self.intrinsics is None:
            status = 'waiting for camera_info'
        text(f'point {min(len(self.pairs) + 1, self.num_points)}/{self.num_points} | {status}',
             (6, 20), (0, 255, 0), 0.6)
        if self.robot_pending is not None and self.state in ('ready_scan', 'capturing'):
            rp = self.robot_pending
            text(f'robot input: {rp[0]:.1f} {rp[1]:.1f} {rp[2]:.1f}', (6, 42), (255, 200, 0), 0.55)
        if self.msg and time.monotonic() < self.msg_until:
            text(self.msg[:90], (6, img.shape[0] - 8), (255, 255, 255), 0.5)
        cv2.imshow(self.win, img)


def fit_from_csv(path, model, out):
    P, Q = load_pairs_csv(path)
    r = calibrate(P, Q, model)
    print(format_report(P, Q, r))
    CameraToRobot(r['M']).save(out, P, Q, r)
    print(f'บันทึก {out}')


def main(args=None):
    argv = sys.argv[1:] if args is None else list(args)
    if '--fit' in argv:
        ap = argparse.ArgumentParser()
        ap.add_argument('--fit', required=True, help='CSV ที่ node บันทึกไว้ (calib_points.csv)')
        ap.add_argument('--model', default='rigid', choices=['rigid', 'affine'])
        ap.add_argument('--out', default='handeye_calibration.json')
        a, _ = ap.parse_known_args(argv)
        fit_from_csv(a.fit, a.model, a.out)
        return
    rclpy.init(args=args)
    node = YellowCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
