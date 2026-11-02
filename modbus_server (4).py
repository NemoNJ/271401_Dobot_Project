# modbus_server.py
# Modbus TCP Slave: รับข้อมูลจาก Ubuntu (detect_edit_4.py) / คีย์บอร์ด แล้วให้ Dobot มาอ่านค่าไปเคลื่อนที่
#
# Register map (Holding Registers)
#   0       : คำสั่ง  0 = MovJ(P1), 1 = Go(InitialPose), 2 = MovJ ไปพิกัดใน reg 1-8,
#                     3 = เดินตามเส้นทาง waypoint ใน reg 20.. (จาก detect_edit_4.py)
#   1-8     : พิกัดเป้าหมาย x, y, z, r  (float32 ค่าละ 2 register, big-endian, word สูงก่อน)
#   9       : ลำดับคำสั่ง (seq) เพิ่มขึ้นทุกครั้งที่มีคำสั่งใหม่ -> Dobot ทำงานเมื่อ seq เปลี่ยน
#   10      : สถานะที่ Dobot เขียนกลับมา  0 = ว่าง/เสร็จแล้ว, 1 = กำลังเคลื่อนที่, 2 = หยุดรอเพราะพบมือ (hold)
#   11      : รหัสยืนยันตัวตน (MAGIC = 0x4442) ค่าคงที่ ห้ามเขียน ; Lua อ่านตอนเริ่มเพื่อเช็กว่าต่อถูกเครื่อง
#   12      : hand guard จาก Ubuntu (detect_edit_12.py เขียนทุกครั้งที่ตรวจมือ ~5 ครั้ง/วินาที)
#             0 = ไม่มีมือ, 1 = พบมือระหว่างหุ่นทำงาน ให้หยุด, 2 = ปิด hand guard (ไม่ต้องเฝ้าสัญญาณ)
#   13      : hold ที่ Lua อ่าน (server คำนวณ ห้ามเขียน) 1 = หยุดรอ, 0 = เดินได้
#             = 1 เมื่อ reg 12 = 1 หรือใช้ hand guard อยู่แต่ไม่ได้รับ reg 12 เกิน HOLD_TIMEOUT_S (Ubuntu ค้าง/หลุด)
#   18      : หมุน R กลับที่ตำแหน่งที่ 2  1 = หมุนกลับ (ปุ่ม 6 ฝั่ง Ubuntu), 0 = คง R ตามมุม tilt ตลอดทาง (ปุ่ม 7)
#   19      : มุมที่ต้องบวก/ลบให้ R ของหัวจับ (องศา x 10, int16) จาก detect_edit_4.py ; Lua เอาไปบวกกับ R เริ่มต้น
#   20      : จำนวน waypoint N (1-40)
#   21..    : x1,y1,z1, x2,y2,z2, ... (int16 two's complement, ค่า = mm x PATH_SCALE)
#
# detect_edit_4.py ส่งแบบนี้ (กด 's' ที่หน้าต่างภาพ):
#   1) function 0x10 เขียน [N, x1,y1,z1, ...] ที่ modbus_data_reg
#   2) function 0x06 เขียน modbus_run_cmd ลง modbus_cmd_reg
# จึงต้องรัน detect_edit_4.py ด้วย parameter ให้ตรงกับ map นี้:
#   ros2 run ... detect_edit_4.py --ros-args \
#       -p handeye_file:="$PWD/handeye_calibration.json" \
#       -p modbus_enable:=true -p modbus_ip:=<IP เครื่องนี้> \
#       -p modbus_data_reg:=20 -p modbus_run_cmd:=3 -p modbus_scale:=10.0 -p modbus_r_reg:=19 -p modbus_rret_reg:=18

import socket
import struct
import threading
import time
from datetime import datetime

HOST = "0.0.0.0"
PORT = 502         # Linux/Mac ต้อง sudo, Windows รันแบบ Administrator

UBUNTU_IP = "192.168.0.247"

# None = รับทุก IP, หรือใส่ {"192.168.0.247", "<IP ของ Dobot>"} เพื่อล็อก
ALLOWED_IPS = None

LOG_EVERY_READ = False
LOG_PATH_READS = True     # log การที่ Dobot อ่านเส้นทาง/มุม R (reg 19 ขึ้นไป) ไว้ตรวจว่า Dobot อ่านถึงไหน (อ่านเฉพาะตอนมีคำสั่งใหม่ ไม่ถี่)

# ขอบเขตพิกัดที่ยอมให้ส่ง (mm / องศา) — ** ต้องปรับให้ตรงกับพื้นที่ทำงานจริงของหุ่น **
# ค่าที่อยู่นอกช่วงนี้จะถูกปฏิเสธ ไม่ส่งไปให้หุ่น (ใช้ตรวจทุก waypoint ของเส้นทางด้วย)
LIMITS = {
    "x": (150.0, 400.0),
    "y": (-250.0, 250.0),
    "z": (-150.0, 150.0),
    "r": (-180.0, 180.0),
}

REG_CMD, REG_TARGET, REG_SEQ, REG_STATUS = 0, 1, 9, 10
REG_MAGIC, MAGIC = 11, 0x4442   # ต้องตรงกับ MAGIC_REG / MAGIC ในสคริปต์ Lua
REG_HOLD_REQ, REG_HOLD = 12, 13  # 12: Ubuntu เขียน (hand guard), 13: Lua อ่าน (HOLD_REG ใน Lua)
HOLD_TIMEOUT_S = 1.5             # ใช้ hand guard อยู่แต่ไม่ได้รับสัญญาณเกินนี้ = สั่งหยุดไว้ก่อน
STATUS_TEXT = {0: 'เสร็จแล้ว / ว่าง', 1: 'กำลังเคลื่อนที่...', 2: 'หยุดรอ (พบมือ)'}
REG_PATH_RRET, REG_PATH_R, REG_PATH_N, REG_PATH_DATA = 18, 19, 20, 21
R_SCALE = 10.0           # ต้องตรงกับ modbus_r_scale ใน detect_edit_4.py
R_MAX_DEG = 90.0         # มุม R ที่ยอมรับ (บวก/ลบ) เกินนี้ปฏิเสธ
PATH_MAX = 40            # ต้องตรงกับ MODBUS_MAX_WAYPOINTS ใน detect_edit_4.py
PATH_SCALE = 10.0        # ต้องตรงกับ modbus_scale ใน detect_edit_4.py
PATH_END = REG_PATH_DATA + 3 * PATH_MAX          # register ตัวถัดจากพื้นที่เส้นทาง
CMD_P1, CMD_HOME, CMD_TARGET, CMD_PATH = 0, 1, 2, 3
NUM_REGS = 256

registers = [0] * NUM_REGS
registers[REG_CMD] = CMD_HOME
registers[REG_MAGIC] = MAGIC
lock = threading.Lock()
hold_state = {'active': False, 'last': 0.0}   # active = Ubuntu ใช้ hand guard (เคยส่ง 0/1 มา)
print_lock = threading.Lock()


def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    with print_lock:
        print(f"\r[{ts}] {msg}\nค่า> ", end="", flush=True)


def label(ip):
    return "Ubuntu" if ip == UBUNTU_IP else "Dobot/อื่นๆ"


def read_target(regs):
    """อ่านพิกัด x,y,z,r (float32) จาก register 1-8"""
    raw = struct.pack(">8H", *regs[REG_TARGET:REG_TARGET + 8])
    return struct.unpack(">4f", raw)


def read_target_locked():
    return read_target(registers)


def path_r_deg(regs):
    """มุม R ที่ต้องบวก/ลบ (องศา) จาก reg 19"""
    v = regs[REG_PATH_R]
    return (v - 65536 if v >= 32768 else v) / R_SCALE


def path_rret(regs):
    """True = หมุน R กลับที่ตำแหน่งที่ 2 (reg 18 = 1)"""
    return regs[REG_PATH_RRET] == 1


def decode_path(regs):
    """อ่านเส้นทางจาก reg 20.. -> (list ของ (x,y,z) mm, None) หรือ (None, ข้อความ error)"""
    n = regs[REG_PATH_N]
    if not 2 <= n <= PATH_MAX:
        return None, f"จำนวน waypoint = {n} (ต้องอยู่ระหว่าง 2-{PATH_MAX})"
    raw = regs[REG_PATH_DATA:REG_PATH_DATA + 3 * n]
    vals = [(v - 65536 if v >= 32768 else v) / PATH_SCALE for v in raw]
    return [tuple(vals[i:i + 3]) for i in range(0, 3 * n, 3)], None


def fmt_path(pts):
    return " -> ".join(f"({x:.1f},{y:.1f},{z:.1f})" for x, y, z in pts)


def describe(cmd, target=None, r_deg=None, rret=False):
    if cmd == CMD_P1:
        return "MovJ(P1)"
    if cmd == CMD_TARGET:
        x, y, z, r = target
        return f"MovJ ไป ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})"
    if cmd == CMD_PATH:
        rtxt = f" หมุนหัวจับ R {r_deg:+.1f}°" if r_deg is not None else ""
        if rret:
            rtxt += " แล้วหมุนกลับที่ตำแหน่งที่ 2"
        return (f"เดินตามเส้นทาง {len(target)} waypoint{rtxt}" if target else "เดินตามเส้นทาง (ข้อมูลไม่ถูกต้อง)")
    return "Go(InitialPose)"


def target_for(cmd, regs):
    if cmd == CMD_TARGET:
        return read_target(regs)
    if cmd == CMD_PATH:
        return decode_path(regs)[0]
    return None


def bump_seq_locked():
    registers[REG_SEQ] = registers[REG_SEQ] % 65535 + 1


# ------------------------------------------------------- สั่งงานหุ่น ---

def check_limits(x, y, z, r):
    for name, val in zip("xyzr", (x, y, z, r)):
        lo, hi = LIMITS[name]
        if not lo <= val <= hi:
            return f"{name}={val:.1f} อยู่นอกช่วง {lo}..{hi}"
    return None


def check_path(regs):
    """ตรวจเส้นทางใน regs ; คืนข้อความ error หรือ None ถ้าใช้ได้"""
    pts, err = decode_path(regs)
    if err:
        return err
    for i, (x, y, z) in enumerate(pts, 1):
        problem = check_limits(x, y, z, 0.0)
        if problem:
            return f"waypoint #{i} {problem}"
    return None


def send_command(cmd, target=None, source="คีย์บอร์ด"):
    """ตั้งคำสั่งให้ Dobot: cmd 0/1, 2 พร้อม target=(x,y,z,r) หรือ 3 (เส้นทางที่รับไว้ล่าสุด)"""
    if cmd == CMD_TARGET:
        problem = check_limits(*target)
        if problem:
            log(f"[x] {source}: ปฏิเสธพิกัด เพราะ {problem}")
            return False
        words = struct.unpack(">8H", struct.pack(">4f", *target))
    with lock:
        if cmd == CMD_PATH:
            problem = check_path(registers)
            if problem:
                log(f"[x] {source}: ปฏิเสธเส้นทาง เพราะ {problem}")
                return False
            target = decode_path(registers)[0]
        if cmd == CMD_TARGET:
            registers[REG_TARGET:REG_TARGET + 8] = words
        registers[REG_CMD] = cmd
        bump_seq_locked()
        seq = registers[REG_SEQ]
        r_deg = path_r_deg(registers)
        rret = path_rret(registers)
    log(f"[S] {source}: คำสั่ง #{seq} -> Dobot จะทำ {describe(cmd, target, r_deg, rret)}")
    return True


def parse_command(text):
    """แปลงข้อความเป็นคำสั่ง
       "0" / "1"                -> คำสั่งเดิม
       "x,y,z,r" หรือ "x y z r"  -> ไปพิกัด (r ไม่ใส่ก็ได้ = 0)
       คืนค่า (cmd, target) หรือ None ถ้าอ่านไม่ออก"""
    parts = text.replace(",", " ").split()
    if parts and parts[0].lower() in ("go", "move", "movj"):
        parts = parts[1:]
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        return None
    if len(nums) == 1 and nums[0].is_integer() and 0 <= nums[0] <= 65535:
        cmd = int(nums[0])
        return (CMD_HOME if cmd not in (CMD_P1, CMD_HOME) else cmd), None
    if len(nums) == 3:
        nums.append(0.0)
    if len(nums) == 4:
        return CMD_TARGET, tuple(nums)
    return None


# ---------------------------------------------------------------- Modbus ---

def looks_like_modbus(data):
    if len(data) < 8:
        return False
    _tid, pid, length = struct.unpack(">HHH", data[:6])
    return pid == 0 and 2 <= length <= 254


def touches(start, end, lo, hi):
    """ช่วง [start, end) ทับกับ [lo, hi) หรือไม่"""
    return start < hi and end > lo


def validate_write_locked(start, vals):
    """ตรวจก่อนยอมรับการเขียน ; คืน (exception code, ข้อความ) หรือ None ถ้ารับได้"""
    end = start + len(vals)
    if start <= REG_MAGIC < end and vals[REG_MAGIC - start] != MAGIC:
        return 0x03, f"ห้ามเขียน reg {REG_MAGIC} (รหัสยืนยันตัวตนของ server)"
    if start <= REG_HOLD < end:
        return 0x03, f"ห้ามเขียน reg {REG_HOLD} (server คำนวณ hold ให้เอง)"
    if start <= REG_HOLD_REQ < end and vals[REG_HOLD_REQ - start] not in (0, 1, 2):
        return 0x03, f"ค่า hand guard (reg {REG_HOLD_REQ}) ต้องเป็น 0, 1 หรือ 2"
    # ห้ามแก้ข้อมูลเส้นทาง/มุม R ขณะหุ่นกำลังเดินตามเส้นทางเดิม (Dobot อ่านเส้นทางทีละช่วง)
    if touches(start, end, REG_PATH_RRET, PATH_END) and registers[REG_STATUS] in (1, 2):
        return 0x06, "Dobot กำลังเคลื่อนที่อยู่ รอให้เสร็จก่อนแล้วกด 's' ใหม่"
    if start <= REG_PATH_RRET < end and vals[REG_PATH_RRET - start] not in (0, 1):
        return 0x03, f"ปฏิเสธค่าหมุน R กลับ = {vals[REG_PATH_RRET - start]} (ต้องเป็น 0 หรือ 1)"
    if start <= REG_PATH_R < end:
        v = vals[REG_PATH_R - start]
        r_deg = (v - 65536 if v >= 32768 else v) / R_SCALE
        if abs(r_deg) > R_MAX_DEG:
            return 0x03, f"ปฏิเสธมุม R {r_deg:+.1f}° (เกิน ±{R_MAX_DEG:g}°)"
    if not start <= REG_CMD < end:
        return None
    preview = registers[:]
    preview[start:end] = vals
    cmd = preview[REG_CMD]
    if cmd == CMD_TARGET:
        problem = check_limits(*read_target(preview))
        if problem:
            return 0x03, f"ปฏิเสธพิกัด เพราะ {problem}"
    elif cmd == CMD_PATH:
        problem = check_path(preview)
        if problem:
            return 0x03, f"ปฏิเสธเส้นทาง เพราะ {problem}"
    return None


def update_hold_locked():
    """คำนวณ reg 13 (hold ที่ Lua อ่าน) ; คืนข้อความถ้าค่าเปลี่ยน (ถือ lock อยู่)"""
    why = ''
    if not hold_state['active']:
        eff = 0
    elif time.monotonic() - hold_state['last'] > HOLD_TIMEOUT_S:
        eff, why = 1, f'ไม่ได้รับสัญญาณ hand guard จาก Ubuntu เกิน {HOLD_TIMEOUT_S:g} s'
    elif registers[REG_HOLD_REQ] == 1:
        eff, why = 1, 'Ubuntu พบมือในพื้นที่'
    else:
        eff = 0
    if eff == registers[REG_HOLD]:
        return None
    registers[REG_HOLD] = eff
    return f"[H] สั่งหุ่นหยุด: {why}" if eff else "[H] ปลดการหยุด: หุ่นเคลื่อนที่ต่อได้"


def hold_watchdog():
    """เช็กสัญญาณ hand guard ทุก 0.2 s (Ubuntu หลุดระหว่างใช้งาน = สั่งหยุด)"""
    while True:
        time.sleep(0.2)
        with lock:
            msg = update_hold_locked()
        if msg:
            log(msg)


def on_registers_written(who, start, old_vals, new_vals):
    """เรียกหลังมีการเขียน register ผ่าน Modbus (ถือ lock อยู่)"""
    end = start + len(new_vals)
    msgs = []
    if start <= REG_HOLD_REQ < end:              # hand guard (เขียนถี่ ~5 ครั้ง/วินาที: log เฉพาะตอนเปลี่ยน)
        v = new_vals[REG_HOLD_REQ - start]
        was_active = hold_state['active']
        hold_state['active'], hold_state['last'] = v != 2, time.monotonic()
        if v == 2 and was_active is not False:
            msgs.append(f"[H] {who}: ปิด hand guard (ไม่เฝ้าสัญญาณ)")
        elif v != 2 and not was_active:
            msgs.append(f"[H] {who}: เริ่มใช้ hand guard (ขาดสัญญาณเกิน {HOLD_TIMEOUT_S:g} s = สั่งหุ่นหยุด)")
        m = update_hold_locked()
        if m:
            msgs.append(m)
        if end - start == 1:
            return msgs
    if start <= REG_STATUS < end:
        status = new_vals[REG_STATUS - start]
        msgs.append(f"[D] สถานะ Dobot: {STATUS_TEXT.get(status, status)}")
    if start <= REG_PATH_RRET < end:
        msgs.append(f"[P] {who}: " + ("หมุน R กลับที่ตำแหน่งที่ 2 (ปุ่ม 6)" if path_rret(registers)
                                      else "ไม่หมุน R กลับ คง R ตามมุม tilt ตลอดทาง (ปุ่ม 7)"))
    if start <= REG_PATH_R < end:
        msgs.append(f"[P] {who}: มุมหมุนหัวจับ R {path_r_deg(registers):+.1f}° (บวก/ลบจาก R เริ่มต้นฝั่ง Dobot)")
    if touches(start, end, REG_PATH_N, PATH_END):
        pts, err = decode_path(registers)
        if err:
            msgs.append(f"[P] {who}: เขียนข้อมูลเส้นทาง แต่ {err}")
        else:
            problem = check_path(registers)
            note = f"  [!] {problem} -> จะไม่ยอมให้สั่งรัน" if problem else ""
            msgs.append(f"[P] {who}: รับเส้นทาง {len(pts)} waypoint (mm): {fmt_path(pts)}{note}")
    if start <= REG_CMD < end:      # Ubuntu เขียนคำสั่งตรงผ่าน Modbus
        bump_seq_locked()
        cmd = registers[REG_CMD]
        msgs.append(f"[W] {who}: คำสั่ง #{registers[REG_SEQ]} -> Dobot จะทำ "
                    f"{describe(cmd, target_for(cmd, registers), path_r_deg(registers), path_rret(registers))}")
    if not msgs:
        msgs.append(f"[W] {who} เขียน addr {start}..{end - 1}: {old_vals} -> {list(new_vals)}")
    return msgs


def handle_modbus(conn, who, buf):
    last_read = {}

    def need(n):
        nonlocal buf
        while len(buf) < n:
            chunk = conn.recv(4096)
            if not chunk:
                raise ConnectionError("client disconnected")
            buf += chunk

    while True:
        need(7)
        tid, pid, length, uid = struct.unpack(">HHHB", buf[:7])
        need(6 + length)
        pdu = buf[7:6 + length]
        buf = buf[6 + length:]
        fc = pdu[0]
        msgs = []

        if fc == 0x03:  # Read Holding Registers
            start, qty = struct.unpack(">HH", pdu[1:5])
            if qty < 1 or qty > 125 or start + qty > len(registers):
                resp = bytes([fc | 0x80, 0x02])
                msgs.append(f"[!] {who} อ่าน addr {start} x{qty} -> Illegal data address")
            else:
                with lock:
                    vals = registers[start:start + qty]
                resp = bytes([fc, qty * 2]) + struct.pack(f">{qty}H", *vals)
                key = (start, qty)
                if start >= REG_PATH_RRET:      # อ่านค่าหมุนกลับ / มุม R / เส้นทาง: log แบบสั้น (ไม่แสดงค่า) เพื่อดูว่า Dobot อ่านครบไหม
                    if LOG_PATH_READS:
                        msgs.append(f"[R] {who} อ่านเส้นทาง/R addr {start}..{start + qty - 1} ({qty} reg)")
                elif LOG_EVERY_READ or last_read.get(key) != vals:
                    msgs.append(f"[R] {who} อ่าน addr {start}..{start + qty - 1} ได้ค่า {vals}")
                    last_read[key] = vals

        elif fc in (0x06, 0x10):  # Write Single / Multiple Registers
            if fc == 0x06:
                start, value = struct.unpack(">HH", pdu[1:5])
                vals, qty = [value], 1
            else:
                start, qty, _bc = struct.unpack(">HHB", pdu[1:6])
                vals = list(struct.unpack(f">{qty}H", pdu[6:6 + qty * 2])) if qty else []
            if qty < 1 or qty > 123 or start + qty > len(registers) or len(vals) != qty:
                resp = bytes([fc | 0x80, 0x02])
                msgs.append(f"[!] {who} เขียน addr {start} x{qty} -> Illegal data address")
            else:
                with lock:
                    problem = validate_write_locked(start, vals)
                    if problem is None:
                        old = registers[start:start + qty]
                        registers[start:start + qty] = vals
                        msgs += on_registers_written(who, start, old, vals)
                if problem:
                    code, text = problem
                    resp = bytes([fc | 0x80, code])
                    msgs.append(f"[x] {who}: {text} (ไม่ส่งให้หุ่น)")
                else:
                    resp = pdu[:5] if fc == 0x06 else struct.pack(">BHH", fc, start, qty)

        else:
            resp = bytes([fc | 0x80, 0x01])
            msgs.append(f"[!] {who} ส่ง function code 0x{fc:02X} ที่ไม่รองรับ")

        conn.sendall(struct.pack(">HHHB", tid, 0, len(resp) + 1, uid) + resp)
        for m in msgs:
            log(m)


# ------------------------------------------------- ข้อความธรรมดา (raw TCP) ---

def handle_text(conn, who, buf):
    """Ubuntu ส่งเป็นข้อความ เช่น "0", "1", "250,0,50,0" (ปิดท้ายด้วย \\n หรือไม่ก็ได้)"""
    conn.settimeout(0.2)

    def process(text):
        text = text.strip()
        if not text:
            return
        parsed = parse_command(text)
        if parsed is None:
            log(f"[T] {who} ส่งข้อความ: {text!r} (อ่านเป็นคำสั่งไม่ได้ จึงไม่ทำอะไร)")
            return
        log(f"[T] {who} ส่งมา: {text!r}")
        send_command(*parsed, source=who)

    while True:
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            process(line.decode("utf-8", errors="replace"))
        try:
            chunk = conn.recv(4096)
        except socket.timeout:
            if buf:
                process(buf.decode("utf-8", errors="replace"))
                buf = b""
            continue
        if not chunk:
            if buf:
                process(buf.decode("utf-8", errors="replace"))
            raise ConnectionError("client disconnected")
        buf += chunk


# --------------------------------------------------------------- server ---

def handle_client(conn, addr):
    ip = addr[0]
    who = f"{label(ip)} {ip}:{addr[1]}"

    if ALLOWED_IPS is not None and ip not in ALLOWED_IPS:
        log(f"[x] ปฏิเสธการเชื่อมต่อจาก {ip} (ไม่อยู่ใน ALLOWED_IPS)")
        conn.close()
        return

    log(f"[+] {who} เชื่อมต่อเข้ามา")
    try:
        first = conn.recv(4096)
        if not first:
            raise ConnectionError("client disconnected")
        if looks_like_modbus(first):
            handle_modbus(conn, who, first)
        else:
            log(f"[i] {who} ไม่ได้ส่งเป็น Modbus -> อ่านเป็นข้อความธรรมดา")
            handle_text(conn, who, first)
    except (ConnectionError, OSError):
        pass
    finally:
        conn.close()
        log(f"[-] {who} ตัดการเชื่อมต่อ")


def server_loop():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Windows: SO_REUSEADDR ยอมให้ 2 โปรแกรม bind port 502 ซ้ำกันได้ (Dobot อาจไปต่อกับตัวเก่า)
    # จึงใช้ SO_EXCLUSIVEADDRUSE แทน ถ้ามีตัวอื่นจับ port 502 อยู่ จะ error ทันทีให้รู้ตัว
    if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
    else:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)
    log(f"Modbus TCP server รออยู่ที่ {HOST}:{PORT}  (รอ Ubuntu {UBUNTU_IP} และ Dobot)")
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()


def main():
    print("\nสั่งหุ่นได้จาก Ubuntu (detect_edit_4.py กด 's') หรือพิมพ์ที่นี่:")
    print("  0                -> MovJ(P1)")
    print("  1                -> Go(InitialPose)")
    print("  x y z r          -> MovJ ไปพิกัดนั้น เช่น 250 0 50 0  (หรือ 250,0,50,0)")
    print("  p                -> สั่งเดินตามเส้นทางที่รับจาก detect_edit_4.py ล่าสุดอีกครั้ง")
    print("  s                -> ดูสถานะ / ค่า register / เส้นทางตอนนี้")
    print("  q                -> ออกจากโปรแกรม")
    print(f"  ขอบเขตที่อนุญาต: {LIMITS}")
    print(f"  detect_edit_4.py ต้องใช้: -p modbus_data_reg:={REG_PATH_N} "
          f"-p modbus_run_cmd:={CMD_PATH} -p modbus_cmd_reg:={REG_CMD} -p modbus_scale:={PATH_SCALE} -p modbus_r_reg:={REG_PATH_R} "
          f"-p modbus_rret_reg:={REG_PATH_RRET}\n")

    threading.Thread(target=server_loop, daemon=True).start()
    threading.Thread(target=hold_watchdog, daemon=True).start()

    while True:
        text = input("ค่า> ").strip()
        if not text:
            continue
        low = text.lower()
        if low in ("q", "quit", "exit"):
            break
        if low in ("s", "r", "status"):
            with lock:
                cmd, seq, status = registers[REG_CMD], registers[REG_SEQ], registers[REG_STATUS]
                target = target_for(cmd, registers)
                snap = registers[:11]
                pts, err = decode_path(registers)
            log(f"[i] คำสั่งล่าสุด #{seq}: {describe(cmd, target, path_r_deg(registers), path_rret(registers))} | "
                f"Dobot: {STATUS_TEXT.get(status, status)} | hold reg 13 = {registers[REG_HOLD]} | reg 0-10 = {snap}")
            log(f"[i] เส้นทางที่เก็บไว้: {fmt_path(pts) if pts else err}")
            continue
        if low in ("p", "path"):
            send_command(CMD_PATH)
            continue
        parsed = parse_command(text)
        if parsed is None:
            print("รูปแบบไม่ถูกต้อง: ใส่ 0, 1, p หรือ x y z r")
            continue
        send_command(*parsed)


if __name__ == "__main__":
    main()