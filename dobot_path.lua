-- Version: Lua 5.3.5
-- อ่านคำสั่งจาก modbus_server.py (Windows LAN 192.168.1.24) แล้วเคลื่อนที่
-- reg 0 = คำสั่ง (0=P1, 1=ท่าเริ่มต้น HOME, 2=พิกัด, 3=เส้นทาง), reg 1-8 = x,y,z,r (float32),
-- reg 9 = seq, reg 10 = สถานะ (0 ว่าง, 1 เคลื่อนที่, 2 หยุดรอเพราะพบมือ), reg 11 = รหัสยืนยันตัวตนของ server (MAGIC),
-- reg 13 = hold จาก hand guard (1 = หยุดรอ, 0 = เดินได้ ; modbus_server.py คำนวณจากที่ detect_edit_12.py ส่งมา)
-- reg 18 = หมุน R กลับที่ตำแหน่งที่ 2 (1 = ปุ่ม 6, 0 = ปุ่ม 7/ไม่หมุนกลับ),
-- reg 19 = มุมที่ต้องบวก/ลบให้ R (int16, องศา x 10),
-- reg 20 = จำนวน waypoint N, reg 21.. = x,y,z (int16, mm x 10)

local SERVER_IP = "192.168.1.24"   -- ** ต้องเป็น IP ของเครื่อง Windows ที่รัน modbus_server.py (เช็กด้วย ipconfig) **

err, id = ModbusCreate(SERVER_IP, 502, 1)
if err ~= 0 then
    print("Create failed:", err)
    return
end

-- ---------------- ตรวจว่าต่อถูก server ----------------
-- modbus_server.py ตั้ง reg 11 = 0x4442 ไว้เสมอ ถ้าอ่านได้ค่าอื่น แปลว่า IP นี้ไม่ใช่ server ของเรา
local MAGIC_REG = 11
local MAGIC     = 0x4442
local m = GetHoldRegs(id, MAGIC_REG, 1, "U16")
if m == nil or m[1] == nil or math.floor(m[1] + 0.5) ~= MAGIC then
    print("ต่อ " .. SERVER_IP .. ":502 ได้ แต่ไม่ใช่ modbus_server.py (reg 11 = " .. tostring(m and m[1]) ..
          ") -> เช็ก IP ของเครื่อง Windows / ปิด server ตัวเก่าที่ค้างอยู่ แล้วรันใหม่")
    ModbusClose(id)
    return
end

-- ---------------- ตั้งค่าเส้นทาง ----------------
local PATH_N_REG    = 20
local PATH_DATA_REG = 21
local PATH_MAX      = 40
local PATH_SCALE    = 10.0   -- ต้องตรงกับ modbus_scale ของ detect_edit_4.py
local PATH_R        = 103.57 -- R พื้นฐานของหัวจับ (= R ของท่าเริ่มต้น ตอนที่หัวจับขนานกับแกนภาพ/เส้นกึ่งกลางกล้อง)
local R_REG         = 19     -- register มุมที่ต้องบวก/ลบให้ R (จาก detect_edit_4.py)
local R_SCALE       = 10.0   -- ต้องตรงกับ modbus_r_scale
local R_MAX         = 90     -- มุมบวก/ลบสูงสุดที่ยอมรับ (องศา)
local R_RET_REG     = 18     -- 1 = หมุน R กลับเป็น PATH_R ที่ตำแหน่งที่ 2 (ปุ่ม 6), 0 = คง R ตามมุม tilt ตลอดทาง (ปุ่ม 7)
local curR          = PATH_R -- R ที่ใช้อยู่ตอนนี้ (ตั้งใหม่ทุกครั้งที่เริ่มเส้นทาง)

-- ---------------- หยุดเมื่อพบมือ (hand guard) ----------------
-- MG400 สั่งหยุดกลางคำสั่ง MovL/MovJ จาก Lua ไม่ได้ จึงแบ่งการเดินตามเส้นทางเป็นช่วงสั้น ๆ
-- แล้วอ่าน reg 13 ก่อนทุกช่วง: 1 = หยุดรอที่ปลายช่วงปัจจุบันจนกว่าจะเป็น 0 แล้วเดินต่อจากจุดเดิม
-- (ช่วงสั้น = หยุดได้เร็วขึ้น แต่การเคลื่อนที่จะกระตุกมากขึ้น)
local HOLD_REG = 13
-- ความถี่การเช็ก: ช่วงยาวขึ้น = เช็กน้อยลง กระตุกน้อยลง แต่หุ่นเดินต่อได้ไกลขึ้นก่อนหยุด
local STEP_PATH_MM    = 45   -- ช่วงตามเส้นทางระหว่าง node (เดิม 15 -> เช็กน้อยลง 3 เท่า)
local STEP_TRANSIT_MM = 180  -- ขาเข้า (ท่าเริ่มต้น -> เหนือ node ที่ 1) และขาออก (เหนือ node สุดท้าย -> ท่าเริ่มต้น)
                             -- (เดิม 30 -> เช็กน้อยลง 6 เท่า)
local STEP_DEG        = 45   -- มุม R ต่อช่วง (เดิม 15 -> น้อยลง 3 เท่า)
local cur      = nil     -- ตำแหน่งที่สั่งไปล่าสุด {x, y, z, r} (nil = ไม่ทราบ)

-- ---------------- ตั้งค่าหัวดูด ----------------
local DO_PRESSURE     = 1    -- DO1 = เป่าลม (ช่วยปล่อยชิ้นงาน)
local DO_VACUUM       = 2    -- DO2 = ดูด
local SUCK_WAIT_MS    = 500  -- รอให้ดูดติดก่อนยก
local BLOW_MS         = 200  -- เวลาเป่าลมตอนปล่อย (0 = ไม่เป่า)
local RELEASE_WAIT_MS = 200  -- รอหลังปล่อยก่อนยกขึ้น

-- ท่าเริ่มต้น: รออยู่ที่นี่จนกว่าจะมีเส้นทางส่งมา และกลับมาที่นี่ทุกครั้งหลังทำเสร็จ
local HOME = {coordinate = {-2.11, 255.91, 0, 103.57}, tool = 0, user = 0}
local SAFE_Z = 0         -- ความสูง (mm) ตอนเข้าไปเหนือจุดหยิบ และตอนยกขึ้นหลังวางก่อนกลับท่าเริ่มต้น

-- รวม 2 register (word สูง, word ต่ำ) เป็น float32
local function f32(hi, lo)
    return (string.unpack(">f", string.pack(">I2I2", hi, lo)))
end

-- GetHoldRegs ของ Dobot คืนค่าเป็น float (เช่น 6.0) -> แปลงเป็นจำนวนเต็มจริง
-- สำคัญมาก: ถ้าส่ง float เป็น address/จำนวน register ให้ GetHoldRegs จะอ่านไม่สำเร็จ (คืน nil)
local function toint(v)
    if v == nil then return nil end
    return math.tointeger(v) or math.floor(v + 0.5)
end

-- uint16 -> int16 (two's complement)
local function s16(v)
    if v >= 32768 then return v - 65536 end
    return v
end

local function setStatus(v)
    SetHoldRegs(id, 10, 1, {v}, "U16")
end

-- อ่านมุมที่ต้องบวก/ลบให้ R (องศา) ; อ่านไม่ได้ = 0
local function readRDelta()
    local rr = GetHoldRegs(id, R_REG, 1, "U16")
    if rr == nil or rr[1] == nil then return 0 end
    local d = s16(toint(rr[1])) / R_SCALE
    if math.abs(d) > R_MAX then
        print("มุม R เกินขอบเขต: " .. tostring(d) .. " -> ใช้ 0")
        return 0
    end
    return d
end

-- อ่านว่าให้หมุน R กลับที่ตำแหน่งที่ 2 หรือไม่ ; อ่านไม่ได้/ค่าอื่น = ไม่หมุนกลับ
local function readRReturn()
    local rr = GetHoldRegs(id, R_RET_REG, 1, "U16")
    return rr ~= nil and toint(rr[1]) == 1
end

-- อ่านเส้นทางทั้งหมดจาก server (อ่านเป็นช่วงละไม่เกิน 60 register)
local function readPath()
    local nr = GetHoldRegs(id, PATH_N_REG, 1, "U16")
    if nr == nil or nr[1] == nil then
        print("อ่านจำนวน waypoint (reg " .. PATH_N_REG .. ") ไม่ได้")
        return nil
    end
    local n = toint(nr[1])
    if n < 2 or n > PATH_MAX then
        print("จำนวน waypoint ไม่ถูกต้อง: " .. tostring(n))
        return nil
    end
    local raw, addr, left = {}, PATH_DATA_REG, 3 * n
    while left > 0 do
        local cnt = math.min(left, 60)
        local part = GetHoldRegs(id, addr, cnt, "U16")
        if part == nil or #part < cnt then
            print(string.format("อ่าน waypoint reg %d..%d ไม่ได้", addr, addr + cnt - 1))
            return nil
        end
        for i = 1, cnt do raw[#raw + 1] = toint(part[i]) end
        addr, left = addr + cnt, left - cnt
    end
    local pts = {}
    for i = 0, n - 1 do
        pts[i + 1] = { s16(raw[3 * i + 1]) / PATH_SCALE,
                       s16(raw[3 * i + 2]) / PATH_SCALE,
                       s16(raw[3 * i + 3]) / PATH_SCALE }
    end
    return pts
end

local function P(p)
    return {coordinate = {p[1], p[2], p[3], curR}, tool = 0, user = 0}
end

-- ปิดทั้งดูดและเป่า
local function airOff()
    DO(DO_VACUUM, OFF)
    DO(DO_PRESSURE, OFF)
end

-- on = true : ดูดชิ้นงาน (DO2 ON, DO1 OFF)
-- on = false: ปล่อยชิ้นงาน (DO2 OFF แล้วเป่า DO1 สั้น ๆ)
local function grip(on)
    Sync()                                   -- ให้ถึงตำแหน่งก่อนสั่งหัวดูด
    if on then
        DO(DO_PRESSURE, OFF)
        DO(DO_VACUUM, ON)
        Sleep(SUCK_WAIT_MS)
    else
        DO(DO_VACUUM, OFF)
        if BLOW_MS > 0 then
            DO(DO_PRESSURE, ON)
            Sleep(BLOW_MS)
            DO(DO_PRESSURE, OFF)
        end
        Sleep(RELEASE_WAIT_MS)
    end
end

-- hold จาก server: 1 = หยุด ; อ่านไม่ได้ = หยุดไว้ก่อน (ไม่รู้ว่ามีมืออยู่หรือเปล่า)
local function holdActive()
    local h = GetHoldRegs(id, HOLD_REG, 1, "U16")
    if h == nil or h[1] == nil then return true end
    return toint(h[1]) ~= 0
end

-- ถ้ามี hold: หยุดรอ (สถานะ 2) จนกว่าจะปลด แล้วกลับเป็นสถานะ 1
local function waitClear()
    if not holdActive() then return end
    print("พบมือในพื้นที่ -> หยุดรอ")
    setStatus(2)
    while holdActive() do Sleep(50) end
    setStatus(1)
    print("ไม่พบมือแล้ว -> เคลื่อนที่ต่อ")
end

local function lerp(a, b, t) return a + (b - a) * t end

local function angDiff(a, b)
    local d = b - a
    if d > math.pi then d = d - 2 * math.pi elseif d < -math.pi then d = d + 2 * math.pi end
    return d
end

-- จุดระหว่าง a -> b ที่สัดส่วน t (a, b = {x, y, z, r})
-- linear = เส้นตรง (MovL) ; ไม่ linear = ไล่รัศมี/มุมรอบฐานหุ่น (คล้ายแนวที่ MovJ เดินจริง ไม่ตัดเข้าใกล้ฐาน)
local function interp(a, b, t, linear)
    if linear then
        return {lerp(a[1], b[1], t), lerp(a[2], b[2], t), lerp(a[3], b[3], t), lerp(a[4], b[4], t)}
    end
    local ra, rb = math.sqrt(a[1] ^ 2 + a[2] ^ 2), math.sqrt(b[1] ^ 2 + b[2] ^ 2)
    local ta = math.atan(a[2], a[1])
    local th = ta + angDiff(ta, math.atan(b[2], b[1])) * t
    local rr = lerp(ra, rb, t)
    return {rr * math.cos(th), rr * math.sin(th), lerp(a[3], b[3], t), lerp(a[4], b[4], t)}
end

local function nSteps(a, b, linear, stepMM)
    local dist
    if linear then
        dist = math.sqrt((b[1] - a[1]) ^ 2 + (b[2] - a[2]) ^ 2 + (b[3] - a[3]) ^ 2)
    else
        local ra, rb = math.sqrt(a[1] ^ 2 + a[2] ^ 2), math.sqrt(b[1] ^ 2 + b[2] ^ 2)
        local d = angDiff(math.atan(a[2], a[1]), math.atan(b[2], b[1]))
        dist = math.sqrt((rb - ra) ^ 2 + ((ra + rb) / 2 * d) ^ 2 + (b[3] - a[3]) ^ 2)
    end
    return math.max(1, math.ceil(dist / stepMM), math.ceil(math.abs(b[4] - a[4]) / STEP_DEG))
end

local function pose(q)
    return {coordinate = {q[1], q[2], q[3], q[4]}, tool = 0, user = 0}
end

-- เคลื่อนที่ไป p = {x, y, z, r} ทีละช่วง (ช่วงละไม่เกิน stepMM, ค่าเริ่มต้น STEP_PATH_MM) เช็ก hold ก่อนทุกช่วง
local function moveTo(p, linear, stepMM)
    local a = cur
    local n = (a == nil) and 1 or nSteps(a, p, linear, stepMM or STEP_PATH_MM)
    for k = 1, n do
        waitClear()
        local q = (k == n) and p or interp(a, p, k / n, linear)
        if linear then MovL(pose(q)) else MovJ(pose(q)) end
        Sync()
        cur = q
    end
end

-- pts[1] = จุดหยิบ, pts[2] = เหนือจุดหยิบ, ..., pts[n-1] = เหนือจุดวาง, pts[n] = จุดวาง
-- เข้าหยิบ: ท่าเริ่มต้น -> x, y ของจุดหยิบที่ z = SAFE_Z -> ลงตรง ๆ ไปจุดหยิบ
-- วางเสร็จ: ยกขึ้น pts[n-1] -> ยกตรงขึ้นถึง z = SAFE_Z (xy เดิม) -> กลับท่าเริ่มต้น
-- rReturn = true (ปุ่ม 6): ยกขึ้นถึง pts[2] แล้วหมุน R กลับเป็น PATH_R (หมุนกลับเท่ามุม tilt) ก่อนเดินต่อ
-- rReturn = false (ปุ่ม 7 / ค่าเริ่มต้น): คง R = PATH_R + rDelta ตลอดเส้นทางเหมือนเดิม
-- ทุกการเคลื่อนที่ผ่าน moveTo: พบมือ (reg 13 = 1) = หยุดรอ แล้วเดินต่อเมื่อไม่มีมือ
local function runPath(pts, rDelta, rReturn)
    local n = #pts
    local function at(p) return {p[1], p[2], p[3], curR} end
    curR = PATH_R + rDelta                   -- หมุนหัวจับให้ขนานกับวัตถุ
    moveTo({pts[1][1], pts[1][2], SAFE_Z, curR}, false, STEP_TRANSIT_MM)   -- ขาเข้า: ไป x, y ของจุดหยิบที่ z = SAFE_Z
    moveTo(at(pts[1]), true)                 -- ลงตรง ๆ ไปจุดหยิบ
    waitClear()
    grip(true)                               -- ดูด
    moveTo(at(pts[2]), true)                 -- ยกขึ้นตรง ๆ ถึงตำแหน่งที่ 2 (เหนือจุดหยิบ)
    if rReturn and rDelta ~= 0 then
        curR = PATH_R                        -- หมุน R กลับตามมุม tilt ที่ตำแหน่งที่ 2 (xyz เดิม หมุนอย่างเดียว)
        moveTo(at(pts[2]), false)
    end
    for i = 3, n do moveTo(at(pts[i]), true) end   -- เดินตามเส้นทาง (เส้นตรงทีละช่วง) จนถึงจุดวาง
    waitClear()
    grip(false)                              -- ปล่อย
    moveTo(at(pts[n - 1]), true)             -- ยกขึ้นจากจุดวาง
    moveTo({pts[n - 1][1], pts[n - 1][2], SAFE_Z, curR}, true)   -- ยกขึ้นตรง ๆ ต่อจนถึง z = SAFE_Z
    curR = PATH_R
    moveTo(HOME.coordinate, false, STEP_TRANSIT_MM)   -- ขาออก: กลับท่าเริ่มต้น รอคำสั่งรอบถัดไป
end

-- เริ่มต้น: ปิดลมทั้งหมดก่อน กันค่าค้างจากรอบก่อน
airOff()

print("ไปท่าเริ่มต้น แล้วรอเส้นทางจาก server")
MovJ(HOME)
Sync()
cur = HOME.coordinate
setStatus(0)

-- เริ่มจาก seq ปัจจุบัน เพื่อไม่ให้ทำคำสั่งเก่าที่ค้างอยู่ตอนเปิดสคริปต์
-- อ่านซ้ำจนกว่าจะได้ (ถ้าอ่านพลาดแล้วใช้ 0 จะไปทำคำสั่งเก่าทันที)
local first = GetHoldRegs(id, 9, 1, "U16")
while first == nil or first[1] == nil do
    Sleep(100)
    first = GetHoldRegs(id, 9, 1, "U16")
end
local lastSeq = toint(first[1])

while true do
    local r = GetHoldRegs(id, 0, 10, "U16")   -- r[1]=reg0 ... r[10]=reg9

    if r ~= nil and r[10] ~= nil and toint(r[10]) ~= lastSeq then
        lastSeq = toint(r[10])
        local cmd = toint(r[1])
        local tag = string.format("#%d", lastSeq)
        setStatus(1)

        if cmd == 0 then
            if P1 == nil then
                -- ยังไม่ได้สอนจุด P1 ใน Points ของโปรเจกต์นี้ -> MovJ(P1) จะ error แล้วสคริปต์หยุด
                print(tag .. " ไม่มีจุด P1 ในโปรเจกต์ (ไปสอนจุดก่อน) -> กลับท่าเริ่มต้นแทน")
                MovJ(HOME)
                cur = HOME.coordinate
            else
                print(tag .. " MovJ(P1)")
                MovJ(P1)
                cur = (type(P1) == "table" and type(P1.coordinate) == "table") and P1.coordinate or nil
            end
        elseif cmd == 1 then
            print(tag .. " กลับท่าเริ่มต้น")
            MovJ(HOME)
            cur = HOME.coordinate
        elseif cmd == 2 then
            local x  = f32(r[2], r[3])
            local y  = f32(r[4], r[5])
            local z  = f32(r[6], r[7])
            local rr = f32(r[8], r[9])
            print(string.format("%s MovJ -> %.1f, %.1f, %.1f, %.1f", tag, x, y, z, rr))
            MovJ({coordinate = {x, y, z, rr}, tool = 0, user = 0})
            cur = {x, y, z, rr}
        elseif cmd == 3 then
            local pts = readPath()
            if pts then
                local rDelta = readRDelta()
                local rReturn = readRReturn()
                print(string.format("%s เดินตามเส้นทาง %d waypoint, R = %.2f %+.1f = %.2f, %s",
                                    tag, #pts, PATH_R, rDelta, PATH_R + rDelta,
                                    rReturn and "หมุน R กลับเป็น " .. string.format("%.2f", PATH_R) .. " ที่ตำแหน่งที่ 2"
                                            or "คง R ตลอดทาง"))
                for i, p in ipairs(pts) do
                    print(string.format("   %d: %.1f, %.1f, %.1f", i, p[1], p[2], p[3]))
                end
                runPath(pts, rDelta, rReturn)
            else
                print(tag .. " อ่านเส้นทางไม่สำเร็จ ข้ามคำสั่งนี้ (อยู่ที่ท่าเริ่มต้น)")
            end
        else
            print(tag .. " คำสั่งไม่รู้จัก (" .. tostring(cmd) .. ") ไม่ขยับ")
        end

        Sync()        -- รอให้เคลื่อนที่เสร็จก่อน
        setStatus(0)
    end

    Sleep(50)
end
