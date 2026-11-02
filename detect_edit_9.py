#!/usr/bin/env python3
"""
detect_edit_12.py  (ต่อจาก detect_edit_11.py + Hand guard ด้วย MediaPipe Hands)
---------------------
Hand guard (ใหม่ใน detect_edit_12): ตรวจมือด้วย MediaPipe Hands hand_hz ครั้งต่อวินาที (ค่าเริ่มต้น 5) บนภาพเต็มของกล้อง
  - ระหว่างหุ่นทำงาน: พบมือ = สั่งหุ่นหยุดทันที ; ไม่พบมือต่อเนื่อง hand_clear_s (1 s) = หุ่นเคลื่อนที่ต่อจากจุดที่หยุด
    (ไม่มีภาพใหม่จากกล้อง = ไม่ถือว่าพบมือ ไม่สั่งหยุดหุ่น)
  - หุ่นว่าง: ชูนิ้วชี้นิ้วเดียว = เครื่องหมายบวกสีเขียว (ตัวเดียวกับที่ใช้คลิก) เคลื่อนตามปลายนิ้วชี้
    ค้างนิ้วนิ่ง pointer_dwell_s (1.5 s, วงแหวนรอบเครื่องหมายบวกเต็มวง) = เลือกเป็นจุดวาง (end point) เหมือนคลิกซ้าย
  - มีมือในภาพ: dynamic scan หยุดชั่วคราว (มือจะกลายเป็นสิ่งกีดขวางใน map) และกด Start ไม่ได้
  ส่งสถานะผ่าน modbus_server.py: เขียน reg hand_hold_reg (12) ทุกรอบที่ตรวจ: 0 = ไม่มีมือ, 1 = หยุด, 2 = ปิด hand guard
    server คำนวณ reg 13 ให้ Lua อ่าน (และสั่งหยุดเองถ้าไม่ได้รับสัญญาณเกิน 1.5 s ระหว่างใช้ hand guard)
  ** ต้องใช้ modbus_server.py และ dobot_path.lua ชุดเดียวกัน ** (Lua แบ่งการเคลื่อนที่เป็นช่วงสั้น ๆ แล้วเช็ก reg 13 ก่อนทุกช่วง)
  ** เป็นระบบช่วยเท่านั้น ไม่ใช่อุปกรณ์ความปลอดภัย: ต้องมีปุ่มหยุดฉุกเฉินของหุ่นพร้อมกดเสมอ **
  ติดตั้ง: pip install mediapipe
    mediapipe รุ่นที่มี mp.solutions (แบบ finger_tag.py) ใช้ได้เลย ; รุ่นใหม่ (0.10.3x) ไม่มี mp.solutions แล้ว ต้องโหลดโมเดล:
    wget -O hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
    (วางไว้โฟลเดอร์เดียวกับสคริปต์ หรือ -p hand_model_path:=/path/hand_landmarker.task)
  parameters: hand_enable (true), hand_hz (5.0), hand_zone ('frame' = ทั้งภาพกล้อง, 'crop' = เฉพาะกรอบ crop + 20 px),
              hand_clear_s (1.0), hand_hold_reg (12), hand_max_num (2), hand_model_complexity (1),
              hand_min_detection (0.6), hand_min_tracking (0.5), hand_pause_dynamic (true),
              pointer_enable (true), pointer_dwell_s (1.5), pointer_dwell_px (8.0 px ของภาพ crop), hand_model_path ('')

detect_edit_11.py  (ต่อจาก detect_edit_10.py + Dynamic scan map)
---------------------
Dynamic scan map (ใหม่ใน detect_edit_11):
  สแกน map อัตโนมัติ dynamic_scan_hz ครั้งต่อวินาที (ค่าเริ่มต้น 1 fps): point cloud / solid / วางเส้นทาง / แผนที่ 3 มิติ /
  Depth Image / Segmentation อัปเดตเองบน dashboard โดยไม่ต้องกด Scan
  ระหว่างที่หุ่นทำงาน จะ "ไม่สแกน" จนกว่าหุ่นจะทำงานเสร็จ (map และเส้นทางค้างไว้ตามรอบสุดท้ายก่อนหุ่นขยับ):
    - หยุดทันทีที่กด Start (ระหว่างส่ง Modbus + รอหุ่นเริ่มทำงาน)
    - อ่านสถานะหุ่นจาก modbus_server.py reg robot_status_reg (10: 1 = กำลังเคลื่อนที่, 0 = ว่าง ซึ่ง Lua เขียนไว้)
      เห็น 1 แล้วกลับเป็น 0 = ทำงานเสร็จ -> สแกนต่อทันที ; หุ่นไม่เริ่มภายใน robot_start_timeout_s = สแกนต่อ (แจ้งเตือน)
    - หุ่นถูกสั่งจากที่อื่น (สถานะเป็น 1) ก็หยุดสแกนเหมือนกัน
  ปุ่ม Scan (spacebar) ยังใช้ได้: สแกน 1 ครั้ง + เซฟภาพลง dataset ; dynamic scan ไม่เซฟไฟล์ (dynamic_save:=true ถ้าต้องการ)
  เปิด/ปิดด้วยสวิตช์ "Dynamic scan" บน dashboard หรือปุ่ม 'd'
  log ของ dynamic scan ไม่พิมพ์ซ้ำทุกวินาที (เฉพาะคำเตือนที่ต่างจากเดิม)
  parameters: dynamic_scan (true), dynamic_scan_hz (1.0), dynamic_save (false),
              robot_status_reg (10), robot_poll_hz (4.0), robot_start_timeout_s (5.0), robot_max_wait_s (180.0)

detect_edit_10.py  (ต่อจาก detect_edit_9.py + Dashboard บนหน้าเว็บ)
---------------------
Dashboard เต็มจอ (เปิดใน browser อัตโนมัติ ที่ http://localhost:8080/) แทนหน้าต่าง OpenCV / matplotlib เดิมทั้งหมด:
  จอใหญ่แสดงตามปุ่มขวามือ:
    Camera Viewer  = ภาพสด "RealSense D455 - RGB + Depth" (ชี้เมาส์ดูระยะ, คลิกซ้ายตั้งจุดวาง, คลิกขวา = จุดวาง default)
    3D Voxel Map   = "3D height map" (Plotly: ลากเมาส์หมุน, ล้อเมาส์ซูม)
    Depth Image    = "Depth colormap (captured)"
    Segmentation   = "Solids (segmentation)"
  Start = กด 's' เดิม (ส่งเส้นทางทาง Modbus) | Scan = กด spacebar เดิม
  Choose Algorithm = กด 1-5 เดิม (5 = Comparison: เทียบทุก algorithm และ "ส่งเส้นทางที่สั้นที่สุด" ให้หุ่น)
  Choose Mode      = กด 6 / 7 เดิม (หมุน R กลับที่ตำแหน่งที่ 2 / ไม่หมุนกลับ)
  ช่อง Shortest Path & Computational Time: algorithm เดียว = ระยะทาง + เวลาค้นหา ; Comparison = กราฟเทียบทุก algorithm
  ปุ่มลัดคีย์บอร์ดเดิมยังใช้ได้บนหน้า dashboard: SPACE, s, 1-7, a, q
  parameters: dashboard_port (8080), dashboard_host (127.0.0.1 = เปิดได้เฉพาะเครื่องนี้ ; 0.0.0.0 = เปิดจากเครื่องอื่นใน LAN ได้),
              dashboard_open_browser (true), dashboard_plotly_js ('' = หาเองจาก pip plotly ไม่เจอใช้ CDN), dashboard_jpeg_quality (85)
  แผนที่ 3 มิติใช้ Plotly: ถ้าเครื่องไม่มีอินเทอร์เน็ต ให้ pip install plotly (โหนดจะเสิร์ฟไฟล์จากเครื่องเอง)

(คำอธิบายเดิมของ detect_edit_9 / rgbd_stereo_depth_8.py ด้านล่าง: "กดปุ่มที่หน้าต่างภาพ" = ใช้ปุ่มบน dashboard แทน)
ROS 2 Jazzy node สำหรับแสดงภาพ RGB พร้อมค่าความลึก (depth) จากกล้อง
Intel RealSense D455 โดยแสดงระยะทาง (เมตร) ณ ตำแหน่งที่เมาส์ชี้อยู่บนภาพ

Topics ที่ subscribe:
  - /camera/camera/color/image_raw        (RGB)
  - /camera/camera/depth/image_rect_raw   (Depth, หน่วย mm เป็น uint16)

*** สำคัญ: ต้อง launch realsense2_camera พร้อมเปิด align_depth ก่อน ***
เพื่อให้ depth image ถูก align (จับคู่ pixel ต่อ pixel) กับ color image แล้ว
มิฉะนั้นตำแหน่ง pixel ของสอง stream จะไม่ตรงกัน เพราะมุมมอง/ความละเอียด
ของเลนส์ RGB กับเลนส์ depth ไม่เท่ากัน

    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

เมื่อเปิด align_depth แล้ว topic depth ที่ align แล้วจะอยู่ที่:
    /camera/camera/aligned_depth_to_color/image_raw

วิธีใช้:
  - เอาเมาส์ไปชี้จุดใดก็ได้บนภาพ จะมีตัวเลขระยะทาง (เมตร) ขึ้นที่ตำแหน่งนั้น
  - กด "spacebar" เพื่อแคปภาพ (color + depth) เก็บลง folder dataset
    พร้อมสร้างภาพ depth แบบระบายสีตามความลึก (top view: พื้น = สีเข้มสุด,
    ยิ่งสูงขึ้นมาใกล้กล้อง สีไล่เฉดสว่างขึ้น) แสดงในหน้าต่างใหม่ และเซฟเป็น PNG
    และ publish point cloud (PointCloud2) ไปที่ topic /rgbd_viewer/points เพื่อดูใน RViz
  - กด "q" เพื่อออก

ดู point cloud ใน RViz:
  Add > PointCloud2 > Topic: /rgbd_viewer/points
  Reliability = Reliable, Durability = Transient Local, Color Transformer = RGB8
  Fixed Frame = camera_link (หรือ camera_color_optical_frame)

ขนาดหน้าต่าง:
  display_scale : ขยาย/ย่อหน้าต่างภาพ (ค่าเริ่มต้น 3.0 เพราะภาพที่ crop เล็ก ~300x208 px) ใช้กับหน้าต่างภาพสด,
                  depth colormap, solids, mask ; ปรับตอนรันได้ เช่น -p display_scale:=4.0
  หน้าต่างแผนที่ 3 มิติ (matplotlib) ใหญ่ขึ้นเป็น 14x10 นิ้ว (ลากขอบหน้าต่างขยายเพิ่มได้)

Parameters ที่เกี่ยวกับภาพสี depth:
  depth_near_mm : ระยะใกล้สุดที่ใช้ map สี (mm) ค่าเริ่มต้น 500
  depth_far_mm  : ระยะไกลสุด = พื้น (mm) ค่าเริ่มต้น 650
                  (ตั้งทั้งสองเป็น 0 เพื่อให้คำนวณช่วงอัตโนมัติจากภาพแต่ละเฟรม)
  colormap      : turbo | jet | inferno | viridis | plasma  (ค่าเริ่มต้น turbo)
  cloud_topic   : topic ของ point cloud (ค่าเริ่มต้น /rgbd_viewer/points)
  cloud_color   : depth = สีตามความลึก | rgb = สีจริงจากกล้อง (ค่าเริ่มต้น depth)
  cloud_stride  : ข้าม pixel ทุก N จุดเพื่อลดจำนวนจุด (ค่าเริ่มต้น 1 = ครบทุกจุด)
  cloud_max_depth_m : ตัดจุดที่ไกลกว่านี้ทิ้ง (เมตร) ค่าเริ่มต้น 1.0

ตรวจจับวัตถุสีเหลือง (HSV) + วัดขนาดจริง (mm) จาก bounding box:
  ทำงานทุกเฟรมบนภาพที่ crop แล้ว: BGR -> HSV -> inRange -> morphology -> contour
  -> boundingRect แล้วแปลงขนาด pixel เป็น mm ด้วย pinhole:
      width_mm  = w_px * Z / fx
      height_mm = h_px * Z / fy      (Z = median ของ depth ในพื้นที่สีเหลือง, mm)
  (Z คือระยะถึงผิวด้านบนของวัตถุ ขนาดที่ได้จึงเป็นขนาดที่ระดับผิวบน)
  กด spacebar จะ log ขนาดของวัตถุสีเหลืองทุกชิ้นด้วย
  yellow_enable        : เปิด/ปิด (ค่าเริ่มต้น true)
  yellow_h_low/high    : ช่วง Hue ของ OpenCV (0-179) ค่าเริ่มต้น 20 - 35
  yellow_s_low/high    : ช่วง Saturation ค่าเริ่มต้น 100 - 255
  yellow_v_low/high    : ช่วง Value ค่าเริ่มต้น 120 - 255
  yellow_min_area_px   : พื้นที่ contour ขั้นต่ำ (ค่าเริ่มต้น 300)
  yellow_show_mask     : แสดงหน้าต่าง mask เพื่อจูนค่า HSV (ค่าเริ่มต้น false)
  ตัวอย่าง: --ros-args -p yellow_h_low:=22 -p yellow_h_high:=34 -p yellow_show_mask:=true

A* 3 มิติ วางเส้นทางไปจุดวางปลายทาง (ทำหลังสแกน 3D map):
  ตอนกด spacebar ลำดับคือ แคป -> point cloud -> solid -> วางเส้นทาง A* 3 มิติ -> วาดแผนที่ 3 มิติ (มีเส้นทางซ้อน)
  จุดเริ่ม = วัตถุสีเหลืองที่ใหญ่สุดที่ตรวจเจอ (ตำแหน่งใดก็ได้), ปลายทาง = จุดที่คลิก (ไม่คลิก = default)
  วางเส้นทางของยอดวัตถุที่ถือ (TCP) บน voxel grid (x, y, ความสูง) 26 ทิศ จาก height map ของ depth ที่แคป:
    voxel ว่างเมื่อ สูงกว่า (สิ่งกีดขวางที่ขยายด้วยรัศมีวัตถุ) + ความสูงวัตถุ + ระยะปลอดภัย
    จึงยกข้ามของเตี้ย / อ้อมของสูง (เพดาน path_zmax_mm) ; เริ่มและจบด้วยการยกขึ้น/ลงแนวตั้ง path_lift_mm
    รางอลูมิเนียมถูกนับเป็นสิ่งกีดขวางตามความสูงจริง (ยกข้ามได้ถ้าเพดานสูงพอ)
  ผลลัพธ์: เส้นม่วงบนภาพสด (ค้างจนสแกนใหม่), เส้นม่วง (พร้อมเงาบนพื้น) ใน 3D height map ของ matplotlib,
           MarkerArray ที่ /rgbd_viewer/path (Fixed Frame เดียวกับ solid), log waypoint (pixel+สูง และ xyz กล้อง mm)
  handeye_file : ไฟล์ handeye_calibration.json จาก detect_edit_calibrate_1.py -> แปลง waypoint เป็นพิกัดหุ่น (mm) แล้ว log + เซฟ CSV
  handeye_csv  : ไฟล์ CSV ที่เซฟ waypoint พิกัดหุ่นล่าสุด (ค่าเริ่มต้น path_robot.csv)
  ตัวอย่าง: -p handeye_file:="$PWD/handeye_calibration.json"
  ส่ง waypoint (พิกัดหุ่น) ผ่าน Modbus TCP: กด SPACE สแกน+วางเส้นทาง ตรวจเส้นทางบนหน้าจอ แล้วกด "s" เพื่อส่ง
    (ไม่ส่งอัตโนมัติเพื่อความปลอดภัย ; ต้องตั้ง handeye_file และ modbus_enable:=true)
    ลำดับ: เขียน [จำนวนจุด N, x1,y1,z1, ..., xN,yN,zN] ลง holding register ตั้งแต่ modbus_data_reg (function 0x10)
           ค่า = int16 (mm x modbus_scale, ค่าเริ่มต้น x10 = 0.1 mm, ติดลบเป็น two's complement)
           แล้วเขียน modbus_run_cmd ลง modbus_cmd_reg (function 0x06) เป็นอย่างสุดท้าย เพื่อให้ฝั่งหุ่นเริ่มเคลื่อนที่
    modbus_enable(false), modbus_ip(192.168.0.215), modbus_port(502), modbus_unit(1),
    modbus_cmd_reg(0), modbus_run_cmd(2), modbus_data_reg(10), modbus_scale(10.0)  (สูงสุด 40 waypoint)

  มุมหมุนหัวจับ (R): fit สี่เหลี่ยมหมุนได้ (minAreaRect) บน contour วัตถุสีเหลือง -> วัดมุมเอียงจากแกนภาพ (เส้นกึ่งกลางกล้อง)
    ทำให้อยู่ในช่วง -45..+45 องศา (สี่เหลี่ยมซ้ำทุก 90) แล้วแปลงเป็นมุมในกรอบหุ่นด้วยส่วนหมุนของ hand-eye
    R ที่สั่ง = grip_r_sign x มุมในกรอบหุ่น  (ส่งเป็น 'ค่าที่ต้องบวก/ลบ' เข้า register modbus_r_reg ; ฝั่ง Lua เอาไปบวกกับ R เริ่มต้น)
    grip_r_enable (true), grip_r_sign (1.0 ; ถ้าหัวจับหมุนผิดทางให้ใช้ -1.0), grip_r_min_deg (3.0 ; เอียงน้อยกว่านี้ไม่หมุน),
    grip_r_max_deg (45.0), modbus_r_reg (19), modbus_r_scale (10.0 ; ค่า register = องศา x scale แบบ int16)
  path_enable, path_topic, path_cell_px (5), path_dz_mm (6.0), path_zmax_mm (150.0), path_lift_mm (30.0),
  path_margin_px (4.0), path_z_margin_mm (5.0), path_goal_x_frac (-1 = กลางโซนขวา), path_goal_y_frac (0.5)

  A* หลบสิ่งกีดขวางด้วย "solid ประมาณของ obstacle ที่ขยายด้วยขนาดวัตถุสีเหลือง" (path_obstacle_source:=solids, ค่าเริ่มต้น):
    1) ประมาณ solid ของวัตถุสีเหลือง: fit รูปทรงจาก contour (สี่เหลี่ยม/วงกลม/...) -> กว้าง x ยาว (mm) + สูง (mm)
       (สูง = ระยะพื้น - ระยะยอดวัตถุ) ; log + วาดใน 3D map เป็นกรอบสีเหลือง
    2) สร้าง solid ของ obstacle ทุกชิ้นบนภาพที่ crop (ทุกโซน ตัดวัตถุที่จะหยิบออก) ด้วย pipeline เดียวกับ solid_*
    3) ขยาย solid ของ obstacle แต่ละชิ้นด้วยขนาดวัตถุสีเหลือง (Minkowski sum: วัตถุถูกหยิบด้วย R คงที่ จึงเลื่อนอย่างเดียวไม่หมุน)
         กว้างใหม่ = กว้างเดิม + กว้างวัตถุ + 2 x extra     ยาวใหม่ = ยาวเดิม + ยาวของวัตถุ + 2 x extra
         สูงใหม่   = สูงเดิม + สูงวัตถุ   (ยอดวัตถุที่ถือ = TCP ต้องสูงกว่ายอดที่ขยายแล้ว + path_z_margin_mm)
       แปลง mm -> pixel ที่ระดับยอดของ obstacle ชิ้นนั้นเอง (ใกล้กล้องกว่า = pixel มากกว่า) แล้ววาด A* บนแผนที่ที่ขยายแล้ว
    4) ของที่ไม่ถูกทำเป็น solid (เล็ก/เตี้ยเกิน solid_min_area_px, solid_min_height_mm) ยังถูกนับเป็นสิ่งกีดขวางจาก height map
       แล้วขยายด้วยขนาดวัตถุเหมือนกัน (path_residual_raw:=true) กันของที่หลุดการ fit
    บนภาพสด: เส้นส้ม = ขอบ obstacle ที่ขยายแล้ว, กรอบฟ้า = ขนาดวัตถุที่จุดหยิบ/จุดวาง ; ใน 3D map: กรอบส้มประ = solid ที่ขยายแล้ว
    path_obstacle_source : solids (ค่าเริ่มต้น) | raw (แบบเดิม dilate height map ด้วยรัศมีจาก path_margin_px)
    path_inflate_mode    : shape (ค่าเริ่มต้น ขยายด้วยรูปร่างจริงของวัตถุ = contour สีเหลือง แน่นสุด ใช้เมื่อหัวจับไม่หมุนวัตถุ)
                           | box (กล่องกรอบแกนภาพ) | circle (วงกลมล้อมวัตถุ กรณีวัตถุอาจหมุน)
    path_inflate_extra_mm (3.0)  : ระยะปลอดภัย + ส่วนของหัวจับที่ยื่นเกินขอบวัตถุ (mm) ต่อด้าน
                                   (ควรไม่น้อยกว่า RMSE ของ hand-eye ; ส่วนที่ขยายหลักคือครึ่งขนาดวัตถุ ซึ่งจำเป็น เพราะเส้นทางคือจุดกึ่งกลางวัตถุ)
    path_shape_mode (contour), path_poly_eps (0.015) : รูปร่าง obstacle ที่ใช้วางเส้นทาง (ตามขอบจริง ไม่ snap เป็นสี่เหลี่ยม)
    path_residual_raw (true), path_show_inflated (true)

  เลือกอัลกอริทึมวางเส้นทาง (กดปุ่มตัวเลขที่หน้าต่างภาพ ; ไม่กดอะไร = mode 2 A*):
    "1" = mode 1 Dijkstra | "2" = mode 2 A* (ค่าเริ่มต้น) | "3" = mode 3 Bidirectional Dijkstra
    "4" = mode 4 Bidirectional A* | "5" = mode 5 รันทั้ง 4 แบบบน voxel grid เดียวกัน แล้วเปิดกราฟเทียบ
          computational time (เวลาค้นหา ms) และ distance รวม (mm) ของแต่ละแบบ (เซฟกราฟ PNG ลง dataset ด้วย)
          ใน mode 5 เส้นทางที่ใช้วาด/ส่งหุ่น = ผลของ A*
    ถ้ามีภาพที่สแกนไว้แล้ว เปลี่ยน mode แล้ววางเส้นทางใหม่ทันทีด้วยภาพสแกนเดิม ; ถ้ายังไม่สแกน จะใช้ตอนกด SPACE
    (ทุกแบบใช้ voxel grid 26 ทิศ กันตัดมุม และ smooth line-of-sight เหมือนกัน ต่างกันแค่วิธีค้นหา)

  หมุน R กลับตามมุม tilt ที่ตำแหน่งที่ 2 (กดปุ่มที่หน้าต่างภาพ ; ไม่กดอะไร = แบบ "7"):
    "6" = หยิบด้วย R ที่หมุนตามมุมวัตถุ (R เริ่มต้น + มุม tilt) แล้วพอยกขึ้นถึงตำแหน่งที่ 2 (เหนือจุดหยิบ)
          หมุน R กลับเป็น R เริ่มต้น (หมุนกลับเท่ามุม tilt) -> วัตถุตั้งตรงกับแกนภาพตลอดทางที่เหลือและตอนวาง
          วางเส้นทางด้วยรูปร่างวัตถุที่ "กวาด" ตั้งแต่มุมเดิมถึงมุมที่หมุนกลับแล้ว (กันชนตอนหมุน + ตลอดทาง)
          กรอบฟ้าที่จุดวางแสดงรูปร่างวัตถุหลังหมุนกลับ
    "7" = ไม่หมุนกลับ: คง R ตามมุม tilt ตลอดเส้นทางเหมือนเดิม (ค่าเริ่มต้น)
    ส่งให้หุ่นผ่าน register modbus_rret_reg (ค่าเริ่มต้น 18): 1 = หมุนกลับที่ตำแหน่งที่ 2, 0 = ไม่หมุนกลับ
    ถ้ามีภาพที่สแกนไว้แล้ว กด 6/7 แล้ววางเส้นทางใหม่ทันที (กด 's' ส่งได้เลย)

  เลือกจุดวางสุดท้าย: เอาเมาส์ (กากบาทเขียว) ไปชี้ตำแหน่งไหนก็ได้บนภาพ แล้วคลิกซ้าย = ตั้งเป็นจุดวาง
    ถ้ามีภาพที่สแกนไว้แล้ว จะวางเส้นทางใหม่ทันทีด้วยภาพสแกนเดิม (กด 's' ส่งเส้นทางใหม่ได้เลย) ; ถ้ายังไม่สแกน จะใช้ตอนกด SPACE
    กด 'a' (หรือคลิกขวา) = กลับไปใช้ตำแหน่ง default (path_goal_x_frac / path_goal_y_frac)

Solid (extrude boss) มุมมอง 3 มิติ:
  ตอนกด spacebar จะวิเคราะห์ depth ที่แคป แล้ว publish โมเดล 3 มิติไปที่ /rgbd_viewer/solids
  (visualization_msgs/MarkerArray) ขั้นตอน:
    1) หาระนาบพื้น (RANSAC แบบวนซ้ำบน 1/Z) แล้วคำนวณ "ความสูงเหนือพื้น" ต่อ pixel
       (pixel ที่ไม่มีข้อมูล depth ถือว่าเป็นพื้น -> ขอบดำรอบวัตถุหลุดออกเอง)
    2) กรอง median + bilateral (รักษาขอบ ไม่ทำให้ขอบเบลอ)
    3) แยกวัตถุตามระดับความสูงของยอด (component tree) -> ชิ้นที่มีรอยต่อนุ่ม ๆ ติดกัน
       แต่ยอดสูงต่างกันจะถูกแยกเป็นคนละชิ้น แล้วขยาย seed กลับด้วย watershed
    4) ตัดขอบแต่ละชิ้นที่ solid_edge_level x ความสูงยอดของชิ้นนั้นเอง
    5) fit รูปทรงให้คม: สี่เหลี่ยม (ทนต่อส่วนยื่น/รอยต่อ) / วงกลม-วงรี / สามเหลี่ยม / polygon
    6) extrude ฝาบน (แบน ที่ความลึก median ของยอด) ลงไปถึงระนาบพื้น
  solid_*       : parameters ปรับแต่ง (ดูตอน declare_parameter ด้านล่าง)
                  หมายเหตุ: parameter ชนิด double ต้องใส่จุดทศนิยม เช่น -p solid_floor_mm:=650.0
  solid_shape_mode:=contour  ใช้ contour จริง (simplify เป็น polygon) แทนการ snap เป็นรูปทรงมาตรฐาน

ดู solid ใน RViz:
  Fixed Frame = topview   (เฟรมนี้โหนดสร้างให้เอง: z ชี้ขึ้น พื้น z=0 ไม่กลับหัว)
  topview_yaw_deg: หมุนมุมมองรอบแกนตั้ง (เริ่มต้น 90 = ภาพตรงกับภาพกล้อง; ถ้าหันผิดทิศลอง 0, 180, -90)
  Add > MarkerArray > Topic: /rgbd_viewer/solids
  Reliability = Reliable, Durability = Transient Local
  (เอา PointCloud2 มาซ้อนด้วยได้ เพื่อเทียบ solid กับข้อมูลดิบ; กด Orbit/หมุนมุมมองเพื่อดู 3 มิติ)

Crop ภาพก่อนเข้า process ทั้งหมด (แสดงผล / ดูระยะด้วยเมาส์ / เซฟ / point cloud / grid):
  crop ทันทีที่รับเฟรม color+depth ที่ sync กันแล้ว ทั้งสองภาพถูกตัดด้วยกรอบเดียวกัน
  (พิกัดอ้างอิงภาพต้นฉบับ มุมซ้ายบน = (0,0)) และปรับ cx, cy ของ intrinsics ให้อัตโนมัติ
  จึงได้ point cloud / grid ที่พิกัดถูกต้องเหมือนเดิม
  crop_x, crop_y          : มุมซ้ายบนของกรอบ (pixel) ค่าเริ่มต้น 0, 0
  crop_width, crop_height : ขนาดกรอบ (pixel) ค่าเริ่มต้น 0 = ไปจนสุดขอบภาพ
                            (ถ้าไม่ตั้งค่าเลย = ไม่ crop)
  ตัวอย่าง: --ros-args -p crop_x:=160 -p crop_y:=80 -p crop_width:=640 -p crop_height:=480

จุดเริ่ม / จุดวาง (ไม่มีโซนแล้ว ใช้ทั้งภาพที่ crop):
  START = วัตถุสีเหลืองที่ใหญ่สุดที่ตรวจเจอ อยู่ตรงไหนของภาพก็ได้ (ภาพสดขึ้นป้าย START ที่วัตถุ)
  GOAL  = ตำแหน่งที่คลิกซ้ายด้วยกากบาทเขียว ; ไม่คลิก/กด 'a' = ตำแหน่ง default (path_goal_x_frac, path_goal_y_frac)
  วัตถุสีเหลืองที่ตรวจเจอ "ไม่ถูกนำมาทำแผนที่": ตัดออกจาก point cloud / solid / แผนที่ 3 มิติ / grid / สิ่งกีดขวางของ A*
    map_exclude_yellow : all (ค่าเริ่มต้น ตัดทุกชิ้น) | start (ตัดเฉพาะชิ้นที่เป็น START ชิ้นอื่นยังนับเป็นสิ่งกีดขวาง) | off
    map_yellow_margin_px (4) : ขยายขอบที่ตัดออก (ขอบวัตถุ/เงา depth)
  (การเซฟ color/depth ลงไฟล์ยังเป็นภาพเต็มไม่ตัดอะไร)
  โหมดแบ่งโซนแบบเดิม (ซ้าย=จุดวางวัตถุ | กลาง=scan map | ขวา=จุดวางของ) ยังมีให้ แต่ปิดเป็นค่าเริ่มต้น: zone_enable:=true
  zone_enable (false), zone_left_frac (0.168), zone_right_frac (0.851), zone_show_lines
  ตัวอย่าง: --ros-args -p zone_left_frac:=0.2 -p zone_right_frac:=0.8

Grid map (nav_msgs/OccupancyGrid) มุมมองจากด้านบน:
  ตอนกด spacebar จะ publish grid ไปที่ /rgbd_viewer/grid ด้วย
  ค่าในแต่ละช่อง = ความสูงเหนือพื้น 0..100 (พื้น=0, ใกล้กล้องสุด=100), ไม่มีข้อมูล = -1
  ใช้ช่วง depth_near_mm / depth_far_mm ตัวเดียวกับภาพสี
  grid_topic      : topic ของ grid (ค่าเริ่มต้น /rgbd_viewer/grid)
  grid_resolution : ขนาดช่อง (เมตร) ค่าเริ่มต้น 0.005 (ถ้ามีรูโหว่เยอะให้เพิ่มเป็น 0.01)

ดู grid ใน RViz:
  Fixed Frame = camera_color_optical_frame
  Add > Map > Topic: /rgbd_viewer/grid
  Reliability = Reliable, Durability = Transient Local, Color Scheme = costmap

แผนที่ 3 มิติด้วย matplotlib:
  ตอนกด spacebar จะเปิด/อัปเดตหน้าต่าง matplotlib 3 มิติ แสดง surface ความสูงเหนือพื้น (mm)
  ระบายสีตามความสูง + โครงขอบของ solid ที่ fit ได้พร้อม label (ลากเมาส์หมุน, ล้อเมาส์ซูม)
  แกน: X = ขวา, Y = บนภาพ, Z = ความสูงเหนือพื้น (mm) ใช้ floor/ตัวกรองชุดเดียวกับ solid_*
  ต้องติดตั้ง: pip install matplotlib  และ  sudo apt install python3-tk  (สำหรับ backend TkAgg)
  plot3d_enable      : เปิด/ปิดแผนที่ 3 มิติ (ค่าเริ่มต้น true)
  plot3d_backend     : backend ของ matplotlib (ค่าเริ่มต้น TkAgg; ใส่ '' เพื่อใช้ค่า default)
  plot3d_stride      : ข้าม pixel ทุก N จุด (ค่าเริ่มต้น 3; ลดเป็น 2 ถ้าอยากละเอียดขึ้น แต่หมุนจะช้าลง)
  plot3d_cmap        : colormap ของ matplotlib (ค่าเริ่มต้น turbo)
  plot3d_z_aspect    : สัดส่วนความสูงแกน Z ต่อแกนที่ยาวสุด (ค่าเริ่มต้น 0.35; 0 = สเกลจริง)
  plot3d_show_solids : วาดโครงขอบ solid ซ้อนบน surface (ค่าเริ่มต้น true)
  plot3d_full        : แสดงทั้งภาพที่ crop พร้อมเส้นแบ่งโซนสีแดง เพื่อให้เห็นเส้นทาง A* ตั้งแต่จุดหยิบถึงจุดวาง (ค่าเริ่มต้น true; false = เฉพาะโซน scan)
"""

import math
import os
import socket
import struct
import threading
import time
from datetime import datetime
import json
import queue
import re
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
)
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from builtin_interfaces.msg import Time
import cv2


# encoding -> (numpy dtype, จำนวน channel)
# แปลง sensor_msgs/Image เป็น numpy array เอง โดยไม่ใช้ cv_bridge
# (cv_bridge ของ Jazzy compile มากับ NumPy 1.x จึงใช้กับ NumPy 2 ไม่ได้)
_ENCODINGS = {
    'bgr8': (np.uint8, 3),
    'rgb8': (np.uint8, 3),
    'bgra8': (np.uint8, 4),
    'rgba8': (np.uint8, 4),
    'mono8': (np.uint8, 1),
    '8UC1': (np.uint8, 1),
    'mono16': (np.uint16, 1),
    '16UC1': (np.uint16, 1),
}


# ---------------------------------------------------------------- Modbus TCP (ส่ง waypoint ให้หุ่น)
MODBUS_MAX_WAYPOINTS = 40       # 1 register จำนวนจุด + 3 x N <= 123 (ขีดจำกัดของ function 0x10)


def _recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('server ปิดการเชื่อมต่อ')
        buf += chunk
    return buf


def _modbus_txn(sock, tid, unit, pdu):
    sock.sendall(struct.pack('>HHHB', tid, 0, len(pdu) + 1, unit) + pdu)
    head = _recv_exact(sock, 7)
    body = _recv_exact(sock, struct.unpack('>H', head[4:6])[0] - 1)
    if body[0] & 0x80:
        raise RuntimeError(f'server ตอบ error (function 0x{body[0]:02X}, code {body[1] if len(body) > 1 else "?"})')
    return body


def modbus_write_reg(sock, tid, unit, reg, value):
    """function 0x06: เขียน 1 register"""
    return _modbus_txn(sock, tid, unit, struct.pack('>BHH', 0x06, reg, value))


def modbus_write_regs(sock, tid, unit, reg, values):
    """function 0x10: เขียนหลาย register ต่อเนื่อง (สูงสุด 123)"""
    n = len(values)
    pdu = struct.pack('>BHHB', 0x10, reg, n, 2 * n) + struct.pack(f'>{n}H', *values)
    return _modbus_txn(sock, tid, unit, pdu)


def modbus_read_regs(sock, tid, unit, reg, count):
    """function 0x03: อ่าน holding register ต่อเนื่อง"""
    body = _modbus_txn(sock, tid, unit, struct.pack('>BHH', 0x03, reg, count))
    n = body[1] // 2
    return list(struct.unpack(f'>{n}H', body[2:2 + 2 * n]))


def encode_waypoints(points_mm, scale=10.0):
    """[[x,y,z],...] mm -> [N, x1,y1,z1, ...] เป็น uint16 (int16 two's complement, ค่า = mm x scale)"""
    pts = np.asarray(points_mm, float).reshape(-1, 3)
    if not 1 <= len(pts) <= MODBUS_MAX_WAYPOINTS:
        raise ValueError(f'จำนวน waypoint {len(pts)} ต้องอยู่ระหว่าง 1-{MODBUS_MAX_WAYPOINTS}')
    vals = [len(pts)]
    for v in pts.ravel():
        iv = int(round(v * scale))
        if not -32768 <= iv <= 32767:
            raise ValueError(f'ค่า {v:.1f} mm x{scale:g} เกินช่วง int16')
        vals.append(iv & 0xFFFF)
    return vals


def imgmsg_to_array(msg: Image) -> np.ndarray:
    """แปลง Image msg เป็น numpy array รูปทรง (H, W) หรือ (H, W, C) ตาม encoding เดิม"""
    if msg.encoding not in _ENCODINGS:
        raise ValueError(f'ไม่รองรับ encoding: {msg.encoding}')
    dtype, ch = _ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype)
    row_bytes = msg.width * ch * dtype.itemsize

    # msg.step อาจมากกว่า row_bytes (มี padding ท้ายแถว) จึงต้องตัดออก
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    buf = np.ascontiguousarray(buf[:, :row_bytes])

    if dtype.itemsize > 1 and msg.is_bigendian:
        dtype = dtype.newbyteorder('>')
    arr = buf.view(dtype).reshape(msg.height, msg.width, ch)
    arr = arr.astype(dtype.newbyteorder('='), copy=False)
    return arr[:, :, 0] if ch == 1 else arr


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """แปลง Image msg เป็นภาพ BGR 8-bit สำหรับ OpenCV"""
    arr = imgmsg_to_array(msg)
    code = {
        'bgr8': None,
        'rgb8': cv2.COLOR_RGB2BGR,
        'bgra8': cv2.COLOR_BGRA2BGR,
        'rgba8': cv2.COLOR_RGBA2BGR,
        'mono8': cv2.COLOR_GRAY2BGR,
        '8UC1': cv2.COLOR_GRAY2BGR,
    }.get(msg.encoding, 'unsupported')
    if code == 'unsupported':
        raise ValueError(f'แปลง {msg.encoding} เป็น bgr8 ไม่ได้')
    return arr if code is None else cv2.cvtColor(arr, code)


def compute_crop_rect(img_w, img_h, x, y, w, h):
    """
    คำนวณกรอบ crop (x0, y0, w, h) ที่อยู่ในขอบเขตภาพจริง
    w/h <= 0 หมายถึงไปจนสุดขอบภาพ, ค่าที่เกินขอบจะถูกตัดให้พอดี
    """
    x0 = min(max(int(x), 0), img_w - 1)
    y0 = min(max(int(y), 0), img_h - 1)
    cw = img_w - x0 if w <= 0 else min(int(w), img_w - x0)
    ch = img_h - y0 if h <= 0 else min(int(h), img_h - y0)
    return x0, y0, cw, ch


# ---------------------------------------------------- depth flat-field correction
# (จาก Depth_flat_calibrate.py) แก้ depth ด้วย z_new = 1 / (1/z + delta)
def load_flat_calibration(path):
    if not path:
        return None
    z = np.load(path)
    return {'delta': z['delta'].astype(np.float32),
            'crop': tuple(int(v) for v in z['crop']),
            'img_size': tuple(int(v) for v in z['img_size'])}


def flat_slice(flat, rect, img_size=None):
    """ตัดแผนที่ delta ให้ตรงกรอบ crop rect=(x0,y0,w,h); None ถ้ากรอบอยู่นอกพื้นที่ที่ calibrate"""
    if flat is None:
        return None
    if img_size is not None and tuple(flat['img_size']) != tuple(img_size):
        return None
    cx0, cy0, cw, ch = flat['crop']
    x0, y0, w, h = rect
    if x0 < cx0 or y0 < cy0 or x0 + w > cx0 + cw or y0 + h > cy0 + ch:
        return None
    return flat['delta'][y0 - cy0:y0 - cy0 + h, x0 - cx0:x0 - cx0 + w]


def apply_flat_correction(depth_mm, delta):
    d = depth_mm.astype(np.float32)
    valid = d > 0
    inv = np.where(valid, 1.0 / np.maximum(d, 1.0), 0.0) + delta
    ok = valid & (inv > 1e-6)
    out = np.where(ok, 1.0 / np.maximum(inv, 1e-6), 0.0)
    return np.clip(np.rint(out), 0, 65535).astype(np.uint16)


# ---------------------------------------------------- yellow detection (HSV) + size in mm
def detect_yellow_objects(bgr, depth_mm, intr, hsv_lo, hsv_hi,
                          min_area_px=300, open_px=3, close_px=7):
    """
    ตรวจจับวัตถุสีเหลืองด้วย HSV แล้ววัดขนาดจริง (mm) จาก bounding box
    bgr/depth_mm : ภาพสีและ depth (uint16 mm) ที่ crop แล้ว ขนาดเท่ากัน (aligned)
    intr         : (fx, fy, cx, cy) หรือ None
    คืน (list ของ dict, mask)  dict = {bbox:(x,y,w,h), z_mm, w_mm, h_mm, area_px, touches_edge}
    w_mm/h_mm = None ถ้าไม่มี depth หรือ intrinsics
    """
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lo, np.uint8), np.array(hsv_hi, np.uint8))
    if open_px > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_px, open_px)))
    if close_px > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_px, close_px)))

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    H, W = mask.shape
    same_size = depth_mm is not None and depth_mm.shape[:2] == (H, W)
    results = []
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(c)
        if area < min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(c)

        z_mm = None
        if same_size:
            obj = np.zeros((H, W), np.uint8)
            cv2.drawContours(obj, [c], -1, 1, thickness=cv2.FILLED)
            inner = cv2.erode(obj, np.ones((5, 5), np.uint8))
            if inner.sum() < 20:
                inner = obj
            dz = depth_mm[inner > 0]
            dz = dz[dz > 0]
            if dz.size > 0:
                z_mm = float(np.median(dz))

        w_mm = h_mm = None
        if z_mm is not None and intr is not None:
            fx, fy = intr[0], intr[1]
            w_mm = w * z_mm / fx
            h_mm = h * z_mm / fy
        results.append({
            'bbox': (x, y, w, h), 'z_mm': z_mm, 'w_mm': w_mm, 'h_mm': h_mm,
            'area_px': area,
            'touches_edge': x <= 0 or y <= 0 or x + w >= W or y + h >= H,
            'contour': c,
        })
    return results, mask


# ---------------------------------------------------- A* 3 มิติ (วางเส้นทางหลังสแกน 3D map)
def build_height_field(depth_mm, params):
    """(plane, height_mm, floor_z_mm) ของภาพ depth ที่ crop แล้ว ; None ถ้าหาระนาบพื้นไม่ได้"""
    plane = fit_floor_plane(depth_mm, floor_mm=params['floor_mm'])
    if plane is None:
        return None
    fz = floor_map(plane, depth_mm.shape)
    h = make_height_map(depth_mm, fz, params['median_ksize'], params['bilateral_sigma_mm'])
    return plane, h, fz


def cell_max(a, cell):
    """ค่าสูงสุดในแต่ละช่อง cell x cell (ลดความละเอียดแบบไม่ทำให้สิ่งกีดขวางหาย)"""
    H, W = a.shape
    gh, gw = -(-H // cell), -(-W // cell)
    pad = np.zeros((gh * cell, gw * cell), a.dtype)
    pad[:H, :W] = a
    return pad.reshape(gh, cell, gw, cell).max(axis=(1, 3))


# ---------------------------------------------------------------- อัลกอริทึมค้นหาเส้นทาง (เลือกด้วยปุ่ม 1-5)
PATH_ALGOS = ('dijkstra', 'astar', 'bidir_dijkstra', 'bidir_astar')
PATH_MODE_ALGO = {1: 'dijkstra', 2: 'astar', 3: 'bidir_dijkstra', 4: 'bidir_astar', 5: 'compare'}
PATH_ALGO_LABEL = {'dijkstra': 'Dijkstra', 'astar': 'A*',
                   'bidir_dijkstra': 'Bi-Dijkstra', 'bidir_astar': 'Bi-A*'}
PATH_DEFAULT_MODE = 2           # ไม่กดอะไร = A*


def _moves_3d(step_mm):
    """26 ทิศ พร้อมระยะจริง (mm) ของแต่ละก้าว"""
    sy, sx, sz = step_mm
    return [(dr, dc, dk, math.sqrt((dr * sy) ** 2 + (dc * sx) ** 2 + (dk * sz) ** 2))
            for dr in (-1, 0, 1) for dc in (-1, 0, 1) for dk in (-1, 0, 1)
            if (dr, dc, dk) != (0, 0, 0)]


def _search_uni_3d(free, start, goal, step_mm, use_heur, stats):
    """Dijkstra (use_heur=False) / A* (use_heur=True) ทิศทางเดียว จาก start ไป goal"""
    import heapq
    gh, gw, K = free.shape
    fl = free.ravel().tobytes()
    sy, sx, sz = step_mm
    offs = _moves_3d(step_mm)

    def ix(r, c, k):
        return (r * gw + c) * K + k

    if use_heur:
        def heur(r, c, k):
            return math.sqrt(((r - goal[0]) * sy) ** 2 + ((c - goal[1]) * sx) ** 2
                             + ((k - goal[2]) * sz) ** 2)
    else:
        def heur(r, c, k):
            return 0.0

    g = {start: 0.0}
    parent = {}
    closed = set()
    heap = [(heur(*start), 0.0, start)]
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if cur == goal:
            stats['expanded'] = len(closed) + 1
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        r, c, k = cur
        for dr, dc, dk, cost in offs:
            nr, nc, nk = r + dr, c + dc, k + dk
            if nr < 0 or nr >= gh or nc < 0 or nc >= gw or nk < 0 or nk >= K:
                continue
            if not fl[ix(nr, nc, nk)]:
                continue
            if (dr != 0) + (dc != 0) + (dk != 0) >= 2:       # กันตัดมุม
                if ((dr and not fl[ix(r + dr, c, k)]) or (dc and not fl[ix(r, c + dc, k)])
                        or (dk and not fl[ix(r, c, k + dk)])):
                    continue
            ng = gc + cost
            nxt = (nr, nc, nk)
            if ng < g.get(nxt, 1e18):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(heap, (ng + heur(nr, nc, nk), ng, nxt))
    stats['expanded'] = len(closed)
    return None


def _search_bi_3d(free, start, goal, step_mm, use_heur, stats):
    """
    Bidirectional Dijkstra (use_heur=False) / Bidirectional A* (use_heur=True)
    ค้นหาพร้อมกันจาก start (ไปข้างหน้า) และจาก goal (ย้อนกลับ) ขยายฝั่งที่ key ต่ำกว่าก่อน
    หยุดเมื่อพิสูจน์ได้ว่าเส้นทางที่ต่อกันได้ดีสุด (mu) สั้นสุดแล้ว:
      Dijkstra : top_f + top_b >= mu      A* (heuristic consistent) : max(fmin_f, fmin_b) >= mu
    เงื่อนไขขอบ (กันตัดมุม) ตรวจตามทิศเดินจริงเสมอ ฝั่งย้อนกลับจึงตรวจขอบ u -> cur
    """
    import heapq
    if start == goal:
        stats['expanded'] = 0
        return [start]
    gh, gw, K = free.shape
    fl = free.ravel().tobytes()
    sy, sx, sz = step_mm
    offs = _moves_3d(step_mm)
    INF = float('inf')

    def dist(a, b):
        return math.sqrt(((a[0] - b[0]) * sy) ** 2 + ((a[1] - b[1]) * sx) ** 2
                         + ((a[2] - b[2]) * sz) ** 2)

    if use_heur:
        hfun = (lambda n: dist(n, goal), lambda n: dist(n, start))
    else:
        hfun = (lambda n: 0.0, lambda n: 0.0)
    g = ({start: 0.0}, {goal: 0.0})
    par = ({}, {})
    closed = (set(), set())
    heaps = ([(hfun[0](start), 0.0, start)], [(hfun[1](goal), 0.0, goal)])
    push, pop = heapq.heappush, heapq.heappop
    mu, meet = INF, None
    while heaps[0] and heaps[1]:
        kf, kb = heaps[0][0][0], heaps[1][0][0]
        if (max(kf, kb) if use_heur else kf + kb) >= mu:
            break
        d = 0 if kf <= kb else 1
        _, gc, cur = pop(heaps[d])
        gd, cd, pd, hd, hq = g[d], closed[d], par[d], hfun[d], heaps[d]
        if cur in cd or gc > gd.get(cur, INF):
            continue
        cd.add(cur)
        other = g[1 - d]
        r, c, k = cur
        for dr, dc, dk, cost in offs:
            nr, nc, nk = r + dr, c + dc, k + dk
            if nr < 0 or nr >= gh or nc < 0 or nc >= gw or nk < 0 or nk >= K:
                continue
            if not fl[(nr * gw + nc) * K + nk]:
                continue
            if (dr != 0) + (dc != 0) + (dk != 0) >= 2:       # กันตัดมุม: ตรวจตามทิศเดินจริง
                # ข้างหน้า: ขอบ cur -> nxt (ต้นทาง = cur) ; ย้อนกลับ: ขอบ nxt -> cur (ต้นทาง = nxt, ก้าว = -d)
                if d == 0:
                    br, bc, bk, er, ec, ek = r, c, k, dr, dc, dk
                else:
                    br, bc, bk, er, ec, ek = nr, nc, nk, -dr, -dc, -dk
                if ((er and not fl[((br + er) * gw + bc) * K + bk])
                        or (ec and not fl[(br * gw + bc + ec) * K + bk])
                        or (ek and not fl[(br * gw + bc) * K + bk + ek])):
                    continue
            nxt = (nr, nc, nk)
            if nxt in cd:
                continue
            ng = gc + cost
            if ng < gd.get(nxt, INF):
                gd[nxt] = ng
                pd[nxt] = cur
                push(hq, (ng + hd(nxt), ng, nxt))
                og = other.get(nxt)
                if og is not None and ng + og < mu:
                    mu, meet = ng + og, nxt
    stats['expanded'] = len(closed[0]) + len(closed[1])
    if meet is None:
        return None
    path, n = [meet], meet
    while n in par[0]:
        n = par[0][n]
        path.append(n)
    path.reverse()
    n = meet
    while n in par[1]:
        n = par[1][n]
        path.append(n)
    return path


def search_3d(free, start, goal, step_mm, algo='astar', stats=None):
    """
    ค้นหาเส้นทางบน voxel grid 26 ทิศ  free = bool (rows, cols, levels)  start/goal = (r, c, k)
    algo : dijkstra | astar | bidir_dijkstra | bidir_astar
    step_mm = (mm ต่อ 1 แถว, mm ต่อ 1 คอลัมน์, mm ต่อ 1 ระดับ) ใช้คิดระยะจริง (heuristic = ระยะตรง)
    ไม่ตัดมุมสิ่งกีดขวาง คืน list ของ (r, c, k) หรือ None ; stats (dict) ได้ 'expanded' = จำนวน node ที่ขยาย
    """
    if stats is None:
        stats = {}
    if algo == 'dijkstra':
        return _search_uni_3d(free, start, goal, step_mm, False, stats)
    if algo == 'bidir_dijkstra':
        return _search_bi_3d(free, start, goal, step_mm, False, stats)
    if algo == 'bidir_astar':
        return _search_bi_3d(free, start, goal, step_mm, True, stats)
    return _search_uni_3d(free, start, goal, step_mm, True, stats)


def astar_3d(free, start, goal, step_mm):
    """A* บน voxel grid 26 ทิศ (คงไว้เพื่อความเข้ากันได้ ; ใช้ search_3d)"""
    return search_3d(free, start, goal, step_mm, 'astar')


def cells_length_mm(cells, step_mm):
    """ระยะทางรวม (mm) ของเส้นทางบน grid"""
    if cells is None or len(cells) < 2:
        return 0.0
    a = np.asarray(cells, np.float64) * np.asarray(step_mm, np.float64)
    return float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())


def _line_free_3d(free, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]), abs(b[2] - a[2]))) * 2 + 1
    p = np.rint(np.linspace(a, b, n)).astype(int)
    return bool(free[p[:, 0], p[:, 1], p[:, 2]].all())


def smooth_path_3d(free, path):
    """ลดจุดเลี้ยวด้วย line-of-sight ใน 3 มิติ (เหลือเฉพาะจุดที่จำเป็น)"""
    if len(path) <= 2:
        return list(path)
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_free_3d(free, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_path_3d(h, start_xy, goal_xy, h_obj, cell_px=5, dz_mm=6.0, zmax_mm=150.0,
                 lift_mm=30.0, radius_px=12.0, z_margin_mm=5.0, cell_mm=4.0, obstacle=None,
                 algo='astar', stats=None):
    """
    วางเส้นทางของ \"ปลายวัตถุที่ถือ\" (TCP = ยอดวัตถุ) จากจุดหยิบไปจุดวาง ใน 3 มิติ
      h        : height map (mm เหนือพื้น) ของภาพที่ crop แล้ว (ตัดวัตถุที่จะหยิบออกแล้ว)
      start_xy, goal_xy : pixel (x, y) ของจุดหยิบ / จุดวาง ;  h_obj = ความสูงวัตถุที่ถือ (mm)
    voxel (r, c, k) ว่าง เมื่อ ระดับ z_k >= (ความสูงสิ่งกีดขวางที่ขยายด้วยรัศมีวัตถุ) + h_obj + z_margin
    (ตัววัตถุยื่นลงใต้ TCP เท่ากับ h_obj) จึงยกข้ามของเตี้ยได้ และอ้อมของที่สูงเกิน zmax_mm
    obstacle : (H, W) mm ความสูงสิ่งกีดขวาง \"ที่ขยายแล้ว\" (กว้าง/ยาว/สูง บวกขนาดวัตถุที่ถือแล้ว)
               ถ้าให้มา จะไม่ dilate และไม่บวก h_obj ซ้ำ (ใช้ radius_px ไม่ได้) ; None = dilate h ด้วย radius_px แบบเดิม
    algo     : dijkstra | astar | bidir_dijkstra | bidir_astar
    stats    : dict (ไม่บังคับ) ได้ 'search_ms' (เวลาค้นหาอย่างเดียว), 'expanded', 'raw_len_mm' (ระยะเส้นทางบน grid ก่อน smooth)
    คืน (waypoints Nx3 [(u_px, v_px, TCP สูงเหนือพื้น mm)] หรือ None, ข้อความสาเหตุ)
    """
    if stats is None:
        stats = {}
    cell = max(int(cell_px), 1)
    h_cell = cell_max(h.astype(np.float32), cell)                   # ความสูงจริง (ไว้หาพื้นผิวที่จุดวาง)
    if obstacle is None:
        r = max(int(math.ceil(radius_px)), 1)
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        hc = cell_max(cv2.dilate(h.astype(np.float32), kern), cell)     # ขยายสิ่งกีดขวางด้วยรัศมีวัตถุ
        need = hc + h_obj + z_margin_mm
    else:
        need = cell_max(obstacle.astype(np.float32), cell) + z_margin_mm
    gh, gw = h_cell.shape
    K = int(zmax_mm // dz_mm) + 1
    zs = np.arange(K, dtype=np.float32) * dz_mm
    free = zs[None, None, :] >= need[:, :, None]

    def to_cell(p):
        return (int(np.clip(p[1] // cell, 0, gh - 1)), int(np.clip(p[0] // cell, 0, gw - 1)))

    sc, gc = to_cell(start_xy), to_cell(goal_xy)
    place_h = float(h_cell[gc])                                      # พื้นผิวที่จุดวาง
    ks = int(math.ceil((h_obj + lift_mm) / dz_mm))
    kg = int(math.ceil((place_h + h_obj + lift_mm) / dz_mm))
    if ks >= K or kg >= K:
        return None, 'zmax too low'
    s3, g3 = (sc[0], sc[1], ks), (gc[0], gc[1], kg)
    if not free[s3]:
        return None, 'start blocked'
    if not free[g3]:
        return None, 'goal blocked'
    step = (cell_mm, cell_mm, dz_mm)
    t0 = time.perf_counter()
    cells = search_3d(free, s3, g3, step, algo, stats)
    stats['search_ms'] = 1000.0 * (time.perf_counter() - t0)
    if cells is None:
        return None, 'no path'
    stats['raw_len_mm'] = cells_length_mm(cells, step)
    cells = smooth_path_3d(free, cells)
    pts = [[(c + 0.5) * cell, (rr + 0.5) * cell, k * dz_mm] for rr, c, k in cells]
    pts[0][:2] = [float(start_xy[0]), float(start_xy[1])]
    pts[-1][:2] = [float(goal_xy[0]), float(goal_xy[1])]
    grasp = [float(start_xy[0]), float(start_xy[1]), float(h_obj)]            # จุดหยิบ (ยอดวัตถุ)
    place = [float(goal_xy[0]), float(goal_xy[1]), float(place_h + h_obj)]    # จุดวาง
    return np.array([grasp] + pts + [place], np.float32), ''


def contour_rect_tilt(contour):
    """
    fit สี่เหลี่ยมหมุนได้ (minAreaRect) บน contour แล้ววัดมุมเอียงจากแกนภาพ (เส้นกึ่งกลางกล้องแนวนอน/แนวตั้ง)
    คืน dict {box 4x2 px, center, rw, rh, tilt} ; tilt อยู่ใน [-45, 45) องศา
    บวก = หมุนตามเข็มนาฬิกาบนภาพ (แกน y ของภาพชี้ลง) ; สี่เหลี่ยมซ้ำทุก 90 องศา จึงเลือกมุมที่หมุนน้อยที่สุด
    """
    if contour is None or len(contour) < 5:
        return None
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(np.float32)
    e = box[1] - box[0]
    a = math.degrees(math.atan2(float(e[1]), float(e[0])))
    return {'box': box, 'center': (float(rect[0][0]), float(rect[0][1])),
            'rw': float(rect[1][0]), 'rh': float(rect[1][1]), 'tilt': ((a + 45.0) % 90.0) - 45.0}


def rotate_pts_deg(pts, deg):
    """หมุนจุด Nx2 (px สัมพัทธ์กับจุด TCP) รอบจุด TCP บนภาพ ; บวก = ตามเข็มนาฬิกาบนภาพ (แกน y ชี้ลง) เหมือน tilt"""
    a = math.radians(deg)
    c, s_ = math.cos(a), math.sin(a)
    p = np.asarray(pts, np.float32)
    return np.stack([c * p[:, 0] - s_ * p[:, 1], s_ * p[:, 0] + c * p[:, 1]], 1).astype(np.float32)


def object_kernel(w_mm, l_mm, extra_mm, z_mm, fx, fy, mode='shape', footprint=None):
    """
    structuring element ของวัตถุที่ถือ (ใช้ขยาย obstacle) คืน (kernel, add_w_mm, add_l_mm)
    add_* = ขนาดที่ obstacle กว้าง/ยาวขึ้นหลังขยาย (รวมระยะเผื่อ 2 x extra)
      shape  : รูปร่างจริงของวัตถุ (convex hull ของ contour สีเหลือง, เลื่อนอย่างเดียว ไม่หมุน)  <- แน่นสุด
      box    : กล่องกว้าง w x ยาว l (กรอบแกนภาพ) + extra
      circle : วงกลมล้อมวัตถุ รัศมี = ครึ่งเส้นทแยง + extra  (กรณีวัตถุอาจหมุนระหว่างเคลื่อนที่)
    footprint = {'pts': Nx2 px สัมพัทธ์กับจุด TCP (กึ่งกลางกรอบ), 'z': ความลึกที่วัด pts (mm)}
    z_mm = ความลึกของระดับที่ขยาย ใช้แปลง mm -> pixel (ใกล้กล้อง = pixel มาก)
    """
    z_mm = max(float(z_mm), 1.0)
    if mode == 'shape' and footprint is not None:
        pts = np.asarray(footprint['pts'], np.float32) * (float(footprint['z']) / z_mm)
        ex_x, ex_y = extra_mm * fx / z_mm, extra_mm * fy / z_mm
        rx = int(math.ceil(float(np.abs(pts[:, 0]).max() + ex_x))) + 2
        ry = int(math.ceil(float(np.abs(pts[:, 1]).max() + ex_y))) + 2
        k = np.zeros((2 * ry + 1, 2 * rx + 1), np.uint8)
        cv2.fillPoly(k, [np.round(pts + np.array([rx, ry], np.float32)).astype(np.int32)], 1)
        ax_, ay_ = max(int(math.ceil(ex_x)), 1), max(int(math.ceil(ex_y)), 1)     # เผื่อรอบรูปร่าง (อย่างน้อย 1 px)
        k = cv2.dilate(k, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * ax_ + 1, 2 * ay_ + 1)))
        fp = np.asarray(footprint['pts'], np.float32)
        return (k, float(np.ptp(fp[:, 0])) * footprint['z'] / fx + 2.0 * extra_mm,
                float(np.ptp(fp[:, 1])) * footprint['z'] / fy + 2.0 * extra_mm)
    if mode == 'circle':
        hx = hy = 0.5 * math.hypot(w_mm, l_mm) + extra_mm
        shape = cv2.MORPH_ELLIPSE
    else:
        hx, hy = 0.5 * w_mm + extra_mm, 0.5 * l_mm + extra_mm
        shape = cv2.MORPH_RECT
    rx = max(int(math.ceil(hx * fx / z_mm)), 1)
    ry = max(int(math.ceil(hy * fy / z_mm)), 1)
    return cv2.getStructuringElement(shape, (2 * rx + 1, 2 * ry + 1)), 2.0 * hx, 2.0 * hy


def inflate_solids(solids, floor_z, shape, intr, w_mm, l_mm, h_obj, extra_mm=3.0, mode='shape', footprint=None):
    """
    ขยาย solid ของ obstacle แต่ละชิ้นด้วยขนาดวัตถุที่ถือ (กว้าง/ยาว/สูง) -> แผนที่ความสูงสิ่งกีดขวาง
      solids  : list จาก build_solids() (ต้องมี poly_px, z_top, height_mm, top)
      floor_z : ความลึกของพื้นต่อ pixel (mm) ; shape = (H, W) ; intr = (fx, fy, cx, cy) ของภาพนั้น
    ฝาบนของ solid แบน ที่ความลึก z_top: ความสูงเหนือพื้นของ pixel = floor_z - z_top
    คืน (obstacle (H, W) float32 = ความสูงยอดที่ขยายแล้ว + h_obj, info list)
      info = {kind, w, l, h (ขนาดเดิม mm), W, L, H (ขนาดหลังขยาย mm), poly_infl (Nx2 px), z_top}
    """
    fx, fy = intr[0], intr[1]
    H, W = shape
    obs = np.zeros((H, W), np.float32)
    info = []
    for sol in solids:
        poly = sol.get('poly_px')
        if poly is None or len(poly) < 3:
            continue
        z_top = float(sol['z_top'])
        m = np.zeros((H, W), np.uint8)
        cv2.fillPoly(m, [np.round(poly).astype(np.int32)], 1)
        kern, add_w, add_l = object_kernel(w_mm, l_mm, extra_mm, z_top, fx, fy, mode, footprint)
        mi = cv2.dilate(m, kern)
        top_h = np.maximum(floor_z - z_top, 0.0).astype(np.float32)
        np.maximum(obs, np.where(mi > 0, top_h + h_obj, 0.0).astype(np.float32), out=obs)
        cnts, _ = cv2.findContours(mi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pi = np.zeros((0, 2), np.float32)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            pi = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True).reshape(-1, 2).astype(np.float32)
        top = sol['top']
        w0 = float(np.ptp(top[:, 0])) * 1000.0
        l0 = float(np.ptp(top[:, 1])) * 1000.0
        info.append({'kind': sol['kind'], 'w': w0, 'l': l0, 'h': float(sol['height_mm']),
                     'W': w0 + add_w, 'L': l0 + add_l, 'H': float(sol['height_mm']) + h_obj,
                     'poly_infl': pi, 'z_top': z_top})
    return obs, info


def residual_obstacle(h, solids, min_height_mm, intr, w_mm, l_mm, h_obj, z_floor_mm,
                      extra_mm=3.0, mode='shape', keep_out=None, footprint=None):
    """
    สิ่งกีดขวางที่ไม่ได้ถูกทำเป็น solid (เล็ก/เตี้ยเกิน threshold ของ build_solids) : เอาจาก height map ตรง ๆ
    แล้วขยายด้วยขนาดวัตถุเหมือน solid ; keep_out = (x, y, w, h) กรอบวัตถุที่จะหยิบ (ไม่นับ)
    คืน (obstacle float32 หรือ None ถ้าไม่มี, จำนวน pixel)
    """
    H, W = h.shape
    cov = np.zeros((H, W), np.uint8)
    for sol in solids:
        poly = sol.get('poly_px')
        if poly is not None and len(poly) >= 3:
            cv2.fillPoly(cov, [np.round(poly).astype(np.int32)], 1)
    cov = cv2.dilate(cov, np.ones((7, 7), np.uint8))           # เผื่อขอบเอียงของ solid
    res = np.where((h > min_height_mm) & (cov == 0), h, 0.0).astype(np.float32)
    if keep_out is not None:
        x, y, bw, bh = keep_out
        k = 8
        res[max(y - k, 0):y + bh + k, max(x - k, 0):x + bw + k] = 0.0
    n = int(np.count_nonzero(res))
    if n == 0:
        return None, 0
    z_ref = max(float(z_floor_mm) - float(res.max()), 100.0)   # ยอดที่สูงสุด = ใกล้กล้องสุด = pixel มากสุด (ปลอดภัย)
    kern, _, _ = object_kernel(w_mm, l_mm, extra_mm, z_ref, intr[0], intr[1], mode, footprint)
    rm = cv2.dilate(res, kern)
    return np.where(rm > 0, rm + h_obj, 0.0).astype(np.float32), n


def estimate_yellow_solid(o, plane, floor_z, intr, params):
    """
    ประมาณ solid ของวัตถุสีเหลืองจาก contour: fit รูปทรง + extrude ลงพื้น
    คืน solid dict (เหมือน build_solids + w_mm, l_mm = ขนาดบนระนาบของรูปทรงที่ fit) หรือ None
    """
    c = o.get('contour')
    if c is None or o['z_mm'] is None:
        return None
    H, W = floor_z.shape
    m = np.zeros((H, W), np.uint8)
    cv2.drawContours(m, [c], -1, 1, thickness=cv2.FILLED)
    if int(m.sum()) < 20:
        return None
    kind, poly = fit_primitive(m, params['shape_mode'], params['poly_eps'], params['rect_support'])
    if len(poly) < 3:
        return None
    z_top = float(o['z_mm'])
    sol = extrude(poly, z_top, plane, intr)
    cy_, cx_ = float(np.mean(poly[:, 1])), float(np.mean(poly[:, 0]))
    z_fl = float(floor_z[int(np.clip(cy_, 0, H - 1)), int(np.clip(cx_, 0, W - 1))])
    xy = (pix_to_xyz(poly, np.full(len(poly), z_top), intr)[:, :2] * 1000.0).astype(np.float32)
    (_, _), (a, b), _ = cv2.minAreaRect(xy)
    sol.update(kind='YELLOW ' + kind, height_mm=z_fl - z_top, color_idx=3, z_top=z_top, poly_px=poly,
               w_mm=float(max(a, b)), l_mm=float(min(a, b)),
               line_color=(0.85, 0.6, 0.0), line_width=2.4)
    return sol


_COLORMAPS = {
    'turbo': cv2.COLORMAP_TURBO,
    'jet': cv2.COLORMAP_JET,
    'inferno': cv2.COLORMAP_INFERNO,
    'viridis': cv2.COLORMAP_VIRIDIS,
    'plasma': cv2.COLORMAP_PLASMA,
}

# ใช้ LUT แค่ช่วง [LUT_LO, LUT_HI] จาก 0-255 เพื่อไม่ให้ปลายสองด้านของ colormap
# เข้มจนเกือบดำ (ซึ่งจะสับสนกับ pixel ที่ไม่มีข้อมูล depth ที่เราลงเป็นสีดำสนิท)
_LUT_LO = 20
_LUT_HI = 235


def colorize_depth(depth_mm, near_mm, far_mm, colormap):
    """
    แปลง depth (uint16, mm) เป็นภาพสี BGR ตามความลึก
      - ไกลสุด (far_mm, เช่น พื้น)  -> ปลายเข้มของ colormap
      - ใกล้สุด (near_mm, เช่น ยอดวัตถุ) -> ปลายสว่างของ colormap
      - pixel ที่ไม่มีข้อมูล (depth = 0) -> ดำสนิท
    ถ้า near_mm/far_mm ไม่ถูกต้อง (<=0 หรือ far<=near) จะคำนวณช่วงอัตโนมัติจากภาพ
    คืนค่า (ภาพสี, near_mm ที่ใช้จริง, far_mm ที่ใช้จริง)
    """
    valid = depth_mm > 0
    d = depth_mm.astype(np.float32)

    if near_mm <= 0 or far_mm <= near_mm:
        if valid.any():
            near_mm = float(d[valid].min())
            far_mm = float(np.percentile(d[valid], 99))
        if far_mm - near_mm < 1.0:  # ไม่มีข้อมูล หรือแทบไม่มีช่วงต่าง
            far_mm = near_mm + 1.0

    norm = np.clip((far_mm - d) / (far_mm - near_mm), 0.0, 1.0)  # ใกล้=1, ไกล=0
    idx = (_LUT_LO + norm * (_LUT_HI - _LUT_LO)).astype(np.uint8)
    color = cv2.applyColorMap(idx, colormap)
    color[~valid] = 0
    return color, near_mm, far_mm


def add_depth_legend(img, near_mm, far_mm, colormap, panel_w=120, n_ticks=6):
    """ต่อแถบสี (color bar) พร้อมตัวเลขระยะ (เมตร) ไว้ด้านขวาของภาพ"""
    h = img.shape[0]
    margin = 20
    bar_h = max(h - 2 * margin, 2)
    bar_w = 30

    # บน = ใกล้ (สว่าง), ล่าง = ไกล (เข้ม)
    grad = np.linspace(_LUT_HI, _LUT_LO, bar_h).astype(np.uint8)
    bar = cv2.applyColorMap(np.repeat(grad[:, None], bar_w, axis=1), colormap)

    panel = np.full((h, panel_w, 3), 255, dtype=np.uint8)
    panel[margin:margin + bar_h, 10:10 + bar_w] = bar
    cv2.rectangle(panel, (10, margin), (10 + bar_w, margin + bar_h - 1), (0, 0, 0), 1)

    for i in range(n_ticks):
        t = i / (n_ticks - 1)
        y = int(margin + t * (bar_h - 1))
        value_m = (near_mm + t * (far_mm - near_mm)) / 1000.0
        cv2.line(panel, (10 + bar_w, y), (10 + bar_w + 6, y), (0, 0, 0), 1)
        cv2.putText(
            panel, f'{value_m:.2f} m', (10 + bar_w + 10, y + 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA
        )
    return np.hstack([img, panel])


def depth_to_cloud_msg(depth_mm, bgr, intrinsics, header, max_depth_m=1.0, stride=1):
    """
    แปลง depth (uint16, mm) + ภาพสี BGR (ขนาดเท่ากัน) เป็น sensor_msgs/PointCloud2 (xyz + rgb)
    ใช้สูตร pinhole: X = (u-cx)*Z/fx, Y = (v-cy)*Z/fy, Z = depth
    พิกัดอยู่ใน frame ของ depth image (optical frame: X ขวา, Y ลง, Z พุ่งออกจากกล้อง)
    """
    fx, fy, cx, cy = intrinsics
    stride = max(int(stride), 1)
    d = depth_mm[::stride, ::stride]
    c = bgr[::stride, ::stride]
    h, w = d.shape

    v, u = np.mgrid[0:h, 0:w]
    u = (u * stride).astype(np.float32)
    v = (v * stride).astype(np.float32)
    z = d.astype(np.float32) * 0.001

    valid = (d > 0) & (z <= max_depth_m)
    z = z[valid]
    u = u[valid]
    v = v[valid]
    cv_ = c[valid]
    b = cv_[:, 0].astype(np.uint32)
    g = cv_[:, 1].astype(np.uint32)
    r = cv_[:, 2].astype(np.uint32)

    pts = np.empty(z.size, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    pts['x'] = (u - cx) * z / fx
    pts['y'] = (v - cy) * z / fy
    pts['z'] = z
    pts['rgb'] = (r << 16) | (g << 8) | b  # packed RGB ตามที่ RViz อ่าน

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(pts.size)
    msg.fields = [
        PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
        for n, o in (('x', 0), ('y', 4), ('z', 8), ('rgb', 12))
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * int(pts.size)
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def depth_to_grid_msg(depth_mm, intrinsics, header, near_mm, far_mm,
                      resolution=0.005, max_depth_m=1.0):
    """
    แปลง depth (uint16, mm) เป็น nav_msgs/OccupancyGrid มุมมองจากด้านบน
      - ค่าช่อง = ความสูงเหนือพื้น 0..100 (พื้น/far_mm = 0, ใกล้กล้องสุด/near_mm = 100)
      - ช่องที่ไม่มีข้อมูล = -1 (unknown)
      - แต่ละช่องเก็บจุดที่สูงที่สุด (ใกล้กล้องที่สุด) ที่ตกลงในช่องนั้น
    คืนค่า None ถ้าไม่มีจุดที่ใช้ได้เลย
    """
    fx, fy, cx, cy = intrinsics
    v, u = np.nonzero((depth_mm > 0) & (depth_mm <= max_depth_m * 1000))
    if v.size == 0:
        return None
    z = depth_mm[v, u].astype(np.float32) * 0.001
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    near, far = near_mm / 1000.0, far_mm / 1000.0
    if near <= 0 or far <= near:  # คำนวณช่วงอัตโนมัติเหมือนภาพสี
        near, far = float(z.min()), float(np.percentile(z, 99))
        far = max(far, near + 0.001)

    x_min, y_max = float(x.min()), float(y.max())
    W = int((float(x.max()) - x_min) / resolution) + 1
    H = int((y_max - float(y.min())) / resolution) + 1
    col = ((x - x_min) / resolution).astype(np.int32)
    row = ((y_max - y) / resolution).astype(np.int32)

    height = np.clip((far - z) / (far - near), 0.0, 1.0)
    val = (height * 100).astype(np.int8)

    grid = np.full(H * W, -1, dtype=np.int8)
    np.maximum.at(grid, row * W + col, val)

    msg = OccupancyGrid()
    msg.header.frame_id = header.frame_id
    msg.header.stamp = Time()  # stamp = 0 -> RViz ใช้ TF ล่าสุดเสมอ ไม่ติดเรื่อง timestamp เก่า
    msg.info.map_load_time = Time()
    msg.info.resolution = float(resolution)
    msg.info.width = W
    msg.info.height = H
    # หมุน 180 องศารอบแกน x เพื่อให้มองจากด้านบนแล้วภาพไม่กลับด้าน
    # (optical frame: x ขวา, y ลง, z ออกจากกล้อง)
    msg.info.origin.position.x = x_min
    msg.info.origin.position.y = y_max
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.x = 1.0
    msg.info.origin.orientation.y = 0.0
    msg.info.origin.orientation.z = 0.0
    msg.info.origin.orientation.w = 0.0
    msg.data = grid.tolist()
    return msg


# ============================================================================
#  Solid (extrude boss) pipeline: depth -> แยกวัตถุ -> fit รูปทรง -> extrude ลงพื้น
# ============================================================================
# ---------------------------------------------------------------- floor plane
def fit_floor_plane(depth_mm, iters=4, tol_mm=4.0, far_fraction=0.5, floor_mm=0.0):
    """Fit plane on inverse depth: 1/Z = a*u + b*v + c. Returns (a,b,c) or None."""
    h, w = depth_mm.shape
    if floor_mm and floor_mm > 0:
        return (0.0, 0.0, 1.0 / float(floor_mm))
    valid = depth_mm > 0
    if valid.sum() < 100:
        return None
    v, u = np.nonzero(valid)
    z = depth_mm[v, u].astype(np.float64)
    keep = z >= np.percentile(z, 100 * (1.0 - far_fraction))
    A = np.stack([u, v, np.ones_like(u)], 1).astype(np.float64)
    inv = 1.0 / z
    sol = None
    for _ in range(iters):
        if keep.sum() < 50:
            break
        sol, *_ = np.linalg.lstsq(A[keep], inv[keep], rcond=None)
        pred_z = 1.0 / np.maximum(A @ sol, 1e-9)
        keep = np.abs(pred_z - z) < tol_mm
    return None if sol is None else (float(sol[0]), float(sol[1]), float(sol[2]))


def floor_map(plane, shape):
    a, b, c = plane
    v, u = np.mgrid[0:shape[0], 0:shape[1]].astype(np.float32)
    return 1.0 / np.maximum(a * u + b * v + c, 1e-9)


# ------------------------------------------------------------------ filtering
def make_height_map(depth_mm, floor_z, median_ksize=5, bilateral_sigma_mm=6.0):
    """Height above floor in mm. Invalid / below-floor pixels -> 0 (so black rims drop out)."""
    valid = depth_mm > 0
    h = np.where(valid, floor_z - depth_mm.astype(np.float32), 0.0).astype(np.float32)
    h = np.maximum(h, 0.0)
    if median_ksize >= 3:
        h = cv2.medianBlur(h, int(median_ksize))
    if bilateral_sigma_mm > 0:
        h = cv2.bilateralFilter(h, 7, float(bilateral_sigma_mm), 3.0)
    return h


# ------------------------------------------------------------- segmentation
def _components(mask):
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=4)
    return n, lab, stats


def split_by_height(h, base_mask, step_mm=1.5, min_area_px=150, min_prom_mm=4.0):
    """
    Component-tree splitting: raise the threshold step by step; whenever a component splits
    into >=2 sub-components that are each big (min_area_px) and prominent (peak - t >= min_prom_mm)
    they become separate seeds.  Returns list of seed masks (bool arrays).
    """
    seeds = []

    def recurse(mask, t):
        start = mask
        while True:
            t2 = t + step_mm
            sub = mask & (h > t2)
            if not sub.any():
                seeds.append(start)
                return
            n, lab, stats = _components(sub)
            good = []
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] < min_area_px:
                    continue
                m = lab == i
                if h[m].max() - t2 >= min_prom_mm:
                    good.append(m)
            if len(good) == 0:
                seeds.append(start)
                return
            if len(good) == 1:
                mask, t = good[0], t2      # same object, keep climbing (shrinks to the plateau)
                continue
            for m in good:
                recurse(m, t2)
            return

    n, lab, stats = _components(base_mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area_px:
            recurse(lab == i, 0.0)
    return seeds


def expand_seeds(h, base_mask, seeds):
    """Grow seeds back over base_mask (watershed on -height): each pixel goes to one object only."""
    if not seeds:
        return np.zeros(h.shape, np.int32)
    markers = np.zeros(h.shape, np.int32)
    for i, m in enumerate(seeds, start=1):
        markers[m] = i
    bg = len(seeds) + 1
    markers[~base_mask] = bg
    hh = np.clip(h / max(float(h.max()), 1.0) * 255.0, 0, 255).astype(np.uint8)
    img = cv2.cvtColor(255 - hh, cv2.COLOR_GRAY2BGR)       # tall = dark = flooded first
    cv2.watershed(img, markers)
    markers[markers == bg] = 0
    markers[markers < 0] = 0
    return markers


def refine_object(h, region, edge_level=0.6, close_px=5, open_px=3):
    """
    Cut one object at edge_level * (its own top height), fill holes, remove thin spurs.
    Returns (mask uint8, top_h_mm)
    """
    if not region.any():
        return None, 0.0
    top = float(np.percentile(h[region], 75))
    m = (region & (h > edge_level * top)).astype(np.uint8)
    if close_px > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    if open_px > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return None, top
    c = max(cnts, key=cv2.contourArea)
    out = np.zeros_like(m)
    cv2.drawContours(out, [c], -1, 1, thickness=cv2.FILLED)   # fills interior holes
    return out, top


# --------------------------------------------------------------- primitives
def _robust_rect(mask, support=0.7):
    """Rectangle from row/column support profiles in the rotated frame (ignores smears / spurs)."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)
    (cx, cy), (rw, rh), ang = cv2.minAreaRect(c)
    M = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)          # rotate by +ang -> axis aligned
    big = int(np.ceil(np.hypot(*mask.shape))) + 4
    M[0, 2] += big / 2 - cx
    M[1, 2] += big / 2 - cy
    rot = cv2.warpAffine(mask, M, (big, big), flags=cv2.INTER_NEAREST)
    cols = rot.sum(0).astype(np.float32)
    rows = rot.sum(1).astype(np.float32)
    xs = np.nonzero(cols >= support * cols.max())[0]
    ys = np.nonzero(rows >= support * rows.max())[0]
    x0, x1, y0, y1 = xs[0], xs[-1] + 1, ys[0], ys[-1] + 1
    pts = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)
    Mi = cv2.invertAffineTransform(M)
    pts = (pts @ Mi[:, :2].T + Mi[:, 2]).astype(np.float32)
    return pts


def fit_primitive(mask, mode='auto', poly_eps=0.03, rect_support=0.7, circle_pts=48):
    """Return (kind, polygon Nx2 float32 in pixel coords, CCW-ish)."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    hull = cv2.convexHull(c)
    hull_area = max(cv2.contourArea(hull), 1.0)
    per = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, poly_eps * per, True).reshape(-1, 2).astype(np.float32)
    if mode == 'contour':
        return 'polygon', approx
    solidity = area / hull_area
    hper = cv2.arcLength(hull, True)
    circ = 4 * np.pi * hull_area / max(hper * hper, 1.0)
    n = len(approx)
    if len(c) >= 5 and circ > 0.88 and n >= 7 and solidity > 0.93:
        (ex, ey), (ma, mi), ang = cv2.fitEllipse(hull)
        a, b = ma / 2.0, mi / 2.0
        if max(a, b) / max(min(a, b), 1e-6) < 1.12:
            a = b = np.sqrt(a * b)
        t = np.linspace(0, 2 * np.pi, circle_pts, endpoint=False)
        ca, sa = np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))
        x, y = a * np.cos(t), b * np.sin(t)
        poly = np.stack([ex + x * ca - y * sa, ey + x * sa + y * ca], 1).astype(np.float32)
        return ('circle' if a == b else 'ellipse'), poly
    rect_pts = _robust_rect(mask, rect_support)
    rmask = np.zeros_like(mask)
    cv2.fillPoly(rmask, [np.round(rect_pts).astype(np.int32)], 1)
    inter = int(np.count_nonzero(rmask & mask))
    union = int(np.count_nonzero(rmask | mask))
    iou = inter / max(union, 1)
    if iou > 0.88 or (n == 4 and iou > 0.8):
        return 'rectangle', rect_pts.astype(np.float32)
    if n == 3:
        return 'triangle', approx
    return 'polygon', approx


# ------------------------------------------------------------------ meshing
def ear_clip(poly):
    """Triangulate simple polygon (Nx2). Returns list of (i,j,k) index triples (CCW)."""
    n = len(poly)
    idx = list(range(n))
    area2 = np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1])
    if area2 < 0:
        idx.reverse()
    tris = []

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def inside(p, a, b, c):
        return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        clipped = False
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if cross(a, b, c) <= 1e-9:
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            clipped = True
            break
        if not clipped:
            break
    if len(idx) == 3:
        tris.append((idx[0], idx[1], idx[2]))
    return tris


def pix_to_xyz(poly_px, z_mm, intr):
    fx, fy, cx, cy = intr
    z = np.asarray(z_mm, np.float64) * 0.001
    x = (poly_px[:, 0] - cx) * z / fx
    y = (poly_px[:, 1] - cy) * z / fy
    return np.stack([x, y, z], 1)


def extrude(poly_px, z_top_mm, plane, intr):
    """Extrude polygon from the top plane (flat, z_top) to the floor plane. Returns dict of faces."""
    a, b, c = plane
    zb = 1.0 / np.maximum(a * poly_px[:, 0] + b * poly_px[:, 1] + c, 1e-9)
    top = pix_to_xyz(poly_px, np.full(len(poly_px), z_top_mm), intr)
    bot = pix_to_xyz(poly_px, zb, intr)
    tris = ear_clip(poly_px)
    return {'top': top, 'bot': bot, 'tris': tris}



# ทิศ "ไปหาแหล่งกำเนิดแสง" (optical frame: -z = เข้าหากล้อง = ด้านบนของวัตถุ) -> ฝาบนสว่างสุด
_LIGHT = np.array([-0.35, 0.30, -0.89])
_LIGHT = _LIGHT / np.linalg.norm(_LIGHT)
_PALETTE = [(0.95, 0.35, 0.30), (0.25, 0.65, 0.95), (0.35, 0.85, 0.45),
            (0.95, 0.80, 0.25), (0.75, 0.45, 0.95), (0.95, 0.55, 0.20)]


def _shade(base_rgb, normal):
    """
    อบแสงเงาลงใน vertex color (RViz แสดง TRIANGLE_LIST แบบไม่มีเงา จึงต้องอบเอง)
    normal ต้องชี้ "ออกนอกวัตถุ" แล้ว ฝาบน (ชี้เข้าหากล้อง) จะสว่างที่สุด
    """
    n = normal / max(np.linalg.norm(normal), 1e-9)
    lam = max(float(np.dot(n, _LIGHT)), 0.0)
    k = 0.35 + 0.65 * lam
    return base_rgb[0] * k, base_rgb[1] * k, base_rgb[2] * k


def solids_to_markers(solids, header, lifetime_s=0.0, show_floor_rect=None, alpha=1.0,
                      show_labels=True, label_size=0.012):
    """
    solids: list ของ dict {'kind','top','bot','tris','color_idx'} จาก extrude()
    คืน MarkerArray: ผิวทึบ (TRIANGLE_LIST) + เส้นขอบ (LINE_LIST) ต่อวัตถุ + พื้น (ถ้ามี)
    ทุก Marker ใช้ frame เดียวกับ point cloud
    """
    arr = MarkerArray()
    clear = Marker()
    clear.header.frame_id = header.frame_id
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    def new_marker(mid, ns, mtype):
        m = Marker()
        m.header.frame_id = header.frame_id
        m.header.stamp = Time()       # stamp = 0 -> ใช้ TF ล่าสุด (ไม่แตะ header ต้นฉบับ)
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec = int(lifetime_s)
        return m

    def P(v):
        return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

    for oid, s in enumerate(solids):
        base = _PALETTE[s['color_idx'] % len(_PALETTE)]
        top, bot, tris = s['top'], s['bot'], s['tris']
        n = len(top)

        mesh = new_marker(oid, 'solid', Marker.TRIANGLE_LIST)
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
        mesh.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=float(alpha))

        centroid = np.vstack([top, bot]).mean(axis=0)

        def add_tri(a, b, c):
            normal = np.cross(b - a, c - a)
            if np.dot(normal, (a + b + c) / 3.0 - centroid) < 0:   # บังคับให้ normal ชี้ออกนอกวัตถุ
                normal = -normal
            r, g, bl = _shade(base, normal)
            for v in (a, b, c):
                mesh.points.append(P(v))
                mesh.colors.append(ColorRGBA(r=r, g=g, b=bl, a=float(alpha)))

        for i, j, k in tris:                       # ฝาบน (top face) และฝาล่าง
            add_tri(top[i], top[j], top[k])
            add_tri(bot[i], bot[k], bot[j])
        for i in range(n):                         # ผนังข้าง
            j = (i + 1) % n
            add_tri(top[i], top[j], bot[j])
            add_tri(top[i], bot[j], bot[i])
        arr.markers.append(mesh)

        edges = new_marker(oid, 'edges', Marker.LINE_LIST)
        edges.scale.x = 0.0015
        edges.color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=1.0)
        for i in range(n):
            j = (i + 1) % n
            for a, b in ((top[i], top[j]), (bot[i], bot[j]), (top[i], bot[i])):
                edges.points.append(P(a))
                edges.points.append(P(b))
        arr.markers.append(edges)

        if show_labels:
            label = new_marker(oid, 'label', Marker.TEXT_VIEW_FACING)
            c = top.mean(axis=0)
            label.pose.position.x = float(c[0])
            label.pose.position.y = float(c[1])
            label.pose.position.z = float(c[2]) - 2.0 * label_size   # ลอยเหนือฝาบน (optical: -z = ขึ้น)
            label.scale.z = float(label_size)
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"{s['kind']} {s['height_mm']:.0f}mm"
            arr.markers.append(label)

    if show_floor_rect is not None:
        fl = new_marker(0, 'floor', Marker.TRIANGLE_LIST)
        fl.scale.x = fl.scale.y = fl.scale.z = 1.0
        fl.color = ColorRGBA(r=0.6, g=0.6, b=0.65, a=0.35)
        q = show_floor_rect
        for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
            for v in (a, b, c):
                fl.points.append(P(v))
        arr.markers.append(fl)
    return arr


def build_solids(depth_mm, intr, params):
    """
    pipeline เต็ม: depth (uint16 mm, crop แล้ว) -> list ของ solid dict
    params: dict ของ solid_* parameters. คืน (solids, floor_rect_xyz, debug_vis_bgr)
    """
    plane = fit_floor_plane(depth_mm, floor_mm=params['floor_mm'])
    if plane is None:
        return [], None, None
    fz = floor_map(plane, depth_mm.shape)
    h = make_height_map(depth_mm, fz, params['median_ksize'], params['bilateral_sigma_mm'])
    base = h > params['min_height_mm']
    seeds = split_by_height(
        h, base, step_mm=params['split_step_mm'],
        min_area_px=params['min_area_px'], min_prom_mm=params['min_prominence_mm'])
    labels = expand_seeds(h, base, seeds)

    solids = []
    dbg = np.zeros(depth_mm.shape + (3,), np.uint8)
    for lab_id in range(1, int(labels.max()) + 1):
        m, top_h = refine_object(
            h, labels == lab_id, params['edge_level'], params['close_px'], params['open_px'])
        if m is None or int(m.sum()) < params['min_area_px']:
            continue
        kind, poly = fit_primitive(
            m, params['shape_mode'], params['poly_eps'], params['rect_support'])
        if len(poly) < 3:
            continue
        # ความสูงยอด: median ของ depth ในพื้นที่ (erode เข้าข้างในเล็กน้อย) = ฝาบนแบน
        inner = cv2.erode(m, np.ones((5, 5), np.uint8)).astype(bool)
        if inner.sum() < 20:
            inner = m.astype(bool)
        dz = depth_mm[inner]
        dz = dz[dz > 0]
        if dz.size == 0:
            continue
        z_top = float(np.median(dz))
        sol = extrude(poly, z_top, plane, intr)
        cy_, cx_ = np.mean(poly[:, 1]), np.mean(poly[:, 0])
        z_floor_c = float(fz[int(np.clip(cy_, 0, fz.shape[0] - 1)), int(np.clip(cx_, 0, fz.shape[1] - 1))])
        sol.update(kind=kind, height_mm=z_floor_c - z_top, color_idx=len(solids), z_top=z_top,
                   poly_px=poly)
        solids.append(sol)
        col = tuple(int(255 * c) for c in _PALETTE[sol['color_idx'] % len(_PALETTE)][::-1])
        dbg[m > 0] = col
        cv2.polylines(dbg, [poly.astype(np.int32).reshape(-1, 1, 2)], True, (255, 255, 255), 1)

    hh, ww = depth_mm.shape
    corners = np.array([[0, 0], [ww, 0], [ww, hh], [0, hh]], np.float32)
    a, b, c = plane
    zc = 1.0 / np.maximum(a * corners[:, 0] + b * corners[:, 1] + c, 1e-9)
    floor_rect = pix_to_xyz(corners, zc, intr)
    return solids, floor_rect, dbg


# ============================================================================
#  แผนที่ 3 มิติ (แทนหน้าต่าง matplotlib เดิม) -> ข้อมูล JSON ให้ dashboard วาดด้วย Plotly (หมุน/ซูมด้วยเมาส์)
# ============================================================================
def plotly_colorscale(name='turbo', n=11):
    """colormap -> colorscale ของ Plotly [[0, 'rgb(..)'], ..., [1, 'rgb(..)']] (ใช้ matplotlib ถ้ามี ไม่งั้นใช้ OpenCV)"""
    cols = None
    try:
        import matplotlib
        cm = matplotlib.colormaps[name]
        cols = [cm(i / (n - 1))[:3] for i in range(n)]
    except Exception:
        lut = cv2.applyColorMap(np.linspace(0, 255, n).astype(np.uint8).reshape(-1, 1),
                                _COLORMAPS.get(str(name).lower(), cv2.COLORMAP_TURBO)).reshape(-1, 3)
        cols = [(c[2] / 255.0, c[1] / 255.0, c[0] / 255.0) for c in lut]
    return [[round(i / (n - 1), 4), 'rgb(%d,%d,%d)' % tuple(int(round(255 * v)) for v in c)]
            for i, c in enumerate(cols)]


def _rgb_css(c):
    return 'rgb(%d,%d,%d)' % tuple(int(round(255 * float(v))) for v in c[:3])


def _r1(a):
    """numpy -> list ทศนิยม 1 ตำแหน่ง (NaN/inf -> 0) สำหรับ JSON"""
    return np.round(np.nan_to_num(np.asarray(a, np.float64), nan=0.0, posinf=0.0, neginf=0.0), 1).tolist()


class Map3DData:
    """
    สร้างข้อมูลแผนที่ความสูง 3 มิติชุดเดียวกับหน้าต่าง \"3D height map\" เดิม แล้วให้ browser วาดด้วย Plotly
      - surface ของ \"ความสูงเหนือพื้น\" (mm) ระบายสีตามความสูง
      - โครงขอบ (บน/ล่าง/แนวตั้ง) ของ solid ที่ fit ได้ พร้อม label รูปทรง + ความสูง (solid ที่ขยายแล้ว = ส้มประ)
      - เส้นทาง 3 มิติ (ม่วง) + เงาบนพื้น + ป้าย pick / place, เส้นแบ่งโซน (แดง)
    แกน: X = ขวา, Y = ขึ้นบนภาพ, Z = ความสูงเหนือพื้น (หน่วย mm ทั้งหมด)
    """

    def __init__(self, stride=3, cmap='turbo', z_aspect=0.35, title='3D height map'):
        self.stride = max(int(stride), 1)
        self.z_aspect = float(z_aspect)
        self.title = title
        self.colorscale = plotly_colorscale(cmap)

    def build(self, depth_mm, intrinsics, params, solids=None, path=None, zones=None, path_note=None):
        """คืน dict สำหรับ Plotly หรือ None ถ้าหาระนาบพื้นไม่ได้ (พารามิเตอร์เหมือน Map3DViewer.update เดิม)"""
        plane = fit_floor_plane(depth_mm, floor_mm=params['floor_mm'])
        if plane is None:
            return None
        H, W = depth_mm.shape
        fz = floor_map(plane, depth_mm.shape)
        h = make_height_map(depth_mm, fz, params['median_ksize'], params['bilateral_sigma_mm'])

        s = self.stride
        ds, fzs, Z = depth_mm[::s, ::s], fz[::s, ::s], h[::s, ::s]
        v, u = np.mgrid[0:H:s, 0:W:s]
        fx, fy, cx, cy = intrinsics
        zd = np.where(ds > 0, np.minimum(ds.astype(np.float32), fzs), fzs)
        X = (u - cx) * zd / fx          # mm
        Y = -(v - cy) * zd / fy         # mm (บนภาพ = +Y)
        zmax = max(float(np.nanmax(Z)), 1.0)

        groups = {}
        labels = []
        for sol in solids or []:
            top, bot = sol['top'], sol['bot']
            xs, ys = top[:, 0] * 1000.0, -top[:, 1] * 1000.0
            zt = (bot[:, 2] - top[:, 2]) * 1000.0
            n = len(xs)
            key = (_rgb_css(sol.get('line_color', (0.05, 0.05, 0.05))),
                   'solid',      # เส้นประใน Plotly 3D ขาดเป็นท่อน ๆ อ่านยาก: solid ที่ขยายแล้วใช้สีส้มแยกแทน
                   round(float(sol.get('line_width', 1.6)) * 1.6, 1))
            gx, gy, gz = groups.setdefault(key, ([], [], []))
            vstep = max(1, n // 8)

            def seg(p, q):
                gx.extend([p[0], q[0], None])
                gy.extend([p[1], q[1], None])
                gz.extend([p[2], q[2], None])
            for i in range(n):
                j = (i + 1) % n
                seg((xs[i], ys[i], zt[i]), (xs[j], ys[j], zt[j]))      # ขอบฝาบน
                seg((xs[i], ys[i], 0.0), (xs[j], ys[j], 0.0))          # ขอบล่าง (พื้น)
                if i % vstep == 0:
                    seg((xs[i], ys[i], 0.0), (xs[i], ys[i], zt[i]))    # เสาแนวตั้ง
            labels.append([round(float(xs.mean()), 1), round(float(ys.mean()), 1),
                           round(float(zt.mean()) + 0.08 * zmax, 1), f"{sol['kind']} {sol['height_mm']:.0f}mm"])

        def rl(vals):
            return [None if q is None else round(float(q), 1) for q in vals]
        lines = [{'color': k[0], 'dash': k[1], 'width': k[2], 'x': rl(g[0]), 'y': rl(g[1]), 'z': rl(g[2])}
                 for k, g in groups.items()]

        zone_lines = []
        if zones is not None:
            u_cols = np.arange(0, W, s)
            for col in zones:
                i = int(np.argmin(np.abs(u_cols - col)))
                zone_lines.append({'x': _r1(X[:, i]), 'y': _r1(Y[:, i]), 'z': _r1(Z[:, i] + 1.0)})

        ztop = zmax
        x0, x1, y0, y1 = float(X.min()), float(X.max()), float(Y.min()), float(Y.max())
        path_d = None
        if path is not None and len(path):
            pa = np.asarray(path, np.float64)
            path_d = {'x': _r1(pa[:, 0]), 'y': _r1(pa[:, 1]), 'z': _r1(pa[:, 2]),
                      'label_dz': round(0.06 * zmax, 1)}
            ztop = max(zmax, float(pa[:, 2].max()))
            x0, x1 = min(x0, float(pa[:, 0].min())), max(x1, float(pa[:, 0].max()))
            y0, y1 = min(y0, float(pa[:, 1].min())), max(y1, float(pa[:, 1].max()))
        span = max(x1 - x0, y1 - y0, 1e-6)
        zr = self.z_aspect if self.z_aspect > 0 else (ztop * 1.15) / span     # 0 = สเกลจริง
        title = f'{self.title}  ({W}x{H}px, {len(solids or [])} solids)'
        if path_note:
            title += f'\n{path_note}'
        return {
            'title': title,
            'surface': {'x': _r1(X), 'y': _r1(Y), 'z': _r1(Z), 'cmax': round(zmax, 1)},
            'colorscale': self.colorscale,
            'lines': lines, 'labels': labels, 'zones': zone_lines, 'path': path_d,
            'ranges': {'x': [round(x0, 1), round(x1, 1)], 'y': [round(y0, 1), round(y1, 1)],
                       'z': [0.0, round(ztop * 1.15, 1)]},
            'aspect': {'x': round((x1 - x0) / span, 4), 'y': round((y1 - y0) / span, 4), 'z': round(zr, 4)},
        }


# ============================================================================
#  Dashboard (หน้าเว็บ) : แทนหน้าต่าง OpenCV / matplotlib เดิมทั้งหมด
# ============================================================================
PLOTLY_CDN = 'https://cdn.plot.ly/plotly-2.35.2.min.js'
DASH_ALGO_OPTIONS = ((1, 'Dijkstra'), (2, 'A*'), (3, 'Bidirectional Dijkstra'),
                     (4, 'Bidirectional A*'), (5, 'Comparison'))


# ============================================================================
#  Hand guard (MediaPipe Hands): หยุดหุ่นเมื่อพบมือ + ใช้นิ้วชี้เลือกจุดวาง (กติกานับนิ้วจาก finger_tag.py)
# ============================================================================
HAND_FINGERS = (                     # ชื่อนิ้ว, landmark ปลายนิ้ว, ข้อกลาง, สี BGR
    ('Thumb', 4, 3, (0, 165, 255)),
    ('Index', 8, 6, (0, 255, 0)),
    ('Middle', 12, 10, (255, 200, 0)),
    ('Ring', 16, 14, (255, 0, 255)),
    ('Pinky', 20, 18, (0, 0, 255)),
)
HAND_WRIST, HAND_PINKY_MCP = 0, 17
HAND_CONNECTIONS = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
                    (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20))
HOLD_CLEAR, HOLD_HAND, HOLD_GUARD_OFF = 0, 1, 2     # ค่าที่เขียนลง reg hand_hold_reg ของ modbus_server.py


def hand_fingers_extended(pts):
    """pts = landmark 21 จุด (pixel) -> [Thumb, Index, Middle, Ring, Pinky] ว่าเหยียดไหม (ใช้ระยะ ไม่ขึ้นกับการหมุนมือ)"""
    def d(a, b):
        return float(np.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1]))
    out = []
    for name, tip, pip, _ in HAND_FINGERS:
        if name == 'Thumb':
            out.append(d(tip, HAND_PINKY_MCP) > d(pip, HAND_PINKY_MCP) * 1.05)
        else:
            out.append(d(tip, HAND_WRIST) > d(pip, HAND_WRIST) * 1.08)
    return out


def is_index_pointing(up):
    """ชูนิ้วชี้นิ้วเดียว: นิ้วชี้เหยียด นิ้วกลาง/นาง/ก้อยงอ (ไม่นับนิ้วโป้ง เพราะท่าชี้มักกางโป้งออกเอง)"""
    return bool(up[1]) and not any(up[2:])


def dedupe_hands(hands, iou_max=0.3):
    """MediaPipe (max_num_hands=2) บางเฟรมเห็นมือเดียวเป็น 2 มือซ้อนกัน: ตัดตัวที่กรอบทับกันมาก เก็บ score สูงกว่า"""
    def box(h):
        p = h['pts']
        return p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()
    keep = []
    for h in sorted(hands, key=lambda h: -h['score']):
        a = box(h)
        dup = False
        for k in keep:
            b = box(k)
            iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
            ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
            inter = iw * ih
            union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
            if union > 0 and inter / union > iou_max:
                dup = True
                break
        if not dup:
            keep.append(h)
    return keep


def put_outlined(img, text, org, color, scale=0.4):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


HAND_TASK_URL = ('https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/'
                 'float16/1/hand_landmarker.task')


class HandDetector:
    """MediaPipe Hands ได้ทั้ง 2 แบบ:
    - mediapipe รุ่นเก่า (มี mp.solutions เหมือน finger_tag.py) ใช้ได้เลย
    - mediapipe รุ่นใหม่ (0.10.3x ตัด mp.solutions ออกแล้ว) ใช้ HandLandmarker ซึ่งต้องมีไฟล์โมเดล hand_landmarker.task
    process(rgb) -> [(landmark 21x2 แบบ 0..1, label ของ MediaPipe, score), ...]"""

    def __init__(self, max_num, complexity, min_det, min_track, model_path=''):
        import mediapipe as mp
        self.mp = mp
        sol = getattr(mp, 'solutions', None)
        if sol is not None and hasattr(sol, 'hands'):
            self.kind = 'mp.solutions.hands'
            self.h = sol.hands.Hands(static_image_mode=False, max_num_hands=max_num, model_complexity=complexity,
                                     min_detection_confidence=min_det, min_tracking_confidence=min_track)
            return
        from mediapipe.tasks import python as mp_tasks
        from mediapipe.tasks.python import vision
        cands = [model_path] if model_path else []
        cands.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hand_landmarker.task'))
        path = next((p for p in cands if p and os.path.isfile(p)), None)
        if path is None:
            raise RuntimeError(
                f'mediapipe {getattr(mp, "__version__", "")} ไม่มี mp.solutions แล้ว ต้องใช้ไฟล์โมเดล hand_landmarker.task: '
                f'wget -O hand_landmarker.task {HAND_TASK_URL} (วางไว้ข้างสคริปต์ หรือ -p hand_model_path:=...)')
        opts = vision.HandLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=path),
            running_mode=vision.RunningMode.VIDEO, num_hands=max_num,
            min_hand_detection_confidence=min_det, min_hand_presence_confidence=min_det,
            min_tracking_confidence=min_track)
        self.h = vision.HandLandmarker.create_from_options(opts)
        self.kind = f'HandLandmarker ({os.path.basename(path)})'
        self._ts = 0

    def process(self, rgb):
        out = []
        if self.kind == 'mp.solutions.hands':
            res = self.h.process(rgb)
            hd = res.multi_handedness or []
            for i, hand in enumerate(res.multi_hand_landmarks or []):
                c = hd[i].classification[0] if i < len(hd) else None
                out.append(([(p.x, p.y) for p in hand.landmark], c.label if c else '?', float(c.score) if c else 0.0))
            return out
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        self._ts = max(self._ts + 1, int(time.monotonic() * 1000))     # VIDEO mode: เวลาต้องเพิ่มขึ้นเสมอ
        res = self.h.detect_for_video(img, self._ts)
        for i, lms in enumerate(res.hand_landmarks or []):
            cat = res.handedness[i][0] if i < len(res.handedness or []) and res.handedness[i] else None
            out.append(([(p.x, p.y) for p in lms], cat.category_name if cat else '?', float(cat.score) if cat else 0.0))
        return out

    def close(self):
        try:
            self.h.close()
        except Exception:
            pass


class _ModbusLink:
    """socket Modbus ของ thread hand guard (ต่อค้างไว้ ต่อใหม่เองเมื่อหลุด)"""

    def __init__(self, node):
        self.node, self.sock, self.tid, self.warned = node, None, 0, False

    def write(self, reg, val):
        n = self.node
        try:
            if self.sock is None:
                self.sock = socket.create_connection((n.modbus_ip, n.modbus_port), timeout=1.0)
            self.tid = (self.tid + 1) % 65536
            modbus_write_reg(self.sock, self.tid, n.modbus_unit, reg, int(val))
            if self.warned:
                n.get_logger().info(f'Hand guard: ส่งสถานะไป server (reg {reg}) ได้แล้ว')
                self.warned = False
            return True
        except Exception as e:
            self.close()
            if not self.warned:
                n.get_logger().warn(f'Hand guard: ส่งสถานะไป server (reg {reg}) ไม่ได้: {e} '
                                    '(server จะสั่งหุ่นหยุดเองถ้าขาดสัญญาณระหว่างหุ่นทำงาน)')
                self.warned = True
            return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class _LogTap:
    """ห่อ logger ของ ROS: log ไปที่ terminal ตามเดิม และเก็บข้อความไว้แสดงบน dashboard
    (แยก method ละระดับ เพราะ rclpy จำระดับ log ต่อจุดที่เรียก ถ้าใช้จุดเดียวกันหลายระดับจะ error)"""

    def __init__(self, logger, sink):
        self._l = logger
        self._sink = sink
        self.quiet_thread = None     # thread ที่กำลัง dynamic scan: ไม่พิมพ์ info, คำเตือนซ้ำ ๆ พิมพ์ครั้งเดียว
        self._seen = {}

    def _quiet(self):
        return self.quiet_thread is not None and self.quiet_thread == threading.get_ident()

    def _dup(self, msg):
        """คำเตือนเดิม (ไม่นับตัวเลข) ภายใน 30 วินาที = ซ้ำ"""
        key = re.sub(r'[-+]?\d+(\.\d+)?', '#', str(msg))
        now = time.monotonic()
        t = self._seen.get(key)
        if t is not None and now - t < 30.0:
            return True
        if len(self._seen) > 300:
            self._seen.clear()
        self._seen[key] = now
        return False

    def _tap(self, level, msg):
        try:
            self._sink(level, str(msg))
        except Exception:
            pass

    def debug(self, msg, *a, **k):
        return self._l.debug(msg, *a, **k)

    def info(self, msg, *a, **k):
        if self._quiet():
            return self._l.debug(msg, *a, **k)
        self._tap('info', msg)
        return self._l.info(msg, *a, **k)

    def warn(self, msg, *a, **k):
        return self.warning(msg, *a, **k)

    def warning(self, msg, *a, **k):
        if self._quiet() and self._dup(msg):
            return None
        self._tap('warn', msg)
        return self._l.warning(msg, *a, **k)

    def error(self, msg, *a, **k):
        if self._quiet() and self._dup(msg):
            return None
        self._tap('error', msg)
        return self._l.error(msg, *a, **k)

    def __getattr__(self, name):
        return getattr(self._l, name)


def find_plotly_js(user_path=''):
    """หาไฟล์ plotly.min.js ในเครื่อง (ใช้ได้แม้ไม่มีอินเทอร์เน็ต) ; ไม่เจอ = None -> ให้ browser โหลดจาก CDN"""
    cands = []
    if user_path:
        cands.append(user_path)
    cands.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plotly.min.js'))
    try:
        import plotly
        cands.append(os.path.join(os.path.dirname(plotly.__file__), 'package_data', 'plotly.min.js'))
    except Exception:
        pass
    for p in cands:
        if p and os.path.isfile(p):
            return p
    return None


def make_dashboard_handler(node, plotly_path):
    cache = {}

    class Handler(BaseHTTPRequestHandler):
        server_version = 'RGBDDashboard/1.0'

        def log_message(self, fmt, *args):      # ไม่พิมพ์ log ของทุก request
            pass

        def _send(self, code, body, ctype, cache_s=0):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', f'max-age={cache_s}' if cache_s else 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode('utf-8'), 'application/json; charset=utf-8')

        def _empty(self, code):
            self.send_response(code)
            self.send_header('Content-Length', '0')
            self.end_headers()

        def do_GET(self):
            u = urlparse(self.path)
            p = u.path
            try:
                if p in ('/', '/index.html'):
                    self._send(200, DASHBOARD_HTML.encode('utf-8'), 'text/html; charset=utf-8')
                elif p == '/plotly.js':
                    if plotly_path:
                        if 'js' not in cache:
                            with open(plotly_path, 'rb') as f:
                                cache['js'] = f.read()
                        self._send(200, cache['js'], 'application/javascript', cache_s=86400)
                    else:
                        self.send_response(302)
                        self.send_header('Location', PLOTLY_CDN)
                        self.send_header('Content-Length', '0')
                        self.end_headers()
                elif p == '/favicon.ico':
                    self._empty(204)
                elif p == '/stream.mjpg':
                    self._stream()
                elif p in ('/img/depth.png', '/img/seg.png'):
                    data = node._img.get(p[5:-4])
                    if data is None:
                        self._empty(404)
                    else:
                        self._send(200, data, 'image/png')
                elif p == '/api/state':
                    self._json(node.dashboard_state())
                elif p == '/api/map3d':
                    data = node._map3d_json
                    if data is None:
                        self._empty(404)
                    else:
                        self._send(200, data, 'application/json; charset=utf-8')
                elif p == '/api/mouse':
                    q = parse_qs(u.query)
                    try:
                        node.dashboard_mouse(float(q.get('x', ['-1'])[0]), float(q.get('y', ['-1'])[0]))
                    except ValueError:
                        pass
                    self._empty(204)
                else:
                    self._empty(404)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            try:
                if urlparse(self.path).path != '/api/cmd':
                    self._empty(404)
                    return
                # ต้องมี header นี้: หน้าเว็บอื่นที่เปิดใน browser เดียวกันจะสั่งหุ่นแทนเราไม่ได้ (โดน CORS preflight กั้น)
                if self.headers.get('X-Dashboard') != '1':
                    self._json({'ok': False, 'msg': 'forbidden'}, 403)
                    return
                n = min(int(self.headers.get('Content-Length') or 0), 4096)
                try:
                    body = json.loads(self.rfile.read(n) or b'{}')
                except ValueError:
                    body = {}
                ok, msg, ticket = node.dashboard_command(body if isinstance(body, dict) else {})
                self._json({'ok': ok, 'msg': msg, 'ticket': ticket}, 200 if ok else 400)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _stream(self):
            """MJPEG ของภาพ \"RealSense D455 - RGB + Depth\" (ภาพเดียวกับหน้าต่างเดิม)"""
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            with node._jpeg_cond:
                node._stream_clients += 1
            last = -1
            try:
                while not node._dash_stop:
                    with node._jpeg_cond:
                        node._jpeg_cond.wait_for(lambda: node._jpeg_seq != last or node._dash_stop, timeout=1.0)
                        seq, jpg = node._jpeg_seq, node._jpeg
                    if jpg is None or seq == last:
                        continue
                    last = seq
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with node._jpeg_cond:
                    node._stream_clients -= 1

    return Handler


class DashboardServer:
    def __init__(self, node, host, port, plotly_path):
        self.httpd = ThreadingHTTPServer((host, port), make_dashboard_handler(node, plotly_path))
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, kwargs={'poll_interval': 0.3}, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass


DASHBOARD_HTML = r'''<!doctype html>
<html lang="th">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Project 271401 AI for Robotics</title>
<style>
:root {
  --paper: #F2F4FB; --panel: #FFFFFF; --ink: #2235C4; --ink-2: #8B98EA; --wash: #E5E9FB;
  --text: #141A38; --muted: #5B6488; --stage: #0F121A; --stage-text: #C9CFE6;
  --ok: #14804F; --warn: #B4530A; --err: #C2262E;
  color-scheme: light;
}
@media (prefers-color-scheme: dark) {
  :root {
    --paper: #0C1124; --panel: #121933; --ink: #93A6FF; --ink-2: #4A5AB4; --wash: #1B2347;
    --text: #E7EAF8; --muted: #9AA3C8; --stage: #05070D; --stage-text: #AEB6D6;
    --ok: #4ADE9B; --warn: #F5A65B; --err: #FF7A80;
    color-scheme: dark;
  }
}
* { box-sizing: border-box; }
[hidden] { display: none !important; }
html, body { height: 100%; margin: 0; }
body {
  background: var(--paper); color: var(--text);
  font-family: "IBM Plex Sans Thai", "Noto Sans Thai", "Sarabun", "Leelawadee UI", system-ui, sans-serif;
  font-size: 15px; line-height: 1.4;
  display: grid; grid-template-rows: auto minmax(0, 1fr); gap: 12px;
  padding: 14px 16px 16px; overflow: hidden;
}
button, select { font: inherit; }
:focus-visible { outline: 3px solid var(--ink); outline-offset: 2px; }

/* ---------- header ---------- */
.top { display: flex; align-items: baseline; gap: 18px; position: relative; min-width: 0; }
.top h1 { margin: 0; font-size: clamp(20px, 2.1vw, 32px); font-weight: 700; color: var(--ink); letter-spacing: -0.01em; white-space: nowrap; }
.top .byline { margin: 0; color: var(--muted); font-size: clamp(14px, 1.1vw, 18px); white-space: nowrap; }
.status {
  margin-left: auto; min-width: 0; flex: 0 1 46vw; display: flex; align-items: center; gap: 8px; justify-content: flex-end;
  background: none; border: 0; color: var(--muted); cursor: pointer; padding: 4px 6px; border-radius: 6px; font-size: 14px;
}
.status:hover { background: var(--wash); }
.status span:last-child { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.dot { width: 9px; height: 9px; border-radius: 50%; background: var(--muted); flex: none; }
.lv-info .dot { background: var(--ok); } .lv-warn .dot { background: var(--warn); } .lv-error .dot { background: var(--err); }
.lv-warn { color: var(--warn); } .lv-error { color: var(--err); }
.ghost { background: none; border: 2px solid var(--ink); color: var(--ink); border-radius: 6px; padding: 3px 12px; cursor: pointer; white-space: nowrap; }
.ghost:hover { background: var(--wash); }
.logpanel {
  position: absolute; right: 0; top: calc(100% + 6px); z-index: 20; width: min(720px, 70vw); max-height: 55vh; overflow: auto;
  background: var(--panel); border: 2px solid var(--ink); border-radius: 8px; padding: 6px 0; margin: 0; list-style: none;
}
.logpanel li { padding: 5px 14px 5px 12px; border-left: 3px solid transparent; font-size: 13.5px; display: grid; grid-template-columns: 64px 1fr; gap: 8px; }
.logpanel li time { color: var(--muted); font-variant-numeric: tabular-nums; }
.logpanel li.lv-warn { border-left-color: var(--warn); } .logpanel li.lv-error { border-left-color: var(--err); }

/* ---------- board ---------- */
.board {
  display: grid; min-height: 0; gap: 12px;
  grid-template-columns: minmax(0, 1fr) clamp(200px, 16vw, 300px);
  grid-template-rows: minmax(0, 1fr) clamp(178px, 27vh, 310px);
}
.stage {
  grid-column: 1; grid-row: 1; min-height: 0; display: flex; flex-direction: column;
  border: 2px solid var(--ink); border-radius: 8px; background: var(--stage); overflow: hidden;
}
.stage-cap { display: flex; align-items: center; gap: 12px; padding: 7px 14px; color: var(--stage-text); font-size: 14px; border-bottom: 1px solid rgba(255,255,255,.08); }
.stage-cap .hint { opacity: .75; font-size: 13px; white-space: nowrap; }
.busy { color: #FFD27A; display: flex; align-items: center; gap: 6px; }
.busy::before { content: ""; width: 8px; height: 8px; border-radius: 50%; background: currentColor; animation: pulse 1s ease-in-out infinite; }
@keyframes pulse { 50% { opacity: .25; } }
@media (prefers-reduced-motion: reduce) { .busy::before { animation: none; } }
.panes { position: relative; flex: 1; min-height: 0; }
.pane { position: absolute; inset: 0; }
.pane[hidden] { display: none; }
.pane img { width: 100%; height: 100%; object-fit: contain; display: block; }
.pane img.pixelated { image-rendering: pixelated; }
#camImg { cursor: crosshair; }
#plot3d { position: absolute; inset: 0; background: #fff; }
.empty { position: absolute; inset: 0; margin: 0; display: grid; place-items: center; text-align: center; padding: 24px; color: var(--stage-text); font-size: 17px; }
.empty[hidden] { display: none; }

/* ---------- view buttons (ขวามือ) ---------- */
.views { grid-column: 2; grid-row: 1 / span 2; display: flex; flex-direction: column; gap: 12px; min-height: 0; }
.vbtn {
  flex: 1; min-height: 0; display: flex; flex-direction: column; justify-content: center; gap: 4px; text-align: left;
  padding: 12px 16px; border: 2px solid var(--ink); border-radius: 8px; background: var(--panel); color: var(--ink); cursor: pointer;
}
.vbtn b { font-size: clamp(16px, 1.3vw, 21px); font-weight: 700; }
.vbtn small { color: var(--muted); font-size: 13px; }
.vbtn:hover { background: var(--wash); }
.vbtn[aria-pressed="true"] { background: var(--ink); color: var(--panel); }
.vbtn[aria-pressed="true"] small { color: var(--panel); opacity: .8; }

/* ---------- controls (ล่าง) ---------- */
.controls { grid-column: 1; grid-row: 2; min-height: 0; display: grid; gap: 12px;
  grid-template-columns: minmax(170px, .9fr) minmax(150px, .75fr) minmax(320px, 2.1fr); }
.pickers { display: flex; flex-direction: column; justify-content: space-evenly; gap: 6px; min-width: 0; }
.pickers label { font-weight: 700; color: var(--ink); font-size: 16px; }
.pickers select {
  width: 100%; height: 44px; padding: 0 10px; border: 2px solid var(--ink); border-radius: 6px;
  background: var(--panel); color: var(--text); font-size: 15px; cursor: pointer;
}
.actions { display: flex; flex-direction: column; gap: 10px; min-height: 0; }
.actions button {
  flex: 1; min-height: 0; border-radius: 8px; border: 2px solid var(--ink); font-size: clamp(18px, 1.6vw, 26px); font-weight: 700; cursor: pointer;
  display: flex; align-items: center; justify-content: center; gap: 10px;
}
.primary { background: var(--ink); color: var(--panel); }
.primary:hover:not(:disabled) { filter: brightness(1.12); }
.secondary { background: var(--panel); color: var(--ink); }
.secondary:hover:not(:disabled) { background: var(--wash); }
.actions button:disabled { opacity: .45; cursor: not-allowed; border-style: dashed; }
kbd { font-family: inherit; font-size: 12px; font-weight: 600; line-height: 1; padding: 3px 6px; border-radius: 4px; border: 1.5px solid currentColor; opacity: .75; }
.actions .hint { margin: 0; font-size: 13px; color: var(--muted); line-height: 1.3; min-height: 2.6em; }
.switch { display: flex; align-items: center; gap: 8px; font-weight: 600; color: var(--ink); cursor: pointer; font-size: 15px; }
.switch input { width: 18px; height: 18px; accent-color: var(--ink); margin: 0; cursor: pointer; }
.switch small { font-weight: 400; color: var(--muted); }
.scanstate { margin-left: auto; font-size: 13px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.scanstate.paused { color: #FFD27A; }
.pill { font-size: 13px; padding: 1px 9px; border-radius: 999px; border: 1px solid currentColor; white-space: nowrap; }
.pill.idle { color: #7FE0B0; } .pill.moving { color: #FFB86B; } .pill.unknown { color: #9AA3C8; }
.pill.stop { color: #fff; background: #D02A31; border-color: #D02A31; font-weight: 700; }
.pill.point { color: #7CFF9B; }

.result { min-width: 0; min-height: 0; display: flex; flex-direction: column; border: 2px solid var(--ink); border-radius: 8px; background: var(--panel); padding: 10px 16px 12px; }
.result h2 { margin: 0 0 6px; font-size: 16px; color: var(--ink); }
#resultBody { flex: 1; min-height: 0; display: flex; flex-direction: column; justify-content: center; }
.muted { color: var(--muted); margin: auto 0; }
.metrics { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; margin: 0 0 8px; }
.m-label { display: block; color: var(--muted); font-size: 13.5px; }
.m-val { font-size: clamp(26px, 2.7vw, 44px); font-weight: 700; font-variant-numeric: tabular-nums; line-height: 1.1; color: var(--text); }
.m-val small { font-size: .45em; font-weight: 600; color: var(--muted); margin-left: 4px; }
.m-foot { margin: 0; font-size: 13.5px; color: var(--muted); }
.m-algo { font-weight: 700; color: var(--ink); }
.fail { color: var(--err); font-weight: 600; margin: auto 0; }
.charts { flex: 1; min-height: 0; display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.charts svg { width: 100%; height: 100%; display: block; overflow: visible; }
.charts .t { font-size: 11px; fill: var(--muted); }
.charts .v { font-size: 11px; fill: var(--text); font-weight: 600; font-variant-numeric: tabular-nums; }
.charts .h { font-size: 12px; fill: var(--ink); font-weight: 700; }
.charts .bar { fill: var(--ink-2); }
.charts .bar.chosen { fill: var(--ink); }
.charts .axis { stroke: var(--muted); stroke-width: 1; }
.cap { margin: 4px 0 0; font-size: 13.5px; }

.offline {
  position: fixed; left: 50%; bottom: 20px; transform: translateX(-50%); z-index: 30;
  background: var(--err); color: #fff; padding: 10px 18px; border-radius: 8px; font-weight: 600;
}
.offline[hidden] { display: none; }
</style>
<script src="/plotly.js" defer></script>
</head>
<body>
<header class="top">
  <h1>Project 271401 AI for Robotics</h1>
  <p class="byline">by Nemo and Frame</p>
  <button id="statusBtn" class="status" aria-expanded="false" aria-controls="logPanel" title="ดูข้อความล่าสุดจากโหนด">
    <span class="dot"></span><span id="statusText">กำลังเชื่อมต่อกับโหนด…</span>
  </button>
  <button id="fsBtn" class="ghost" type="button">เต็มจอ</button>
  <ol id="logPanel" class="logpanel" hidden></ol>
</header>

<main class="board">
  <section class="stage" aria-label="จอแสดงผล">
    <div class="stage-cap">
      <span id="viewTitle">RealSense D455 - RGB + Depth</span>
      <span id="busyBadge" class="busy" hidden></span>
      <span id="scanState" class="scanstate"></span>
      <span id="robotPill" class="pill unknown" hidden></span>
      <span id="handPill" class="pill unknown" hidden></span>
      <span id="stageHint" class="hint">คลิกซ้าย = จุดวาง, คลิกขวา = default</span>
    </div>
    <div class="panes">
      <div class="pane" data-view="camera">
        <img id="camImg" alt="ภาพสดจาก RealSense D455 พร้อมค่า depth">
        <p class="empty" id="camEmpty">รอภาพจากกล้อง: ตรวจว่า launch realsense2_camera ด้วย align_depth.enable:=true แล้ว</p>
      </div>
      <div class="pane" data-view="map3d" hidden>
        <div id="plot3d"></div>
        <p class="empty" id="mapEmpty">ยังไม่มีแผนที่ 3 มิติ กด Scan เพื่อสแกนพื้นที่</p>
      </div>
      <div class="pane" data-view="depth" hidden>
        <img id="depthImg" alt="ภาพ depth ระบายสีตามความลึกจากการสแกนล่าสุด">
        <p class="empty" id="depthEmpty">ยังไม่มีภาพ depth กด Scan เพื่อแคปภาพ</p>
      </div>
      <div class="pane" data-view="seg" hidden>
        <img id="segImg" class="pixelated" alt="ผล segmentation ของวัตถุจากการสแกนล่าสุด">
        <p class="empty" id="segEmpty">ยังไม่มีผล segmentation กด Scan เพื่อแยกวัตถุ</p>
      </div>
    </div>
  </section>

  <nav class="views" aria-label="เลือกสิ่งที่แสดงบนจอ">
    <button class="vbtn" data-view="camera" aria-pressed="true"><b>Camera Viewer</b><small>ภาพสด RGB + Depth</small></button>
    <button class="vbtn" data-view="map3d" aria-pressed="false"><b>3D Voxel Map</b><small>แผนที่ความสูง 3 มิติ + เส้นทาง</small></button>
    <button class="vbtn" data-view="depth" aria-pressed="false"><b>Depth Image</b><small>Depth colormap ตอนสแกน</small></button>
    <button class="vbtn" data-view="seg" aria-pressed="false"><b>Segmentation</b><small>Solids ที่แยกได้ตอนสแกน</small></button>
  </nav>

  <section class="controls">
    <div class="pickers">
      <label for="algoSel">Choose Algorithm</label>
      <select id="algoSel">
        <option value="1">Dijkstra</option>
        <option value="2">A*</option>
        <option value="3">Bidirectional Dijkstra</option>
        <option value="4">Bidirectional A*</option>
        <option value="5">Comparison (เทียบทุก algorithm)</option>
      </select>
      <label for="modeSel">Choose Mode</label>
      <select id="modeSel">
        <option value="7">ไม่หมุน R กลับ (คงมุม tilt ตลอดทาง)</option>
        <option value="6">หมุน R กลับที่ตำแหน่งที่ 2</option>
      </select>
    </div>
    <div class="actions">
      <button id="startBtn" class="primary" type="button" disabled>Start <kbd>S</kbd></button>
      <button id="scanBtn" class="secondary" type="button">Scan <kbd>Space</kbd></button>
      <label class="switch" title="สแกน map อัตโนมัติ หยุดระหว่างหุ่นทำงาน (ปุ่ม D)">
        <input type="checkbox" id="dynChk"> Dynamic scan <small id="dynHz">1 fps</small>
      </label>
      <p id="actionHint" class="hint"></p>
    </div>
    <div class="result" aria-live="polite">
      <h2>Shortest Path &amp; Computational Time</h2>
      <div id="resultBody"><p class="muted">ยังไม่มีเส้นทาง กด Scan เพื่อสแกนและวางเส้นทาง</p></div>
    </div>
  </section>
</main>

<div id="offline" class="offline" hidden>ขาดการเชื่อมต่อกับโหนด: ตรวจว่า detect_edit_10.py ยังรันอยู่</div>

<script>
'use strict';
const $ = (s) => document.querySelector(s);
const VIEW_TITLES = {
  camera: 'RealSense D455 - RGB + Depth', map3d: '3D height map',
  depth: 'Depth colormap (captured)', seg: 'Solids (segmentation)'
};
const VIEW_HINTS = {
  camera: 'คลิกซ้าย = จุดวาง, คลิกขวา = default',
  map3d: 'ลากเมาส์ = หมุน, ล้อเมาส์ = ซูม', depth: '', seg: ''
};
const BUSY_TEXT = {
  scan: 'กำลังสแกนและวางเส้นทาง…', dynamic: 'กำลังเปลี่ยน dynamic scan…', algo: 'กำลังวางเส้นทางใหม่…', mode: 'กำลังวางเส้นทางใหม่…',
  goal: 'กำลังวางเส้นทางไปจุดวางใหม่…', goal_default: 'กำลังวางเส้นทางไปจุดวาง default…', start: 'กำลังส่งเส้นทางให้หุ่น…'
};
let view = 'camera', st = null, failCount = 0, online = true;
const shown = { depth: 0, seg: 0, map3d: 0 };
let map3dData = null, scanTicket = 0, startTicket = 0, flashUntil = 0, resultKey = '', logKey = -1;

function esc(s) { return String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c])); }
function fmt(v, d = 1) { return Number(v).toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d }); }

function setStatus(text, level) {
  const b = $('#statusBtn');
  b.className = 'status lv-' + (level || 'info');
  $('#statusText').textContent = text;
  b.title = text;
}
function flash(text, level) { setStatus(text, level); flashUntil = performance.now() + 4000; }

async function cmd(action, extra = {}) {
  try {
    const r = await fetch('/api/cmd', {
      method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Dashboard': '1' },
      body: JSON.stringify(Object.assign({ action }, extra))
    });
    const j = await r.json();
    if (!j.ok) flash(j.msg || 'คำสั่งไม่สำเร็จ', 'warn');
    return j.ok ? j.ticket : 0;
  } catch (e) {
    flash('ส่งคำสั่งไม่ได้: ขาดการเชื่อมต่อกับโหนด', 'error');
    return 0;
  }
}

/* ---------- เลือกสิ่งที่แสดงบนจอ ---------- */
const camImg = $('#camImg');
function startStream() { camImg.src = '/stream.mjpg?t=' + Date.now(); }
function stopStream() { camImg.removeAttribute('src'); }

function setView(v) {
  view = v;
  document.querySelectorAll('.vbtn').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === v)));
  document.querySelectorAll('.pane').forEach((p) => { p.hidden = p.dataset.view !== v; });
  $('#viewTitle').textContent = VIEW_TITLES[v];
  $('#stageHint').textContent = VIEW_HINTS[v];
  if (v === 'camera') startStream(); else stopStream();
  if (v === 'map3d') requestAnimationFrame(drawMap3d);
}
document.querySelectorAll('.vbtn').forEach((b) => b.addEventListener('click', () => { setView(b.dataset.view); b.blur(); }));

/* ---------- เมาส์บนภาพสด: กากบาท + ค่าระยะ, คลิกตั้งจุดวาง ---------- */
function imgCoords(img, ev) {
  const r = img.getBoundingClientRect(), nw = img.naturalWidth, nh = img.naturalHeight;
  if (!nw || !nh) return null;
  const s = Math.min(r.width / nw, r.height / nh);
  const ox = r.left + (r.width - nw * s) / 2, oy = r.top + (r.height - nh * s) / 2;
  const x = (ev.clientX - ox) / s, y = (ev.clientY - oy) / s;
  if (x < 0 || y < 0 || x >= nw || y >= nh) return null;
  return { x, y };
}
let moveQ = null, moveTimer = null;
function queueMouse(x, y) {
  moveQ = [Math.round(x), Math.round(y)];
  if (!moveTimer) moveTimer = setTimeout(() => {
    moveTimer = null;
    fetch(`/api/mouse?x=${moveQ[0]}&y=${moveQ[1]}`).catch(() => {});
  }, 40);
}
camImg.addEventListener('mousemove', (e) => { const c = imgCoords(camImg, e); if (c) queueMouse(c.x, c.y); else queueMouse(-1, -1); });
camImg.addEventListener('mouseleave', () => queueMouse(-1, -1));
camImg.addEventListener('click', (e) => { const c = imgCoords(camImg, e); if (c) cmd('goal', { x: c.x, y: c.y }); });
camImg.addEventListener('contextmenu', (e) => { e.preventDefault(); cmd('goal_default'); });

/* ---------- ปุ่มควบคุม ---------- */
async function doScan() {
  if ($('#scanBtn').disabled) return;
  $('#scanBtn').disabled = true;
  scanTicket = await cmd('scan');
  if (!scanTicket) $('#scanBtn').disabled = false;
}
async function doStart() {
  if ($('#startBtn').disabled) { if (st) flash(st.start.reason || 'ยังส่งเส้นทางไม่ได้', 'warn'); return; }
  $('#startBtn').disabled = true;
  startTicket = await cmd('start');
}
$('#scanBtn').addEventListener('click', (e) => { e.currentTarget.blur(); doScan(); });
$('#startBtn').addEventListener('click', (e) => { e.currentTarget.blur(); doStart(); });
$('#algoSel').addEventListener('change', (e) => { cmd('algo', { value: Number(e.target.value) }); e.target.blur(); });
$('#dynChk').addEventListener('change', (e) => { cmd('dynamic', { value: e.target.checked }); e.target.blur(); });
$('#modeSel').addEventListener('change', (e) => { cmd('mode', { value: Number(e.target.value) }); e.target.blur(); });

/* ---------- ปุ่มลัดคีย์บอร์ดเดิม ---------- */
document.addEventListener('keydown', (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey || e.repeat) return;
  const tag = e.target.tagName;
  if (tag === 'SELECT' && [' ', 'Enter', 'ArrowUp', 'ArrowDown'].includes(e.key)) return;
  const k = e.key;
  if (k === ' ') { e.preventDefault(); doScan(); }
  else if (k === 's' || k === 'S') doStart();
  else if (k >= '1' && k <= '5') { $('#algoSel').value = k; cmd('algo', { value: Number(k) }); }
  else if (k === '6' || k === '7') { $('#modeSel').value = k; cmd('mode', { value: Number(k) }); }
  else if (k === 'a' || k === 'A') cmd('goal_default');
  else if (k === 'd' || k === 'D') { const c = $('#dynChk'); c.checked = !c.checked; cmd('dynamic', { value: c.checked }); }
  else if (k === 'q' || k === 'Q') { if (confirm('ปิดโปรแกรม detect_edit_10.py ?')) cmd('quit'); }
  else return;
  if (tag === 'BUTTON') e.preventDefault();     // กัน space/enter ไปกดปุ่มที่ focus อยู่ซ้ำ
});
document.addEventListener('keyup', (e) => { if (e.key === ' ' && e.target.tagName === 'BUTTON') e.preventDefault(); });

/* ---------- header: log, เต็มจอ ---------- */
$('#statusBtn').addEventListener('click', () => {
  const p = $('#logPanel'); p.hidden = !p.hidden;
  $('#statusBtn').setAttribute('aria-expanded', String(!p.hidden));
});
document.addEventListener('click', (e) => {
  if (!e.target.closest('#logPanel') && !e.target.closest('#statusBtn')) { $('#logPanel').hidden = true; $('#statusBtn').setAttribute('aria-expanded', 'false'); }
});
$('#fsBtn').addEventListener('click', () => {
  if (document.fullscreenElement) document.exitFullscreen(); else document.documentElement.requestFullscreen().catch(() => flash('browser ไม่ยอมให้เต็มจอ ลองกด F11', 'warn'));
});
document.addEventListener('fullscreenchange', () => { $('#fsBtn').textContent = document.fullscreenElement ? 'ออกจากเต็มจอ' : 'เต็มจอ'; });

/* ---------- แผนที่ 3 มิติ (Plotly) ---------- */
let dragging3d = false, pending3d = false;
$('#plot3d').addEventListener('pointerdown', () => { dragging3d = true; });
window.addEventListener('pointerup', () => { dragging3d = false; if (pending3d) { pending3d = false; drawMap3d(); } });
function drawMap3d() {
  const div = $('#plot3d'), empty = $('#mapEmpty');
  if (dragging3d) { pending3d = true; return; }
  if (!map3dData) { empty.hidden = false; return; }
  if (typeof Plotly === 'undefined') {
    empty.textContent = 'โหลด Plotly ไม่ได้: ต่ออินเทอร์เน็ต หรือติดตั้งในเครื่องด้วย pip install plotly แล้วรันโหนดใหม่';
    empty.hidden = false; return;
  }
  empty.hidden = true;
  const d = map3dData, tr = [];
  tr.push({
    type: 'surface', x: d.surface.x, y: d.surface.y, z: d.surface.z, surfacecolor: d.surface.z,
    colorscale: d.colorscale, cmin: 0, cmax: d.surface.cmax,
    colorbar: { title: { text: 'Height above floor (mm)', side: 'right' }, len: 0.6, thickness: 14 },
    lighting: { ambient: 0.7, diffuse: 0.55, specular: 0.05, roughness: 0.9 },
    lightposition: { x: -10000, y: 10000, z: 20000 },
    hovertemplate: 'X %{x:.0f} mm<br>Y %{y:.0f} mm<br>สูง %{z:.1f} mm<extra></extra>'
  });
  for (const L of d.lines) tr.push({ type: 'scatter3d', mode: 'lines', x: L.x, y: L.y, z: L.z, line: { color: L.color, width: L.width, dash: L.dash }, hoverinfo: 'skip' });
  for (const Zl of d.zones) tr.push({ type: 'scatter3d', mode: 'lines', x: Zl.x, y: Zl.y, z: Zl.z, line: { color: 'rgb(255,0,0)', width: 4 }, hoverinfo: 'skip' });
  if (d.labels.length) tr.push({
    type: 'scatter3d', mode: 'text', x: d.labels.map((l) => l[0]), y: d.labels.map((l) => l[1]), z: d.labels.map((l) => l[2]),
    text: d.labels.map((l) => l[3]), textfont: { size: 12, color: '#111' }, hoverinfo: 'skip'
  });
  if (d.path) {
    const p = d.path, n = p.x.length;
    tr.push({ type: 'scatter3d', mode: 'lines', x: p.x, y: p.y, z: p.x.map(() => 0), line: { color: 'rgb(90,90,90)', width: 3, dash: 'dash' }, hoverinfo: 'skip' });
    tr.push({
      type: 'scatter3d', mode: 'lines+markers', x: p.x, y: p.y, z: p.z,
      line: { color: 'rgb(255,0,255)', width: 7 }, marker: { size: 4, color: 'rgb(255,0,255)' },
      hovertemplate: 'waypoint %{pointNumber}<br>X %{x:.0f} mm, Y %{y:.0f} mm<br>สูง %{z:.0f} mm<extra></extra>'
    });
    tr.push({
      type: 'scatter3d', mode: 'text', x: [p.x[0], p.x[n - 1]], y: [p.y[0], p.y[n - 1]],
      z: [p.z[0] + p.label_dz, p.z[n - 1] + p.label_dz], text: ['pick', 'place'], textfont: { size: 13, color: '#000' }, hoverinfo: 'skip'
    });
  }
  const layout = {
    title: { text: d.title.replace(/\n/g, '<br>'), font: { size: 14 } },
    margin: { l: 0, r: 0, t: 52, b: 0 }, paper_bgcolor: '#ffffff', font: { color: '#1a1f36' },
    showlegend: false, uirevision: 'map3d',
    scene: {
      xaxis: { title: { text: 'X (mm, image right)' }, range: d.ranges.x },
      yaxis: { title: { text: 'Y (mm, image up)' }, range: d.ranges.y },
      zaxis: { title: { text: 'Height (mm)' }, range: d.ranges.z, nticks: 4 },
      aspectmode: 'manual', aspectratio: d.aspect,
      camera: { eye: { x: 0, y: -1.02, z: 0.86 }, up: { x: 0, y: 0, z: 1 } }, domain: { x: [0, 1], y: [0, 1] }
    }
  };
  Plotly.react(div, tr, layout, { responsive: true, displaylogo: false });
  requestAnimationFrame(() => { if (view === 'map3d') Plotly.Plots.resize(div); });
}
window.addEventListener('resize', () => { if (view === 'map3d' && typeof Plotly !== 'undefined' && map3dData) Plotly.Plots.resize($('#plot3d')); });
async function loadMap3d(v) {
  try {
    const r = await fetch('/api/map3d?v=' + v, { cache: 'no-store' });
    if (!r.ok) return;
    map3dData = await r.json();
    shown.map3d = v;
    if (view === 'map3d') drawMap3d();
  } catch (e) { /* ลองใหม่รอบหน้า */ }
}

/* ---------- Shortest Path & Computational Time ---------- */
function barChart(title, unit, items, digits, emptyText = 'no path') {
  const W = 300, H = 170, top = 34, base = 132, n = items.length;
  const max = Math.max(1e-9, ...items.map((i) => (i.ok ? i.value : 0)));
  const slot = (W - 20) / n, bw = Math.min(46, slot * 0.6);
  let s = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="${esc(title)}"><text class="h" x="0" y="13">${esc(title)}</text>`;
  s += `<line class="axis" x1="0" x2="${W}" y1="${base}" y2="${base}"/>`;
  items.forEach((it, i) => {
    const cx = 10 + slot * i + slot / 2, h = it.ok ? Math.max(2, (it.value / max) * (base - top)) : 0;
    s += `<g><title>${esc(it.tip)}</title>`;
    if (it.ok) {
      s += `<rect class="bar${it.chosen ? ' chosen' : ''}" x="${cx - bw / 2}" y="${base - h}" width="${bw}" height="${h}" rx="2"/>`;
      s += `<text class="v" x="${cx}" y="${base - h - 5}" text-anchor="middle">${fmt(it.value, digits)}</text>`;
    } else {
      s += `<text class="v" x="${cx}" y="${base - 6}" text-anchor="middle">${esc(emptyText)}</text>`;
    }
    s += `<text class="t" x="${cx}" y="${base + 16}" text-anchor="middle">${esc(it.label)}</text></g>`;
  });
  return s + `<text class="t" x="${W}" y="13" text-anchor="end">${esc(unit)}</text></svg>`;
}

function renderResult(s) {
  const key = JSON.stringify([s.runs, s.path_msg, s.path && s.path.goal_src]);
  if (key === resultKey) return;
  resultKey = key;
  const el = $('#resultBody'), R = s.runs;
  if (!R) {
    el.innerHTML = s.path_msg ? `<p class="fail">วางเส้นทางไม่ได้: ${esc(s.path_msg)}</p>`
                              : '<p class="muted">ยังไม่มีเส้นทาง กด Scan เพื่อสแกนและวางเส้นทาง</p>';
    return;
  }
  if (!R.compare) {
    const a = R.algos[0];
    if (!a.ok) {
      el.innerHTML = `<p class="fail"><span class="m-algo">${esc(a.label)}</span> หาเส้นทางไม่ได้ (${esc(a.reason)})</p>`
        + `<p class="m-foot">ย้ายสิ่งกีดขวาง เลือกจุดวางใหม่ หรือกด A ใช้จุดวาง default</p>`;
      return;
    }
    el.innerHTML = `<div class="metrics">
        <div><span class="m-label">ระยะทางที่สั้นที่สุด (<span class="m-algo">${esc(a.label)}</span>)</span>
             <span class="m-val">${fmt(a.final_len_mm)}<small>mm</small></span></div>
        <div><span class="m-label">Computational time (ค้นหา)</span>
             <span class="m-val">${fmt(a.search_ms, a.search_ms < 100 ? 2 : 1)}<small>ms</small></span></div>
      </div>
      <p class="m-foot">${a.n_wp} waypoint, ขยาย ${a.expanded.toLocaleString('en-US')} node, ระยะบน grid ก่อน smooth ${fmt(a.raw_len_mm)} mm, รวมทั้งขั้นตอน ${fmt(a.total_ms, 0)} ms</p>`;
    return;
  }
  const items = (f, d) => R.algos.map((a) => ({
    label: a.label, ok: a.ok, value: a[f], chosen: a.key === R.chosen && a.ok,
    tip: a.ok ? `${a.label}: ${fmt(a.final_len_mm)} mm, ค้นหา ${fmt(a.search_ms, 2)} ms, ขยาย ${a.expanded} node, ${a.n_wp} waypoint`
              : `${a.label}: หาเส้นทางไม่ได้ (${a.reason})`
  }));
  const timeItems = R.algos.map((a) => ({        // ไม่ได้เริ่มค้นหา (จุดหยิบ/จุดวางติดสิ่งกีดขวาง) = ไม่มีเวลา
    label: a.label, ok: a.search_ms > 0, value: a.search_ms, chosen: a.key === R.chosen && a.ok,
    tip: a.search_ms > 0 ? `${a.label}: ค้นหา ${fmt(a.search_ms, 2)} ms, ขยาย ${a.expanded} node`
                         : `${a.label}: ไม่ได้ค้นหา (${a.reason})`
  }));
  const best = R.algos.find((a) => a.key === R.chosen && a.ok);
  el.innerHTML = `<div class="charts">${barChart('Computational time', 'ms', timeItems, 1, '–')}${barChart('Distance (final 3D path)', 'mm', items('final_len_mm'), 1)}</div>`
    + (best ? `<p class="cap">เส้นทางที่ใช้ส่งหุ่น = สั้นที่สุด: <span class="m-algo">${esc(best.label)}</span> ${fmt(best.final_len_mm)} mm (ค้นหา ${fmt(best.search_ms, 2)} ms)</p>`
            : '<p class="cap fail">ไม่มี algorithm ไหนหาเส้นทางได้</p>');
}

/* ---------- อ่านสถานะจากโหนด ---------- */
function apply(s) {
  $('#camEmpty').hidden = !!s.frames;
  if (document.activeElement !== $('#algoSel')) $('#algoSel').value = String(s.algo);
  if (document.activeElement !== $('#modeSel')) $('#modeSel').value = s.r_return ? '6' : '7';

  const scanPending = scanTicket && s.cmd_done < scanTicket;
  if (!scanPending) scanTicket = 0;
  $('#scanBtn').disabled = !!scanPending || s.busy === 'scan' || s.robot.busy;
  const startPending = startTicket && (s.cmd_done < startTicket || s.modbus.busy);
  if (!startPending) startTicket = 0;
  $('#startBtn').disabled = !s.start.ready || !!startPending;
  $('#startBtn').title = s.start.ready ? 'ส่งเส้นทางให้หุ่นทาง Modbus' : s.start.reason;
  let hint = s.start.ready ? `พร้อมส่ง ${s.path.n_wp} waypoint ไป ${s.modbus.target}` : s.start.reason;
  if (s.path && s.path.ok && s.path.r_delta) hint += `, หมุน R ${s.path.r_delta > 0 ? '+' : ''}${fmt(s.path.r_delta)}°`;
  $('#actionHint').textContent = hint;

  const D = s.dynamic, RB = s.robot;
  if (document.activeElement !== $('#dynChk')) $('#dynChk').checked = !!D.enabled;
  $('#dynHz').textContent = `${fmt(D.hz, D.hz < 1 ? 1 : 0)} fps`;
  const ss = $('#scanState');
  ss.className = 'scanstate' + (D.paused ? ' paused' : '');
  if (!D.enabled) ss.textContent = 'Dynamic scan ปิดอยู่';
  else if (D.paused) ss.textContent = `หยุด scan map ชั่วคราว: ${D.reason}`;
  else if (D.last) ss.textContent = `Dynamic scan ${D.rate ? fmt(D.rate, 1) + ' fps, ' : ''}อัปเดต ${D.last.t} (${D.last.ms} ms)`;
  else ss.textContent = 'Dynamic scan รอภาพแรก…';
  const rp = $('#robotPill');
  rp.hidden = !RB.monitor;
  if (RB.monitor) {
    const moving = RB.busy || RB.status === 1 || RB.status === 2;
    rp.className = 'pill ' + (RB.status === 2 ? 'stop' : !RB.conn && !moving ? 'unknown' : moving ? 'moving' : 'idle');
    rp.textContent = RB.status === 2 ? 'หุ่น: หยุดรอ (พบมือ)' : moving ? 'หุ่น: กำลังทำงาน'
                   : RB.conn ? 'หุ่น: ว่าง' : 'หุ่น: อ่านสถานะไม่ได้';
  }
  const HD = s.hand, hp = $('#handPill');
  hp.hidden = !HD.enabled;
  if (HD.hold) { hp.className = 'pill stop'; hp.textContent = `หยุดหุ่น: ${HD.reason}`; }
  else if (!HD.ok) { hp.className = 'pill unknown'; hp.textContent = HD.err ? 'Hand guard ใช้ไม่ได้' : 'Hand guard กำลังเริ่ม…'; hp.title = HD.err; }
  else if (HD.pointer) { hp.className = 'pill point'; hp.textContent = `นิ้วชี้: ค้างไว้เลือกจุดวาง ${Math.round(HD.dwell * 100)}%`; }
  else if (HD.present) { hp.className = 'pill moving'; hp.textContent = `พบมือ ${HD.count || ''} ข้าง`.replace('  ', ' '); }
  else { hp.className = 'pill idle'; hp.textContent = `มือ: ไม่พบ (${fmt(HD.rate || HD.hz, 0)} ครั้ง/วิ)`; }
  $('#scanBtn').title = (s.robot.busy ? 'สแกนได้เมื่อหุ่นทำงานเสร็จ' : 'สแกน 1 ครั้ง + เซฟภาพลง dataset');

  const busy = s.busy || (s.queued ? 'queue' : '');
  $('#busyBadge').hidden = !busy;
  $('#busyBadge').textContent = BUSY_TEXT[busy] || 'รอคิวคำสั่ง…';

  for (const k of ['depth', 'seg']) {
    const v = s.versions[k];
    $('#' + k + 'Empty').hidden = v > 0 && s.has_img[k];
    if (v !== shown[k]) {
      shown[k] = v;
      if (s.has_img[k]) $('#' + k + 'Img').src = `/img/${k}.png?v=${v}`; else $('#' + k + 'Img').removeAttribute('src');
    }
  }
  if (s.versions.map3d > 0 && s.versions.map3d !== shown.map3d) loadMap3d(s.versions.map3d);

  renderResult(s);

  const logs = s.log || [];
  const lastId = logs.length ? logs[logs.length - 1].id : 0;
  if (lastId !== logKey) {
    logKey = lastId;
    $('#logPanel').innerHTML = logs.slice().reverse().map((l) =>
      `<li class="lv-${l.level}"><time>${esc(l.t)}</time><span>${esc(l.msg)}</span></li>`).join('') || '<li><time></time><span>ยังไม่มีข้อความ</span></li>';
    if (logs.length && performance.now() > flashUntil) setStatus(logs[logs.length - 1].msg, logs[logs.length - 1].level);
  }
}

async function poll() {
  try {
    const r = await fetch('/api/state', { cache: 'no-store' });
    st = await r.json();
    failCount = 0;
    if (!online) { online = true; $('#offline').hidden = true; if (view === 'camera') startStream(); }
    apply(st);
  } catch (e) {
    failCount += 1;
    if (failCount >= 3 && online) { online = false; $('#offline').hidden = false; setStatus('ขาดการเชื่อมต่อกับโหนด', 'error'); }
  }
  setTimeout(poll, 400);
}

setView('camera');
poll();
</script>
</body>
</html>
'''


class RGBDViewerNode(Node):
    def __init__(self):
        super().__init__('rgbd_viewer_node')

        # ---- dashboard (หน้าเว็บ): สร้างก่อนอย่างอื่น เพราะห่อ logger ไว้เก็บข้อความไปแสดงบนหน้าเว็บ
        self._log = deque(maxlen=200)
        self._log_id = 0
        self._log_tap = _LogTap(super().get_logger(), self._push_log)
        self.dashboard = None
        self._dash_stop = False
        self._cmd_q = queue.Queue()          # คำสั่งจากหน้าเว็บ -> ทำใน thread ของ ROS (display_callback)
        self._cmd_lock = threading.Lock()
        self._cmd_seq = 0
        self._cmd_done = 0
        self._busy = ''
        self._jpeg = None                    # ภาพสดล่าสุด (JPEG) สำหรับ MJPEG stream
        self._jpeg_seq = 0
        self._jpeg_cond = threading.Condition()
        self._stream_clients = 0
        self._img = {'depth': None, 'seg': None}        # PNG ของ Depth colormap / Segmentation ตอนสแกนล่าสุด
        self._ver = {'depth': 0, 'seg': 0, 'map3d': 0}  # เปลี่ยนทุกครั้งที่มีภาพ/แผนที่ใหม่ (หน้าเว็บโหลดใหม่)
        self._map3d_json = None
        self.last_runs = None                # ผลของทุก algorithm ที่รันล่าสุด (ช่อง Shortest Path & Computational Time)
        self.path_msg = ''                   # สาเหตุที่วางเส้นทางไม่ได้ (ก่อนถึงขั้นค้นหา)
        self.modbus_last = None
        self.scan_count = 0
        for _n, _v in (('dashboard_host', '127.0.0.1'), ('dashboard_port', 8080), ('dashboard_open_browser', True),
                       ('dashboard_plotly_js', ''), ('dashboard_jpeg_quality', 85)):
            self.declare_parameter(_n, _v)
        self.dash_jpeg_quality = int(self.get_parameter('dashboard_jpeg_quality').value)

        # ---- dynamic scan map: สแกนอัตโนมัติทุก 1/dynamic_scan_hz วินาที ยกเว้นตอนหุ่นทำงาน
        for _n, _v in (('dynamic_scan', True), ('dynamic_scan_hz', 1.0), ('dynamic_save', False),
                       ('robot_status_reg', 10), ('robot_poll_hz', 4.0), ('robot_start_timeout_s', 5.0),
                       ('robot_max_wait_s', 180.0)):
            self.declare_parameter(_n, _v)
        _dp = lambda n: self.get_parameter(n).value
        self.dyn_enable = bool(_dp('dynamic_scan'))
        self.dyn_hz = max(float(_dp('dynamic_scan_hz')), 0.05)
        self.dyn_save = bool(_dp('dynamic_save'))
        self.robot_status_reg = int(_dp('robot_status_reg'))
        self.robot_poll_hz = max(float(_dp('robot_poll_hz')), 0.5)
        self.robot_start_timeout = float(_dp('robot_start_timeout_s'))
        self.robot_max_wait = float(_dp('robot_max_wait_s'))
        self.robot_status = None             # None = อ่านไม่ได้/ไม่มี Modbus, 0 = ว่าง, 1 = กำลังเคลื่อนที่
        self.robot_conn = False
        self._await_robot = None             # หลังกด Start: รอหุ่นเริ่ม (เห็น 1) แล้วเสร็จ (กลับเป็น 0)
        self._dyn_next = 0.0
        self._dyn_running = False
        self._dyn_last = None
        self._dyn_count = 0
        self._dyn_times = deque(maxlen=6)
        self._path_hold = None

        # ---- hand guard (MediaPipe Hands)
        for _n, _v in (('hand_enable', True), ('hand_hz', 5.0), ('hand_max_num', 2), ('hand_model_complexity', 1),
                       ('hand_min_detection', 0.6), ('hand_min_tracking', 0.5), ('hand_zone', 'frame'),
                       ('hand_clear_s', 1.0), ('hand_hold_reg', 12),
                       ('hand_pause_dynamic', True), ('pointer_enable', True), ('pointer_dwell_s', 1.5),
                       ('pointer_dwell_px', 8.0), ('hand_model_path', '')):
            self.declare_parameter(_n, _v)
        self.hand_enable = bool(_dp('hand_enable'))
        self.hand_hz = min(max(float(_dp('hand_hz')), 0.5), 30.0)
        self.hand_max_num = int(_dp('hand_max_num'))
        self.hand_model_complexity = int(_dp('hand_model_complexity'))
        self.hand_min_det = float(_dp('hand_min_detection'))
        self.hand_min_track = float(_dp('hand_min_tracking'))
        self.hand_zone = str(_dp('hand_zone')).lower()
        self.hand_clear_s = float(_dp('hand_clear_s'))
        self.hand_hold_reg = int(_dp('hand_hold_reg'))
        self.hand_pause_dynamic = bool(_dp('hand_pause_dynamic'))
        self.pointer_enable = bool(_dp('pointer_enable'))
        self.pointer_dwell_s = max(float(_dp('pointer_dwell_s')), 0.2)
        self.pointer_dwell_px = float(_dp('pointer_dwell_px'))
        self.hand_model_path = str(_dp('hand_model_path'))
        self.latest_full = None              # ภาพสีเต็ม (ก่อน crop) สำหรับตรวจมือ
        self.latest_full_off = (0, 0)        # มุมซ้ายบนของกรอบ crop บนภาพเต็ม
        self.latest_full_t = 0.0
        self.hand_ok = False
        self.hand_err = ''
        self.hands_view = []                 # มือที่เห็นรอบล่าสุด (พิกัดภาพ crop)
        self.hand_view_t = 0.0
        self._hand_last_seen = -1e9
        self._hand_times = deque(maxlen=8)
        self.hand_hold = False               # กำลังสั่งหุ่นหยุดเพราะพบมือ
        self.hand_hold_reason = ''
        self.finger_target = None            # ปลายนิ้วชี้ (พิกัดภาพ crop) ตอนชูนิ้วชี้นิ้วเดียว
        self.finger_cursor = None
        self._dwell = None
        self.dwell_progress = 0.0

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter('display_scale', 3.0)
        self.declare_parameter('display_fps', 30.0)
        self.declare_parameter('dataset_dir', 'dataset')
        # กล้องอยู่สูงจากพื้นประมาณ 60 cm (top view) -> พื้นอยู่ที่ ~600 mm
        self.declare_parameter('depth_near_mm', 500.0)
        self.declare_parameter('depth_far_mm', 650.0)
        self.declare_parameter('colormap', 'turbo')
        self.declare_parameter('cloud_topic', '/rgbd_viewer/points')
        self.declare_parameter(
            'info_topic', '/camera/camera/aligned_depth_to_color/camera_info'
        )
        self.declare_parameter('cloud_color', 'depth')  # 'depth' หรือ 'rgb'
        self.declare_parameter('cloud_stride', 1)
        self.declare_parameter('cloud_max_depth_m', 1.0)
        # กรอบ crop (pixel บนภาพต้นฉบับ) width/height = 0 -> ไปจนสุดขอบภาพ
        self.declare_parameter('crop_x', 487)
        self.declare_parameter('crop_y', 310)
        self.declare_parameter('crop_width', 300)
        self.declare_parameter('crop_height', 208)

        self.crop_x = self.get_parameter('crop_x').get_parameter_value().integer_value
        self.crop_y = self.get_parameter('crop_y').get_parameter_value().integer_value
        self.crop_w = self.get_parameter('crop_width').get_parameter_value().integer_value
        self.crop_h = self.get_parameter('crop_height').get_parameter_value().integer_value
        # แบ่งภาพ (หลัง crop) เป็น 3 โซน: ซ้าย = จุดวางวัตถุ | กลาง = พื้นที่ scan map | ขวา = จุดวางของ
        # ค่าเป็นสัดส่วนความกว้างของภาพที่ crop แล้ว (0..1) ; zone_enable=False = ปิดการแบ่ง
        self.declare_parameter('zone_enable', False)   # ไม่แบ่งโซน (เปิดโหมดเดิมด้วย zone_enable:=true)
        self.declare_parameter('zone_left_frac', 0.168)
        self.declare_parameter('zone_right_frac', 0.851)
        self.declare_parameter('zone_show_lines', True)
        self.zone_enable = self.get_parameter('zone_enable').get_parameter_value().bool_value
        self.zone_left_frac = self.get_parameter('zone_left_frac').get_parameter_value().double_value
        self.zone_right_frac = self.get_parameter('zone_right_frac').get_parameter_value().double_value
        self.zone_show_lines = self.get_parameter('zone_show_lines').get_parameter_value().bool_value
        self.crop_enabled = any(v > 0 for v in (self.crop_x, self.crop_y, self.crop_w, self.crop_h))
        # offset จริงของกรอบ (หลัง clamp) ใช้ชดเชย cx, cy ของ intrinsics
        self.crop_x0 = 0
        self.crop_y0 = 0
        self._crop_logged = False
        self._crop_mismatch_warned = False

        # ---- ตรวจจับวัตถุสีเหลืองด้วย HSV (Hue ของ OpenCV = 0-179)
        self.declare_parameter('yellow_enable', True)
        self.declare_parameter('yellow_h_low', 20)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('yellow_s_low', 100)
        self.declare_parameter('yellow_s_high', 255)
        self.declare_parameter('yellow_v_low', 120)
        self.declare_parameter('yellow_v_high', 255)
        self.declare_parameter('yellow_min_area_px', 300)
        self.declare_parameter('yellow_show_mask', False)
        _g = self.get_parameter
        self.yellow_enable = _g('yellow_enable').get_parameter_value().bool_value
        self.yellow_lo = (_g('yellow_h_low').get_parameter_value().integer_value,
                          _g('yellow_s_low').get_parameter_value().integer_value,
                          _g('yellow_v_low').get_parameter_value().integer_value)
        self.yellow_hi = (_g('yellow_h_high').get_parameter_value().integer_value,
                          _g('yellow_s_high').get_parameter_value().integer_value,
                          _g('yellow_v_high').get_parameter_value().integer_value)
        self.yellow_min_area = _g('yellow_min_area_px').get_parameter_value().integer_value
        self.yellow_show_mask = _g('yellow_show_mask').get_parameter_value().bool_value
        self.yellow_results = []

        # ---- A* 3 มิติ: วางเส้นทางหลังสแกน 3D map (กด spacebar) จากวัตถุสีเหลือง (โซนซ้าย) ไปจุดวาง (โซนขวา)
        self.declare_parameter('path_enable', True)
        self.declare_parameter('path_topic', '/rgbd_viewer/path')
        self.declare_parameter('path_cell_px', 5)                 # ขนาดช่อง voxel ในแนวนอน (pixel)
        self.declare_parameter('path_dz_mm', 6.0)                 # ความละเอียดแนวสูง (mm ต่อระดับ)
        self.declare_parameter('path_zmax_mm', 150.0)             # เพดานสูงสุดของ TCP เหนือพื้น (mm)
        self.declare_parameter('path_lift_mm', 30.0)              # ยกสูงเหนือยอดวัตถุก่อนเคลื่อนที่/ก่อนลง (mm)
        self.declare_parameter('path_margin_px', 4.0)             # ระยะปลอดภัยแนวนอนเพิ่มจากรัศมีวัตถุ (pixel)
        self.declare_parameter('path_z_margin_mm', 5.0)           # ระยะปลอดภัยแนวตั้ง (mm)
        self.declare_parameter('path_goal_x_frac', -1.0)          # จุดวาง (สัดส่วนกว้างภาพ crop) -1 = กลางโซนขวา
        self.declare_parameter('path_goal_y_frac', 0.5)           # จุดวาง (สัดส่วนสูงภาพ crop)
        self.declare_parameter('path_obstacle_source', 'solids')  # solids = solid ที่ขยายด้วยขนาดวัตถุ | raw = dilate height map แบบเดิม
        self.declare_parameter('path_inflate_mode', 'shape')      # shape = รูปร่างจริงของวัตถุ | box = กล่องขนาดวัตถุ | circle = วงกลมล้อมวัตถุ
        self.declare_parameter('path_inflate_extra_mm', 4.0)      # ระยะปลอดภัย + หัวจับที่ยื่นเกินวัตถุ (mm ต่อด้าน)
        self.declare_parameter('path_shape_mode', 'contour')      # รูปร่าง obstacle ที่ใช้วางเส้นทาง: contour = ตามขอบจริง | auto = snap เป็นสี่เหลี่ยม/วงกลม
        self.declare_parameter('path_poly_eps', 0.015)            # ความละเอียด polygon ของ obstacle (เล็ก = เกาะขอบแน่นขึ้น)
        self.declare_parameter('path_residual_raw', True)         # นับของที่ไม่ถูกทำเป็น solid จาก height map ด้วย
        self.declare_parameter('path_show_inflated', True)        # วาด solid ที่ขยายแล้ว (ส้มประ) ใน 3D map
        self.declare_parameter('map_exclude_yellow', 'all')       # all = ตัดวัตถุสีเหลืองทุกชิ้นออกจากแผนที่ | start = เฉพาะชิ้น START | off
        self.declare_parameter('map_yellow_margin_px', 4)         # ขยายขอบที่ตัดออก (pixel)
        self.path_enable = _g('path_enable').get_parameter_value().bool_value
        self.path_cell_px = _g('path_cell_px').get_parameter_value().integer_value
        self.path_dz_mm = _g('path_dz_mm').get_parameter_value().double_value
        self.path_zmax_mm = _g('path_zmax_mm').get_parameter_value().double_value
        self.path_lift_mm = _g('path_lift_mm').get_parameter_value().double_value
        self.path_margin_px = _g('path_margin_px').get_parameter_value().double_value
        self.path_z_margin_mm = _g('path_z_margin_mm').get_parameter_value().double_value
        self.path_goal_x_frac = _g('path_goal_x_frac').get_parameter_value().double_value
        self.path_goal_y_frac = _g('path_goal_y_frac').get_parameter_value().double_value
        self.path_obstacle_source = _g('path_obstacle_source').get_parameter_value().string_value.lower()
        self.path_inflate_mode = _g('path_inflate_mode').get_parameter_value().string_value.lower()
        self.path_inflate_extra_mm = _g('path_inflate_extra_mm').get_parameter_value().double_value
        self.path_residual_raw = _g('path_residual_raw').get_parameter_value().bool_value
        self.path_shape_mode = _g('path_shape_mode').get_parameter_value().string_value.lower()
        self.path_poly_eps = _g('path_poly_eps').get_parameter_value().double_value
        self.path_show_inflated = _g('path_show_inflated').get_parameter_value().bool_value
        self.map_exclude_yellow = _g('map_exclude_yellow').get_parameter_value().string_value.lower()
        self.map_yellow_margin_px = int(_g('map_yellow_margin_px').get_parameter_value().integer_value)
        # hand-eye calibration (ไฟล์จาก detect_edit_calibrate_1.py): p_robot = R @ p_cam + t (mm)
        self.declare_parameter('handeye_file', '')
        self.declare_parameter('handeye_csv', 'path_robot.csv')
        self.handeye_M = None
        self.handeye_csv = self.get_parameter('handeye_csv').get_parameter_value().string_value
        he_path = self.get_parameter('handeye_file').get_parameter_value().string_value
        if he_path:
            try:
                import json
                with open(he_path, encoding='utf-8') as f:
                    he = json.load(f)
                self.handeye_M = np.asarray(he['M'], np.float64).reshape(3, 4)
                self.get_logger().info(
                    f"โหลด hand-eye: {he_path} (model={he.get('model')}, RMSE {he.get('rmse_mm', float('nan')):.2f} mm)")
            except Exception as e:
                self.get_logger().error(f'โหลด hand-eye {he_path} ไม่สำเร็จ: {e}')
        # Modbus TCP: ส่ง waypoint พิกัดหุ่นไปยังฝั่งที่คุมหุ่น (กด 's')
        for _n, _v in (('modbus_enable', False), ('modbus_ip', '192.168.0.215'), ('modbus_port', 502),
                       ('modbus_unit', 1), ('modbus_cmd_reg', 0), ('modbus_run_cmd', 3),
                       ('modbus_data_reg', 20), ('modbus_scale', 10.0),
                       ('modbus_r_reg', 19), ('modbus_r_scale', 10.0),
                       ('modbus_rret_reg', 18),     # 1 = หมุน R กลับที่ตำแหน่งที่ 2 (ปุ่ม 6), 0 = ไม่หมุนกลับ (ปุ่ม 7) ; -1 = ไม่ส่ง
                       ('grip_r_enable', True), ('grip_r_sign', 1.0), ('grip_r_min_deg', 3.0), ('grip_r_max_deg', 45.0)):
            self.declare_parameter(_n, _v)
        _mp = lambda n: self.get_parameter(n).value
        self.modbus_enable = bool(_mp('modbus_enable'))
        self.modbus_ip, self.modbus_port, self.modbus_unit = str(_mp('modbus_ip')), int(_mp('modbus_port')), int(_mp('modbus_unit'))
        self.modbus_cmd_reg, self.modbus_run_cmd = int(_mp('modbus_cmd_reg')), int(_mp('modbus_run_cmd'))
        self.modbus_data_reg, self.modbus_scale = int(_mp('modbus_data_reg')), float(_mp('modbus_scale'))
        self.modbus_r_reg, self.modbus_r_scale = int(_mp('modbus_r_reg')), float(_mp('modbus_r_scale'))
        self.modbus_rret_reg = int(_mp('modbus_rret_reg'))
        self.grip_r_enable, self.grip_r_sign = bool(_mp('grip_r_enable')), float(_mp('grip_r_sign'))
        self.grip_r_min_deg, self.grip_r_max_deg = float(_mp('grip_r_min_deg')), float(_mp('grip_r_max_deg'))
        self._modbus_busy = False
        self._modbus_tid = 0
        self.path_result = None   # ผลวางเส้นทางล่าสุด (วาดซ้อนบนภาพสดจนกว่าจะสแกนใหม่)
        self.path_plot = None     # จุดของเส้นทางในพิกัดแผนที่ 3 มิติ (X, Y, สูง) ให้ matplotlib
        self.path_mode = PATH_DEFAULT_MODE   # 1 Dijkstra | 2 A* (ค่าเริ่มต้น) | 3 Bi-Dijkstra | 4 Bi-A* | 5 เทียบทั้ง 4 แบบ
        self.compare_fig = None              # หน้าต่างกราฟเทียบ (mode 5)
        self.r_return = False                # True = ปุ่ม 6 หมุน R กลับที่ตำแหน่งที่ 2 | False = ปุ่ม 7 / ค่าเริ่มต้น ไม่หมุนกลับ

        # depth flat-field calibration (ไฟล์จาก Depth_flat_calibrate.py)
        self.declare_parameter('depth_flat_file', '')
        self.flat = None
        self._flat_warned = False
        flat_path = self.get_parameter('depth_flat_file').get_parameter_value().string_value
        if flat_path:
            try:
                self.flat = load_flat_calibration(flat_path)
                self.get_logger().info(f'โหลด depth flat calibration: {flat_path} (crop {self.flat["crop"]})')
            except Exception as e:
                self.get_logger().error(f'โหลด {flat_path} ไม่สำเร็จ: {e}')

        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.display_scale = self.get_parameter('display_scale').get_parameter_value().double_value
        display_fps = self.get_parameter('display_fps').get_parameter_value().double_value

        self.depth_near_mm = self.get_parameter('depth_near_mm').get_parameter_value().double_value
        self.depth_far_mm = self.get_parameter('depth_far_mm').get_parameter_value().double_value
        cmap_name = self.get_parameter('colormap').get_parameter_value().string_value.lower()
        if cmap_name not in _COLORMAPS:
            self.get_logger().warn(
                f'ไม่รู้จัก colormap "{cmap_name}" ใช้ turbo แทน (เลือกได้: {", ".join(_COLORMAPS)})'
            )
            cmap_name = 'turbo'
        self.colormap = _COLORMAPS[cmap_name]
        self.depth_window_name = 'Depth colormap (captured)'
        self.depth_window_open = False

        self.cloud_color = self.get_parameter('cloud_color').get_parameter_value().string_value.lower()
        self.cloud_stride = self.get_parameter('cloud_stride').get_parameter_value().integer_value
        self.cloud_max_depth_m = self.get_parameter('cloud_max_depth_m').get_parameter_value().double_value
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('info_topic').get_parameter_value().string_value

        # latch (transient local) เพื่อให้ RViz ที่เปิดทีหลังยังได้รับ cloud ที่แคปล่าสุด
        cloud_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_topic, cloud_qos)

        # grid map (OccupancyGrid) มุมมองจากด้านบน ใช้ QoS แบบ latch เหมือน point cloud
        self.declare_parameter('grid_topic', '/rgbd_viewer/grid')
        self.declare_parameter('grid_resolution', 0.005)  # เมตรต่อช่อง
        grid_topic = self.get_parameter('grid_topic').get_parameter_value().string_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.grid_pub = self.create_publisher(OccupancyGrid, grid_topic, cloud_qos)
        self.get_logger().info(f'Grid topic: {grid_topic} (resolution {self.grid_resolution} m)')
        self.last_grid = None

        # ---- solid (extrude boss) : แยกวัตถุ -> fit รูปทรง -> extrude ลงพื้น -> publish MarkerArray
        self.declare_parameter('solid_enable', True)
        self.declare_parameter('solid_topic', '/rgbd_viewer/solids')
        self.declare_parameter('solid_floor_mm', 0.0)           # 0 = หาระนาบพื้นอัตโนมัติ
        self.declare_parameter('solid_min_height_mm', 12.0)     # ต่ำกว่านี้ถือว่าเป็นพื้น
        self.declare_parameter('solid_min_area_px', 300)
        self.declare_parameter('solid_min_prominence_mm', 4.0)  # ยอดต้องสูงกว่าจุดเชื่อมอย่างน้อยเท่านี้ถึงจะแยกเป็นคนละชิ้น
        self.declare_parameter('solid_split_step_mm', 1.5)
        self.declare_parameter('solid_edge_level', 0.6)         # ตัดขอบที่กี่ % ของความสูงยอด (0.5-0.8)
        self.declare_parameter('solid_shape_mode', 'auto')      # auto = snap เป็น สี่เหลี่ยม/วงกลม/สามเหลี่ยม | contour
        self.declare_parameter('solid_poly_eps', 0.03)          # ความละเอียดของ polygon (สัดส่วนของเส้นรอบรูป)
        self.declare_parameter('solid_rect_support', 0.7)       # ความทนต่อส่วนยื่น/รอยต่อตอน fit สี่เหลี่ยม
        self.declare_parameter('solid_median_ksize', 5)
        self.declare_parameter('solid_bilateral_sigma_mm', 6.0)
        self.declare_parameter('solid_close_px', 5)
        self.declare_parameter('solid_open_px', 3)
        self.declare_parameter('solid_show_floor', True)
        self.declare_parameter('solid_alpha', 1.0)
        self.declare_parameter('solid_show_labels', True)
        self.declare_parameter('solid_label_size_m', 0.012)   # ความสูงตัวอักษร (เมตร)
        gp = self.get_parameter
        self.solid_enable = gp('solid_enable').get_parameter_value().bool_value
        solid_topic = gp('solid_topic').get_parameter_value().string_value
        self.solid_show_floor = gp('solid_show_floor').get_parameter_value().bool_value
        self.solid_alpha = gp('solid_alpha').get_parameter_value().double_value
        self.solid_show_labels = gp('solid_show_labels').get_parameter_value().bool_value
        self.solid_label_size = gp('solid_label_size_m').get_parameter_value().double_value
        self.solid_params = {
            'floor_mm': gp('solid_floor_mm').get_parameter_value().double_value,
            'min_height_mm': gp('solid_min_height_mm').get_parameter_value().double_value,
            'min_area_px': gp('solid_min_area_px').get_parameter_value().integer_value,
            'min_prominence_mm': gp('solid_min_prominence_mm').get_parameter_value().double_value,
            'split_step_mm': gp('solid_split_step_mm').get_parameter_value().double_value,
            'edge_level': gp('solid_edge_level').get_parameter_value().double_value,
            'shape_mode': gp('solid_shape_mode').get_parameter_value().string_value.lower(),
            'poly_eps': gp('solid_poly_eps').get_parameter_value().double_value,
            'rect_support': gp('solid_rect_support').get_parameter_value().double_value,
            'median_ksize': gp('solid_median_ksize').get_parameter_value().integer_value,
            'bilateral_sigma_mm': gp('solid_bilateral_sigma_mm').get_parameter_value().double_value,
            'close_px': gp('solid_close_px').get_parameter_value().integer_value,
            'open_px': gp('solid_open_px').get_parameter_value().integer_value,
        }
        self.solid_pub = self.create_publisher(MarkerArray, solid_topic, cloud_qos)
        self.path_pub = self.create_publisher(
            MarkerArray, gp('path_topic').get_parameter_value().string_value, cloud_qos)
        self.solid_window_name = 'Solids (segmentation)'
        self.solid_window_open = False
        self.get_logger().info(f'Solid topic: {solid_topic} (enable={self.solid_enable})')
        self.last_solids = []   # solid ล่าสุดที่ build_solids() คำนวณได้ (ใช้วาดซ้อนในแผนที่ 3 มิติ)

        # ---- แผนที่ 3 มิติด้วย matplotlib (อัปเดตทุกครั้งที่กด spacebar)
        self.declare_parameter('plot3d_enable', True)
        self.declare_parameter('plot3d_backend', 'TkAgg')       # '' = ใช้ค่าเริ่มต้นของ matplotlib
        self.declare_parameter('plot3d_stride', 3)              # ข้าม pixel ทุก N จุด (เล็ก = ละเอียดแต่ช้า)
        self.declare_parameter('plot3d_cmap', 'turbo')
        self.plot_backend = gp('plot3d_backend').get_parameter_value().string_value
        self.declare_parameter('plot3d_z_aspect', 0.35)         # สัดส่วนความสูงแกน Z ต่อแกนที่ยาวสุด (0 = สเกลจริง)
        self.declare_parameter('plot3d_show_solids', True)
        self.declare_parameter('plot3d_full', True)             # True = แสดงทั้งภาพที่ crop (เห็นเส้นทางครบ) | False = เฉพาะโซน scan
        self.plot3d_show_solids = gp('plot3d_show_solids').get_parameter_value().bool_value
        self.plot3d_full = gp('plot3d_full').get_parameter_value().bool_value
        self.map3d = None
        if gp('plot3d_enable').get_parameter_value().bool_value:
            self.map3d = Map3DData(
                stride=gp('plot3d_stride').get_parameter_value().integer_value,
                cmap=gp('plot3d_cmap').get_parameter_value().string_value,
                z_aspect=gp('plot3d_z_aspect').get_parameter_value().double_value,
            )
            self.get_logger().info('แผนที่ 3 มิติจะแสดงที่ปุ่ม 3D Voxel Map บน dashboard เมื่อกด Scan')

        # ---- เฟรม "topview": z ชี้ขึ้น, พื้นอยู่ที่ z=0, x ขวา, y ขึ้น (เหมือนมองภาพจากด้านบน)
        # แก้ปัญหา RViz แสดงกลับหัว เพราะ optical frame มี z ชี้ออกจากกล้อง ไม่ใช่ชี้ขึ้น
        self.declare_parameter('topview_enable', True)
        self.declare_parameter('topview_frame', 'topview')
        self.declare_parameter('topview_parent', 'camera_color_optical_frame')
        # หมุนเฟรมรอบแกน z: 90 = ภาพใน RViz (มุมมองเริ่มต้น) ตรงกับภาพจากกล้อง (ขวา=ขวา, บน=บน)
        self.declare_parameter('topview_yaw_deg', 90.0)
        self.topview_enable = gp('topview_enable').get_parameter_value().bool_value
        self.topview_frame = gp('topview_frame').get_parameter_value().string_value
        self.topview_parent = gp('topview_parent').get_parameter_value().string_value
        self.topview_yaw_deg = gp('topview_yaw_deg').get_parameter_value().double_value
        self.tf_static = StaticTransformBroadcaster(self)
        if self.topview_enable:
            # ค่าเริ่มต้นก่อนแคปครั้งแรก: ใช้ depth_far_mm เป็นระยะพื้น แล้วอัปเดตทุกครั้งที่กด spacebar
            self.publish_topview_tf(self.depth_far_mm / 1000.0, self.topview_parent)
        # publish grid ล่าสุดซ้ำทุก 1 วินาที เผื่อ RViz เพิ่ง subscribe หรือพลาดข้อความแรก
        self.grid_timer = self.create_timer(1.0, self.republish_grid)
        self.intrinsics = None  # (fx, fy, cx, cy) จาก camera_info
        self.latest_header = None
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            ),
        )
        self.get_logger().info(f'Cloud topic: {cloud_topic} | Camera info: {info_topic}')

        self.dataset_dir = os.path.abspath(
            self.get_parameter('dataset_dir').get_parameter_value().string_value
        )
        os.makedirs(self.dataset_dir, exist_ok=True)
        self.save_msg = ''
        self.save_msg_until = 0.0  # เวลา (monotonic) ที่จะเลิกแสดงข้อความ "saved"

        # เก็บเฟรมล่าสุดที่ sync กันแล้ว (color + depth คู่กัน)
        self.latest_color = None
        self.latest_depth = None  # ยังเป็นความละเอียดเต็ม ไม่ resize เพื่อคำนวณระยะแม่นยำ
        self.frame_lock = threading.Lock()

        # ตำแหน่งเมาส์ล่าสุดบนภาพที่แสดงผล (พิกัดหลัง resize)
        self.mouse_x = -1          # -1 = เมาส์ไม่ได้อยู่บนภาพ (ไม่วาดกากบาท)
        self.mouse_y = -1
        self.goal_user = None        # จุดวางที่ผู้ใช้คลิกเลือก (พิกัดภาพที่ crop) ; None = ใช้ตำแหน่ง default
        self._goal_pending = False   # มีการเปลี่ยนจุดวาง -> วางเส้นทางใหม่จากภาพสแกนเดิม
        self._scan_snap = None       # ภาพ depth + ผลตรวจวัตถุสีเหลืองตอนสแกนล่าสุด (ไว้วางเส้นทางใหม่เมื่อเปลี่ยนจุดวาง)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ใช้ message_filters เพื่อ sync สอง topic เข้าด้วยกันตาม timestamp
        # เพราะ color และ depth ถูก publish คนละเวลากันเล็กน้อย
        self.color_sub = Subscriber(self, Image, color_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=5,
            slop=0.05,  # ยอมให้ timestamp ต่างกันได้ไม่เกิน 50ms
        )
        self.sync.registerCallback(self.synced_callback)

        self.window_name = 'RealSense D455 - RGB + Depth'
        self.start_dashboard()               # หน้าเว็บแทนหน้าต่าง OpenCV / matplotlib เดิม
        self.start_robot_monitor()           # อ่านสถานะหุ่น (reg 10) ไว้หยุด dynamic scan ระหว่างหุ่นทำงาน
        self.start_hand_guard()              # MediaPipe: พบมือระหว่างหุ่นทำงาน = หยุด, นิ้วชี้ = เลือกจุดวาง

        self.display_timer = self.create_timer(1.0 / display_fps, self.display_callback)

        self.get_logger().info(f'Color topic: {color_topic}')
        self.get_logger().info(f'Depth topic: {depth_topic}')
        self.get_logger().info(
            'ถ้าไม่เห็นภาพ ให้เช็คว่า launch driver ด้วย align_depth.enable:=true แล้วหรือยัง'
        )
        self.get_logger().info(f'Dataset folder: {self.dataset_dir}')
        self.get_logger().info(
            'Dashboard: Camera Viewer ชี้เมาส์ดูระยะ, คลิกซ้ายตั้งจุดวาง (คลิกขวา / "a" = default) | '
            'Scan (spacebar) | Start ("s") ส่งเส้นทางทาง Modbus | "q" ออก')
        self.get_logger().info(
            'Choose Algorithm ("1"-"5"): Dijkstra | A* (ค่าเริ่มต้น) | Bidirectional Dijkstra | Bidirectional A* | '
            'Comparison (เทียบทุกแบบ ส่งเส้นทางที่สั้นที่สุด)')
        self.get_logger().info(
            'Choose Mode: "6" หมุน R กลับตามมุม tilt ที่ตำแหน่งที่ 2 | "7" ไม่หมุนกลับ (ค่าเริ่มต้น)')

    # ================================================================ dashboard (หน้าเว็บ)
    def get_logger(self):
        """logger เดิม + เก็บข้อความไปแสดงบน dashboard"""
        tap = self.__dict__.get('_log_tap')
        return tap if tap is not None else super().get_logger()

    def _push_log(self, level, msg):
        self._log_id += 1
        self._log.append({'id': self._log_id, 't': datetime.now().strftime('%H:%M:%S'), 'level': level, 'msg': msg})

    def start_dashboard(self):
        gp = self.get_parameter
        host = str(gp('dashboard_host').value)
        port = int(gp('dashboard_port').value)
        plotly_path = find_plotly_js(str(gp('dashboard_plotly_js').value))
        try:
            self.dashboard = DashboardServer(self, host, port, plotly_path)
        except OSError as e:
            self.dashboard = None
            self.get_logger().error(
                f'เปิด dashboard ที่ {host}:{port} ไม่ได้ ({e}) -> พอร์ตอาจถูกใช้อยู่ เปลี่ยนด้วย -p dashboard_port:=8081')
            return
        self.dashboard.start()
        url = f"http://{'localhost' if host in ('0.0.0.0', '127.0.0.1', '') else host}:{port}/"
        self.get_logger().info(
            f'Dashboard: {url}  (Plotly: {plotly_path or PLOTLY_CDN + " (ต้องมีอินเทอร์เน็ต)"})')
        if bool(gp('dashboard_open_browser').value):
            threading.Thread(target=webbrowser.open, args=(url,), daemon=True).start()

    def stop_dashboard(self):
        self._dash_stop = True
        with self._jpeg_cond:
            self._jpeg_cond.notify_all()
        if self.dashboard is not None:
            self.dashboard.stop()
        t = getattr(self, '_hand_thread', None)
        if t is not None:
            t.join(timeout=2.0)          # ให้ hand guard แจ้ง server ก่อนปิด

    def dashboard_mouse(self, x, y):
        """ตำแหน่งเมาส์บนภาพสด (พิกัดภาพที่ขยายด้วย display_scale แล้ว เหมือน on_mouse เดิม) ; -1 = ออกนอกภาพ"""
        if x < 0 or y < 0:
            self.mouse_x = self.mouse_y = -1
        else:
            self.mouse_x, self.mouse_y = int(x), int(y)

    def dashboard_command(self, body):
        """รับคำสั่งจากหน้าเว็บ (thread ของ http) -> เข้าคิว ไปทำใน thread ของ ROS ; คืน (ok, ข้อความ, ticket)"""
        a = str(body.get('action', ''))
        arg = None
        try:
            if a in ('scan', 'start', 'goal_default', 'quit'):
                pass
            elif a == 'algo':
                arg = int(body.get('value', 0))
                if arg not in PATH_MODE_ALGO:
                    return False, f'algorithm {arg} ไม่ถูกต้อง', 0
            elif a == 'mode':
                arg = int(body.get('value', 0))
                if arg not in (6, 7):
                    return False, f'mode {arg} ไม่ถูกต้อง', 0
            elif a == 'dynamic':
                arg = bool(body.get('value'))
            elif a == 'goal':
                sc = self.display_scale if self.display_scale > 0 else 1.0
                arg = (float(body['x']) / sc, float(body['y']) / sc)   # พิกัดภาพที่ crop (ก่อนขยาย)
            else:
                return False, f'ไม่รู้จักคำสั่ง "{a}"', 0
        except (KeyError, TypeError, ValueError):
            return False, f'ข้อมูลของคำสั่ง "{a}" ไม่ถูกต้อง', 0
        return True, '', self._enqueue(a, arg)

    def _enqueue(self, a, arg=None):
        """เข้าคิวคำสั่งให้ thread ของ ROS ทำ (thread ไหนเรียกก็ได้) ; คืน ticket"""
        with self._cmd_lock:
            self._cmd_seq += 1
            t = self._cmd_seq
            self._cmd_q.put((t, a, arg))
        return t

    def process_commands(self, color, depth, header):
        """ทำคำสั่งจาก dashboard ที่รอในคิว (เรียกจาก display_callback = thread ของ ROS เหมือนการกดปุ่มเดิม)"""
        for _ in range(16):
            try:
                t, a, arg = self._cmd_q.get_nowait()
            except queue.Empty:
                return
            self._busy = a
            try:
                self._run_command(a, arg, color, depth, header)
            except Exception as e:      # ไม่ให้คำสั่งเดียวทำให้โหนดล่ม
                self.get_logger().error(f'ทำคำสั่ง {a} ไม่สำเร็จ: {e}')
            finally:
                self._busy = ''
                self._cmd_done = t
            if a == 'quit':
                return

    def _run_command(self, a, arg, color, depth, header):
        if a == 'scan':                                  # = กด spacebar เดิม
            busy, why = self.robot_busy()
            if busy:
                self.get_logger().warn(f'ยังสแกนไม่ได้: {why} (สแกนได้เมื่อหุ่นทำงานเสร็จ)')
                return
            self.do_scan(color, depth, header)
        elif a == 'dynamic':                             # สวิตช์ Dynamic scan / ปุ่ม 'd'
            self.dyn_enable = bool(arg)
            self._dyn_next = 0.0
            self.get_logger().info(f'Dynamic scan: {"เปิด" if arg else "ปิด"} ({self.dyn_hz:g} fps)')
        elif a == 'start':                               # = กด 's' เดิม
            self.send_path_modbus()
        elif a == 'algo':                                # = กด 1-5 เดิม
            self.set_path_mode(arg)
        elif a == 'mode':                                # = กด 6 / 7 เดิม
            self.set_r_return(arg == 6)
        elif a == 'goal':                                # = คลิกซ้ายบนภาพเดิม
            self.goal_user = arg
            self.get_logger().info(f'จุดวางที่เลือก = ({arg[0]:.0f}, {arg[1]:.0f}) px')
            self.replan_goal()
        elif a == 'goal_default':                        # = กด 'a' / คลิกขวาเดิม
            self.goal_user = None
            self.get_logger().info('จุดวาง: กลับไปใช้ตำแหน่ง default')
            self.replan_goal()
        elif a == 'quit':                                # = กด 'q' เดิม
            self.get_logger().info('ผู้ใช้สั่งปิดโปรแกรมจาก dashboard...')
            rclpy.shutdown()

    def do_scan(self, color, depth, header, dynamic=False):
        """Scan: แคป -> point cloud -> solid -> วางเส้นทาง -> แผนที่ 3 มิติ (เหมือนกด spacebar เดิม)
        dynamic=True (สแกนอัตโนมัติ): ไม่เซฟไฟล์ลง dataset (ยกเว้น dynamic_save) และไม่นับเป็นการกด Scan"""
        if color is None or depth is None:
            self.get_logger().warn('ยังไม่มีภาพจากกล้อง จึงสแกนไม่ได้ (เช็คว่า launch realsense2_camera แล้ว)')
            return
        # เซฟจากภาพ color ต้นฉบับ (ไม่มี crosshair/ตัวเลขติดไปด้วย)
        self.save_frame(color, depth, header, save=(not dynamic) or self.dyn_save)
        if dynamic:
            return
        for i, o in enumerate(self.yellow_results):
            if o['w_mm'] is not None:
                self.get_logger().info(
                    f"yellow #{i + 1}: {o['w_mm']:.1f} x {o['h_mm']:.1f} mm "
                    f"(bbox {o['bbox']}, Z={o['z_mm']:.0f} mm)")
        self.scan_count += 1

    def publish_frame(self, img):
        """ภาพสด (เหมือนหน้าต่าง RealSense D455 - RGB + Depth เดิม) -> JPEG ให้ stream (เฉพาะตอนมีคนดู)"""
        if self._stream_clients <= 0:
            return
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.dash_jpeg_quality])
        if not ok:
            return
        with self._jpeg_cond:
            self._jpeg = buf.tobytes()
            self._jpeg_seq += 1
            self._jpeg_cond.notify_all()

    def _store_img(self, key, img):
        """เก็บภาพ (BGR) เป็น PNG ให้หน้าเว็บ ; img = None = ล้างภาพ"""
        data = None
        if img is not None:
            ok, buf = cv2.imencode('.png', img)
            data = buf.tobytes() if ok else None
        self._img[key] = data
        self._ver[key] += 1

    # ================================================================ hand guard (MediaPipe Hands)
    def start_hand_guard(self):
        if not self.hand_enable:
            self.get_logger().warn('Hand guard ปิดอยู่ (hand_enable:=false): หุ่นจะไม่หยุดเมื่อมีมือเข้าพื้นที่')
            if self.modbus_enable:      # บอก server ว่าไม่ได้ใช้ hand guard (ไม่ต้องเฝ้าสัญญาณ)
                threading.Thread(target=lambda: _ModbusLink(self).write(self.hand_hold_reg, HOLD_GUARD_OFF),
                                 daemon=True).start()
            return
        self._hand_thread = threading.Thread(target=self._hand_loop, daemon=True)
        self._hand_thread.start()

    def _hand_loop(self):
        """ตรวจมือ hand_hz ครั้ง/วินาที (thread แยก ไม่ติดการสแกน) แล้วเขียน hold ไป server ทุกรอบ (= heartbeat)"""
        log = self.get_logger()
        link = _ModbusLink(self) if self.modbus_enable else None
        try:
            hands = HandDetector(self.hand_max_num, self.hand_model_complexity, self.hand_min_det,
                                 self.hand_min_track, self.hand_model_path)
        except ImportError as e:
            self.hand_err = f'ไม่มี mediapipe ({e}) -> pip install mediapipe'
        except Exception as e:
            self.hand_err = f'เริ่ม MediaPipe ไม่ได้: {e}'
        if self.hand_err:
            log.error(f'Hand guard: {self.hand_err} ; ตอนนี้หุ่นจะไม่หยุดเมื่อมีมือเข้าพื้นที่')
            if link is not None:
                link.write(self.hand_hold_reg, HOLD_GUARD_OFF)
            return
        self.hand_ok = True
        log.info(f'Hand guard: MediaPipe {hands.kind} ตรวจ {self.hand_hz:g} ครั้ง/วินาที '
                 f'(พบมือระหว่างหุ่นทำงาน = หยุด, ไม่พบมือ {self.hand_clear_s:g} s = ทำงานต่อ ; '
                 f'หุ่นว่าง + ชูนิ้วชี้ = เลือกจุดวาง)')
        period = 1.0 / self.hand_hz
        err_logged = False
        try:
            while not self._dash_stop:
                t0 = time.monotonic()
                with self.frame_lock:
                    full, off, ft = self.latest_full, self.latest_full_off, self.latest_full_t
                    crop = self.latest_color.shape[:2] if self.latest_color is not None else None
                found = []
                if full is not None:
                    try:
                        found = self.detect_hands(hands, full, off)
                        err_logged = False
                    except Exception as e:
                        if not err_logged:
                            log.error(f'Hand guard: ตรวจมือไม่สำเร็จ: {e}')
                            err_logged = True
                # ไม่มีภาพจากกล้อง = found ว่าง: ไม่สั่งหยุดหุ่น (หยุดเฉพาะตอนเห็นมือ)
                hold = self.hand_update(found, t0, (crop[1], crop[0]) if crop else None)
                if link is not None:
                    link.write(self.hand_hold_reg, HOLD_HAND if hold else HOLD_CLEAR)
                self._hand_times.append(time.monotonic())
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            hands.close()
            if link is not None:
                if not self.robot_working():
                    # ปิดโปรแกรมตอนหุ่นว่าง: บอก server ให้เลิกเฝ้าสัญญาณ (ปิดตอนหุ่นทำงาน = ปล่อยให้ server สั่งหยุด)
                    link.write(self.hand_hold_reg, HOLD_GUARD_OFF)
                link.close()

    def detect_hands(self, hands, full, off):
        """MediaPipe บนภาพเต็ม (ไม่ crop: เห็นทั้งมือแม้ปลายนิ้วเพิ่งเข้าพื้นที่) -> พิกัด pixel ของภาพที่ crop"""
        rgb = cv2.cvtColor(full, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        H, W = full.shape[:2]
        out = []
        for lm, mp_label, score in hands.process(rgb):
            pts = np.array(lm, np.float32) * np.array([W, H], np.float32)
            # MediaPipe ถือว่าภาพกลับกระจกแล้ว ภาพกล้องไม่ได้กลับ จึงสลับซ้าย/ขวา (เหมือน finger_tag.py)
            label = {'Right': 'Left', 'Left': 'Right'}.get(mp_label, mp_label)
            out.append({'pts': pts - np.array(off, np.float32), 'up': hand_fingers_extended(pts),
                        'label': label, 'score': score})
        return dedupe_hands(out)

    def hand_update(self, found, now, crop_wh):
        """ตัดสิน hold + โหมดนิ้วชี้จากผลตรวจรอบนี้ ; คืน True = ให้หุ่นหยุด"""
        if self.hand_zone == 'crop' and crop_wh is not None:
            cw, ch, m = crop_wh[0], crop_wh[1], 20.0
            present = [h for h in found
                       if np.any((h['pts'][:, 0] > -m) & (h['pts'][:, 0] < cw + m)
                                 & (h['pts'][:, 1] > -m) & (h['pts'][:, 1] < ch + m))]
        else:
            present = list(found)
        self.hands_view, self.hand_view_t = found, now
        if present:
            self._hand_last_seen = now
        working = self.robot_working()
        reason = ''
        if working and present:
            hold, reason = True, f'พบมือ {len(present)} ข้าง'
        elif working and self.hand_hold and now - self._hand_last_seen < self.hand_clear_s:
            hold, reason = True, f'รอให้ไม่มีมือครบ {self.hand_clear_s:g} s'
        else:
            hold = False
        if hold and not self.hand_hold:
            self.get_logger().warn(f'Hand guard: {reason} ระหว่างหุ่นทำงาน -> สั่งหุ่นหยุดทันที')
        elif self.hand_hold and not hold:
            self.get_logger().info('Hand guard: ไม่พบมือแล้ว -> หุ่นทำงานต่อ' if working
                                   else 'Hand guard: ปลดการหยุด (หุ่นไม่ได้ทำงานแล้ว)')
        self.hand_hold, self.hand_hold_reason = hold, reason
        self.pointer_update([] if working else present, now, crop_wh)
        return hold

    def pointer_update(self, hands, now, crop_wh):
        """หุ่นว่าง + ชูนิ้วชี้นิ้วเดียว (มือเดียว): ปลายนิ้ว = เครื่องหมายบวก ; ค้างนิ่ง pointer_dwell_s = เลือกจุดวาง"""
        tgt = None
        if self.pointer_enable and crop_wh is not None:
            pointing = [h for h in hands if is_index_pointing(h['up'])]
            if len(pointing) == 1 and len(hands) == 1:
                x, y = (float(v) for v in pointing[0]['pts'][8])
                if 0 <= x < crop_wh[0] and 0 <= y < crop_wh[1]:
                    tgt = (x, y)
        self.finger_target = tgt
        if tgt is None:
            # หลุดแค่รอบเดียว (MediaPipe พลาดบางเฟรม) ไม่นับใหม่ ; หลุดเกิน 0.7 s = เริ่มนับใหม่
            if self._dwell is not None and now - self._dwell.get('seen', now) > 0.7:
                self._dwell = None
                self.dwell_progress = 0.0
            return
        d = self._dwell
        if d is None or math.hypot(tgt[0] - d['x'], tgt[1] - d['y']) > self.pointer_dwell_px:
            self._dwell = {'x': tgt[0], 'y': tgt[1], 't0': now, 'seen': now, 'done': False}
            self.dwell_progress = 0.0
            return
        d['seen'] = now
        p = (now - d['t0']) / self.pointer_dwell_s
        self.dwell_progress = min(p, 1.0)
        if p >= 1.0 and not d['done']:
            d['done'] = True
            self.get_logger().info(f'นิ้วชี้: เลือกจุดวางที่ ({d["x"]:.0f}, {d["y"]:.0f}) px')
            self._enqueue('goal', (d['x'], d['y']))

    def robot_working(self):
        """หุ่นกำลังทำงาน (รวมกำลังส่ง/รอเริ่ม/หยุดรอเพราะพบมือ)"""
        return bool(self._modbus_busy or self._await_robot is not None or self.robot_status in (1, 2))

    def hand_recent(self):
        return time.monotonic() - self._hand_last_seen < self.hand_clear_s

    def update_finger_cursor(self):
        """ตำแหน่งเครื่องหมายบวกตามปลายนิ้ว (ตรวจ 5 ครั้ง/วินาที จึงเลื่อนแบบนุ่มทุกเฟรมภาพ)"""
        tgt = self.finger_target
        if tgt is None or time.monotonic() - self.hand_view_t > 1.0:
            self.finger_cursor = None
            return None
        c = self.finger_cursor
        self.finger_cursor = tgt if c is None else (c[0] + 0.35 * (tgt[0] - c[0]), c[1] + 0.35 * (tgt[1] - c[1]))
        return self.finger_cursor

    def draw_hands(self, img):
        """โครงมือ + แท็กปลายนิ้ว (แบบ finger_tag.py) + ป้ายเตือนเมื่อสั่งหุ่นหยุด"""
        H, W = img.shape[:2]
        if self.hands_view and time.monotonic() - self.hand_view_t < 1.0:
            for hnd in self.hands_view:
                p = [(int(round(x)), int(round(y))) for x, y in hnd['pts']]
                for a, b in HAND_CONNECTIONS:
                    cv2.line(img, p[a], p[b], (255, 255, 255), 1, cv2.LINE_AA)
                for (name, tip, _, color), up in zip(HAND_FINGERS, hnd['up']):
                    cv2.circle(img, p[tip], 5 if up else 3, color, -1 if up else 1, cv2.LINE_AA)
                    if 0 <= p[tip][0] < W and 0 <= p[tip][1] < H:
                        put_outlined(img, name, (p[tip][0] + 6, p[tip][1] - 5), color, 0.32)
                wx = min(max(p[HAND_WRIST][0] - 40, 2), W - 150)
                wy = min(max(p[HAND_WRIST][1] + 14, 12), H - 4)
                put_outlined(img, f"{hnd['label']} hand  up: {sum(hnd['up'])}", (wx, wy), (0, 255, 255), 0.36)
        if self.hand_hold:
            txt = 'HAND DETECTED - ROBOT STOPPED' if 'มือ' in self.hand_hold_reason else 'ROBOT STOPPED'
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            x0 = max((W - tw) // 2 - 6, 0)
            cv2.rectangle(img, (x0, H // 2 - th - 8), (x0 + tw + 12, H // 2 + 6), (0, 0, 200), -1)
            cv2.putText(img, txt, (x0 + 6, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def draw_dwell(self, img, c):
        """วงแหวนนับเวลาค้างนิ้ว (ครบวง = เลือกจุดวาง)"""
        p = self.dwell_progress
        cv2.circle(img, c, 13, (0, 0, 0), 3, cv2.LINE_AA)
        if p > 0:
            cv2.ellipse(img, c, (13, 13), -90, 0, int(360 * p), (0, 255, 0), 2, cv2.LINE_AA)
        put_outlined(img, 'INDEX', (c[0] - 16, c[1] + 26), (0, 255, 0), 0.36)

    def hand_state(self):
        ts = list(self._hand_times)
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) >= 2 and ts[-1] > ts[0] else 0.0
        return {'enabled': self.hand_enable, 'ok': self.hand_ok, 'err': self.hand_err, 'hz': self.hand_hz,
                'rate': round(rate, 1), 'count': len(self.hands_view) if time.monotonic() - self.hand_view_t < 1.0 else 0,
                'present': self.hand_recent(), 'pointer': self.finger_target is not None,
                'dwell': round(self.dwell_progress, 2), 'hold': self.hand_hold, 'reason': self.hand_hold_reason}

    # ================================================================ dynamic scan + สถานะหุ่น
    def start_robot_monitor(self):
        if not self.modbus_enable:
            self.get_logger().info('Modbus ปิดอยู่: dynamic scan ทำงานต่อเนื่อง (ไม่มีสถานะหุ่นให้รอ)')
            return
        threading.Thread(target=self._robot_monitor, daemon=True).start()

    def _robot_monitor(self):
        """อ่าน reg สถานะหุ่น (Lua เขียน 1 = กำลังเคลื่อนที่, 0 = ว่าง) จาก modbus_server.py เป็นระยะ"""
        sock, tid, warned = None, 0, False
        period = 1.0 / self.robot_poll_hz
        while not self._dash_stop:
            try:
                if sock is None:
                    sock = socket.create_connection((self.modbus_ip, self.modbus_port), timeout=1.5)
                tid = (tid + 1) % 65536
                v = modbus_read_regs(sock, tid, self.modbus_unit, self.robot_status_reg, 1)[0]
                if not self.robot_conn:
                    self.get_logger().info(
                        f'อ่านสถานะหุ่นจาก {self.modbus_ip}:{self.modbus_port} reg {self.robot_status_reg} ได้แล้ว')
                self.robot_conn, warned = True, False
                self._on_robot_status(v)
                time.sleep(period)
            except Exception as e:
                if not warned:
                    self.get_logger().warn(
                        f'อ่านสถานะหุ่น (reg {self.robot_status_reg}) จาก {self.modbus_ip}:{self.modbus_port} '
                        f'ไม่ได้: {e} -> จะลองใหม่เรื่อย ๆ')
                    warned = True
                self.robot_conn = False
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
                    sock = None
                self._on_robot_status(None)
                time.sleep(1.0)

    def _on_robot_status(self, v):
        prev, self.robot_status = self.robot_status, v
        aw = self._await_robot
        log = self.get_logger()
        if aw is not None:
            waited = time.monotonic() - aw['t0']
            if v in (1, 2):
                if not aw['seen']:
                    log.info('หุ่นเริ่มทำงาน: หยุด scan map จนกว่าหุ่นจะทำงานเสร็จ')
                aw['seen'] = True
            elif v == 0 and aw['seen']:
                self._await_robot = None
                log.info('หุ่นทำงานเสร็จแล้ว: กลับมา scan map ต่อ')
            elif not aw['seen'] and waited > self.robot_start_timeout:
                self._await_robot = None
                log.warn(f'หุ่นไม่เริ่มทำงานภายใน {self.robot_start_timeout:g} วินาทีหลังส่งเส้นทาง '
                         '(สคริปต์ Lua บน Dobot รันอยู่ไหม?) -> กลับมา scan map ต่อ')
            elif waited > self.robot_max_wait:
                self._await_robot = None
                log.warn(f'รอหุ่นเกิน {self.robot_max_wait:g} วินาที (อ่านสถานะไม่ได้) -> กลับมา scan map ต่อ')
        elif prev == 0 and v in (1, 2):
            log.info('หุ่นเริ่มทำงาน (สั่งจากที่อื่น): หยุด scan map จนกว่าหุ่นจะทำงานเสร็จ')
        elif prev in (1, 2) and v == 0:
            log.info('หุ่นทำงานเสร็จแล้ว: กลับมา scan map ต่อ')

    def robot_busy(self):
        """(True, เหตุผล) ถ้าหุ่นกำลังทำงาน/กำลังจะทำงาน -> ห้ามสแกน"""
        if self._modbus_busy:
            return True, 'กำลังส่งเส้นทางให้หุ่น'
        aw = self._await_robot
        if aw is not None:
            return True, 'หุ่นกำลังทำงาน' if aw['seen'] else 'รอหุ่นเริ่มทำงาน'
        if self.robot_status == 1:
            return True, 'หุ่นกำลังทำงาน'
        if self.robot_status == 2:
            return True, 'หุ่นหยุดรอ (พบมือ)'
        return False, ''

    def dynamic_tick(self, color, depth, header):
        """สแกน map อัตโนมัติตามรอบ dynamic_scan_hz ; หุ่นทำงานอยู่ = ข้าม (map/เส้นทางค้างไว้รอบสุดท้าย)"""
        if not self.dyn_enable or self.intrinsics is None or color is None:
            return
        if self.robot_busy()[0]:
            self._dyn_next = 0.0             # หุ่นเสร็จเมื่อไหร่ สแกนรอบใหม่ทันที
            return
        if self.hand_pause_dynamic and self.hand_recent():
            self._dyn_next = 0.0             # มือจะกลายเป็นสิ่งกีดขวางใน map: รอเอามือออกก่อน
            return
        t0 = time.monotonic()
        if t0 < self._dyn_next:
            return
        # ระหว่างสแกน path_result ถูกล้างชั่วคราว: ให้หน้าเว็บเห็นผลรอบก่อนไปจนกว่ารอบใหม่จะเสร็จ (ปุ่ม Start ไม่กะพริบ)
        self._path_hold = (self.path_result, self.last_runs, self.path_msg)
        self._dyn_running = True
        self._log_tap.quiet_thread = threading.get_ident()
        try:
            self.do_scan(color, depth, header, dynamic=True)
        except Exception as e:
            self.get_logger().error(f'dynamic scan ไม่สำเร็จ: {e}')
        finally:
            self._log_tap.quiet_thread = None
            self._dyn_running = False
            self._path_hold = None
        t1 = time.monotonic()
        # รอบถัดไป: ตามรอบที่ตั้งไว้ แต่เว้นอย่างน้อย 0.15 วินาที ให้ภาพสดยังขยับได้ถ้าสแกนช้ากว่ารอบ
        self._dyn_next = max(t0 + 1.0 / self.dyn_hz, t1 + 0.15)
        self._dyn_count += 1
        self._dyn_times.append(t1)
        self._dyn_last = {'t': datetime.now().strftime('%H:%M:%S'), 'ms': int(round(1000 * (t1 - t0)))}

    def dynamic_state(self):
        busy, why = self.robot_busy()
        if not busy and self.hand_pause_dynamic and self.hand_recent():
            busy, why = True, 'มีมือในภาพ'
        ts = list(self._dyn_times)
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) >= 2 and ts[-1] > ts[0] else 0.0
        if ts and time.monotonic() - ts[-1] > 3.0 / self.dyn_hz + 2.0:
            rate = 0.0                       # ไม่ได้สแกนมาสักพัก
        return {'enabled': self.dyn_enable, 'hz': self.dyn_hz, 'paused': self.dyn_enable and busy,
                'reason': why, 'last': self._dyn_last, 'count': self._dyn_count, 'rate': round(rate, 2)}

    def start_status(self, r):
        if not self.modbus_enable:
            return False, 'Modbus ปิดอยู่: รันด้วย -p modbus_enable:=true'
        busy, why = self.robot_busy()
        if busy:
            return False, f'{why}: กด Start ได้อีกเมื่อหุ่นทำงานเสร็จ'
        if self.hand_recent():
            return False, 'มีมือในภาพ: เอามือออกจากพื้นที่ทำงานก่อนกด Start'
        if r is None:
            return False, 'กด Scan เพื่อสแกนและวางเส้นทางก่อน'
        if r.get('pts') is None:
            return False, 'ยังไม่มีเส้นทางที่ใช้ได้: ย้ายสิ่งกีดขวางหรือเลือกจุดวางใหม่ แล้ว Scan อีกครั้ง'
        if r.get('robot_xyz') is None:
            return False, 'ไม่มี hand-eye: รันด้วย -p handeye_file:=".../handeye_calibration.json"'
        if self._modbus_busy:
            return False, 'กำลังส่งเส้นทางให้หุ่น…'
        return True, ''

    def dashboard_state(self):
        hold = self._path_hold
        r, runs, pmsg = hold if hold is not None else (self.path_result, self.last_runs, self.path_msg)
        path = None
        if r is not None:
            ok = r.get('pts') is not None
            g = r.get('grip') or {}
            path = {'ok': ok, 'reason': r.get('reason', ''),
                    'algo': PATH_ALGO_LABEL.get(r.get('algo', 'astar'), 'A*'),
                    'n_wp': int(len(r['pts'])) if ok else 0,
                    'peak_mm': round(float(r['pts'][:, 2].max()), 1) if ok else None,
                    'goal_src': r.get('goal_src', 'default'),
                    'r_delta': round(float(g.get('r_delta', 0.0)), 1),
                    'r_return': bool(r.get('r_return')),
                    'robot': r.get('robot_xyz') is not None}
        ready, reason = self.start_status(r)
        return {
            'frames': self.latest_color is not None,
            'intrinsics': self.intrinsics is not None,
            'algo': self.path_mode,
            'r_return': self.r_return,
            'busy': self._busy,
            'queued': self._cmd_q.qsize(),
            'cmd_done': self._cmd_done,
            'scan_count': self.scan_count,
            'versions': dict(self._ver),
            'has_img': {k: self._img.get(k) is not None for k in ('depth', 'seg')},
            'runs': runs,
            'path_msg': pmsg,
            'path': path,
            'goal': 'click' if self.goal_user is not None else 'default',
            'start': {'ready': ready, 'reason': reason},
            'modbus': {'enabled': self.modbus_enable, 'target': f'{self.modbus_ip}:{self.modbus_port}',
                       'busy': self._modbus_busy, 'last': self.modbus_last},
            'log': list(self._log)[-40:],
            'dynamic': self.dynamic_state(),
            'hand': self.hand_state(),
            'robot': {'monitor': self.modbus_enable, 'conn': self.robot_conn, 'status': self.robot_status,
                      'busy': self.robot_busy()[0]},
        }

    def info_callback(self, msg: CameraInfo):
        if self.intrinsics is not None:
            return  # intrinsics ไม่เปลี่ยนระหว่างใช้งาน เก็บครั้งแรกพอ
        k = msg.k
        if k[0] > 0 and k[4] > 0:
            self.intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy
            self.get_logger().info(
                f'ได้ intrinsics: fx={k[0]:.1f} fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}'
            )

    def synced_callback(self, color_msg: Image, depth_msg: Image):
        try:
            color_img = imgmsg_to_bgr(color_msg)
            # depth จาก RealSense ปกติเป็น 16UC1 หน่วย millimeter ต่อ pixel
            depth_img = imgmsg_to_array(depth_msg)
        except Exception as e:
            self.get_logger().error(f'แปลงภาพไม่สำเร็จ: {e}')
            return
        full_color, full_off = color_img, (0, 0)       # ภาพเต็มสำหรับ hand guard

        # crop ก่อนเข้า process ทุกอย่าง (แสดงผล, เมาส์, เซฟ, point cloud, grid)
        if self.crop_enabled:
            if color_img.shape[:2] != depth_img.shape[:2] and not self._crop_mismatch_warned:
                self._crop_mismatch_warned = True
                self.get_logger().warn(
                    f'ขนาด color {color_img.shape[1]}x{color_img.shape[0]} ไม่ตรงกับ depth '
                    f'{depth_img.shape[1]}x{depth_img.shape[0]}: crop จะคลาดเคลื่อน '
                    '(ต้องใช้ aligned_depth_to_color)'
                )
            ih, iw = color_img.shape[:2]
            x0, y0, cw, ch = compute_crop_rect(
                iw, ih, self.crop_x, self.crop_y, self.crop_w, self.crop_h
            )
            self.crop_x0, self.crop_y0 = x0, y0
            full_off = (x0, y0)
            color_img = np.ascontiguousarray(color_img[y0:y0 + ch, x0:x0 + cw])
            depth_img = np.ascontiguousarray(depth_img[y0:y0 + ch, x0:x0 + cw])
            if not self._crop_logged:
                self._crop_logged = True
                self.get_logger().info(
                    f'Crop: x={x0} y={y0} กว้าง={cw} สูง={ch} (จากภาพ {iw}x{ih})'
                )
            if self.flat is not None:
                delta = flat_slice(self.flat, (x0, y0, cw, ch), (iw, ih))
                if delta is not None:
                    depth_img = apply_flat_correction(depth_img, delta)
                elif not self._flat_warned:
                    self._flat_warned = True
                    self.get_logger().warn('depth flat: กรอบ crop/ขนาดภาพไม่ตรงกับตอน calibrate จึงไม่แก้ค่า')

        with self.frame_lock:
            self.latest_color = color_img
            self.latest_depth = depth_img
            self.latest_header = depth_msg.header
            self.latest_full, self.latest_full_off = full_color, full_off
            self.latest_full_t = time.monotonic()

    def zone_cols(self, width):
        """คืน (x_left, x_right) เป็นคอลัมน์ pixel ของเส้นแบ่งบนภาพที่ crop แล้ว"""
        xl = int(round(min(max(self.zone_left_frac, 0.0), 1.0) * width))
        xr = int(round(min(max(self.zone_right_frac, 0.0), 1.0) * width))
        if xr - xl < 2:          # ค่าผิดปกติ -> ใช้ทั้งภาพ
            return 0, width
        return xl, xr

    def plan_path_3d(self, depth_full, header):
        """
        เรียกหลังสแกน 3D map ตอนกด spacebar: วาง A* 3 มิติ จากวัตถุสีเหลือง (โซนซ้าย) ไปจุดวาง (โซนขวา)
        depth_full = depth ที่ crop แล้ว (ทั้งภาพ ไม่ตัดโซน) ; ผลเก็บใน self.path_result / self.path_plot
        """
        self.path_result = None
        self.path_plot = None
        self.last_runs = None
        self.path_msg = ''
        if not (self.path_enable and self.yellow_enable) or self.intrinsics is None:
            self.path_msg = ('ยังไม่ได้รับ camera_info (intrinsics)' if self.intrinsics is None
                             else 'ปิดการวางเส้นทางอยู่ (path_enable / yellow_enable)')
            return
        H, W = depth_full.shape[:2]
        o = self.start_yellow(W)                       # START = วัตถุสีเหลืองชิ้นใหญ่สุด (ที่ไหนก็ได้) ที่มี depth
        if o is None:
            self.get_logger().warn('path: ไม่พบวัตถุสีเหลือง (ที่มีค่า depth) จึงยังวางเส้นทางไม่ได้')
            self.path_msg = 'ไม่พบวัตถุสีเหลือง (ที่มีค่า depth) ให้หยิบ'
            return
        hf = build_height_field(depth_full, self.solid_params)
        if hf is None:
            self.get_logger().warn('path: หาระนาบพื้นไม่ได้ จึงวางเส้นทางไม่ได้')
            self.path_msg = 'หาระนาบพื้นไม่ได้'
            return
        plane, h, fz = hf
        x, y, bw, bh = o['bbox']
        start = (x + bw / 2.0, y + bh / 2.0)
        goal = self.default_goal(W, H)
        goal_src = 'default'
        if self.goal_user is not None:                 # ผู้ใช้คลิกเลือกจุดวางเอง (ที่ไหนก็ได้บนภาพ)
            goal = (float(np.clip(self.goal_user[0], 0, W - 1)), float(np.clip(self.goal_user[1], 0, H - 1)))
            goal_src = 'click'
        z_floor_s = float(fz[int(np.clip(start[1], 0, H - 1)), int(np.clip(start[0], 0, W - 1))])
        h_obj = max(z_floor_s - o['z_mm'], 5.0)        # ความสูงวัตถุเหนือพื้น (mm)
        h = h.copy()
        pad = 4
        h[max(y - pad, 0):y + bh + pad, max(x - pad, 0):x + bw + pad] = 0.0   # ไม่นับวัตถุที่จะหยิบเป็นสิ่งกีดขวาง
        fx, fy, cx, cy = self.intrinsics
        intr_full = (fx, fy, cx - self.crop_x0, cy - self.crop_y0)
        extra = {'start': start, 'box_px': (bw, bh), 'infl_polys': [], 'goal_src': goal_src,
                 'r_return': self.r_return}
        self.get_logger().info(f'path: จุดวาง ({goal_src}) = ({goal[0]:.0f}, {goal[1]:.0f}) px')
        grip = self.compute_grip(o)
        if grip is not None:
            extra['grip'] = grip
        obstacle = None
        if self.path_obstacle_source == 'solids':
            try:
                obstacle, extra = self.build_path_obstacle(
                    depth_full, o, h, fz, plane, intr_full, h_obj, z_floor_s, extra)
            except Exception as e:      # ไม่ให้โหนดล่ม: ถอยไปใช้ height map แบบเดิม
                obstacle = None
                self.get_logger().error(f'path: สร้าง solid ของ obstacle ไม่สำเร็จ ({e}) ใช้ height map แบบเดิมแทน')
        kw = dict(cell_px=self.path_cell_px, dz_mm=self.path_dz_mm,
                  zmax_mm=self.path_zmax_mm, lift_mm=self.path_lift_mm,
                  radius_px=max(bw, bh) / 2.0 + self.path_margin_px,
                  z_margin_mm=self.path_z_margin_mm,
                  cell_mm=self.path_cell_px * z_floor_s / fx, obstacle=obstacle)
        mode = self.path_mode
        algo = PATH_MODE_ALGO.get(mode, 'astar')
        compare = algo == 'compare'
        run_algos = PATH_ALGOS if compare else (algo,)
        runs = {}
        for a in run_algos:
            st = {}
            t0 = time.monotonic()
            pts_a, reason_a = plan_path_3d(h, start, goal, h_obj, algo=a, stats=st, **kw)
            st['total_ms'] = 1000 * (time.monotonic() - t0)
            if pts_a is not None:
                xyz_a = self._pts_to_xyz(pts_a, plane)
                st['final_len_mm'] = float(np.linalg.norm(np.diff(xyz_a, axis=0), axis=1).sum())
                st['n_wp'] = len(pts_a)
            runs[a] = (pts_a, reason_a, st)
        if compare:
            # mode 5 (Comparison): เส้นทางที่ใช้วาด/ส่งหุ่น = algorithm ที่ได้ระยะรวมสั้นที่สุด (เท่ากัน = ค้นหาเร็วกว่า)
            ok_algos = [a for a in run_algos if runs[a][0] is not None]
            algo = (min(ok_algos, key=lambda a: (runs[a][2]['final_len_mm'], runs[a][2].get('search_ms', 0.0)))
                    if ok_algos else 'astar')
            if ok_algos:
                self.get_logger().info(
                    f"Comparison: เส้นทางที่สั้นที่สุด = {PATH_ALGO_LABEL[algo]} "
                    f"({runs[algo][2]['final_len_mm']:.1f} mm) ใช้เป็นเส้นทางหลัก")
        self.last_runs = {
            'mode': mode, 'compare': compare, 'chosen': algo, 'chosen_label': PATH_ALGO_LABEL[algo],
            'algos': [{'key': a, 'label': PATH_ALGO_LABEL[a], 'ok': runs[a][0] is not None,
                       'reason': runs[a][1] or '',
                       'search_ms': round(float(runs[a][2].get('search_ms', 0.0)), 3),
                       'total_ms': round(float(runs[a][2].get('total_ms', 0.0)), 1),
                       'raw_len_mm': round(float(runs[a][2].get('raw_len_mm', 0.0)), 1),
                       'final_len_mm': round(float(runs[a][2].get('final_len_mm', 0.0)), 1),
                       'expanded': int(runs[a][2].get('expanded', 0)),
                       'n_wp': int(runs[a][2].get('n_wp', 0))} for a in run_algos]}
        pts, reason, st_main = runs[algo]
        dt = st_main['total_ms']
        extra['algo'] = algo
        extra['mode'] = mode
        lab = PATH_ALGO_LABEL[algo]
        if len(run_algos) > 1:
            self.log_compare(runs)
            if not self._dyn_running:             # dynamic scan ไม่เซฟกราฟ PNG ทุกวินาที
                self.show_compare(runs)
        if pts is None:
            self.path_result = {'pts': None, 'reason': reason, 'goal': goal, **extra}
            hint = ''
            if goal_src == 'click' and reason in ('goal blocked', 'zmax too low'):
                hint = (" -> จุดวางที่คลิกอยู่ใกล้/บนสิ่งกีดขวางเกินไป (ภายในกรอบส้ม): คลิกจุดใหม่ที่ห่างออกไป "
                        "หรือกด 'a' ใช้ตำแหน่ง default")
            elif obstacle is not None and reason in ('start blocked', 'goal blocked', 'no path'):
                hint = (' -> obstacle ที่ขยายแล้วปิดทาง: ย้ายของให้ห่างจุดหยิบ/จุดวาง/ช่องทางเดินมากขึ้น '
                        'หรือลด path_inflate_extra_mm / เพิ่ม path_zmax_mm')
            self.get_logger().warn(f'path [{lab}]: วางเส้นทาง 3 มิติไม่ได้ ({reason}) ({dt:.0f} ms){hint}')
            return
        # แปลง (u, v, สูงเหนือพื้น) -> พิกัด optical ของกล้อง (mm): Z = ความลึกพื้น - ความสูง
        xyz = self._pts_to_xyz(pts, plane)
        X, Y = xyz[:, 0], xyz[:, 1]
        self.path_result = {'pts': pts, 'xyz': xyz, 'goal': goal, 'h_obj': h_obj, 'reason': '', **extra}
        self.path_plot = np.stack([X, -Y, pts[:, 2]], 1)     # แกนเดียวกับแผนที่ 3 มิติ (Y กลับเครื่องหมาย)
        if self.handeye_M is not None:
            rob = xyz @ self.handeye_M[:, :3].T + self.handeye_M[:, 3]
            self.path_result['robot_xyz'] = rob
            try:
                np.savetxt(self.handeye_csv, rob, delimiter=',', fmt='%.2f', header='x,y,z (robot mm)', comments='')
            except Exception as e:
                self.get_logger().warn(f'เซฟ {self.handeye_csv} ไม่สำเร็จ: {e}')
            self.get_logger().info(
                'path xyz หุ่น (mm): ' + ' -> '.join(f'({p[0]:.1f},{p[1]:.1f},{p[2]:.1f})' for p in rob))
            if self.modbus_enable:
                self.get_logger().info("ตรวจเส้นทางแล้วกด Start บน dashboard (หรือ 's') เพื่อส่งไปหุ่นทาง Modbus")
        self.publish_path_markers(xyz, header)
        txt = ' -> '.join(
            f'({p[0]:.0f},{p[1]:.0f})px h={p[2]:.0f}mm' for p in pts)
        self.get_logger().info(
            f'path {lab} 3D (mode {mode}) {len(pts)} waypoint, ยอดสูงสุด {pts[:, 2].max():.0f} mm เหนือพื้น '
            f'(วัตถุสูง {h_obj:.0f} mm) ระยะรวม {st_main["final_len_mm"]:.1f} mm '
            f'(ค้นหา {st_main["search_ms"]:.1f} ms, รวม {dt:.0f} ms): {txt}')
        self.get_logger().info(
            'path xyz กล้อง (mm): ' + ' -> '.join(f'({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})' for p in xyz))

    def _pts_to_xyz(self, pts, plane):
        """แปลง (u, v, สูงเหนือพื้น) -> พิกัด optical ของกล้อง (mm): Z = ความลึกพื้น - ความสูง"""
        fx, fy, cx, cy = self.intrinsics
        a_, b_, c_ = plane
        zf = 1.0 / np.maximum(a_ * pts[:, 0] + b_ * pts[:, 1] + c_, 1e-9)
        Z = zf - pts[:, 2]
        X = (pts[:, 0] - (cx - self.crop_x0)) * Z / fx
        Y = (pts[:, 1] - (cy - self.crop_y0)) * Z / fy
        return np.stack([X, Y, Z], 1)

    def set_path_mode(self, mode):
        """กดปุ่ม 1-5: เปลี่ยนอัลกอริทึม แล้ววางเส้นทางใหม่จากภาพสแกนเดิม (ถ้ามี)"""
        self.path_mode = mode
        algo = PATH_MODE_ALGO[mode]
        name = 'Comparison เทียบทั้ง 4 แบบ (ใช้เส้นทางที่สั้นที่สุด)' if algo == 'compare' else PATH_ALGO_LABEL[algo]
        self.get_logger().info(f'mode {mode}: {name}')
        if self._scan_snap is None:
            self.get_logger().info('ยังไม่ได้สแกน: จะใช้ mode นี้ตอนกด SPACE')
            return
        self.replan_goal()

    def set_r_return(self, on):
        """ปุ่ม 6 (on=True): หมุน R กลับตามมุม tilt ที่ตำแหน่งที่ 2 | ปุ่ม 7 (on=False): ไม่หมุนกลับ"""
        self.r_return = bool(on)
        self.get_logger().info('ปุ่ม 6: หมุน R กลับตามมุม tilt ที่ตำแหน่งที่ 2 (วัตถุตั้งตรงตอนวาง)' if on
                               else 'ปุ่ม 7: ไม่หมุน R กลับ (คง R ตามมุม tilt ตลอดเส้นทาง)')
        if self._scan_snap is None:
            self.get_logger().info('ยังไม่ได้สแกน: จะใช้ค่านี้ตอนกด SPACE')
            return
        self.replan_goal()

    def mode_text(self):
        algo = PATH_MODE_ALGO.get(self.path_mode, 'astar')
        return f"MODE {self.path_mode}: {'COMPARE ALL' if algo == 'compare' else PATH_ALGO_LABEL[algo]}"

    def log_compare(self, runs):
        """log ตารางเทียบ (mode 5)"""
        self.get_logger().info('เทียบอัลกอริทึม (เวลาค้นหา | node ที่ขยาย | ระยะบน grid | ระยะรวมเส้นทางสุดท้าย):')
        for a in PATH_ALGOS:
            pts_a, reason_a, st = runs[a]
            if pts_a is None:
                self.get_logger().info(f'  {PATH_ALGO_LABEL[a]:<12} ไม่พบเส้นทาง ({reason_a})')
                continue
            self.get_logger().info(
                f"  {PATH_ALGO_LABEL[a]:<12} {st['search_ms']:8.1f} ms | {st.get('expanded', 0):7d} node | "
                f"{st['raw_len_mm']:7.1f} mm | {st['final_len_mm']:7.1f} mm ({st['n_wp']} waypoint)")

    def show_compare(self, runs):
        """mode 5: กราฟแท่งเทียบ (แสดงบน dashboard แล้ว) -> เซฟ PNG ลง dataset เหมือนเดิม (ไม่เปิดหน้าต่าง)"""
        try:
            from matplotlib.figure import Figure       # ใช้ Figure ตรง ๆ ไม่ต้องมี GUI backend
        except ImportError:
            return
        try:
            labels = [PATH_ALGO_LABEL[a] for a in PATH_ALGOS]
            colors = ['#4C72B0', '#DD8452', '#55A868', '#C44E52']
            ok = [runs[a][0] is not None for a in PATH_ALGOS]
            t_ms = [runs[a][2].get('search_ms', 0.0) for a in PATH_ALGOS]
            raw = [runs[a][2].get('raw_len_mm', 0.0) if o else 0.0 for a, o in zip(PATH_ALGOS, ok)]
            fin = [runs[a][2].get('final_len_mm', 0.0) if o else 0.0 for a, o in zip(PATH_ALGOS, ok)]
            nodes = [runs[a][2].get('expanded', 0) for a in PATH_ALGOS]
            ticks = [f'{l}\n({n} nodes)' for l, n in zip(labels, nodes)]
            fig = Figure(figsize=(12, 5.5))
            ax1, ax2 = fig.subplots(1, 2)
            x = np.arange(len(PATH_ALGOS))
            b1 = ax1.bar(x, t_ms, color=colors)
            ax1.set_title('Computational time (search only)')
            ax1.set_ylabel('Time (ms)')
            ax1.set_xticks(x)
            ax1.set_xticklabels(ticks, fontsize=9)
            for bar, v in zip(b1, t_ms):
                ax1.annotate(f'{v:.1f}', (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                             ha='center', va='bottom', fontsize=9, xytext=(0, 2), textcoords='offset points')
            w = 0.38
            b2 = ax2.bar(x - w / 2, raw, w, color=colors, alpha=0.45, label='grid path (before smoothing)')
            b3 = ax2.bar(x + w / 2, fin, w, color=colors, label='final path (3D, mm)')
            ax2.set_title('Total distance')
            ax2.set_ylabel('Distance (mm)')
            ax2.set_xticks(x)
            ax2.set_xticklabels(labels, fontsize=9)
            for bars, vals in ((b2, raw), (b3, fin)):
                for bar, v, o in zip(bars, vals, ok):
                    ax2.annotate(f'{v:.1f}' if o else 'no path',
                                 (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                                 ha='center', va='bottom', fontsize=8, xytext=(0, 2), textcoords='offset points')
            ax2.set_ylim(0, max(raw + fin + [1.0]) * 1.28)
            ax2.legend(loc='upper center', ncol=2, fontsize=8)
            fig.suptitle('Path planning: Dijkstra vs A* vs Bidirectional Dijkstra vs Bidirectional A*')
            fig.tight_layout()
            fn = os.path.join(self.dataset_dir, f"compare_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
            fig.savefig(fn, dpi=120)
            self.get_logger().info(f'เซฟกราฟเทียบ: {fn}')
        except Exception as e:      # ไม่ให้การวาดกราฟทำให้โหนดล่ม
            self.get_logger().warn(f'เซฟกราฟเทียบไม่สำเร็จ: {e}')

    def start_yellow(self, W=None):
        """START = วัตถุสีเหลืองชิ้นใหญ่สุดที่มี depth (yellow_results เรียงตามพื้นที่) ; ไม่มี = None
        โหมดโซนเดิม (zone_enable) จะนับเฉพาะชิ้นที่อยู่โซนซ้าย"""
        xl = self.zone_cols(W)[0] if (self.zone_enable and W) else None
        for o in self.yellow_results:
            if o['z_mm'] is None:
                continue
            if xl is not None and o['bbox'][0] + o['bbox'][2] / 2.0 >= xl:
                continue
            return o
        return None

    def default_goal(self, W, H):
        """ตำแหน่ง GOAL ตอนไม่ได้คลิก (หรือกด 'a')"""
        gxf = self.path_goal_x_frac
        if gxf >= 0:
            gx = gxf * W
        elif self.zone_enable:
            gx = (self.zone_cols(W)[1] + W) / 2.0
        else:
            gx = 0.9 * W
        return (gx, self.path_goal_y_frac * H)

    def mask_yellow_depth(self, depth):
        """
        ตัดวัตถุสีเหลืองออกจากแผนที่: depth ในพื้นที่ contour (ขยายขอบ map_yellow_margin_px) = 0 (ถือว่าไม่มีข้อมูล)
        คืน (depth ใหม่, จำนวนชิ้นที่ตัด) ; ไม่แก้ array เดิม
        """
        objs = [o for o in self.yellow_results if o.get('contour') is not None]
        if self.map_exclude_yellow == 'off' or not objs:
            return depth, 0
        if self.map_exclude_yellow == 'start':
            so = self.start_yellow(depth.shape[1])
            objs = [so] if so is not None and so.get('contour') is not None else []
            if not objs:
                return depth, 0
        m = np.zeros(depth.shape[:2], np.uint8)
        cv2.drawContours(m, [o['contour'] for o in objs], -1, 1, thickness=cv2.FILLED)
        k = max(int(self.map_yellow_margin_px), 0)
        if k > 0:
            m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
        out = depth.copy()
        out[m > 0] = 0
        return out, len(objs)

    def draw_start_label(self, img):
        """ป้าย START ที่วัตถุสีเหลืองชิ้นที่ใช้เป็นจุดเริ่ม (ภาพสด)"""
        o = self.start_yellow(img.shape[1])
        if o is None:
            return
        x, y, bw, bh = o['bbox']
        c = (int(x + bw / 2), int(y + bh / 2))
        cv2.drawMarker(img, c, (0, 255, 255), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
        org = (max(x, 2), min(y + bh + 14, img.shape[0] - 4))
        cv2.putText(img, 'START', org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, 'START', org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    def draw_goal_preview(self, img):
        """ยังไม่มีเส้นทาง: โชว์ตำแหน่ง GOAL (ที่คลิก หรือ default) ให้เห็นก่อนสแกน"""
        if self.path_result is not None:
            return                                       # มีเส้นทางแล้ว draw_path วาดจุดวางเอง
        H, W = img.shape[:2]
        clicked = self.goal_user is not None
        g = self.goal_user if clicked else self.default_goal(W, H)
        gu = (int(round(g[0])), int(round(g[1])))
        mag = (255, 0, 255)
        cv2.drawMarker(img, gu, mag, cv2.MARKER_TILTED_CROSS, 16, 2 if clicked else 1, cv2.LINE_AA)
        lab = 'GOAL' if clicked else 'GOAL (default)'
        lx = gu[0] + 9 if gu[0] + 9 + 7 * len(lab) < W else gu[0] - 9 - 7 * len(lab)
        ly = max(gu[1] - 9, 10)
        cv2.putText(img, lab, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, lab, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.36, mag, 1, cv2.LINE_AA)

    def replan_goal(self):
        """เปลี่ยนจุดวาง (คลิก / กด 'a'): วางเส้นทางใหม่จากภาพที่สแกนไว้ล่าสุด โดยไม่ต้องสแกนซ้ำ"""
        snap = self._scan_snap
        if snap is None:
            self.get_logger().info('จุดวางเปลี่ยนแล้ว: ยังไม่ได้สแกน จะใช้ตอนกด SPACE')
            return
        saved = self.yellow_results
        self.yellow_results = snap['yellow']             # ใช้ผลตรวจวัตถุตอนสแกน ไม่ใช่เฟรมสดที่อาจขยับไปแล้ว
        try:
            self.plan_path_3d(snap['depth'], snap['header'])
        finally:
            self.yellow_results = saved
        if self.plot3d_full and self.intrinsics is not None:
            d = snap['depth']
            fx_, fy_, cx_, cy_ = self.intrinsics
            self.show_map3d(d, (fx_, fy_, cx_ - self.crop_x0, cy_ - self.crop_y0),
                            zones=self.zone_cols(d.shape[1]) if self.zone_enable else None)

    def compute_grip(self, o):
        """
        มุมเอียงของวัตถุสีเหลือง -> ค่า R ที่ต้องบวก/ลบให้หัวจับ (องศา)
          1) minAreaRect บน contour -> tilt จากแกนภาพ (-45..45)
          2) แปลงเป็นมุมในกรอบหุ่น: หมุนเวกเตอร์ขอบวัตถุและเวกเตอร์แกน x ของภาพด้วยส่วนหมุนของ hand-eye
             แล้ววัดมุมต่าง (รวมการกลับด้านถ้า hand-eye กลับมือ) ; บวก = ทวนเข็มรอบแกน Z ของหุ่น
          3) R = grip_r_sign x มุมนั้น ; เอียงน้อยกว่า grip_r_min_deg = 0 (ไม่หมุน)
        """
        g = contour_rect_tilt(o.get('contour'))
        if g is None:
            return None
        g['robot_deg'], g['r_delta'] = None, 0.0
        if self.handeye_M is not None:
            Rm = self.handeye_M[:, :3]
            a = math.radians(g['tilt'])
            v_obj = Rm @ np.array([math.cos(a), math.sin(a), 0.0])
            v_ref = Rm @ np.array([1.0, 0.0, 0.0])
            d = math.degrees(math.atan2(v_obj[1], v_obj[0]) - math.atan2(v_ref[1], v_ref[0]))
            d = ((d + 45.0) % 90.0) - 45.0                    # ซ้ำทุก 90 องศา -> หมุนน้อยที่สุด
            g['robot_deg'] = d
            r = self.grip_r_sign * d
            if self.grip_r_enable and abs(r) >= self.grip_r_min_deg:
                g['r_delta'] = float(max(-self.grip_r_max_deg, min(self.grip_r_max_deg, r)))
        log = self.get_logger()
        if g['robot_deg'] is None:
            log.info(f"grip: วัตถุเอียง {g['tilt']:+.1f}° จากแกนภาพ (ไม่มี hand-eye จึงไม่คำนวณ R)")
        else:
            log.info(f"grip: สี่เหลี่ยม {g['rw']:.0f} x {g['rh']:.0f} px เอียง {g['tilt']:+.1f}° จากแกนภาพ "
                     f"-> {g['robot_deg']:+.1f}° ในกรอบหุ่น -> สั่ง R {g['r_delta']:+.1f}° "
                     f"{'(เอียงน้อยกว่า ' + format(self.grip_r_min_deg, 'g') + '° ไม่หมุน)' if g['r_delta'] == 0 and self.grip_r_enable else ''}"
                     f"{'(ปิด grip_r_enable)' if not self.grip_r_enable else ''}")
        return g

    def build_path_obstacle(self, depth_full, o, h, fz, plane, intr, h_obj, z_floor_s, extra):
        """
        solid ของวัตถุสีเหลือง -> ขยาย solid ของ obstacle ด้วย กว้าง/ยาว/สูง ของวัตถุ -> แผนที่สิ่งกีดขวางสำหรับ A*
        คืน (obstacle (H, W) mm, extra dict สำหรับวาด/เก็บใน path_result)
        """
        x, y, bw, bh = o['bbox']
        H, W = depth_full.shape[:2]
        w_mm, l_mm = float(o['w_mm']), float(o['h_mm'])        # กรอบวัตถุ (แกนภาพ) : เลื่อนอย่างเดียว ไม่หมุน
        ex, mode = self.path_inflate_extra_mm, self.path_inflate_mode
        log = self.get_logger()
        t0 = time.monotonic()

        pad = 4
        dm = depth_full.copy()
        dm[max(y - pad, 0):y + bh + pad, max(x - pad, 0):x + bw + pad] = 0   # ตัดวัตถุที่จะหยิบออกจาก obstacle
        sp = dict(self.solid_params)                            # รูปร่าง obstacle เฉพาะตอนวางเส้นทาง (ไม่กระทบ solid ที่แสดง)
        sp['shape_mode'], sp['poly_eps'] = self.path_shape_mode, self.path_poly_eps
        solids, _, _ = build_solids(dm, intr, sp)

        footprint = None                                        # รูปร่างจริงของวัตถุ (hull) สัมพัทธ์กับจุด TCP = กึ่งกลางกรอบ
        if o.get('contour') is not None:
            hull = cv2.convexHull(o['contour']).reshape(-1, 2).astype(np.float32)
            footprint = {'pts': hull - np.array([x + bw / 2.0, y + bh / 2.0], np.float32),
                         'z': float(o['z_mm'])}
            extra['footprint_px'] = footprint['pts']
            g = extra.get('grip') or {}
            if extra.get('r_return') and abs(float(g.get('r_delta', 0.0))) > 0:
                # ปุ่ม 6: วัตถุถูกหมุนกลับ (-tilt บนภาพ) ที่ตำแหน่งที่ 2 -> ใช้รูปร่างที่กวาดตั้งแต่มุมเดิมถึงมุมใหม่
                # (convex hull ของรูปร่างทุก 5 องศา) ขยาย obstacle ตลอดเส้นทาง กันชนทั้งตอนหมุนและหลังหมุน
                tilt = float(g['tilt'])
                fp0 = footprint['pts']
                n_rot = max(int(math.ceil(abs(tilt) / 5.0)), 1)
                rots = [rotate_pts_deg(fp0, -tilt * i / n_rot) for i in range(n_rot + 1)]
                swept = cv2.convexHull(np.vstack(rots).astype(np.float32)).reshape(-1, 2).astype(np.float32)
                footprint = {'pts': swept, 'z': footprint['z']}
                extra['footprint_goal_px'] = rots[-1]               # รูปร่างวัตถุตอนวาง (ตั้งตรงกับแกนภาพแล้ว)
                if mode == 'box':                                  # กล่องแกนภาพต้องครอบรูปร่างที่กวาดด้วย
                    w_mm = float(np.ptp(swept[:, 0])) * footprint['z'] / intr[0]
                    l_mm = float(np.ptp(swept[:, 1])) * footprint['z'] / intr[1]
                log.info(f"path: [ปุ่ม 6] หมุน R กลับ {-g['r_delta']:+.1f}° ที่ตำแหน่งที่ 2 "
                         f"(วัตถุบนภาพหมุน {-tilt:+.1f}°) -> ขยาย obstacle ด้วยรูปร่างที่กวาดระหว่างหมุน")

        yellow = None
        try:
            yellow = estimate_yellow_solid(o, plane, fz, intr, self.solid_params)
        except Exception as e:
            log.warn(f'path: ประมาณ solid วัตถุสีเหลืองไม่ได้ ({e})')
        if yellow is not None:
            log.info(f"path: solid วัตถุสีเหลือง = {yellow['kind']} {yellow['w_mm']:.1f} x {yellow['l_mm']:.1f} x "
                     f"{h_obj:.1f} mm (กว้าง x ยาว x สูง) ; ขยาย obstacle ด้วยกรอบ {w_mm:.1f} x {l_mm:.1f} mm "
                     f"[{mode}{'' if footprint is not None or mode != 'shape' else ' -> ไม่มี contour ใช้ box'}] "
                     f"+ เผื่อ {ex:.1f} mm/ด้าน")

        obs, infl = inflate_solids(solids, fz, (H, W), intr, w_mm, l_mm, h_obj, ex, mode, footprint)
        for i, f in enumerate(infl, 1):
            log.info(f"path: obstacle #{i} {f['kind']} {f['w']:.0f} x {f['l']:.0f} x {f['h']:.0f} mm "
                     f"-> ขยายเป็น {f['W']:.0f} x {f['L']:.0f} x {f['H']:.0f} mm")
        if not infl:
            log.info('path: ไม่พบ solid ของ obstacle (บนภาพที่ crop)')
        if self.path_residual_raw:
            res, n_res = residual_obstacle(
                h, solids, self.solid_params['min_height_mm'], intr, w_mm, l_mm, h_obj, z_floor_s,
                ex, mode, keep_out=(x, y, bw, bh), footprint=footprint)
            if res is not None:
                obs = np.maximum(obs, res)
                log.info(f'path: มีสิ่งกีดขวางที่ไม่ได้เป็น solid {n_res} px (เล็ก/เตี้ยเกินไป) นับจาก height map และขยายด้วย')

        extra['infl_polys'] = [f['poly_infl'].round().astype(np.int32).reshape(-1, 1, 2)
                               for f in infl if len(f['poly_infl']) >= 3]
        if self.plot3d_full:       # แผนที่ 3 มิติแสดงทั้งภาพ จึงแสดง solid ของ obstacle ทุกโซนที่ใช้วางเส้นทางจริง
            vis = list(solids)
            if yellow is not None:
                vis.append(yellow)
            if self.path_show_inflated:
                for f, sol in zip(infl, solids):
                    if len(f['poly_infl']) < 3:
                        continue
                    big = extrude(f['poly_infl'], f['z_top'] - h_obj, plane, intr)
                    big.update(kind='inflated', height_mm=f['H'], color_idx=0, z_top=f['z_top'] - h_obj,
                               line_color=(1.0, 0.45, 0.0), line_style='dashed', line_width=1.2)
                    vis.append(big)
            self.last_solids = vis
        log.info(f'path: สร้าง obstacle ที่ขยายแล้ว ({1000 * (time.monotonic() - t0):.0f} ms)')
        return obs, extra

    def send_path_modbus(self):
        """กด 's': ส่ง waypoint พิกัดหุ่นล่าสุดทาง Modbus (รันใน thread ไม่ให้หน้าต่างค้าง)"""
        if not self.modbus_enable:
            self.get_logger().warn('ยังไม่เปิด Modbus (ตั้ง -p modbus_enable:=true)')
            return
        r = self.path_result
        if r is None or r.get('pts') is None or r.get('robot_xyz') is None:
            self.get_logger().warn('ยังไม่มีเส้นทางพิกัดหุ่น: ต้องตั้ง handeye_file แล้วกด SPACE สแกนก่อน')
            return
        if self._modbus_busy:
            self.get_logger().warn('กำลังส่ง Modbus อยู่')
            return
        try:
            vals = encode_waypoints(r['robot_xyz'], self.modbus_scale)
        except ValueError as e:
            self.get_logger().error(f'ส่ง Modbus ไม่ได้: {e}')
            return
        r_delta = float((r.get('grip') or {}).get('r_delta', 0.0))     # ไม่มีข้อมูลมุม = 0 (ต้องเขียนเสมอ กันค่าเก่าค้าง)
        r_val = int(round(r_delta * self.modbus_r_scale))
        if not -32768 <= r_val <= 32767:
            self.get_logger().error(f'ส่ง Modbus ไม่ได้: R {r_delta:.1f}° x{self.modbus_r_scale:g} เกินช่วง int16')
            return
        if self.modbus_data_reg <= self.modbus_r_reg < self.modbus_data_reg + len(vals):
            self.get_logger().error(
                f'ส่ง Modbus ไม่ได้: modbus_r_reg ({self.modbus_r_reg}) ชนกับพื้นที่ waypoint '
                f'({self.modbus_data_reg}..{self.modbus_data_reg + len(vals) - 1}) -> ตั้ง modbus_data_reg ให้เริ่มหลัง reg นี้ (เช่น 20)')
            return
        rret = 1 if r.get('r_return') else 0              # ใช้ค่าตอนวางเส้นทางนี้ (เส้นทางวางมาสำหรับแบบนั้น)
        if self.modbus_rret_reg >= 0:
            if (self.modbus_data_reg <= self.modbus_rret_reg < self.modbus_data_reg + len(vals)
                    or self.modbus_rret_reg in (self.modbus_r_reg, self.modbus_cmd_reg)):
                self.get_logger().error(
                    f'ส่ง Modbus ไม่ได้: modbus_rret_reg ({self.modbus_rret_reg}) ชนกับ register อื่น '
                    f'(waypoint {self.modbus_data_reg}..{self.modbus_data_reg + len(vals) - 1}, '
                    f'R {self.modbus_r_reg}, cmd {self.modbus_cmd_reg})')
                return
        self._modbus_busy = True
        threading.Thread(target=self._modbus_worker, args=(vals, r_val & 0xFFFF, r_delta, rret), daemon=True).start()

    def _modbus_worker(self, vals, r_val=0, r_delta=0.0, rret=0):
        try:
            with socket.create_connection((self.modbus_ip, self.modbus_port), timeout=3.0) as sock:
                if self.modbus_rret_reg >= 0:
                    self._modbus_tid = (self._modbus_tid + 1) % 65536
                    modbus_write_reg(sock, self._modbus_tid, self.modbus_unit, self.modbus_rret_reg, rret)
                self._modbus_tid = (self._modbus_tid + 1) % 65536
                modbus_write_reg(sock, self._modbus_tid, self.modbus_unit, self.modbus_r_reg, r_val)
                self._modbus_tid = (self._modbus_tid + 1) % 65536
                modbus_write_regs(sock, self._modbus_tid, self.modbus_unit, self.modbus_data_reg, vals)
                self._modbus_tid = (self._modbus_tid + 1) % 65536
                modbus_write_reg(sock, self._modbus_tid, self.modbus_unit, self.modbus_cmd_reg, self.modbus_run_cmd)
            self.modbus_last = {'ok': True, 't': datetime.now().strftime('%H:%M:%S'),
                                'msg': f'ส่ง {vals[0]} waypoint แล้ว'}
            self._await_robot = {'t0': time.monotonic(), 'seen': False}   # หยุด dynamic scan จนหุ่นทำงานเสร็จ
            self.get_logger().info(
                f'ส่ง {vals[0]} waypoint + R {r_delta:+.1f}° '
                f"({'หมุนกลับที่ตำแหน่งที่ 2' if rret else 'ไม่หมุนกลับ'}) ไป {self.modbus_ip}:{self.modbus_port} แล้ว "
                f'(R reg {self.modbus_r_reg}, หมุนกลับ reg {self.modbus_rret_reg}={rret}, '
                f'data reg {self.modbus_data_reg}.., สั่งรัน reg {self.modbus_cmd_reg}={self.modbus_run_cmd})')
        except Exception as e:
            self.modbus_last = {'ok': False, 't': datetime.now().strftime('%H:%M:%S'), 'msg': str(e)}
            self.get_logger().error(f'ส่ง Modbus ไม่สำเร็จ: {e}')
        finally:
            self._modbus_busy = False

    def path_note(self):
        """ข้อความสรุปเส้นทางสำหรับชื่อกราฟ 3 มิติ"""
        r = self.path_result
        if r is None:
            return None
        if r['pts'] is None:
            return f"NO PATH: {r['reason']}"
        lab = PATH_ALGO_LABEL.get(r.get('algo', 'astar'), 'A*')
        return f"{lab} 3D path (mode {r.get('mode', PATH_DEFAULT_MODE)}): {len(r['pts'])} waypoints, peak {r['pts'][:, 2].max():.0f} mm"

    def publish_path_markers(self, xyz_mm, header):
        """publish เส้นทาง 3 มิติเป็น MarkerArray (เส้น + จุด waypoint) frame เดียวกับ solid"""
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = header.frame_id
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        pts = [Point(x=float(p[0]) * 0.001, y=float(p[1]) * 0.001, z=float(p[2]) * 0.001)
               for p in xyz_mm]
        for mid, mtype, scale in ((0, Marker.LINE_STRIP, 0.003), (1, Marker.SPHERE_LIST, 0.007)):
            m = Marker()
            m.header.frame_id = header.frame_id
            m.header.stamp = Time()
            m.ns, m.id, m.type, m.action = 'path', mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = scale
            if mtype == Marker.SPHERE_LIST:
                m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
            m.points = pts
            arr.markers.append(m)
        self.path_pub.publish(arr)

    def draw_path(self, img, info):
        """วาดเส้นทางที่สแกนไว้ล่าสุด (ม่วง, มองจากด้านบน) + จุดวาง ; ไม่มีเส้นทาง = ข้อความแดง"""
        if info is None:
            return
        mag, red = (255, 0, 255), (0, 0, 255)
        gx, gy = int(info['goal'][0]), int(info['goal'][1])
        for pl in info.get('infl_polys') or []:                       # ขอบ obstacle ที่ขยายแล้ว (ส้ม)
            cv2.polylines(img, [pl], True, (0, 140, 255), 1, cv2.LINE_AA)
        bx, st, fp = info.get('box_px'), info.get('start'), info.get('footprint_px')
        if fp is not None and st is not None:                         # รูปร่างวัตถุที่จุดหยิบ/จุดวาง (ฟ้า)
            fpg = info.get('footprint_goal_px')                       # ปุ่ม 6: ที่จุดวางวัตถุถูกหมุนกลับแล้ว
            for cxy, f_ in ((st, fp), (info['goal'], fp if fpg is None else fpg)):
                cv2.polylines(img, [np.round(f_ + np.array(cxy, np.float32)).astype(np.int32).reshape(-1, 1, 2)],
                              True, (255, 255, 0), 1, cv2.LINE_AA)
        elif bx is not None and st is not None:
            for cxy in (st, info['goal']):
                x0, y0 = int(cxy[0] - bx[0] / 2.0), int(cxy[1] - bx[1] / 2.0)
                cv2.rectangle(img, (x0, y0), (x0 + int(bx[0]), y0 + int(bx[1])), (255, 255, 0), 1, cv2.LINE_AA)
        g = info.get('grip')
        if g is not None:                                              # เส้นกึ่งกลาง (แดง) + สี่เหลี่ยมที่ fit (น้ำเงิน) + มุม
            cxg, cyg = int(round(g['center'][0])), int(round(g['center'][1]))
            Lg = 22
            cv2.line(img, (cxg - Lg, cyg), (cxg + Lg, cyg), red, 1, cv2.LINE_AA)
            cv2.line(img, (cxg, cyg - Lg), (cxg, cyg + Lg), red, 1, cv2.LINE_AA)
            cv2.polylines(img, [np.round(g['box']).astype(np.int32).reshape(-1, 1, 2)], True, (255, 0, 0), 1, cv2.LINE_AA)
            lab = f"tilt {g['tilt']:+.1f}" + (f" R{g['r_delta']:+.1f}" if g.get('robot_deg') is not None else '')
            # ป้าย START อยู่ใต้กรอบวัตถุ (ชิดขอบล่าง +14) จึงวางข้อความมุมไว้ต่ำกว่านั้นอีก ไม่ให้ซ้อนกัน
            org = (max(cxg - 28, 2), int(min(float(g['box'][:, 1].max()) + 28, img.shape[0] - 6)))
            cv2.putText(img, lab, org, cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(img, lab, org, cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 160, 0), 1, cv2.LINE_AA)
        cv2.drawMarker(img, (gx, gy), mag, cv2.MARKER_TILTED_CROSS, 16, 2, cv2.LINE_AA)
        gl = 'GOAL' if info.get('goal_src') == 'click' else 'GOAL (default)'
        gl_x = gx + 9 if gx + 9 + 7 * len(gl) < img.shape[1] else gx - 9 - 7 * len(gl)   # ชิดขอบขวา -> เขียนทางซ้าย
        cv2.putText(img, gl, (gl_x, max(gy - 9, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, gl, (gl_x, max(gy - 9, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.36, mag, 1, cv2.LINE_AA)
        if info['pts'] is not None:
            p = info['pts']
            poly = p[:, :2].round().astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [poly], False, mag, 2, cv2.LINE_AA)
            for (u, v), z in zip(poly.reshape(-1, 2), p[:, 2]):
                cv2.circle(img, (int(u), int(v)), 3, mag, -1, cv2.LINE_AA)
            txt = f"{PATH_ALGO_LABEL.get(info.get('algo', 'astar'), 'A*')} 3D peak {p[:, 2].max():.0f}mm"
            col = mag
        else:
            txt = f"NO PATH: {info['reason']}"
            col = red
        cv2.putText(img, txt, (4, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (4, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, col, 1, cv2.LINE_AA)

    def save_frame(self, color, depth, header=None, save=True):
        """บันทึกภาพ color (PNG) และ depth (16-bit PNG, หน่วย mm) ลง dataset_dir แล้วทำ pipeline สแกน
        save=False (dynamic scan): ไม่เขียนไฟล์ ทำแค่ depth colormap + point cloud / solid / เส้นทาง / แผนที่ 3 มิติ"""
        if not save:
            depth_color, near_mm, far_mm = colorize_depth(
                depth, self.depth_near_mm, self.depth_far_mm, self.colormap)
            self.show_depth_colormap(add_depth_legend(depth_color, near_mm, far_mm, self.colormap))
            self.publish_cloud(color, depth, depth_color, header)
            return
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]  # ถึงระดับ ms
        color_path = os.path.join(self.dataset_dir, f'color_{stamp}.png')
        depth_path = os.path.join(self.dataset_dir, f'depth_{stamp}.png')

        depth_color_path = os.path.join(self.dataset_dir, f'depth_color_{stamp}.png')

        ok_color = cv2.imwrite(color_path, color)
        ok_depth = cv2.imwrite(depth_path, depth)  # uint16 -> PNG 16-bit ไม่สูญเสียข้อมูล

        # สร้างภาพ depth แบบระบายสีตามความลึก (พื้น = เข้มสุด, ยิ่งสูงยิ่งสว่าง)
        depth_color, near_mm, far_mm = colorize_depth(
            depth, self.depth_near_mm, self.depth_far_mm, self.colormap
        )
        depth_vis = add_depth_legend(depth_color, near_mm, far_mm, self.colormap)
        ok_vis = cv2.imwrite(depth_color_path, depth_vis)
        self.show_depth_colormap(depth_vis)
        self.publish_cloud(color, depth, depth_color, header)

        if ok_color and ok_depth and ok_vis:
            # self.save_msg = f'Saved: {stamp}'
            self.save_msg = ''
            self.get_logger().info(
                f'บันทึกภาพแล้ว: {color_path} , {depth_path} , {depth_color_path} '
                f'(ช่วงสี {near_mm / 1000.0:.2f}-{far_mm / 1000.0:.2f} m)'
            )
        else:
            self.save_msg = 'Save FAILED'
            self.get_logger().error(f'บันทึกภาพไม่สำเร็จ ({self.dataset_dir})')
        self.save_msg_until = time.monotonic() + 1.0

    def publish_cloud(self, color, depth, depth_color, header):
        """สร้างและ publish point cloud จากเฟรมที่แคป (สีตาม param cloud_color)"""
        if self.intrinsics is None or header is None:
            self.get_logger().warn(
                'ยังไม่ได้รับ camera_info จึงยังสร้าง point cloud ไม่ได้ (เช็ค info_topic)'
            )
            return
        src = color if self.cloud_color == 'rgb' else depth_color
        if src.shape[:2] != depth.shape[:2]:
            self.get_logger().error(
                'ขนาด color ไม่ตรงกับ depth: ต้องใช้ aligned_depth_to_color เท่านั้น'
            )
            return
        # ชดเชยการ crop: จุดกึ่งกลางภาพ (cx, cy) ย้ายตาม offset ของกรอบ
        fx, fy, cx, cy = self.intrinsics
        depth, n_ex = self.mask_yellow_depth(depth)      # วัตถุสีเหลืองไม่นำมาทำแผนที่ (cloud/solid/3D map/grid/A*)
        if n_ex:
            self.get_logger().info(
                f'map: ตัดวัตถุสีเหลือง {n_ex} ชิ้นออกจากแผนที่ (map_exclude_yellow={self.map_exclude_yellow}, '
                f'ขอบ +{self.map_yellow_margin_px}px)')
            if self.map_exclude_yellow == 'all' and n_ex > 1:
                self.get_logger().warn(
                    f'map: พบวัตถุสีเหลือง {n_ex} ชิ้น: ชิ้นใหญ่สุดเป็น START ส่วนชิ้นอื่นถูกตัดออกจากแผนที่ด้วย '
                    '(A* จะไม่หลบ) -> ใช้ -p map_exclude_yellow:=start ถ้าต้องการให้ชิ้นอื่นเป็นสิ่งกีดขวาง')
        depth_full = depth      # ทั้งภาพที่ crop (ไว้วางเส้นทาง) ; ถ้าเปิดโซนจะตัดเหลือโซน scan ด้านล่าง
        zx0 = 0
        if self.zone_enable:
            # scan map เฉพาะโซนกลาง: ตัดคอลัมน์ซ้าย/ขวาออก แล้วเลื่อน cx ตาม
            zx0, zx1 = self.zone_cols(depth.shape[1])
            depth = np.ascontiguousarray(depth[:, zx0:zx1])
            src = np.ascontiguousarray(src[:, zx0:zx1])
        intrinsics = (fx, fy, cx - self.crop_x0 - zx0, cy - self.crop_y0)
        t0 = time.monotonic()
        msg = depth_to_cloud_msg(
            depth, src, intrinsics, header,
            max_depth_m=self.cloud_max_depth_m, stride=self.cloud_stride,
        )
        self.cloud_pub.publish(msg)
        self.get_logger().info(
            f'publish point cloud {msg.width} จุด '
            f'(frame: {header.frame_id}, {1000 * (time.monotonic() - t0):.0f} ms)'
        )

        self.publish_solids(depth, intrinsics, header)
        self._scan_snap = {'depth': depth_full.copy(), 'header': header, 'yellow': list(self.yellow_results)}
        self.plan_path_3d(depth_full, header)   # A* 3 มิติ หลังสแกน 3D map
        if self.plot3d_full:
            # แผนที่ 3 มิติครอบคลุมทั้งภาพที่ crop (จุดวางวัตถุ + scan map + จุดวางของ) เพื่อให้เห็นเส้นทางตลอดทาง
            fx_, fy_, cx_, cy_ = self.intrinsics
            self.show_map3d(
                depth_full, (fx_, fy_, cx_ - self.crop_x0, cy_ - self.crop_y0),
                zones=self.zone_cols(depth_full.shape[1]) if self.zone_enable else None)
        else:
            self.show_map3d(depth, intrinsics)

        t1 = time.monotonic()
        grid = depth_to_grid_msg(
            depth, intrinsics, header,
            self.depth_near_mm, self.depth_far_mm,
            resolution=self.grid_resolution, max_depth_m=self.cloud_max_depth_m,
        )
        if grid is None:
            self.get_logger().warn('ไม่มีจุด depth ที่ใช้ได้ จึงยังสร้าง grid ไม่ได้')
            return
        self.last_grid = grid
        self.grid_pub.publish(grid)
        self.get_logger().info(
            f'publish grid {grid.info.width}x{grid.info.height} ช่อง '
            f'(frame: {header.frame_id}, {1000 * (time.monotonic() - t1):.0f} ms)'
        )

    def publish_topview_tf(self, floor_m, parent):
        """publish static TF: parent(optical) -> topview (หมุน 180 องศารอบ x, ยกไปที่ระดับพื้น)"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = self.topview_frame
        t.transform.translation.z = float(floor_m)
        # R = Rx(180 องศา) * Rz(yaw)  ->  quaternion (x, y, z, w) = (cos(yaw/2), -sin(yaw/2), 0, 0)
        half = math.radians(self.topview_yaw_deg) / 2.0
        t.transform.rotation.x = math.cos(half)
        t.transform.rotation.y = -math.sin(half)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0
        self.tf_static.sendTransform(t)

    def publish_solids(self, depth, intrinsics, header):
        """แยกวัตถุจาก depth, fit รูปทรง, extrude แล้ว publish เป็น MarkerArray ให้ RViz"""
        self.last_solids = []
        self._store_img('seg', None)        # ล้างภาพเก่า: ถ้ารอบนี้แยกวัตถุไม่สำเร็จจะไม่แสดงผลของรอบก่อน
        if not self.solid_enable:
            return
        t0 = time.monotonic()
        try:
            solids, floor_rect, dbg = build_solids(depth, intrinsics, self.solid_params)
        except Exception as e:  # ไม่ให้ pipeline นี้ทำให้โหนดล่ม
            self.get_logger().error(f'สร้าง solid ไม่สำเร็จ: {e}')
            return
        self.last_solids = solids
        if self.topview_enable and floor_rect is not None:
            self.publish_topview_tf(float(floor_rect[:, 2].mean()), header.frame_id)
        if not solids:
            self.get_logger().warn(
                'ไม่พบวัตถุเหนือพื้น (ลองลด solid_min_height_mm หรือ solid_min_area_px)'
            )
        msg = solids_to_markers(
            solids, header,
            show_floor_rect=floor_rect if self.solid_show_floor else None,
            alpha=self.solid_alpha,
            show_labels=self.solid_show_labels, label_size=self.solid_label_size,
        )
        self.solid_pub.publish(msg)
        summary = ', '.join(f"{s['kind']}({s['height_mm']:.0f}mm)" for s in solids)
        self.get_logger().info(
            f'publish solids {len(solids)} ชิ้น [{summary}] ({1000 * (time.monotonic() - t0):.0f} ms)'
        )
        if dbg is not None:                 # "Solids (segmentation)" -> ปุ่ม Segmentation บน dashboard
            sc = self.display_scale if self.display_scale > 0 else 1.0
            self._store_img('seg', cv2.resize(dbg, None, fx=sc, fy=sc, interpolation=cv2.INTER_NEAREST))

    def republish_grid(self):
        if self.last_grid is not None:
            self.grid_pub.publish(self.last_grid)

    def show_map3d(self, depth, intrinsics, zones=None):
        """แผนที่ความสูง 3 มิติ (surface + โครงขอบ solid + เส้นทาง) -> JSON ให้ปุ่ม 3D Voxel Map บน dashboard"""
        if self.map3d is None:
            return
        t0 = time.monotonic()
        try:
            data = self.map3d.build(
                depth, intrinsics, self.solid_params,
                self.last_solids if self.plot3d_show_solids else [],
                path=self.path_plot, zones=zones, path_note=self.path_note(),
            )
            if data is not None:
                self._map3d_json = json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
                self._ver['map3d'] += 1
        except Exception as e:  # ไม่ให้การวาดกราฟทำให้โหนดล่ม
            self.get_logger().error(f'สร้างแผนที่ 3 มิติไม่สำเร็จ: {e}')
            return
        if data is not None:
            self.get_logger().info(
                f'อัปเดตแผนที่ 3 มิติ ({1000 * (time.monotonic() - t0):.0f} ms, {len(self._map3d_json) // 1024} kB)')
        else:
            self.get_logger().warn('หาระนาบพื้นไม่ได้ จึงยังวาดแผนที่ 3 มิติไม่ได้')

    def show_depth_colormap(self, depth_vis):
        """ภาพ depth สีที่แคปไว้ ("Depth colormap (captured)") -> ปุ่ม Depth Image บน dashboard (ค้างจนกว่าจะ Scan ใหม่)"""
        scale = self.display_scale
        if scale != 1.0 and scale > 0:
            depth_vis = cv2.resize(
                depth_vis, None, fx=scale, fy=scale,
                interpolation=cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA
            )
        self._store_img('depth', depth_vis)

    def display_callback(self):
        with self.frame_lock:
            color = self.latest_color
            depth = self.latest_depth
            header = self.latest_header

        if color is None or depth is None:
            self.process_commands(None, None, None)   # เลือก algorithm / mode ได้แม้ยังไม่มีภาพ
            return

        display_img = color.copy()

        # แปลงตำแหน่งเมาส์ (บนภาพที่ resize แล้ว) กลับเป็นพิกัดจริงบนภาพต้นฉบับ
        scale = self.display_scale
        mouse_on = self.mouse_x >= 0 and self.mouse_y >= 0
        orig_x = int(self.mouse_x / scale) if scale != 0 else self.mouse_x
        orig_y = int(self.mouse_y / scale) if scale != 0 else self.mouse_y
        fc = self.update_finger_cursor()
        if fc is not None:                  # ชูนิ้วชี้: เครื่องหมายบวกสีเขียวตามปลายนิ้วแทนเมาส์
            mouse_on, orig_x, orig_y = True, int(round(fc[0])), int(round(fc[1]))

        h, w = depth.shape[:2]
        if mouse_on and 0 <= orig_x < w and 0 <= orig_y < h:
            depth_value_mm = int(depth[orig_y, orig_x])
            if depth_value_mm > 0:
                depth_text = f'{depth_value_mm / 1000.0:.2f} m'
            else:
                depth_text = 'no data'
        else:
            depth_text = '-'

        # ตรวจจับวัตถุสีเหลือง + วาด bounding box พร้อมขนาดจริง (mm)
        if self.yellow_enable:
            self.yellow_results, omask = detect_yellow_objects(
                color, depth, self.intrinsics, self.yellow_lo, self.yellow_hi,
                min_area_px=self.yellow_min_area)
            for i, o in enumerate(self.yellow_results):
                x, y, bw, bh = o['bbox']
                cv2.rectangle(display_img, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
                if o['w_mm'] is not None:
                    star = '*' if o['touches_edge'] else ''
                    lines = [f"#{i + 1} {o['w_mm']:.1f} x {o['h_mm']:.1f} mm{star}",
                             f"Z={o['z_mm']:.0f} mm"]
                else:
                    lines = [f'#{i + 1} {bw}x{bh} px (no depth/intrinsics)']
                for k, t in enumerate(lines):
                    ty = max(y - 8 - 20 * (len(lines) - 1 - k), 15)
                    cv2.putText(display_img, t, (x, ty), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(display_img, t, (x, ty), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 255, 255), 1, cv2.LINE_AA)
            self.draw_start_label(display_img)
            if self.yellow_show_mask:                   # หน้าต่างจูน HSV (ดีบัก) ยังเป็นหน้าต่าง OpenCV แยก
                sc_m = self.display_scale if self.display_scale > 0 else 1.0
                cv2.imshow('Yellow mask', cv2.resize(
                    omask, None, fx=sc_m, fy=sc_m, interpolation=cv2.INTER_NEAREST))
                cv2.waitKey(1)

        # เส้นทาง A* 3 มิติที่วางไว้ตอนสแกนล่าสุด (วาดค้างไว้จนกด Scan ใหม่)
        self.draw_path(display_img, self.path_result)
        self.draw_goal_preview(display_img)
        self.draw_hands(display_img)                     # มือที่ MediaPipe เห็น + ป้ายหยุดหุ่น

        # วาด crosshair + ตัวเลขระยะทางที่ตำแหน่งเมาส์ (บนภาพก่อน resize) เฉพาะตอนเมาส์อยู่บนภาพ
        if mouse_on:
            cv2.drawMarker(
                display_img, (orig_x, orig_y), (0, 255, 0),
                markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2
            )
            cv2.putText(
                display_img, depth_text, (orig_x + 15, orig_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA
            )
            cv2.putText(
                display_img, depth_text, (orig_x + 15, orig_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA
            )
            if fc is not None:
                self.draw_dwell(display_img, (orig_x, orig_y))

        # เส้นแบ่งโซน (แดง) + ชื่อโซน
        if self.zone_enable and self.zone_show_lines:
            xl, xr = self.zone_cols(w)
            red = (0, 0, 255)
            cv2.line(display_img, (xl, 0), (xl, h - 1), red, 2, cv2.LINE_AA)
            cv2.line(display_img, (xr, 0), (xr, h - 1), red, 2, cv2.LINE_AA)
            for lines, tx in ((('OBJECT', 'POINT'), 4), (('SCAN MAP',), xl + 6),
                              (('PLACE', 'POINT'), xr + 6)):
                for k, txt in enumerate(lines):
                    ty = 14 + 13 * k
                    cv2.putText(display_img, txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                                0.38, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(display_img, txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                                0.38, red, 1, cv2.LINE_AA)

        # mode อัลกอริทึมที่เลือกอยู่ (มุมขวาบน)
        for row, mt in enumerate((self.mode_text(), 'R: RETURN @WP2 (6)' if self.r_return else 'R: KEEP TILT (7)')):
            (tw, _), _ = cv2.getTextSize(mt, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
            mx, my = max(w - tw - 4, 2), 12 + 13 * row
            cv2.putText(display_img, mt, (mx, my), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(display_img, mt, (mx, my), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

        # แสดงข้อความยืนยันการบันทึกที่มุมซ้ายบนชั่วคราว
        if time.monotonic() < self.save_msg_until:
            cv2.putText(
                display_img, self.save_msg, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv2.LINE_AA
            )
            cv2.putText(
                display_img, self.save_msg, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA
            )

        if scale != 1.0:
            display_img = cv2.resize(
                display_img, None, fx=scale, fy=scale,
                interpolation=cv2.INTER_CUBIC if scale > 1 else cv2.INTER_AREA
            )

        self.publish_frame(display_img)                  # -> Camera Viewer บน dashboard

        # ปุ่ม / ปุ่มลัดบน dashboard (แทน cv2.waitKey เดิม): Scan, Start, algorithm, mode, จุดวาง, ออก
        self.process_commands(color, depth, header)
        self.dynamic_tick(color, depth, header)          # dynamic scan map (หยุดระหว่างหุ่นทำงาน)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_dashboard()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()