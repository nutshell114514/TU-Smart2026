
import gc
import math
import time

import image
import seekfree
import sensor
import tf
from machine import UART
from seekfree import Timer


IMG_W = 320
IMG_H = 240
MODEL_INPUT_SIZE = 96

YOLO_MODEL_PATH = "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite"
YOLO_SCORE_THRESHOLD = 0.75
YOLO_CENTER_SUPPRESS_PIXELS = 20

CLASSES = ["car", "sandbag", "teddy", "tennis"]
CLASS_IDS = {
    "car": 0,
    "sandbag": 1,
    "teddy": 2,
    "tennis": 3,
}
CLASS_COLORS = {
    0: (255, 80, 80),
    1: (255, 200, 0),
    2: (80, 220, 120),
    3: (255, 0, 255),
}

YELLOW_THRESHOLD = [(21, 100, -23, 6, 40, 127)]

FIELD_WIDTH = 310
FIELD_HEIGHT = 230
XY_SCALE_FACTOR = 1

ORIGIN_X = 160
ORIGIN_Y = 239
ANGLE_START = -165
ANGLE_END = -15
RAY_COUNT = 32
STEP = 6
MAX_DIST = 320

MIN_SEG_LENGTH = 30
COLLINEAR_ANGLE = 20
CORNER_BUFFER = 3
TRIM_END = 2

CAR_ID = 1

CAR_CONFIGS = {
    1: {
        "rod_offset_cm": -4.0,
        "qvga_matrix": [
    2.8297872340426786, 0.0186170213, -425.58510638299254,
    0.0, -2.6003510310, 612.1827672123433,
    2.0946616262002204e-16, 0.17198581560284248, 1.0
]


,
    },
    2: {
        "rod_offset_cm": 27.0,
        "qvga_matrix": None,
    },
}

CAR_SCREEN_X = 160
CAR_SCREEN_Y = 239

CAR_CFG = CAR_CONFIGS.get(CAR_ID, CAR_CONFIGS[1])
ROD_OFFSET_CM = CAR_CFG["rod_offset_cm"]
PERSPECTIVE_MATRIX = CAR_CFG["qvga_matrix"]
if PERSPECTIVE_MATRIX is None:
    raise RuntimeError("CAR_ID=%d qvga_matrix is not calibrated yet" % CAR_ID)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# Camera settings from model/摄像头参数设置.py
FIXED_RGB_GAIN = (107.0, 64.0, 91.0)
MY_GAIN = 2.2
RUN_EXPOSURE = 520
sensor.set_auto_whitebal(False, rgb_gain_db=FIXED_RGB_GAIN)
sensor.set_auto_gain(False, gain_db=MY_GAIN)
sensor.set_auto_exposure(False, exposure_us=RUN_EXPOSURE)
sensor.set_saturation(2)
sensor.skip_frames(time=500)

seekfree.init()
clock = time.clock()
uart = UART(12, baudrate=115200)

uart_buf = bytearray()
current_imu_yaw = 0.0
current_imu_x = 0.0
current_imu_y = 0.0

global_vis_yaw = 999.0
global_rod_x = 999.0
global_rod_y = 999.0

t = 0
send_flag = 0
N = 0
n = 0
TH_YAW = 10
vis_yaw_calc = None


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def clamp_int(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def normalize_angle_360(angle):
    res = angle % 360.0
    if res < 0:
        res += 360.0
    return res


def check_lab_color(img, x, y, thresh):
    pix = img.get_pixel(x, y)
    if pix is None:
        return False
    l, a, b = image.rgb_to_lab(pix)
    t0 = thresh[0]
    return (t0[0] < l < t0[1]) and (t0[2] < a < t0[3]) and (t0[4] < b < t0[5])


def get_dist_point_to_line(p, l1, l2):
    x, y = p
    x1, y1 = l1
    x2, y2 = l2
    a = y2 - y1
    b = x1 - x2
    c = x2 * y1 - x1 * y2
    return abs(a * x + b * y + c) / (math.sqrt(a * a + b * b) + 1e-6)


def calc_len(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def calc_angle_between_lines(line1_start, line1_end, line2_start, line2_end):
    v1x = line1_end[0] - line1_start[0]
    v1y = line1_end[1] - line1_start[1]
    v2x = line2_end[0] - line2_start[0]
    v2y = line2_end[1] - line2_start[1]
    angle1 = math.degrees(math.atan2(v1y, v1x))
    angle2 = math.degrees(math.atan2(v2y, v2x))
    diff = abs(angle1 - angle2)
    if diff > 180:
        diff = 360 - diff
    return diff


def fit_line(points):
    n_pts = len(points)
    if n_pts < 2:
        return points[0], points[-1]
    sum_x = 0
    sum_y = 0
    sum_xy = 0
    sum_xx = 0
    for x, y in points:
        sum_x += x
        sum_y += y
        sum_xy += x * y
        sum_xx += x * x
    denominator = n_pts * sum_xx - sum_x * sum_x
    if denominator == 0:
        return points[0], points[-1]
    k = (n_pts * sum_xy - sum_x * sum_y) / denominator
    b = (sum_y - k * sum_x) / n_pts
    pts_sorted = sorted(points, key=lambda p: p[0])
    start_x = pts_sorted[0][0]
    end_x = pts_sorted[-1][0]
    return (start_x, int(k * start_x + b)), (end_x, int(k * end_x + b))


def transform_point(x, y, matrix):
    z = matrix[6] * x + matrix[7] * y + matrix[8]
    if z == 0:
        z = 1.0
    new_x = (matrix[0] * x + matrix[1] * y + matrix[2]) / z
    new_y = (matrix[3] * x + matrix[4] * y + matrix[5]) / z
    return (new_x, new_y)


def calc_distance_car_to_line(car_pt, line_start, line_end):
    x0, y0 = car_pt
    x1, y1 = line_start
    x2, y2 = line_end
    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1
    distance = abs(a * x0 + b * y0 + c) / (math.sqrt(a * a + b * b) + 1e-6)
    return distance * XY_SCALE_FACTOR


def solve_pose(imu_yaw, t_car, t_p1, t_p2):
    dx = t_p2[0] - t_p1[0]
    dy = t_p2[1] - t_p1[1]
    vis_rel_angle = math.degrees(math.atan2(dy, dx))
    estimated_abs_angle = normalize_angle_360(imu_yaw + vis_rel_angle)
    dist_cm = calc_distance_car_to_line(t_car, t_p1, t_p2)

    coord_type = None
    coord_val = None
    wall_std_angle = None
    margin = 10

    if (estimated_abs_angle >= (315 + margin)) or (estimated_abs_angle < (45 - margin)):
        coord_type = "Y"
        coord_val = FIELD_HEIGHT - dist_cm
        wall_std_angle = 0
    elif (estimated_abs_angle >= (45 + margin)) and (estimated_abs_angle < (135 - margin)):
        coord_type = "X"
        coord_val = FIELD_WIDTH - dist_cm
        wall_std_angle = 90
    elif (estimated_abs_angle >= (135 + margin)) and (estimated_abs_angle < (225 - margin)):
        coord_type = "Y"
        coord_val = dist_cm
        wall_std_angle = 180
    elif (estimated_abs_angle >= (225 + margin)) and (estimated_abs_angle < (315 - margin)):
        coord_type = "X"
        coord_val = dist_cm
        wall_std_angle = 270
    else:
        return None, None, None, None

    vis_calc_yaw = normalize_angle_360(wall_std_angle - vis_rel_angle)
    return coord_type, coord_val, dist_cm, vis_calc_yaw


def build_letterbox_input(img, letterbox_fb):
    img_w = img.width()
    img_h = img.height()
    scale = min(MODEL_INPUT_SIZE / float(img_w), MODEL_INPUT_SIZE / float(img_h))
    scaled_w = int(img_w * scale)
    scaled_h = int(img_h * scale)
    offset_x = (MODEL_INPUT_SIZE - scaled_w) // 2
    offset_y = (MODEL_INPUT_SIZE - scaled_h) // 2

    letterbox_fb.draw_rectangle((0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE), color=(0, 0, 0), fill=True)
    letterbox_fb.draw_image(img, offset_x, offset_y, x_scale=scale, y_scale=scale)

    return {
        "scale": scale,
        "offset_x": offset_x,
        "offset_y": offset_y,
    }


def det_to_pixels(det, img, letterbox_meta):
    x1, y1, x2, y2, label_id, score = det
    label_id = int(label_id)
    score = float(score)

    if score < YOLO_SCORE_THRESHOLD:
        return None
    if label_id < 0 or label_id >= len(CLASSES):
        return None

    scale = letterbox_meta["scale"]
    offset_x = letterbox_meta["offset_x"]
    offset_y = letterbox_meta["offset_y"]

    x1 = int((x1 * MODEL_INPUT_SIZE - offset_x) / scale)
    y1 = int((y1 * MODEL_INPUT_SIZE - offset_y) / scale)
    x2 = int((x2 * MODEL_INPUT_SIZE - offset_x) / scale)
    y2 = int((y2 * MODEL_INPUT_SIZE - offset_y) / scale)

    x1 = clamp(x1, 0, img.width() - 1)
    y1 = clamp(y1, 0, img.height() - 1)
    x2 = clamp(x2, 0, img.width() - 1)
    y2 = clamp(y2, 0, img.height() - 1)

    if x2 <= x1:
        x2 = min(img.width() - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(img.height() - 1, y1 + 1)

    bottom_x = int((x1 + x2) / 2)
    bottom_y = y2

    return {
        "bbox": (x1, y1, x2, y2),
        "label_id": label_id,
        "label": CLASSES[label_id],
        "score": score,
        "bottom": (bottom_x, bottom_y),
    }


def collect_detections(net, img, letterbox_fb):
    letterbox_meta = build_letterbox_input(img, letterbox_fb)
    detections = []
    for det in tf.detect(net, letterbox_fb):
        parsed = det_to_pixels(det, img, letterbox_meta)
        if parsed is not None:
            detections.append(parsed)
    return detections


def apply_center_distance_suppression(detections):
    if len(detections) < 2:
        return detections

    sorted_detections = sorted(detections, key=lambda det: det["score"], reverse=True)
    kept = []
    thresh_sq = YOLO_CENTER_SUPPRESS_PIXELS * YOLO_CENTER_SUPPRESS_PIXELS

    for det in sorted_detections:
        bx, by = det["bottom"]
        suppressed = False
        for kept_det in kept:
            if det["label_id"] != kept_det["label_id"]:
                continue
            kx, ky = kept_det["bottom"]
            dx = bx - kx
            dy = by - ky
            if dx * dx + dy * dy < thresh_sq:
                suppressed = True
                break
        if not suppressed:
            kept.append(det)

    return kept


def draw_detections(img, detections):
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        bottom_x, bottom_y = det["bottom"]
        color = CLASS_COLORS.get(det["label_id"], (255, 255, 255))
        img.draw_rectangle((x1, y1, x2 - x1, y2 - y1), color=color, thickness=2)
        img.draw_cross(bottom_x, bottom_y, color=color, size=6, thickness=1)
        img.draw_string(
            x1,
            max(0, y1 - 12),
            "%s %.2f" % (det["label"], det["score"]),
            color=color,
            scale=1,
        )


def pack_signed_1d_5chars(value):
    sign = "+" if value >= 0 else "-"
    mag = int(round(abs(value) * 10.0))
    mag = clamp_int(mag, 0, 9999)
    return (sign + "{:04d}".format(mag)).encode("ascii")


def parse_5chars(seg):
    if isinstance(seg, bytes):
        seg = seg.decode("ascii")
    sign = -1.0 if seg[0] == "-" else 1.0
    int_part = (ord(seg[1]) - 48) * 100 + (ord(seg[2]) - 48) * 10 + (ord(seg[3]) - 48)
    frac = ord(seg[4]) - 48
    return sign * (int_part + 0.1 * frac)


def uart_get():
    global uart_buf, current_imu_yaw, current_imu_x, current_imu_y
    if uart.any():
        uart_buf += uart.read()
        if b"\n" in uart_buf:
            uart_buf = bytes(uart_buf)
            parts = uart_buf.split(b"\n")
            if len(parts) >= 2:
                latest_data = parts[-2].decode("ascii").strip()
                uart_buf = parts[-1]
                if len(latest_data) >= 15:
                    try:
                        current_imu_yaw = normalize_angle_360(parse_5chars(latest_data[0:5]))
                        current_imu_x = parse_5chars(latest_data[5:10])
                        current_imu_y = parse_5chars(latest_data[10:15])
                    except Exception:
                        pass


def compute_obj_world_pos(pixel_x, pixel_y, rod_x, rod_y, vis_yaw):
    if rod_x >= 900 or rod_y >= 900 or vis_yaw >= 900:
        return 999.0, 999.0

    obj_ground = transform_point(pixel_x, pixel_y, PERSPECTIVE_MATRIX)
    car_ground = transform_point(CAR_SCREEN_X, CAR_SCREEN_Y, PERSPECTIVE_MATRIX)
    dx = obj_ground[0] - car_ground[0]
    dy = obj_ground[1] - car_ground[1]

    distance = math.sqrt(dx * dx + dy * dy) * XY_SCALE_FACTOR
    if distance < 1.0:
        return 999.0, 999.0

    angle_rel = math.degrees(math.atan2(dx, dy))
    angle_world = math.radians(vis_yaw + angle_rel)
    world_x = rod_x + distance * math.sin(angle_world)
    world_y = rod_y + distance * math.cos(angle_world)
    return world_x, world_y


def compute_obj_ipm_pos(pixel_x, pixel_y):
    ground_x, ground_y = transform_point(pixel_x, pixel_y, PERSPECTIVE_MATRIX)
    return ground_x * XY_SCALE_FACTOR, ground_y * XY_SCALE_FACTOR


def build_uart_payload(vis_yaw, rod_x, rod_y, obj_list):
    payload = (
        pack_signed_1d_5chars(float(len(obj_list))) +
        pack_signed_1d_5chars(vis_yaw) +
        pack_signed_1d_5chars(rod_x) +
        pack_signed_1d_5chars(rod_y)
    )
    for obj in obj_list:
        payload += (
            pack_signed_1d_5chars(obj["send_x"]) +
            pack_signed_1d_5chars(obj["send_y"]) +
            pack_signed_1d_5chars(float(obj["class_id"])) +
            pack_signed_1d_5chars(obj["score"])
        )
    return payload + b"\n"


def uart_send_objects(vis_yaw, rod_x, rod_y, obj_list):
    payload = build_uart_payload(vis_yaw, rod_x, rod_y, obj_list)
    uart.write(payload)

    if obj_list:
        desc = []
        for obj in obj_list:
            desc.append(
                "%s(send_x=%.1f, send_y=%.1f, conf=%.2f)" % (
                    obj["label"],
                    obj["send_x"],
                    obj["send_y"],
                    obj["score"],
                )
            )
        obj_text = "; ".join(desc)
    else:
        obj_text = "none"
    print(
        "SEND vis_yaw=%.1f rod=(%.1f, %.1f) count=%d %s" % (
            vis_yaw,
            rod_x,
            rod_y,
            len(obj_list),
            obj_text,
        )
    )


def check_and_prepare_send(imu_yaw, vis_yaw):
    global N, send_flag
    if vis_yaw is None:
        N = 0
        return

    diff = abs(imu_yaw - vis_yaw)
    if diff > 180:
        diff = 360 - diff

    if diff < TH_YAW and ((vis_yaw % 90) > 82 or (vis_yaw % 90) < 8):
        N += 1
        if N >= n:
            send_flag = 1
            N = 0
    else:
        N = 0


def tick(_):
    global t
    t += 1
    uart_get()
    check_and_prepare_send(current_imu_yaw, vis_yaw_calc)


tim = Timer(1, 500)
tim.callback(tick)

net = tf.load(YOLO_MODEL_PATH)
letterbox_fb = sensor.alloc_extra_fb(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, sensor.RGB565)
gc.collect()


while True:
    clock.tick()
    img = sensor.snapshot()

    vis_yaw_calc = None
    cam_x_float = None
    cam_y_float = None

    blobs = img.find_blobs(YELLOW_THRESHOLD, pixels_threshold=50, area_threshold=50, merge=True)
    points = []
    final_draw_cmd = None
    lines_to_process = []

    if blobs:
        max_blob = max(blobs, key=lambda b: b.pixels())
        bx, by, bw, bh = max_blob.rect()
        img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0))

        angle_step = (ANGLE_END - ANGLE_START) / (RAY_COUNT - 1)
        for i in range(RAY_COUNT):
            rad = math.radians(ANGLE_START + i * angle_step)
            cos_a = math.cos(rad)
            sin_a = math.sin(rad)
            for d in range(0, MAX_DIST, STEP):
                x = int(ORIGIN_X + d * cos_a)
                y = int(ORIGIN_Y + d * sin_a)
                if x < 0 or x >= IMG_W or y < 0 or y >= IMG_H:
                    break
                if bx <= x < bx + bw and by <= y < by + bh:
                    if check_lab_color(img, x, y, YELLOW_THRESHOLD):
                        points.append((x, y))
                        break

    if len(points) > (TRIM_END * 2 + CORNER_BUFFER * 2 + 4):
        p_start, p_end = points[0], points[-1]
        max_d = 0
        idx_split = 0
        for i in range(1, len(points) - 1):
            dist = get_dist_point_to_line(points[i], p_start, p_end)
            if dist > max_d:
                max_d = dist
                idx_split = i

        idx_seg1_end = idx_split - CORNER_BUFFER
        idx_seg2_start = idx_split + CORNER_BUFFER
        idx_seg1_start = TRIM_END
        idx_seg2_end = len(points) - 1 - TRIM_END

        points_a = []
        points_b = []
        if idx_seg1_end > idx_seg1_start:
            points_a = points[idx_seg1_start:idx_seg1_end]
        if idx_seg2_end > idx_seg2_start:
            points_b = points[idx_seg2_start:idx_seg2_end]

        valid_a = False
        valid_b = False
        if len(points_a) >= 3:
            line_a_s, line_a_e = fit_line(points_a)
            if calc_len(line_a_s, line_a_e) > MIN_SEG_LENGTH:
                valid_a = True
        if len(points_b) >= 3:
            line_b_s, line_b_e = fit_line(points_b)
            if calc_len(line_b_s, line_b_e) > MIN_SEG_LENGTH:
                valid_b = True

        if valid_a and valid_b:
            angle = calc_angle_between_lines(line_a_s, line_a_e, line_b_s, line_b_e)
            if angle < COLLINEAR_ANGLE:
                merged_s, merged_e = fit_line(points)
                final_draw_cmd = ("Straight", merged_s, merged_e)
                lines_to_process.append((merged_s, merged_e))
            else:
                final_draw_cmd = ("Corner4", line_a_s, line_a_e, line_b_s, line_b_e)
                lines_to_process.append((line_a_s, line_a_e))
                lines_to_process.append((line_b_s, line_b_e))
        elif valid_a:
            final_draw_cmd = ("Straight", line_a_s, line_a_e)
            lines_to_process.append((line_a_s, line_a_e))
        elif valid_b:
            final_draw_cmd = ("Straight", line_b_s, line_b_e)
            lines_to_process.append((line_b_s, line_b_e))

    t_car = transform_point(CAR_SCREEN_X, CAR_SCREEN_Y, PERSPECTIVE_MATRIX)
    best_line = None
    max_len = 0
    for line in lines_to_process:
        curr_len = calc_len(line[0], line[1])
        if curr_len > max_len:
            max_len = curr_len
            best_line = line

    if best_line:
        t_p1 = transform_point(best_line[0][0], best_line[0][1], PERSPECTIVE_MATRIX)
        t_p2 = transform_point(best_line[1][0], best_line[1][1], PERSPECTIVE_MATRIX)
        c_type, c_val, _, v_yaw = solve_pose(current_imu_yaw, t_car, t_p1, t_p2)
        if c_type is not None:
            vis_yaw_calc = v_yaw
            if c_type == "X":
                cam_x_float = c_val
            elif c_type == "Y":
                cam_y_float = c_val

    rod_x_val = 999.0
    rod_y_val = 999.0
    yaw_rad = math.radians(current_imu_yaw)
    if cam_x_float is not None:
        rod_x_val = cam_x_float - ROD_OFFSET_CM * math.sin(yaw_rad)
    if cam_y_float is not None:
        rod_y_val = cam_y_float - ROD_OFFSET_CM * math.cos(yaw_rad)

    detections = collect_detections(net, img, letterbox_fb)
    detections = apply_center_distance_suppression(detections)
    draw_detections(img, detections)

    if vis_yaw_calc is not None:
        global_vis_yaw = vis_yaw_calc
        global_rod_x = rod_x_val
        global_rod_y = rod_y_val

    obj_list = []
    for det in detections:
        bottom_x, bottom_y = det["bottom"]
        ipm_x, ipm_y = compute_obj_ipm_pos(bottom_x, bottom_y)
        world_x, world_y = compute_obj_world_pos(
            bottom_x,
            bottom_y,
            global_rod_x,
            global_rod_y,
            global_vis_yaw,
        )
        det["world"] = (world_x, world_y)
        det["ipm"] = (ipm_x, ipm_y)

        send_x = ipm_x
        send_y = ipm_y
        if world_x < 900 and world_y < 900:
            send_x = world_x
            send_y = world_y

        obj_list.append(
            {
                "send_x": send_x,
                "send_y": send_y,
                "world_x": world_x,
                "world_y": world_y,
                "ipm_x": ipm_x,
                "ipm_y": ipm_y,
                "class_id": CLASS_IDS[det["label"]],
                "label": det["label"],
                "score": det["score"],
            }
        )
        img.draw_string(
            bottom_x,
            min(IMG_H - 10, bottom_y + 4),
            "(%.0f,%.0f)" % (send_x, send_y),
            color=CLASS_COLORS.get(det["label_id"], (255, 255, 255)),
            scale=1,
        )

    send_vis_yaw = global_vis_yaw if send_flag else 999.0
    uart_send_objects(send_vis_yaw, global_rod_x, global_rod_y, obj_list)
    if send_flag:
        send_flag = 0

    if final_draw_cmd and final_draw_cmd[0] == "Straight":
        img.draw_line(
            final_draw_cmd[1][0],
            final_draw_cmd[1][1],
            final_draw_cmd[2][0],
            final_draw_cmd[2][1],
            color=(0, 255, 255),
            thickness=3,
        )

    print("fps=%.2f det=%d" % (clock.fps(), len(detections)))
    gc.collect()
