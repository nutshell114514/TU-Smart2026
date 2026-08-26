#汽院赛道的
import gc
import math
import os
import time

import image
import seekfree
import sensor
import tf
from machine import UART
from seekfree import Timer

DRAW_DEBUG =  False  # Set True only when viewing OpenART image output.


IMG_W = 320
IMG_H = 240
MODEL_INPUT_SIZE = 96

LINE_ENABLE  = True   # 黄线爬线检测开关
MODEL_ENABLE = True   # YOLO目标检测开关
BALL_ENABLE  = True  # 主车标记球检测开关（从车推球阶段由 Frame1 flags bit2 开启；台架单测时临时改 True）

# 每 SAVE_IMAGE_EVERY_N_FRAMES 帧向 SD 写一张 JPEG。q95 的写入会造成几十到一百多毫秒的
# 视觉主循环停顿，而 Frame2 的有效更新率就等于帧率，这个停顿会直接变成下位机的视觉延迟尖峰。
# 只在采数据集时打开，正式跑任务必须为 False。
SAVE_IMAGE_ENABLE = False
SAVE_IMAGE_EVERY_N_FRAMES = 10
SAVE_IMAGE_QUALITY = 95
EPOCH_FILE = "/sd/EPOCHNUM"
DATASET_DIR = "/sd/dataset"

YOLO_MODEL_PATH = "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite"
YOLO_SCORE_THRESHOLD = 0.75
YOLO_CENTER_SUPPRESS_PIXELS = 10

CLASSES = ["car", "sandbag", "teddy", "tennis", "cone", "brick"]
CLASS_IDS = {
    "car": 0,
    "tennis": 1,
    "blue_sandbag": 2,
    "red_sandbag": 3,
    "white_teddy": 4,
    "brown_teddy": 5,
    "cone": 8,
    "brick": 9,
}
# Object id map shared by OpenART and lower controller:
# 0 car, 1 tennis, 2 blue_sandbag, 3 red_sandbag, 4 white_teddy,
# 5 brown_teddy, 8 cone, 9 brick.
OBJECT_COUNT_BY_CLASS = {
    1: 1,  # tennis; 0 = unknown, use MAX_OBJECTS_PER_CLASS
    2: 1,  # blue sandbag
    3: 1,  # red sandbag
    4: 1,  # white teddy
    5: 1,  # brown teddy
}
MAX_OBJECTS_PER_CLASS = 5
SEND_OBJECT_SLOTS = 2
MAX_SEND_OBJECTS = 7
INVALID_OBJECT_ID = 255
_ID_NAME = {
    0: "car",
    1: "tennis",
    2: "blue_sandbag",
    3: "red_sandbag",
    4: "white_teddy",
    5: "brown_teddy",
    8: "cone",
    9: "brick",
}
CLASS_COLORS = {
    0: (255, 80, 80),
    1: (255, 200, 0),
    2: (80, 160, 255),
    3: (255, 70, 70),
    4: (255, 255, 255),
    5: (170, 100, 50),
    8: (80, 255, 80),
    9: (180, 130, 70),
}

YELLOW_THRESHOLD = [(51, 100, -31, 4, 57, 127)]
# 黄线blob筛选：长宽比 < MIN_ASPECT 且 填充率 > MAX_FILL 时判定为圆形物体（如网球），排除。
LINE_BLOB_MIN_ASPECT = 1.8
LINE_BLOB_MAX_FILL   = 0.55
# 场地黄边线在当前相机视角下必然延伸到画面外，因此其 blob 矩形至少一边应贴近
# 320x240 图像边缘。保留少量像素容差，避免阈值分割在边缘漏掉零星像素。
LINE_EDGE_MARGIN     = 4

# ── 主车标记球检测（从车推球阶段用，blob + 霍夫圆两步法）─────────────────
# 类别 id 与下位机共用：6 = 左球（紫，主车推杆左前），7 = 右球（橙，右前）。
BALL_L_CLASS_ID = 6
BALL_R_CLASS_ID = 7
CONE_CLASS_ID = 8
BRICK_CLASS_ID = 9
# LAB 阈值为占位估值，必须在本文件固定曝光/增益下现场标定（勿照搬 IDE 自动曝光下的值）。
BALL_L_THRESHOLD = [(19, 68, 44, 127, -75, -39)]    # 紫
BALL_R_THRESHOLD = [(42, 93, 21, 96, 39, 85)]     # 橙（注意与红沙包拉开距离）
BALL_BLOB_MIN_PIXELS = 20
BALL_BLOB_MIN_AREA   = 20
BALL_ROI_PAD         = 8     # blob 外扩为霍夫 ROI 的边距(px)
BALL_ROI_MAX         = 90    # ROI 边长上限(px)，限制霍夫耗时
# 定位方式：球面顶部高光+底部自阴影会让色块残缺成椭圆，霍夫圆在残缺边缘上
# 圆心每帧漂移（框跳变根源）。质心定位对残缺形状的偏置是恒定的，且跟球环
# 用相对基准误差、恒定偏置在控制上自然抵消——残缺色块下质心模式更稳。
# False = 色块质心 + 形状门控（推荐）；True = blob ROI 内霍夫圆确认（需近完整圆）
BALL_USE_HOUGH = False
BALL_SHAPE_ASPECT_MAX = 2.2   # 形状门控：长宽比上限（按残缺椭圆放宽）
BALL_SHAPE_FILL_MIN   = 0.35  # 形状门控：填充率下限 pixels/(w*h)
# 霍夫强度阈值与球的像素半径正相关（magnitude≈圆周边缘点累加）。
# 实测球在编队距离下仅 r≈7px，1000 永远达不到；按打印的 magnitude 分布定阈值。
BALL_CIRCLE_THRESHOLD = 350
BALL_R_MIN  = 3
BALL_R_MAX  = 45
BALL_R_STEP = 2
BALL_XY_MARGIN = 15   # 霍夫近邻圆合并容差(px)，加大以合并同一球的重复候选
# 时域跟踪：α-β 滤波（位置+速度双状态）。每帧先按速度预测再用观测修正，
# 静止时降噪等同低通，匀速运动时稳态滞后≈0，解决"球一动框就拖后"。
BALL_TRACK_ALPHA       = 0.5  # 位置修正增益(0~1]，越小越稳
BALL_TRACK_BETA        = 0.25 # 速度修正增益，一般取 ALPHA 的一半左右；0=退化为纯低通
BALL_SMOOTH_GATE_PX    = 25   # 观测与预测偏差超过此值(px)=跟踪失效，重置状态直接跟上
BALL_SMOOTH_MISS_RESET = 5    # 连续未检出超过此帧数清除跟踪状态（期间按速度惯性外推）
BALL_PRED_LEAD_FRAMES  = 0.5  # 发送超前帧数：只作用于发给下位机的 rel（补链路延迟），
                              # 画框始终用滤波位置（所见即球）；变向略过冲可降到 0
BALL_VEL_MAX_PX        = 25.0 # 像素速度限幅(px/帧)，防速度估计被野点带飞
# 球心不在地面平面，IPM 结果按相似三角形缩回：rel *= (H_CAM − H_BALL)/H_CAM。
# 约束：球心必须低于摄像头光心，否则射线不与地面相交、IPM 无意义。均需实测。
BALL_CAM_HEIGHT_CM = 15.0    # 摄像头光心离地高度（7.4 实测）
BALL_HEIGHT_CM     = 2.3     # 球心离地高度（7.4 实测）
BALL_IPM_SCALE = (BALL_CAM_HEIGHT_CM - BALL_HEIGHT_CM) / BALL_CAM_HEIGHT_CM
CONE_HEIGHT_CM = 1.5
CONE_IPM_SCALE = (BALL_CAM_HEIGHT_CM - CONE_HEIGHT_CM) / BALL_CAM_HEIGHT_CM

# ── 球检测逐级调试打印（标定用；脱机比赛务必关总开关，print 会拖慢帧率）──
# 排查思路：①blobs=0 → 阈值/最小像素问题；②SKIP → ROI 尺寸过滤挡掉了；
# ③no circle → 霍夫阈值太严（临时把 BALL_CIRCLE_THRESHOLD 降到 400~500，
#   看打印出的圆 magnitude 值分布，再把阈值定在真球 magnitude 的 6~7 成）；
# ④HIT 但 rel 不对 → 高度修正/单应矩阵问题。
BALL_DEBUG_PRINT = 0   # 总开关
BALL_DBG_BLOB    = 0    # ①色块粗筛：每色 blob 数量 + 每个 blob 的矩形/像素数
BALL_DBG_ROI     = 1    # ②ROI 过滤：外扩后的 ROI，及被丢弃的原因（过小/超上限）
BALL_DBG_CIRCLE  = 1    # ③霍夫圆：每个候选圆参数（含 magnitude），无圆也提示
BALL_DBG_POS     = 1    # ④定位：命中球的 像素→IPM→高度修正→rel 逐级数值 / MISS

FIELD_WIDTH = 310
FIELD_HEIGHT = 230
XY_SCALE_FACTOR = 1

ORIGIN_X = 160
ORIGIN_Y = 190
ANGLE_START = -165
ANGLE_END = -15
RAY_COUNT = 30
STEP = 4
MAX_DIST = 320

MIN_SEG_LENGTH = 6
COLLINEAR_ANGLE = 20
CORNER_BUFFER = 2
TRIM_END = 1

CAR_ID = 1

CAR_CONFIGS = {
    1: {#没有黄色胶带的，在这里定义1是主车
        "rod_offset_cm": 10.0,
        "car_screen_x": 160,
        "car_screen_y": 239,
        "qvga_matrix": [-2.7267133952291407, -0.037620429998306, 445.2654278664716, -0.02355629585168749, 2.3577723732666125, -559.7385956341923, -0.0035379221885103646, -0.16030647444359186, 1.0]


,
    },
    2: {#有黄色胶带的，在这里定义2是从车
        "rod_offset_cm": 7.0,
        "car_screen_x": 155,
        "car_screen_y": 239,
        "qvga_matrix": [-2.1918194640338386, 0.06227889417425004, 335.80645212972274, 5.722071046851319e-17, 1.9372004117548556, -462.990905592233, 6.125937005217e-18, -0.13399153737658595, 1.0]
,
    },
}

CAR_CFG = CAR_CONFIGS.get(CAR_ID, CAR_CONFIGS[1])
ROD_OFFSET_CM = CAR_CFG["rod_offset_cm"]
CAR_SCREEN_X = CAR_CFG["car_screen_x"]
CAR_SCREEN_Y = CAR_CFG["car_screen_y"]
PERSPECTIVE_MATRIX = CAR_CFG["qvga_matrix"]
if PERSPECTIVE_MATRIX is None:
    raise RuntimeError("CAR_ID=%d qvga_matrix is not calibrated yet" % CAR_ID)


def load_next_epoch_num():
    try:
        f = open(EPOCH_FILE, "r")
        text = f.read()
        f.close()
        epoch_num = int(text.strip())
    except:
        epoch_num = 0

    epoch_num += 1

    try:
        f = open(EPOCH_FILE, "w")
        f.write(str(epoch_num))
        f.close()
        try:
            os.sync()
        except:
            pass
    except Exception as e:
        print("EPOCHNUM write failed:", e)

    return epoch_num


def ensure_dataset_dir():
    try:
        os.mkdir(DATASET_DIR)
    except:
        pass


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
uart = UART(12, baudrate=230400)

current_imu_yaw = 315.0
current_imu_x = 0.0
current_imu_y = 0.0

t = 0
vis_yaw_calc = None

# 从主控接收的指令（sel_id 保留解析但不用于主车选择）
current_target_sel_id  = 0      # 保留字段：0=自由选 / APPROACH阶段=锁定类型约束
current_target_rel_x   = 999.0  # 从车位置锁定用（999=无锁定）
current_target_rel_y   = 999.0
current_lock_mode      = 0
current_lock_track_id  = 0
current_obj_remain     = [0, 1, 1, 1, 1, 1]
current_obj_count_mode = 0      # 0=已知剩余类别（模式2/3），1=只知道全局总数（模式1）
current_obj_total_remaining = 5
current_obj_edge_mode = 0       # 0=决赛固定类别映射，1=预赛按完成顺序在0°/270°边交替

# 摄像头自主锁定状态（主车 SELECT 阶段）
_sel_locked           = False
_sel_lock_rel_x       = 999.0
_sel_lock_rel_y       = 999.0
_sel_lock_type        = 0
# 主车自由 SEARCH 只考虑车体系直线距离不超过 60 cm 的可搬运目标。
# 最近候选还必须以相同类别、相近位置连续出现 2 个模型推理帧才真正锁定。
_LEADER_SEL_MAX_DIST_CM = 60.0
_LEADER_SEL_CONFIRM_FRAMES = 2
_LEADER_SEL_CONFIRM_GATE_CM = 30.0
# 红沙包只有在YOLO矩形框四边都完整离开画面边缘至少该像素数后，
# 才允许进入跟踪和主车连续两帧确认。brick及其他类别不受影响。
RED_SANDBAG_EDGE_MARGIN_PX = 6
_leader_sel_candidate_type = 0
_leader_sel_candidate_rel_x = 999.0
_leader_sel_candidate_rel_y = 999.0
_leader_sel_candidate_count = 0
# 候选确认期间保持非负，使普通 slot0 用 track_id=254 标记“正在确认”。
_leader_sel_observe_t0 = -1
# 普通槽为空且 slot0.track_id 为该值时，仅表示“自由选物观察中”；
# 下位机用它保持物体优先级，但不会把它当成有效物体。
TARGET_OBSERVING_TRACK_MARKER = 254
_SEL_REACQUIRE_R      = 30.0   # cm，锁定后允许的最大漂移量（跨帧追踪）
_sel_miss_frames      = 0
_SEL_MAX_MISS         = 5      # SELECT 阶段丢失容忍帧数
_SEL_MAX_MISS_APPROACH = 20    # APPROACH 阶段丢失容忍帧数（约2s@10fps）

# 从车按主车目标坐标锁定槽0时，候选物体的 X/Y 相对坐标误差都必须在该范围内。
FOLLOWER_TARGET_XY_GATE = 40.0


TRACK_WORLD_GATE = 25.0
LOCK_WORLD_GATE = 35.0
TRACK_MAX_MISS_FRAMES = 30
TRACK_COUNT_FACTOR = 1.5
MIN_SCORE_CREATE_TRACK = 0.85
MIN_SCORE_UPDATE_TRACK = 0.7

#坐标修正判断方向的阈值
DIAG_LOW = 25
DIAG_HIGH = 65

_tracks = []
_next_track_id = 1

def _reraise_ide_stop(e):
    # OpenMV IDE 停止脚本的方式是向运行中的代码注入 "IDE interrupt" 异常。
    # 主循环里的宽捕获（except Exception）必须放行它，否则停止信号被当普通
    # 错误吞掉，脚本无法从 IDE 终止。所有循环内的异常处理第一行先调用本函数。
    if "IDE interrupt" in str(e):
        raise e


def _blob_is_line(blob):
    _, _, bw, bh = blob.rect()
    long_side  = bw if bw > bh else bh
    short_side = bh if bw > bh else bw
    if short_side == 0:
        return True
    if long_side / short_side >= LINE_BLOB_MIN_ASPECT:
        return True  # 明显细长，肯定是线
    fill = blob.pixels() / (bw * bh)
    return fill <= LINE_BLOB_MAX_FILL  # 填充率低也可能是线（近距离）


def _blob_touches_frame_edge(blob):
    bx, by, bw, bh = blob.rect()
    return (bx <= LINE_EDGE_MARGIN or
            by <= LINE_EDGE_MARGIN or
            bx + bw >= IMG_W - LINE_EDGE_MARGIN or
            by + bh >= IMG_H - LINE_EDGE_MARGIN)


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



def line_trend_img(p1, p2):
    """
    图像坐标系：左上角为原点，x向右，y向下

    返回：
        1  : \  左上到右下
       -1  : /  左下到右上
        0  : 太水平/太竖直，趋势不可靠
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    if abs(dx) < 3 or abs(dy) < 3:
        return 0

    if dx * dy > 0:
        return 1
    else:
        return -1


def use_diag_method(angle):
    """
    每个90度区间中：
    0~25、65~90 用传统方法
    25~65 用斜率+IMU方法
    """
    a = normalize_angle_360(angle)
    r = a % 90.0
    return DIAG_LOW <= r <= DIAG_HIGH

def calc_distance_car_to_line(car_pt, line_start, line_end):
    x0, y0 = car_pt
    x1, y1 = line_start
    x2, y2 = line_end
    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1
    distance = abs(a * x0 + b * y0 + c) / (math.sqrt(a * a + b * b) + 1e-6)
    return distance * XY_SCALE_FACTOR

def wall_by_imu_and_trend(imu_yaw, trend):
    if trend == 0:
        return None

    yaw = normalize_angle_360(imu_yaw)

    # 0~90
    if 0 <= yaw < 90:
        if trend > 0:   # \
            return 90
        else:           # /
            return 0

    # 90~180
    elif 90 <= yaw < 180:
        if trend > 0:   # \
            return 180
        else:           # /
            return 90

    # 180~270
    elif 180 <= yaw < 270:
        if trend > 0:   # \
            return 270
        else:           # /
            return 180

    # 270~360
    else:
        if trend > 0:   # \
            return 0    # 360
        else:           # /
            return 270

def solve_pose(imu_yaw, t_car, t_p1, t_p2, trend=0):
    dx = t_p2[0] - t_p1[0]
    dy = t_p2[1] - t_p1[1]
    vis_rel_angle = math.degrees(math.atan2(dy, dx))
    estimated_abs_angle = normalize_angle_360(imu_yaw + vis_rel_angle)
    dist_cm = calc_distance_car_to_line(t_car, t_p1, t_p2)

    coord_type = None
    coord_val = None
    wall_std_angle = None

    # 1. 靠近45度区域：用你的新方法
    if use_diag_method(imu_yaw):
        wall_std_angle = wall_by_imu_and_trend(imu_yaw, trend)

    # 2. 靠近整90度区域：用传统方法
    if wall_std_angle is None:
        margin = 16

        if (estimated_abs_angle >= (315 + margin)) or (estimated_abs_angle < (45 - margin)):
            wall_std_angle = 0

        elif (estimated_abs_angle >= (45 + margin)) and (estimated_abs_angle < (135 - margin)):
            wall_std_angle = 90

        elif (estimated_abs_angle >= (135 + margin)) and (estimated_abs_angle < (225 - margin)):
            wall_std_angle = 180

        elif (estimated_abs_angle >= (225 + margin)) and (estimated_abs_angle < (315 - margin)):
            wall_std_angle = 270

        else:
            return None, None, None, None

    # 3. 根据最终 wall_std_angle 算坐标
    if wall_std_angle == 0:
        coord_type = "Y"
        coord_val = FIELD_HEIGHT - dist_cm

    elif wall_std_angle == 90:
        coord_type = "X"
        coord_val = FIELD_WIDTH - dist_cm

    elif wall_std_angle == 180:
        coord_type = "Y"
        coord_val = dist_cm

    elif wall_std_angle == 270:
        coord_type = "X"
        coord_val = dist_cm

    else:
        return None, None, None, None

    vis_calc_yaw = normalize_angle_360(wall_std_angle + vis_rel_angle)
    return coord_type, coord_val, dist_cm, vis_calc_yaw

_LB_SCALE    = min(MODEL_INPUT_SIZE / float(IMG_W), MODEL_INPUT_SIZE / float(IMG_H))
_LB_OFFSET_X = (MODEL_INPUT_SIZE - int(IMG_W * _LB_SCALE)) // 2
_LB_OFFSET_Y = (MODEL_INPUT_SIZE - int(IMG_H * _LB_SCALE)) // 2
_LETTERBOX_META = {
    "scale": _LB_SCALE,
    "offset_x": _LB_OFFSET_X,
    "offset_y": _LB_OFFSET_Y,
    "input_size": MODEL_INPUT_SIZE,
}


def build_letterbox_input(img, letterbox_fb):
    letterbox_fb.draw_rectangle((0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE), color=(0, 0, 0), fill=True)
    letterbox_fb.draw_image(img, _LB_OFFSET_X, _LB_OFFSET_Y, x_scale=_LB_SCALE, y_scale=_LB_SCALE)


def det_to_pixels(det, img, letterbox_meta):
    x1, y1, x2, y2, label_id, score = det
    label_id = int(label_id)
    score = float(score)

    if score < YOLO_SCORE_THRESHOLD:
        return None
    if label_id < 0 or label_id >= len(CLASSES):
        return None

    input_size = letterbox_meta["input_size"]
    scale      = letterbox_meta["scale"]
    offset_x   = letterbox_meta["offset_x"]
    offset_y   = letterbox_meta["offset_y"]

    x1 = int((x1 * input_size - offset_x) / scale)
    y1 = int((y1 * input_size - offset_y) / scale)
    x2 = int((x2 * input_size - offset_x) / scale)
    y2 = int((y2 * input_size - offset_y) / scale)

    x1 = clamp(x1, 0, img.width() - 1)
    y1 = clamp(y1, 0, img.height() - 1)
    x2 = clamp(x2, 0, img.width() - 1)
    y2 = clamp(y2, 0, img.height() - 1)

    if x2 <= x1:
        x2 = min(img.width() - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(img.height() - 1, y1 + 1)

    # bottom 点用于 IPM 地面坐标映射（必须保留）
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
    build_letterbox_input(img, letterbox_fb)
    detections = []
    try:
        for det in tf.detect(net, letterbox_fb):
            parsed = det_to_pixels(det, img, _LETTERBOX_META)
            if parsed is not None:
                detections.append(parsed)
    except Exception as e:
        _reraise_ide_stop(e)
        print("detect ERR:", e)
    return detections

'''
def apply_center_distance_suppression(detections):
    if len(detections) < 2:
        return detections

    sorted_detections = sorted(detections, key=lambda det: det["score"], reverse=True)
    kept = []
    thresh_sq = YOLO_CENTER_SUPPRESS_PIXELS * YOLO_CENTER_SUPPRESS_PIXELS

    for det in sorted_detections:
        x1, y1, x2, y2 = det["bbox"]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        suppressed = False
        for kept_det in kept:
            if det["label_id"] != kept_det["label_id"]:
                continue
            kx1, ky1, kx2, ky2 = kept_det["bbox"]
            kcx = (kx1 + kx2) / 2
            kcy = (ky1 + ky2) / 2
            dx = cx - kcx
            dy = cy - kcy
            if dx * dx + dy * dy < thresh_sq:
                suppressed = True
                break
        if not suppressed:
            kept.append(det)

    return kept
'''

def draw_detections(img, detections):
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        w  = x2 - x1
        h  = y2 - y1
        cx = x1 + w // 2
        cy = y1 + h // 2
        color = CLASS_COLORS.get(det["label_id"], (255, 255, 255))
        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)
        img.draw_circle(cx, cy, 3, color=color, thickness=1)
        img.draw_cross(cx, cy, color=color, size=2, thickness=1)


def _rgb_votes(img, bbox):
    x1, y1, x2, y2 = bbox
    w = x2 - x1
    h = y2 - y1
    if w <= 1 or h <= 1:
        return None

    sx0 = x1 + int(w * 0.25)
    sx1 = x1 + int(w * 0.75)
    sy0 = y1 + int(h * 0.08)
    sy1 = y1 + int(h * 0.60)
    if sx1 <= sx0:
        sx1 = sx0 + 1
    if sy1 <= sy0:
        sy1 = sy0 + 1

    red = 0
    blue = 0
    white = 0
    brown = 0
    valid = 0
    bright_sum = 0.0

    for iy in range(5):
        y = sy0 + int((sy1 - sy0) * (iy + 0.5) / 5.0)
        if y < 0:
            y = 0
        if y >= IMG_H:
            y = IMG_H - 1
        for ix in range(5):
            x = sx0 + int((sx1 - sx0) * (ix + 0.5) / 5.0)
            if x < 0:
                x = 0
            if x >= IMG_W:
                x = IMG_W - 1
            pix = img.get_pixel(x, y)
            if pix is None:
                continue
            r, g, b = pix
            bright = (r + g + b) / 3.0
            if bright < 25:
                continue
            valid += 1
            bright_sum += bright
            if r > 120 and r > g * 1.45 and r > b * 1.35:
                red += 1
            if b > 120 and b > r * 1.45 and b > g * 1.10:
                blue += 1
            if bright > 145 and abs(r - g) < 45 and abs(g - b) < 55:
                white += 1
            if r > 65 and r > g * 1.08 and g > b * 1.05 and bright < 155:
                brown += 1

    if valid <= 0:
        return None
    return {
        "valid": valid,
        "bright": bright_sum / valid,
        "red": red,
        "blue": blue,
        "white": white,
        "brown": brown,
    }


def classify_detection(img, det):
    label = det["label"]
    if label == "car":
        return 0, "car"
    if label == "tennis":
        return 1, "tennis"
    if label == "cone":
        return CONE_CLASS_ID, "cone"
    if label == "brick":
        return BRICK_CLASS_ID, "brick"

    votes = _rgb_votes(img, det["bbox"])
    if votes is None:
        if label == "sandbag":
            return 2, "blue_sandbag"
        if label == "teddy":
            return 4, "white_teddy"
        return 0, label

    if label == "sandbag":
        red = votes["red"]
        blue = votes["blue"]
        if red >= 10 and red > blue * 2:
            return 3, "red_sandbag"
        if blue >= 10 and blue > red * 2:
            return 2, "blue_sandbag"
        if red > blue:
            return 3, "red_sandbag"
        return 2, "blue_sandbag"

    if label == "teddy":
        white = votes["white"]
        brown = votes["brown"]
        bright = votes["bright"]
        if white >= 10 and bright >= 150 and brown <= 6:
            return 4, "white_teddy"
        if brown >= 10 and bright <= 150 and white <= 6:
            return 5, "brown_teddy"
        if bright >= 150 or white > brown:
            return 4, "white_teddy"
        return 5, "brown_teddy"

    return 0, label


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


def parse_int3_switch(seg):
    if isinstance(seg, bytes):
        seg = seg.decode("ascii")
    sign = -1 if seg[0] == "-" else 1
    value = ((ord(seg[1]) - 48) * 100 +
             (ord(seg[2]) - 48) * 10 +
             (ord(seg[3]) - 48))
    enabled = (ord(seg[4]) - 48) > 0
    return sign * value, enabled


def parse_unsigned_4d(seg):
    if isinstance(seg, bytes):
        seg = seg.decode("ascii")
    return ((ord(seg[1]) - 48) * 1000 +
            (ord(seg[2]) - 48) * 100 +
            (ord(seg[3]) - 48) * 10 +
            (ord(seg[4]) - 48))


# ── 二进制协议工具（Frame1 接收 / Frame2 发送）────────────────────────────────
# 与 RT1021 comm.py 完全一致：int16×10 大端序，CRC8(poly=0x31,init=0x00)。

def _pack_i16(buf, i, v):
    if v >= 900.0 or v <= -900.0:
        raw = 9990
    else:
        raw = int(round(v * 10.0))
        if raw > 32767:  raw = 32767
        if raw < -32768: raw = -32768
    if raw < 0:
        raw += 65536
    buf[i]     = (raw >> 8) & 0xFF
    buf[i + 1] = raw & 0xFF


def _dec_i16(buf, i):
    raw = (buf[i] << 8) | buf[i + 1]
    if raw > 32767:
        raw -= 65536
    return raw / 10.0


def _pack_i8(buf, i, v):
    raw = int(round(v))
    if raw > 127:
        raw = 127
    if raw < -128:
        raw = -128
    if raw < 0:
        raw += 256
    buf[i] = raw & 0xFF


def _crc8(buf, s, e):
    crc = 0
    for i in range(s, e):
        crc ^= buf[i]
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# Frame2 v3 固定发送7个槽，不再携带恒为7的槽位数字段。
# 2个普通物体 + car + 左球 + 右球 + cone + brick，目标槽之后追加黄色线中心 x。
_FRAME2_TYPE = 0x07
_FRAME2_LEN = 12 + MAX_SEND_OBJECTS * 4

# 双缓冲保留：打包写 build 缓冲，装好后与 send 缓冲整体交换再发送。
# 发送已改为主循环内当场进行，不存在半包被发出的竞态，这里保留只是为了不改动
# 打包代码结构；如果以后要省 40 字节，可以合并成单缓冲。
_tx_frame2_build = bytearray(_FRAME2_LEN)
_tx_frame2_send = bytearray(_FRAME2_LEN)

# 定时器现在只负责收 Frame1，发送已改为在 uart_send_objects 里当场进行。
# 这个值曾被改成 100 试图省 CPU，但那是纯推测、没有测量支撑：改后下位机收到的
# 帧率从预期的 40 Hz 掉到约 5 Hz。第二个参数的单位/取值是否被硬件如实执行没有把握，
# 因此还原成实车验证过的 500，不要再凭空改动。
_UART_TIMER_HZ = 500

# tick 计数，用于实测定时器真实频率（见主循环的 FPS_PRINT_ENABLE）。
_tick_cnt = 0
# uart_send_objects 实际发出的 Frame2 计数，用于和帧率对账。
_tx_frame2_sent_cnt = 0

# Frame1 接收缓冲：固定 bytearray + 长度计数，回调内零堆分配。
# 原来的 `uart_buf += chunk` 和 `uart_buf = bytearray(buf[i:])` 每次调用都分配，
# 在定时器回调里制造 GC 压力，而 GC 停顿会落在视觉主循环上。
_FRAME1_LEN = 17
# 累积缓冲容量对齐改版前的 400：tf.detect / img.save 这类长 C 调用期间
# 定时器回调不会被调度（软中断只在字节码边界执行），期间 Frame1 会成串堆积，
# 缓冲太小会整段丢弃。单次读取上限与 base.py 的 _UART_READ_MAX 保持一致。
_RX_BUF_MAX = 384
_RX_READ_MAX = 120
_rx_buf = bytearray(_RX_BUF_MAX)
_rx_len = 0
_rx_chunk = bytearray(_RX_READ_MAX)
_rx_chunk_mv = memoryview(_rx_chunk)


def uart_get():
    # 解析二进制 Frame1 v3（17字节）：RT1021 → OpenArt。
    # [0-1] 0xAA 0x55  [2] 0x08  [3-4] angle  [5-6] pos_x  [7-8] pos_y
    # [9] control：bit0=LINE_EN，bit1=MODEL_EN，bit2=BALL_EN，
    #     bit3=count_mode，bit4~6=total_remaining，
    #     bit7=edge_mode（0=决赛固定类别映射，1=预赛0°/270°交替）
    # [10] sel_id  [11] remain_mask  [12-13] rel_x  [14-15] rel_y
    # [16] CRC8(覆盖[2..15])
    global _rx_len, current_imu_yaw, current_imu_x, current_imu_y
    global current_target_sel_id, current_target_rel_x, current_target_rel_y
    global current_lock_mode, current_lock_track_id
    global current_obj_remain, current_obj_count_mode, current_obj_total_remaining
    global current_obj_edge_mode
    global LINE_ENABLE, MODEL_ENABLE, BALL_ENABLE

    n = uart.any()
    if n:
        room = _RX_BUF_MAX - _rx_len
        if room <= 0:
            # 缓冲区被残留垃圾占满：整段丢弃重来。Frame1 是周期帧，丢一拍无害。
            _rx_len = 0
            room = _RX_BUF_MAX
        if n > room:
            n = room
        if n > _RX_READ_MAX:
            n = _RX_READ_MAX
        cl = uart.readinto(_rx_chunk, n)
        if cl:
            _rx_buf[_rx_len:_rx_len + cl] = _rx_chunk_mv[:cl]
            _rx_len += cl

    buf = _rx_buf
    cnt = _rx_len
    i = 0
    last_valid = -1

    while i + 2 < cnt:
        if buf[i] != 0xAA or buf[i + 1] != 0x55:
            i += 1
            continue
        if buf[i + 2] != 0x08:
            i += 1
            continue
        if i + 16 >= cnt:
            break
        if _crc8(buf, i + 2, i + 16) != buf[i + 16]:
            i += 1
            continue
        last_valid = i
        i += _FRAME1_LEN

    if last_valid >= 0:
        off = last_valid
        angle    = _dec_i16(buf, off + 3)
        pos_x    = _dec_i16(buf, off + 5)
        pos_y    = _dec_i16(buf, off + 7)
        control  = buf[off + 9]
        sel_id   = buf[off + 10]
        rem_mask = buf[off + 11]
        count_mode = (control >> 3) & 1
        total_remaining = (control >> 4) & 7
        edge_mode = (control >> 7) & 1
        rel_x    = _dec_i16(buf, off + 12)
        rel_y    = _dec_i16(buf, off + 14)

        current_imu_yaw      = normalize_angle_360(angle)
        current_imu_x        = pos_x
        current_imu_y        = pos_y
        LINE_ENABLE          = bool(control & 1)
        MODEL_ENABLE         = bool(control & 2)
        BALL_ENABLE          = bool(control & 4)
        current_target_sel_id = sel_id
        current_obj_count_mode = count_mode
        current_obj_total_remaining = total_remaining
        current_obj_edge_mode = edge_mode
        for j in range(1, 6):
            current_obj_remain[j] = 1 if (rem_mask & (1 << (j - 1))) else 0
        current_target_rel_x  = rel_x
        current_target_rel_y  = rel_y
        current_lock_mode     = 0
        current_lock_track_id = 0

    if i > 0:
        # 把未消费的尾部搬到开头。循环的退出条件保证 rem <= 16（一个不完整帧），
        # 所以这里用逐字节搬移即可，既便宜又不依赖重叠切片赋值的实现细节。
        rem = cnt - i
        k = 0
        while k < rem:
            buf[k] = buf[i + k]
            k += 1
        _rx_len = rem



def compute_obj_ipm_pos(pixel_x, pixel_y):
    ground_x, ground_y = transform_point(pixel_x, pixel_y, PERSPECTIVE_MATRIX)
    return ground_x * XY_SCALE_FACTOR, ground_y * XY_SCALE_FACTOR

def compute_obj_world_from_rel(rel_x, rel_y):
    if current_imu_x >= 900.0 or current_imu_y >= 900.0:
        return 999.0, 999.0, False
    yaw_rad = math.radians(current_imu_yaw)
    c_yaw = math.cos(yaw_rad)
    s_yaw = math.sin(yaw_rad)
    world_x = current_imu_x + c_yaw * rel_x + s_yaw * rel_y
    world_y = current_imu_y - s_yaw * rel_x + c_yaw * rel_y
    return world_x, world_y, True


_BALL_DEFS = (
    (BALL_L_CLASS_ID, BALL_L_THRESHOLD, "ball_L", (160, 0, 255)),
    (BALL_R_CLASS_ID, BALL_R_THRESHOLD, "ball_R", (255, 128, 0)),
)

# 球 α-β 跟踪状态：class_id -> [x, y, r, vx, vy]（浮点，vx/vy 为像素速度/帧），
# 及连续未检出帧数。
_ball_filt = {}
_ball_miss = {}


def detect_balls(img):
    # 检测主车推杆上的左/右标记球：色块粗筛 + 定位（质心+形状门控 / 霍夫圆二选一，
    # 见 BALL_USE_HOUGH）。每色最多取一个，经 α-β 跟踪抑制帧间跳变，
    # 返回与 YOLO 目标同构的 obj dict 列表。
    out = []
    for class_id, thr, name, color in _BALL_DEFS:
        blobs = img.find_blobs(thr, pixels_threshold=BALL_BLOB_MIN_PIXELS,
                               area_threshold=BALL_BLOB_MIN_AREA, merge=True)
        if BALL_DEBUG_PRINT and BALL_DBG_BLOB:
            print("[%s] 1.blobs=%d (px_th=%d area_th=%d)"
                  % (name, len(blobs), BALL_BLOB_MIN_PIXELS, BALL_BLOB_MIN_AREA))
        best_x = 0.0
        best_y = 0.0
        best_r = 0.0
        best_score = -1.0
        bi = -1
        for b in blobs:
            bi += 1
            bx, by, bw, bh = b.rect()
            if BALL_DEBUG_PRINT and BALL_DBG_BLOB:
                print("[%s] 1.b%d rect=(%d,%d,%d,%d) px=%d"
                      % (name, bi, bx, by, bw, bh, b.pixels()))
            if not BALL_USE_HOUGH:
                # ── 质心定位：形状门控（长宽比+填充率）后直接取色块质心 ──
                short_side = bh if bw > bh else bw
                long_side = bw if bw > bh else bh
                if short_side == 0:
                    continue
                aspect = long_side / short_side
                fill = b.pixels() / (bw * bh)
                if aspect > BALL_SHAPE_ASPECT_MAX or fill < BALL_SHAPE_FILL_MIN:
                    if BALL_DEBUG_PRINT and BALL_DBG_CIRCLE:
                        print("[%s] 3.b%d SKIP shape aspect=%.2f(max%.1f) fill=%.2f(min%.2f)"
                              % (name, bi, aspect, BALL_SHAPE_ASPECT_MAX,
                                 fill, BALL_SHAPE_FILL_MIN))
                    continue
                if BALL_DEBUG_PRINT and BALL_DBG_CIRCLE:
                    print("[%s] 3.b%d shape OK aspect=%.2f fill=%.2f px=%d"
                          % (name, bi, aspect, fill, b.pixels()))
                if b.pixels() > best_score:
                    best_score = b.pixels()
                    best_x = float(b.cx())
                    best_y = float(b.cy())
                    best_r = 0.25 * (bw + bh)
                continue
            # ── 霍夫圆定位（BALL_USE_HOUGH=True）──
            rx = bx - BALL_ROI_PAD
            ry = by - BALL_ROI_PAD
            rw = bw + 2 * BALL_ROI_PAD
            rh = bh + 2 * BALL_ROI_PAD
            if rx < 0:
                rw += rx
                rx = 0
            if ry < 0:
                rh += ry
                ry = 0
            if rx + rw > IMG_W:
                rw = IMG_W - rx
            if ry + rh > IMG_H:
                rh = IMG_H - ry
            if rw < 2 * BALL_R_MIN or rh < 2 * BALL_R_MIN:
                if BALL_DEBUG_PRINT and BALL_DBG_ROI:
                    print("[%s] 2.b%d SKIP roi too SMALL w=%d h=%d (need>=%d)"
                          % (name, bi, rw, rh, 2 * BALL_R_MIN))
                continue
            if rw > BALL_ROI_MAX or rh > BALL_ROI_MAX:
                if BALL_DEBUG_PRINT and BALL_DBG_ROI:
                    print("[%s] 2.b%d SKIP roi too BIG w=%d h=%d (max=%d)"
                          % (name, bi, rw, rh, BALL_ROI_MAX))
                continue
            if BALL_DEBUG_PRINT and BALL_DBG_ROI:
                print("[%s] 2.b%d roi=(%d,%d,%d,%d) -> hough"
                      % (name, bi, rx, ry, rw, rh))
            circles = img.find_circles(threshold=BALL_CIRCLE_THRESHOLD,
                                       x_margin=BALL_XY_MARGIN,
                                       y_margin=BALL_XY_MARGIN,
                                       r_margin=BALL_XY_MARGIN,
                                       r_min=BALL_R_MIN, r_max=BALL_R_MAX,
                                       r_step=BALL_R_STEP,
                                       roi=(rx, ry, rw, rh))
            if BALL_DEBUG_PRINT and BALL_DBG_CIRCLE:
                if circles:
                    for c in circles:
                        # 直接打印圆对象，repr 含 x/y/r/magnitude，用于调霍夫阈值
                        print("[%s] 3.b%d circle:" % (name, bi), c)
                else:
                    print("[%s] 3.b%d NO circle (th=%d r=%d..%d step=%d)"
                          % (name, bi, BALL_CIRCLE_THRESHOLD,
                             BALL_R_MIN, BALL_R_MAX, BALL_R_STEP))
            # 同一球常出多个相邻候选；按 magnitude（物理置信度）选，
            # 比按半径选稳定——半径相近时不会帧间来回切换。
            for c in circles:
                if c.magnitude() > best_score:
                    best_score = c.magnitude()
                    best_x = float(c.x())
                    best_y = float(c.y())
                    best_r = float(c.r())
        if best_score < 0:
            miss = _ball_miss.get(class_id, 0) + 1
            _ball_miss[class_id] = miss
            filt = _ball_filt.get(class_id)
            if miss > BALL_SMOOTH_MISS_RESET:
                if filt is not None:
                    del _ball_filt[class_id]
            elif filt is not None:
                # 短暂丢检：按速度惯性外推，重捕获时预测位置贴近真实位置。
                filt[0] += filt[3]
                filt[1] += filt[4]
            if BALL_DEBUG_PRINT and BALL_DBG_POS:
                print("[%s] 4.MISS (miss=%d)" % (name, miss))
            continue
        _ball_miss[class_id] = 0
        # α-β 跟踪：预测→残差→修正位置与速度。匀速运动下稳态滞后≈0。
        zx = best_x
        zy = best_y
        zr = best_r
        filt = _ball_filt.get(class_id)
        if filt is None:
            filt = [zx, zy, zr, 0.0, 0.0]
            _ball_filt[class_id] = filt
        else:
            px = filt[0] + filt[3]
            py = filt[1] + filt[4]
            ex = zx - px
            ey = zy - py
            if ex * ex + ey * ey > BALL_SMOOTH_GATE_PX * BALL_SMOOTH_GATE_PX:
                # 预测失效（急变向/野点后重捕获）：重置为当前观测，速度清零重学。
                filt[0] = zx
                filt[1] = zy
                filt[2] = zr
                filt[3] = 0.0
                filt[4] = 0.0
            else:
                filt[0] = px + BALL_TRACK_ALPHA * ex
                filt[1] = py + BALL_TRACK_ALPHA * ey
                filt[2] = filt[2] + BALL_TRACK_ALPHA * (zr - filt[2])
                vx = filt[3] + BALL_TRACK_BETA * ex
                vy = filt[4] + BALL_TRACK_BETA * ey
                if vx > BALL_VEL_MAX_PX:
                    vx = BALL_VEL_MAX_PX
                elif vx < -BALL_VEL_MAX_PX:
                    vx = -BALL_VEL_MAX_PX
                if vy > BALL_VEL_MAX_PX:
                    vy = BALL_VEL_MAX_PX
                elif vy < -BALL_VEL_MAX_PX:
                    vy = -BALL_VEL_MAX_PX
                filt[3] = vx
                filt[4] = vy
        # 超前只作用于发送的 rel（补采集→处理→发送延迟）；
        # 画框/打印像素用滤波位置本身，屏幕上框应咬住球。
        fx = filt[0] + filt[3] * BALL_PRED_LEAD_FRAMES
        fy = filt[1] + filt[4] * BALL_PRED_LEAD_FRAMES
        fr = filt[2]
        cx = int(filt[0] + 0.5)
        cy = int(filt[1] + 0.5)
        cr = int(fr + 0.5)
        ipm_x, ipm_y = compute_obj_ipm_pos(fx, fy)
        rel_x = ipm_x * BALL_IPM_SCALE
        rel_y = ipm_y * BALL_IPM_SCALE + ROD_OFFSET_CM
        if BALL_DEBUG_PRINT and BALL_DBG_POS:
            print("[%s] 4.HIT px=(%d,%d) r=%d ipm=(%.1f,%.1f) scale=%.2f rel=(%.1f,%.1f)"
                  % (name, cx, cy, cr, ipm_x, ipm_y,
                     BALL_IPM_SCALE, rel_x, rel_y))
        world_x, world_y, world_valid = compute_obj_world_from_rel(rel_x, rel_y)
        out.append(
            {
                "send_x": rel_x,
                "send_y": rel_y,
                "world_x": world_x,
                "world_y": world_y,
                "world_valid": world_valid,
                "class_id": class_id,
                "label": name,
                "score": 0.9,
                "track_id": 0,
                "cx": cx,
                "cy": cy,
                "bbox": (cx - cr, cy - cr, cx + cr, cy + cr),
            }
        )
        if DRAW_DEBUG:
            img.draw_circle(cx, cy, cr, color=color, thickness=2)
            img.draw_string(cx - cr, max(0, cy - cr - 12),
                            "%s(%.0f,%.0f)" % (name, rel_x, rel_y),
                            color=color, scale=1)
    return out


def _track_distance_ok(obj, tr, world_gate):
    if obj["class_id"] != tr["class_id"]:
        return False, 1e9
    if not obj.get("world_valid", False) or not tr.get("world_valid", False):
        return False, 1e9
    dxw = obj["world_x"] - tr["world_x"]
    dyw = obj["world_y"] - tr["world_y"]
    world_d2 = dxw * dxw + dyw * dyw
    if world_d2 > world_gate * world_gate:
        return False, 1e9
    return True, world_d2

def _copy_obj_to_track(tr, obj):
    tr["class_id"] = obj["class_id"]
    tr["label"] = obj.get("label", "")
    tr["score"] = obj.get("score", 0.0)
    tr["send_x"] = obj["send_x"]
    tr["send_y"] = obj["send_y"]
    tr["cx"] = obj["cx"]
    tr["cy"] = obj["cy"]
    tr["world_x"] = obj.get("world_x", 999.0)
    tr["world_y"] = obj.get("world_y", 999.0)
    tr["world_valid"] = obj.get("world_valid", False)
    tr["miss"] = 0

def _class_track_limit(class_id):
    if current_obj_count_mode == 1 and 1 <= class_id <= 5:
        # 未知构成模式允许同类重复；上限随当前全局剩余数变化。
        limit = current_obj_total_remaining
        if limit < 1:
            limit = 1
        if limit > MAX_OBJECTS_PER_CLASS:
            limit = MAX_OBJECTS_PER_CLASS
        return limit
    real_count = OBJECT_COUNT_BY_CLASS.get(class_id, 0)
    if real_count <= 0:
        return MAX_OBJECTS_PER_CLASS
    limit = int(real_count * TRACK_COUNT_FACTOR + 0.999)
    if limit < real_count:
        limit = real_count
    return limit

def _new_track(obj):
    global _next_track_id
    if obj.get("score", 0.0) < MIN_SCORE_CREATE_TRACK:
        obj["track_id"] = 0
        return None
    if not obj.get("world_valid", False):
        obj["track_id"] = 0
        return None
    class_id = obj["class_id"]
    same_class = [tr for tr in _tracks if tr["class_id"] == class_id]
    if len(same_class) >= _class_track_limit(class_id):
        obj["track_id"] = 0
        return None

    used_ids = [tr["track_id"] for tr in _tracks]
    for _ in range(99):
        if _next_track_id not in used_ids:
            break
        _next_track_id += 1
        if _next_track_id > 99:
            _next_track_id = 1
    tr = {
        "track_id": _next_track_id,
        "class_id": obj["class_id"],
        "label": obj.get("label", ""),
        "score": obj.get("score", 0.0),
        "send_x": obj["send_x"],
        "send_y": obj["send_y"],
        "cx": obj["cx"],
        "cy": obj["cy"],
        "world_x": obj.get("world_x", 999.0),
        "world_y": obj.get("world_y", 999.0),
        "world_valid": obj.get("world_valid", False),
        "miss": 0,
    }
    _next_track_id += 1
    if _next_track_id > 99:
        _next_track_id = 1
    _tracks.append(tr)
    obj["track_id"] = tr["track_id"]
    return tr

def _assign_tracks(obj_list):
    global _tracks
    if current_lock_mode == 99 and current_lock_track_id > 0:
        lock_id = current_lock_track_id
        locked = None
        for tr in _tracks:
            if tr["track_id"] == lock_id:
                locked = tr
                break

        if locked is not None:
            best = None
            best_cost = 1e9
            for obj in obj_list:
                if obj.get("score", 0.0) < MIN_SCORE_UPDATE_TRACK:
                    continue
                ok, cost = _track_distance_ok(obj, locked, LOCK_WORLD_GATE)
                if ok and cost < best_cost:
                    best = obj
                    best_cost = cost
            if best is not None:
                _copy_obj_to_track(locked, best)
                best["track_id"] = locked["track_id"]
            else:
                locked["miss"] += 1
                if locked["miss"] > TRACK_MAX_MISS_FRAMES:
                    _tracks = [tr for tr in _tracks if tr["track_id"] != lock_id]

        alive = []
        for tr in _tracks:
            if tr["track_id"] == lock_id:
                alive.append(tr)
            else:
                tr["miss"] += 1
                if tr["miss"] <= TRACK_MAX_MISS_FRAMES:
                    alive.append(tr)
        _tracks = alive
        return

    used_ids = set()
    for obj in obj_list:
        if obj.get("score", 0.0) < MIN_SCORE_UPDATE_TRACK:
            obj["track_id"] = 0
            continue
        best_tr = None
        best_cost = 1e9
        for tr in _tracks:
            if tr["track_id"] in used_ids:
                continue
            ok, cost = _track_distance_ok(obj, tr, TRACK_WORLD_GATE)
            if ok and cost < best_cost:
                best_tr = tr
                best_cost = cost
        if best_tr is not None:
            _copy_obj_to_track(best_tr, obj)
            obj["track_id"] = best_tr["track_id"]
            used_ids.add(best_tr["track_id"])
        else:
            new_tr = _new_track(obj)
            if new_tr is not None:
                used_ids.add(new_tr["track_id"])

    alive = []
    for tr in _tracks:
        if tr["track_id"] in used_ids:
            alive.append(tr)
        else:
            tr["miss"] += 1
            if tr["miss"] <= TRACK_MAX_MISS_FRAMES:
                alive.append(tr)
    _tracks = alive

def uart_send_objects(vis_yaw, rod_x, rod_y, line_cx, obj_list):
    # 发送 Frame2 v3（固定 12 + MAX_SEND_OBJECTS×4=40 字节）：OpenART → RT1021。
    # [0-1] 0xAA 0x55  [2] 0x07  [3-4] vis_yaw  [5-6] rod_x  [7-8] rod_y
    # 从[9]开始固定7槽：slot0/1=普通物体，slot2=car，slot3=ball_l，
    # slot4=ball_r，slot5=cone，slot6=brick。
    # 每槽4字节：[rel_x×1][rel_y×1][class_id][track_id]，class_id=255 表示空槽；
    # 候选连续帧确认期间 slot0 为空且 track_id=254，表示有候选但尚未锁定。
    # [目标槽后2字节] 黄色线矩形框中心原始像素 x（0~320，999 表示无效）。
    # [末] CRC8(覆盖[2..末-1])
    global _tx_frame2_build, _tx_frame2_send, _tx_frame2_sent_cnt
    f = _tx_frame2_build
    f[0] = 0xAA
    f[1] = 0x55
    f[2] = _FRAME2_TYPE
    _pack_i16(f, 3, vis_yaw)
    _pack_i16(f, 5, rod_x)
    _pack_i16(f, 7, rod_y)
    off = 9
    used_idx = -1
    for i in range(SEND_OBJECT_SLOTS):
        best_idx = -1
        if (i == 0 and current_target_sel_id > 0 and len(obj_list)
                and obj_list[0]["class_id"] == current_target_sel_id):
            best_idx = 0
        elif not (i == 0 and current_target_sel_id > 0):
            best_d2 = 99999999.0
            for j in range(len(obj_list)):
                if j == used_idx:
                    continue
                obj = obj_list[j]
                if obj["class_id"] <= 0 or obj["class_id"] > 5:
                    continue
                if i == 1 and current_target_sel_id > 0 and obj["class_id"] == current_target_sel_id:
                    continue
                d2 = obj["send_x"] * obj["send_x"] + obj["send_y"] * obj["send_y"]
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = j
        if best_idx >= 0:
            used_idx = best_idx
            obj = obj_list[best_idx]
            _pack_i8(f, off,     obj["send_x"])
            _pack_i8(f, off + 1, obj["send_y"])
            f[off + 2] = obj["class_id"] & 0xFF
            f[off + 3] = obj.get("track_id", 0) & 0xFF
        else:
            f[off] = 0
            f[off + 1] = 0
            f[off + 2] = INVALID_OBJECT_ID
            if i == 0 and _leader_sel_observe_t0 >= 0:
                f[off + 3] = TARGET_OBSERVING_TRACK_MARKER
            else:
                f[off + 3] = 0
        off += 4
    for i in range(5):
        best_idx = -1
        best_d2 = 99999999.0
        for j in range(len(obj_list)):
            obj = obj_list[j]
            if ((i == 0 and obj["class_id"] != 0) or
                    (i == 1 and obj["class_id"] != BALL_L_CLASS_ID) or
                    (i == 2 and obj["class_id"] != BALL_R_CLASS_ID) or
                    (i == 3 and obj["class_id"] != CONE_CLASS_ID) or
                    (i == 4 and obj["class_id"] != BRICK_CLASS_ID)):
                continue
            d2 = obj["send_x"] * obj["send_x"] + obj["send_y"] * obj["send_y"]
            if d2 < best_d2:
                best_d2 = d2
                best_idx = j
        if best_idx >= 0:
            obj = obj_list[best_idx]
            _pack_i8(f, off,     obj["send_x"])
            _pack_i8(f, off + 1, obj["send_y"])
            f[off + 2] = obj["class_id"] & 0xFF
            f[off + 3] = obj.get("track_id", 0) & 0xFF
        else:
            f[off] = 0
            f[off + 1] = 0
            f[off + 2] = INVALID_OBJECT_ID
            f[off + 3] = 0
        off += 4
    line_off = 9 + MAX_SEND_OBJECTS * 4
    _pack_i16(f, line_off, line_cx)
    crc_end = line_off + 2
    f[crc_end] = _crc8(f, 2, crc_end)

    # 打包完成后当场发送，不再经由定时器的 40 Hz 槽位。
    # 走定时器有两个代价：① 每次发布要等 0~25 ms 的槽位；② 相位跨越时若恰好没有
    # 新发布，这个槽就浪费掉、再等一个周期——旧代码靠无限重传掩盖了这一点，
    # 一旦改成"发完即清"，帧率高于槽位率时就会大量丢发。
    # 40 字节 @230400 只阻塞 1.74 ms，直接发反而更简单更快。
    old_send = _tx_frame2_send
    _tx_frame2_send = f
    _tx_frame2_build = old_send
    uart.write(_tx_frame2_send)
    _tx_frame2_sent_cnt += 1


def with_cars(selected, all_objects):
    out = []
    for obj in selected:
        out.append(obj)
    for obj in all_objects:
        # car/cone/brick 使用各自固定槽，必须保留到最终发送列表中；
        # 前两个普通目标槽会自行忽略这些类别。
        if obj["class_id"] in (0, CONE_CLASS_ID, BRICK_CLASS_ID) and obj not in out:
            out.append(obj)
    return out


def with_route_objects(selected, all_objects):
    out = []
    for obj in selected:
        if obj not in out:
            out.append(obj)
    for obj in all_objects:
        class_id = obj["class_id"]
        if class_id == 0:
            continue
        if not obj_enabled(class_id):
            continue
        if obj not in out:
            out.append(obj)
    for obj in all_objects:
        if obj["class_id"] in (0, CONE_CLASS_ID, BRICK_CLASS_ID) and obj not in out:
            out.append(obj)
    return out


def obj_enabled(class_id):
    if class_id == 0:
        return True
    if 1 <= class_id <= 5:
        if current_obj_count_mode == 1:
            return current_obj_total_remaining > 0
        return current_obj_remain[class_id] > 0
    return False


def tick(_):
    # 只负责收 Frame1。Frame2 的发送已移到 uart_send_objects 里当场完成，
    # 不再受定时器槽位和相位对齐的影响。
    global t, _tick_cnt
    t += 1
    _tick_cnt += 1
    uart_get()


# ── 模型加载 ──────────────────────────────────────────────────
# 1) 先加载模型、后启动 Timer：避免大文件 SD 读取期间被定时器中断
#    （UART 读）干扰，这是间歇性读失败的嫌疑源之一。
# 2) 优先 load_to_fb=True 把模型放进帧缓冲区而非 MicroPython 堆：
#    新代码字节码变大后堆更紧，堆中途分配失败会表现为读取错误。
# 3) 读失败不崩溃：打印警告后禁用 YOLO，黄线/球检测照常运行。
gc.collect()
print("boot: mem_free=", gc.mem_free())
net = None
try:
    try:
        net = tf.load(YOLO_MODEL_PATH, load_to_fb=True)
    except TypeError:
        # 本固件 tf.load 不支持 load_to_fb 参数时退回普通加载
        net = tf.load(YOLO_MODEL_PATH)
except Exception as e:
    print("!! YOLO model load FAILED:", e)
    print("!! mem_free=%d; check SD file size vs PC copy" % gc.mem_free())
letterbox_fb = sensor.alloc_extra_fb(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, sensor.RGB565)
gc.collect()

tim = Timer(1, _UART_TIMER_HZ)
tim.callback(tick)

# 数据集采集关闭时完全不碰 SD 写（建目录/EPOCHNUM 写+sync）：
# SD 状态异常时这两个写操作可能阻塞挂死，导致主循环永远启动不了、
# 一帧不发（握手失败且 IDE 无法停止）。
if SAVE_IMAGE_ENABLE:
    ensure_dataset_dir()
    EPOCHNUM = load_next_epoch_num()
else:
    EPOCHNUM = 0
save_img_num = 0


_last_obj_list = []   # 上一帧目标列表，用于 YOLO 推理前提前发送

# ── 帧率测量（台架联机时用）────────────────────────────────────────────────
# Frame2 的有效更新率就等于这里的帧率，所以它是上下位机延迟的主导项。
# 下位机 LCD 上的 i: 被 40 Hz 发送上限量化成 ~25 ms 台阶，读不出真实值，必须在这里读。
# 脱机跑任务务必置 False：USB CDC 无主机时 print 可能阻塞主循环。
FPS_PRINT_ENABLE = False
FPS_PRINT_EVERY_N_FRAMES = 30
_fps_frame_cnt = 0
_fps_last_ms = time.ticks_ms()
_fps_last_tick = 0
_fps_last_tx = 0

while True:
    clock.tick()
    if FPS_PRINT_ENABLE:
        _fps_frame_cnt += 1
        if _fps_frame_cnt >= FPS_PRINT_EVERY_N_FRAMES:
            _fps_frame_cnt = 0
            _now = time.ticks_ms()
            _el = time.ticks_diff(_now, _fps_last_ms)
            if _el > 0:
                # timerHz 是定时器的实测频率——不要相信 Timer(1,N) 里的 N，以这个数为准。
                # txHz 是 Frame2 的实际发出频率，应当等于 fps×2（每帧发布两次）。
                # 下位机 LCD 的 i: 若明显大于 1000/txHz，问题在链路而不在帧率。
                print("fps %.1f  frame %.0f ms  timerHz %.0f  txHz %.1f  mem %d" % (
                    clock.fps(), _el / float(FPS_PRINT_EVERY_N_FRAMES),
                    (_tick_cnt - _fps_last_tick) * 1000.0 / _el,
                    (_tx_frame2_sent_cnt - _fps_last_tx) * 1000.0 / _el,
                    gc.mem_free()))
            _fps_last_ms = _now
            _fps_last_tick = _tick_cnt
            _fps_last_tx = _tx_frame2_sent_cnt
    try:
        img = sensor.snapshot()
        if SAVE_IMAGE_ENABLE:
            save_img_num += 1
            if save_img_num % SAVE_IMAGE_EVERY_N_FRAMES == 0:
                image_pat = DATASET_DIR + "/" + str(EPOCHNUM) + "_" + str(save_img_num // SAVE_IMAGE_EVERY_N_FRAMES) + ".jpg"
                try:
                    img.save(image_pat, quality=SAVE_IMAGE_QUALITY)
                except Exception as e:
                    _reraise_ide_stop(e)
                    print("save image ERR:", e)
        if DRAW_DEBUG:
            img_copy = img.copy()
    except Exception as e:
        _reraise_ide_stop(e)
        print("snapshot ERR:", e)
        gc.collect()
        continue

    vis_yaw_calc = None
    cam_x_float = None
    cam_y_float = None
    line_rect_cx = 999.0
    line_rect_cx_candidate = 999.0

    blobs = img.find_blobs(YELLOW_THRESHOLD, pixels_threshold=35, area_threshold=35, merge=True,margin=60) if LINE_ENABLE else []
    # 黄边线必须至少有一侧贴住画面边缘；画面内部的黄色网球即使形状筛选误判，
    # 也不能成为 line_rect_cx，更不能参与黄线角度/坐标解算。
    blobs = [b for b in blobs if _blob_is_line(b) and _blob_touches_frame_edge(b)]
    points = []
    final_draw_cmd = None
    lines_to_process = []

    _exclude_rects = []

    if blobs:
        max_blob = max(blobs, key=lambda b: b.pixels())
        bx, by, bw, bh = max_blob.rect()
        line_rect_cx_candidate = bx + bw * 0.5
        if DRAW_DEBUG:
            img.draw_rectangle(bx, by, bw, bh, color=(0, 255, 0))

        # 排除区只在真的要做射线扫描时才构造：没有贴边黄线候选时 points 必然为空，
        # 这两段（尤其第二次全图 find_blobs）的结果根本不会被读到。
        # 排除区 1：上帧 YOLO 检测到的货物 bbox（扩大8px），有一帧延迟但覆盖范围准
        for _eo in _last_obj_list:
            if _eo.get("class_id", 0) != 0:
                _ebp = _eo.get("bbox", None)
                if _ebp is not None:
                    _exclude_rects.append((_ebp[0] - 8, _ebp[1] - 8, _ebp[2] + 8, _ebp[3] + 8))

        # 排除区 2：当帧不merge的黄色blob中，形状不像线的（圆形/方形，如网球），实时无延迟
        _raw_blobs = img.find_blobs(YELLOW_THRESHOLD,
                                    pixels_threshold=50, area_threshold=50, merge=False)
        for _rb in _raw_blobs:
            if not _blob_is_line(_rb):
                _rx, _ry, _rw, _rh = _rb.rect()
                _exclude_rects.append((_rx - 8, _ry - 8, _rx + _rw + 8, _ry + _rh + 8))

        # 每条射线先与 blob 矩形求交，只在相交的 d 区间上步进。
        # 原来固定跑 0..MAX_DIST 共 MAX_DIST/STEP=80 步，其中绝大多数点落在矩形外，
        # 纯粹是空转的 Python 迭代。裁剪保留的 d 集合是原集合的超集（下面仍保留
        # 精确的矩形判定），所以命中点与原来完全一致，只是不再空转。
        _RAY_EPS = 1e-6
        _d_max = float(MAX_DIST - 1)
        # 下界放宽 1 像素：int() 是向零取整而非向下取整，实数 x∈(-1,0] 会被截成 0，
        # 在 bx=0 的矩形里属于合法命中。黄线 blob 必须贴画面边缘，bx/by 为 0 是常态，
        # 按 [bx, ...] 严格裁会把这些边界点整片丢掉。放宽只会多留几个候选 d，
        # 循环内的精确矩形判定照旧过滤。
        _clip_x_lo = bx - 1
        _clip_x_hi = bx + bw
        _clip_y_lo = by - 1
        _clip_y_hi = by + bh
        angle_step = (ANGLE_END - ANGLE_START) / (RAY_COUNT - 1)
        for i in range(RAY_COUNT):
            rad = math.radians(ANGLE_START + i * angle_step)
            cos_a = math.cos(rad)
            sin_a = math.sin(rad)
            d_lo = 0.0
            d_hi = _d_max
            # X 方向限幅：x(d)=ORIGIN_X+d*cos_a 必须落在 [_clip_x_lo, _clip_x_hi]
            if cos_a > _RAY_EPS:
                t0 = (_clip_x_lo - ORIGIN_X) / cos_a
                t1 = (_clip_x_hi - ORIGIN_X) / cos_a
            elif cos_a < -_RAY_EPS:
                t0 = (_clip_x_hi - ORIGIN_X) / cos_a
                t1 = (_clip_x_lo - ORIGIN_X) / cos_a
            else:
                # 射线几乎垂直：x 恒为 ORIGIN_X，不在矩形横向范围内就整条跳过
                if ORIGIN_X < _clip_x_lo or ORIGIN_X > _clip_x_hi:
                    continue
                t0 = d_lo
                t1 = d_hi
            if t0 > d_lo:
                d_lo = t0
            if t1 < d_hi:
                d_hi = t1
            # Y 方向限幅：y(d)=ORIGIN_Y+d*sin_a 必须落在 [_clip_y_lo, _clip_y_hi]
            if sin_a > _RAY_EPS:
                t0 = (_clip_y_lo - ORIGIN_Y) / sin_a
                t1 = (_clip_y_hi - ORIGIN_Y) / sin_a
            elif sin_a < -_RAY_EPS:
                t0 = (_clip_y_hi - ORIGIN_Y) / sin_a
                t1 = (_clip_y_lo - ORIGIN_Y) / sin_a
            else:
                if ORIGIN_Y < _clip_y_lo or ORIGIN_Y > _clip_y_hi:
                    continue
                t0 = d_lo
                t1 = d_hi
            if t0 > d_lo:
                d_lo = t0
            if t1 < d_hi:
                d_hi = t1
            if d_lo > d_hi:
                continue
            # 对齐回原来的 0, STEP, 2*STEP... 网格，保证取样点与改前一致
            d = int(d_lo / STEP) * STEP
            if d < d_lo:
                d += STEP
            while d <= d_hi:
                x = int(ORIGIN_X + d * cos_a)
                y = int(ORIGIN_Y + d * sin_a)
                if bx <= x < bx + bw and by <= y < by + bh:
                    if check_lab_color(img, x, y, YELLOW_THRESHOLD):
                        # 检查是否命中货物排除区，是则继续沿射线穿过物体找真正的黄线
                        _in_excl = False
                        for _er in _exclude_rects:
                            if _er[0] <= x <= _er[2] and _er[1] <= y <= _er[3]:
                                _in_excl = True
                                break
                        if not _in_excl:
                            points.append((x, y))
                            break
                d += STEP

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

    line_dist_cm = None
    if best_line:
        # 只有贴边候选确实采样并拟合出有效直线后，才允许对外发布黄线 x。
        line_rect_cx = line_rect_cx_candidate
        t_p1 = transform_point(best_line[0][0], best_line[0][1], PERSPECTIVE_MATRIX)
        t_p2 = transform_point(best_line[1][0], best_line[1][1], PERSPECTIVE_MATRIX)
        trend = line_trend_img(best_line[0], best_line[1])
        c_type, c_val, line_dist_cm, v_yaw = solve_pose(
            current_imu_yaw,
            t_car,
            t_p1,
            t_p2,
            trend
        )

        #if line_dist_cm is not None:
         #   print("RAW_LINE_DIST_CM=%.2f" % line_dist_cm)
        #else:
         #   print("RAW_LINE_DIST_CM=None")

        if c_type is not None:
            vis_yaw_calc = v_yaw
            if c_type == "X":
                cam_x_float = c_val
            elif c_type == "Y":
                cam_y_float = c_val

    # 可视化：画出所有检测到的边线段（蓝色），并在左上角显示距离
    if DRAW_DEBUG:
        for _vl in lines_to_process:
            img.draw_line(_vl[0][0], _vl[0][1], _vl[1][0], _vl[1][1],
                          color=(0, 100, 255), thickness=2)
        if line_dist_cm is not None:
            img.draw_string(2, 2, "D=%.1fcm" % line_dist_cm, color=(0, 255, 0), scale=1)
            # print("LINE dist=%.1fcm yaw=%.1fdeg" % (
            #     line_dist_cm,
            #     vis_yaw_calc if vis_yaw_calc is not None else 999.0))

    rod_x_val = 999.0
    rod_y_val = 999.0
    yaw_rad = math.radians(current_imu_yaw)
    if cam_x_float is not None:
        rod_x_val = cam_x_float - ROD_OFFSET_CM * math.sin(yaw_rad)
    if cam_y_float is not None:
        rod_y_val = cam_y_float - ROD_OFFSET_CM * math.cos(yaw_rad)


    #if vis_yaw_calc is not None:
     #   print("VIS_FIX yaw=%.1f x=%.1f y=%.1f" % (vis_yaw_calc, rod_x_val, rod_y_val))
    #else:
     #   print("VIS_FIX none")

    # YOLO 前提前发布一次：当前帧位姿 + 上一帧目标，供 40 Hz 定时发送使用。
    uart_send_objects(
        vis_yaw_calc if vis_yaw_calc is not None else 999.0,
        rod_x_val, rod_y_val, line_rect_cx, _last_obj_list)

    _ball_objs = detect_balls(img) if BALL_ENABLE else []

    if MODEL_ENABLE and net is not None:
        #detections = apply_center_distance_suppression(detections)
        if DRAW_DEBUG:
            detections = collect_detections(net, img_copy, letterbox_fb)
            draw_detections(img, detections)
        else:
            detections = collect_detections(net, img, letterbox_fb)
    else:
        detections = []

    obj_list = []
    for det in detections:
        bottom_x, bottom_y = det["bottom"]
        x1, y1, x2, y2 = det["bbox"]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        ipm_x, ipm_y = compute_obj_ipm_pos(bottom_x, bottom_y)
        class_id, class_name = classify_detection(img, det)
        if class_id == 3:
            margin = RED_SANDBAG_EDGE_MARGIN_PX
            if (x1 < margin or y1 < margin
                    or x2 > IMG_W - 1 - margin
                    or y2 > IMG_H - 1 - margin):
                # 边缘裁切会破坏沙包的完整外形，并可能把红砖稳定误分为红沙包。
                # 在obj_list/track/候选确认之前丢弃，使SEARCH的连续两帧计数
                # 在该帧自然中断；brick检测结果仍按原路径保留。
                continue
        color = CLASS_COLORS.get(class_id, (255, 255, 255))
        # 物体相对车中心坐标（车体系，cm）
        # ipm_y 以摄像头底行中点为原点，加上 ROD_OFFSET_CM 平移到车中心
        if class_id == CONE_CLASS_ID:
            rel_x = ipm_x * CONE_IPM_SCALE
            rel_y = ipm_y * CONE_IPM_SCALE + ROD_OFFSET_CM
        else:
            rel_x = ipm_x
            rel_y = ipm_y + ROD_OFFSET_CM
        world_x, world_y, world_valid = compute_obj_world_from_rel(rel_x, rel_y)

        obj_list.append(
            {
                "send_x": rel_x,
                "send_y": rel_y,
                "world_x": world_x,
                "world_y": world_y,
                "world_valid": world_valid,
                "class_id": class_id,
                "label": class_name,
                "score": det["score"],
                "track_id": 0,
                "cx": center_x,
                "cy": center_y,
                "bbox": (x1, y1, x2, y2),
            }
        )
        if DRAW_DEBUG:
            img.draw_string(
                x1,
                max(0, y1 - 12),
                "%s %.2f" % (class_name, det["score"]),
                color=color,
                scale=1,
            )
            img.draw_string(
                bottom_x,
                min(IMG_H - 10, bottom_y + 4),
                "(%.0f,%.0f)" % (rel_x, rel_y),
                color=color,
                scale=1,
            )

    # ── 物体筛选 ──
   # global _sel_locked, _sel_lock_rel_x, _sel_lock_rel_y, _sel_lock_type, _sel_miss_frames

    all_obj_list = obj_list[:]
    obj_list = [obj for obj in obj_list if obj_enabled(obj["class_id"])]

    if current_target_rel_x < 900.0:
        # ── 从车位置锁定模式：按相对坐标找最近匹配 ──
        _sel_locked      = False
        _sel_miss_frames = 0
        _leader_sel_candidate_type = 0
        _leader_sel_candidate_rel_x = 999.0
        _leader_sel_candidate_rel_y = 999.0
        _leader_sel_candidate_count = 0
        _leader_sel_observe_t0 = -1
        if obj_list:
            if current_target_sel_id > 0:
                match_list = [obj for obj in obj_list
                              if obj["class_id"] == current_target_sel_id]
            else:
                match_list = [obj for obj in obj_list
                              if 1 <= obj["class_id"] <= 5]
            best_obj = None
            best_d   = 999999.0
            for obj in match_list:
                dx = obj["send_x"] - current_target_rel_x
                dy = obj["send_y"] - current_target_rel_y
                if abs(dx) > FOLLOWER_TARGET_XY_GATE or abs(dy) > FOLLOWER_TARGET_XY_GATE:
                    continue
                d  = dx * dx + dy * dy
                if d < best_d:
                    best_d   = d
                    best_obj = obj
            if best_obj is not None:
                obj_list = with_route_objects([best_obj], all_obj_list)
            else:
                obj_list = with_route_objects([], all_obj_list) if current_target_sel_id > 0 else with_cars([], all_obj_list)
        else:
            # 即使当前没有可搬运目标，也继续保留 car/cone/brick 的固定槽数据。
            obj_list = with_route_objects([], all_obj_list) if current_target_sel_id > 0 else with_cars([], all_obj_list)

    else:
        # ── 主车自主选择模式：摄像头独立锁定最优搬运物体 ──
        cargo = [obj for obj in obj_list if 1 <= obj["class_id"] <= 5]
        if current_target_sel_id <= 0:
            max_select_d2 = _LEADER_SEL_MAX_DIST_CM * _LEADER_SEL_MAX_DIST_CM
            cargo = [obj for obj in cargo
                     if obj["send_x"] * obj["send_x"] + obj["send_y"] * obj["send_y"]
                     <= max_select_d2]

        if _sel_locked:
            _leader_sel_candidate_type = 0
            _leader_sel_candidate_rel_x = 999.0
            _leader_sel_candidate_rel_y = 999.0
            _leader_sel_candidate_count = 0
            _leader_sel_observe_t0 = -1
            # 尝试找回已锁定目标（按类型 + 距离）
            found   = None
            best_d2 = _SEL_REACQUIRE_R * _SEL_REACQUIRE_R
            for obj in cargo:
                if obj["class_id"] == _sel_lock_type:
                    dx = obj["send_x"] - _sel_lock_rel_x
                    dy = obj["send_y"] - _sel_lock_rel_y
                    d2 = dx * dx + dy * dy
                    if d2 < best_d2:
                        best_d2 = d2
                        found   = obj
            if found is not None:
                _sel_lock_rel_x  = found["send_x"]
                _sel_lock_rel_y  = found["send_y"]
                _sel_miss_frames = 0
                if current_target_sel_id > 0:
                    obj_list = with_route_objects([found], all_obj_list)
                else:
                    obj_list = with_cars([found], all_obj_list)
            else:
                _sel_miss_frames += 1
                max_miss = (_SEL_MAX_MISS_APPROACH if current_target_sel_id > 0
                            else _SEL_MAX_MISS)
                if _sel_miss_frames > max_miss:
                    _sel_locked      = False
                    _sel_miss_frames = 0
                if current_target_sel_id > 0:
                    obj_list = with_route_objects([], all_obj_list)
                else:
                    obj_list = with_cars([], all_obj_list)

        else:
            # 重新选择：APPROACH 阶段只从约束类型选，其余阶段从可搬运目标中选最近物体。
            if current_target_sel_id > 0:
                candidates = [obj for obj in cargo
                              if obj["class_id"] == current_target_sel_id]
            else:
                candidates = cargo
            best = None
            best_dist2 = 1e9
            for obj in candidates:
                dist2 = obj["send_x"] * obj["send_x"] + obj["send_y"] * obj["send_y"]
                if dist2 < best_dist2:
                    best_dist2 = dist2
                    best = obj
            if best is not None:
                if current_target_sel_id > 0:
                    _leader_sel_candidate_type = 0
                    _leader_sel_candidate_rel_x = 999.0
                    _leader_sel_candidate_rel_y = 999.0
                    _leader_sel_candidate_count = 0
                    _leader_sel_observe_t0 = -1
                    _sel_locked      = True
                    _sel_lock_rel_x  = best["send_x"]
                    _sel_lock_rel_y  = best["send_y"]
                    _sel_lock_type   = best["class_id"]
                    _sel_miss_frames = 0
                    obj_list = with_route_objects([best], all_obj_list)
                else:
                    same_candidate = (
                        best["class_id"] == _leader_sel_candidate_type
                        and (
                            (best["send_x"] - _leader_sel_candidate_rel_x)
                            * (best["send_x"] - _leader_sel_candidate_rel_x)
                            + (best["send_y"] - _leader_sel_candidate_rel_y)
                            * (best["send_y"] - _leader_sel_candidate_rel_y)
                            <= _LEADER_SEL_CONFIRM_GATE_CM * _LEADER_SEL_CONFIRM_GATE_CM
                        )
                    )
                    if same_candidate:
                        _leader_sel_candidate_count += 1
                    else:
                        _leader_sel_candidate_type = best["class_id"]
                        _leader_sel_candidate_count = 1
                    _leader_sel_candidate_rel_x = best["send_x"]
                    _leader_sel_candidate_rel_y = best["send_y"]
                    _leader_sel_observe_t0 = time.ticks_ms()
                    if _leader_sel_candidate_count < _LEADER_SEL_CONFIRM_FRAMES:
                        # 首帧候选只标记“正在确认”，不发布普通搬运目标。
                        obj_list = with_cars([], all_obj_list)
                    else:
                        _leader_sel_candidate_type = 0
                        _leader_sel_candidate_rel_x = 999.0
                        _leader_sel_candidate_rel_y = 999.0
                        _leader_sel_candidate_count = 0
                        _leader_sel_observe_t0 = -1
                        _sel_locked      = True
                        _sel_lock_rel_x  = best["send_x"]
                        _sel_lock_rel_y  = best["send_y"]
                        _sel_lock_type   = best["class_id"]
                        _sel_miss_frames = 0
                        obj_list = with_cars([best], all_obj_list)
            else:
                if current_target_sel_id > 0:
                    obj_list = with_route_objects([], all_obj_list)
                else:
                    # 候选中断后，下次重新看到物体时重新累计连续两帧。
                    _leader_sel_candidate_type = 0
                    _leader_sel_candidate_rel_x = 999.0
                    _leader_sel_candidate_rel_y = 999.0
                    _leader_sel_candidate_count = 0
                    _leader_sel_observe_t0 = -1
                    obj_list = with_cars([], all_obj_list)

    # 标记球插在选中目标之后：下位机按顺序截入 CAM_OBJ 槽位，保证球不被后段路障/车目标挤掉。
    if _ball_objs:
        obj_list = obj_list[:1] + _ball_objs + obj_list[1:]

    # ── 调试输出（print，不画图）──
    _last_obj_list = obj_list
    # 视觉 yaw 不再经过“接近90度/接近IMU/累计次数”门控；本帧能解算就直接发布。
    send_vis_yaw = vis_yaw_calc if vis_yaw_calc is not None else 999.0
    uart_send_objects(send_vis_yaw, rod_x_val, rod_y_val, line_rect_cx, obj_list)


    gc.collect()
