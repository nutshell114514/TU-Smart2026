import gc
import time

import pyb
import sensor
import tf


import sensor, time
# ==========================================
# 1. 基础硬件初始化
# ==========================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.skip_frames(time = 2000)   # 等待感光元件上电稳定

# ==========================================
# 2.写死静态白平衡
# ==========================================

FIXED_RGB_GAIN = (107, 64, 91)
# 彻底关闭自动白平衡，强行将底层寄存器锁定为这个比例
sensor.set_auto_whitebal(False, rgb_gain_db=FIXED_RGB_GAIN)

# ==========================================
# 3.锁死硬件增益
# ==========================================
# 增益越低，画面越纯净，赛道反光越弱。
# 比赛场馆灯光亮，建议设为 2.0；如果特别暗，最多加到 5.0。
MY_GAIN = 2.2
sensor.set_auto_gain(False, gain_db=MY_GAIN)

# ==========================================
# 4.设置曝光
# ==========================================

RUN_EXPOSURE = 520
sensor.set_auto_exposure(False, exposure_us=RUN_EXPOSURE)
sensor.set_saturation(2)
sensor.skip_frames(time=500)



CLASSES = ["car", "sandbag", "teddy", "tennis", "cone", "brick"]
CLASS_COLORS = {
    0: (0, 0, 0),
    1: (0, 0, 255),
    2: (80, 220, 120),
    3: (255, 0, 255),
    4: (255, 255, 0),
    5: (255, 0, 0),
}

CFG = {
    "model_path": "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite",
    "default_score_threshold": 0.75,
    "class_score_thresholds": [
        0.99,  # car
        0.75,  # sandbag
        0.70,  # teddy
        0.75,  # tennis
        0.75,  # cone
        0.75,  # brick
    ],
    "center_distance_suppress_pixels": 7,
    "model_input_size": 96
}

PROFILE = {
    "loop_count": 0,
    "last_ms": pyb.millis(),
    "fps": 0.0,
}


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def build_letterbox_input(img, letterbox_fb):
    input_size = CFG["model_input_size"]
    img_w = img.width()
    img_h = img.height()
    scale = min(input_size / float(img_w), input_size / float(img_h))
    scaled_w = int(img_w * scale)
    scaled_h = int(img_h * scale)
    offset_x = (input_size - scaled_w) // 2
    offset_y = (input_size - scaled_h) // 2

    letterbox_fb.draw_rectangle((0, 0, input_size, input_size), color=(0, 0, 0), fill=True)
    letterbox_fb.draw_image(img, offset_x, offset_y, x_scale=scale, y_scale=scale)

    return {
        "scale": scale,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "input_size": input_size,
    }


def det_to_pixels(det, img, letterbox_meta):
    x1, y1, x2, y2, label_id, score = det
    label_id = int(label_id)
    score = float(score)

    if label_id < 0 or label_id >= len(CLASSES):
        return None
    if label_id < len(CFG["class_score_thresholds"]):
        score_threshold = CFG["class_score_thresholds"][label_id]
    else:
        score_threshold = CFG["default_score_threshold"]
    if score < score_threshold:
        return None

    input_size = letterbox_meta["input_size"]
    scale = letterbox_meta["scale"]
    offset_x = letterbox_meta["offset_x"]
    offset_y = letterbox_meta["offset_y"]

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

    return {
        "bbox": (x1, y1, x2, y2),
        "label_id": label_id,
        "label": CLASSES[label_id],
        "score": score,
    }


def collect_detections(net, img, letterbox_fb):
    letterbox_meta = build_letterbox_input(img, letterbox_fb)
    detections = []
    for det in tf.detect(net, letterbox_fb):
        parsed = det_to_pixels(det, img, letterbox_meta)
        if parsed is not None:
            detections.append(parsed)
    return detections





def draw_detections(img, detections):
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w // 2
        cy = y1 + h // 2
        color = CLASS_COLORS.get(det["label_id"], (255, 255, 255))
        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)
        img.draw_circle(cx, cy, 3, color=color, thickness=1)
        img.draw_cross(cx, cy, color=color, size=2, thickness=1)

'''
def update_profile(clock):
    PROFILE["loop_count"] += 1
    now = pyb.millis()
    dt = now - PROFILE["last_ms"]
    if dt >= 1000:
        PROFILE["fps"] = PROFILE["loop_count"] * 1000.0 / dt
        PROFILE["loop_count"] = 0
        PROFILE["last_ms"] = now

def draw_profile(img):
    img.draw_string(2, 2, "fps=%.1f" % PROFILE["fps"], color=(255, 255, 0), scale=1)
'''

def main():
    clock = time.clock()
    net = tf.load(CFG["model_path"], load_to_fb=True)
    letterbox_fb = sensor.alloc_extra_fb(
        CFG["model_input_size"],
        CFG["model_input_size"],
        sensor.RGB565,
    )
    gc.collect()

    while True:
        clock.tick()
        img = sensor.snapshot()
        detections = collect_detections(net, img, letterbox_fb)
        draw_detections(img, detections)
        #update_profile(clock)
        #draw_profile(img)
        print("fps=%.2f det=%d" % (clock.fps(), len(detections)))
        gc.collect()


main()
