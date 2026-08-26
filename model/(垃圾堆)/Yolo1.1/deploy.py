import gc
import time

import pyb
import sensor
import tf

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.skip_frames(time = 2000)   # 等待感光元件上电稳定

# ==========================================
# 2.写死静态白平衡
# ==========================================

FIXED_RGB_GAIN = (107.0, 64.0, 91.0)
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
CLASSES = ["car", "sandbag", "teddy", "tennis"]
CLASS_COLORS = {
    0: (255, 80, 80),
    1: (80, 180, 255),
    2: (80, 220, 120),
    3: (255, 190, 80),
}

ROI = (64, 10, 192, 192)

CFG = {
    "model_path": "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite",
    "score_threshold": 0.75,
    "nms_iou_threshold": 0.15,
    "containment_ratio": 0.85,
    "min_box_size": 12,
    "draw_roi": True,
    "draw_boxes": True,
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


def box_area(box):
    return max(0, box[2] - box[0]) * max(0, box[3] - box[1])


def compute_iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    if inter_area <= 0:
        return 0.0
    union = box_area(box_a) + box_area(box_b) - inter_area
    if union <= 0:
        return 0.0
    return inter_area / float(union)


def contains_most_of(inner_box, outer_box, ratio):
    inter_x1 = max(inner_box[0], outer_box[0])
    inter_y1 = max(inner_box[1], outer_box[1])
    inter_x2 = min(inner_box[2], outer_box[2])
    inter_y2 = min(inner_box[3], outer_box[3])
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    inner_area = box_area(inner_box)
    if inner_area <= 0:
        return False
    return (inter_area / float(inner_area)) >= ratio


def normalize_label_id(label_id):
    label_id = int(label_id)
    if 0 <= label_id < len(CLASSES):
        return label_id
    if 1 <= label_id <= len(CLASSES):
        return label_id - 1
    return -1


def det_to_prediction(det):
    if len(det) < 6:
        return None

    x1, y1, x2, y2, label_id, score = det
    label_id = normalize_label_id(label_id)
    if label_id < 0:
        return None

    rx, ry, rw, rh = ROI
    roi_x_min = rx
    roi_y_min = ry
    roi_x_max = rx + rw - 1
    roi_y_max = ry + rh - 1
    px1 = clamp(int(rx + x1 * rw), roi_x_min, roi_x_max)
    py1 = clamp(int(ry + y1 * rh), roi_y_min, roi_y_max)
    px2 = clamp(int(rx + x2 * rw), roi_x_min, roi_x_max)
    py2 = clamp(int(ry + y2 * rh), roi_y_min, roi_y_max)
    if px2 <= px1:
        px2 = min(sensor.width() - 1, px1 + 1)
    if py2 <= py1:
        py2 = min(sensor.height() - 1, py1 + 1)

    return {
        "bbox": (px1, py1, px2, py2),
        "score": float(score),
        "raw_label_id": label_id,
    }


def collect_raw_predictions(net, img):
    roi_img = img.copy(roi=ROI)
    raw_predictions = []
    for det in tf.detect(net, roi_img):
        pred = det_to_prediction(det)
        if pred is None:
            continue
        if pred["score"] < CFG["score_threshold"]:
            continue
        raw_predictions.append(pred)
    return raw_predictions


def class_agnostic_nms(predictions, iou_threshold):
    pending = sorted(predictions, key=lambda item: item["score"], reverse=True)
    kept = []
    while pending:
        best = pending.pop(0)
        kept.append(best)
        survivors = []
        for pred in pending:
            if compute_iou(best["bbox"], pred["bbox"]) < iou_threshold:
                survivors.append(pred)
        pending = survivors
    return kept


def suppress_contained_boxes(predictions, ratio):
    kept = []
    for pred in sorted(predictions, key=lambda item: item["score"], reverse=True):
        should_drop = False
        for kept_pred in kept:
            if contains_most_of(pred["bbox"], kept_pred["bbox"], ratio):
                should_drop = True
                break
        if not should_drop:
            kept.append(pred)
    return kept


def filter_boxes(predictions, min_box_size):
    filtered = []
    for pred in predictions:
        x1, y1, x2, y2 = pred["bbox"]
        if (x2 - x1) < min_box_size or (y2 - y1) < min_box_size:
            continue
        filtered.append(pred)
    return filtered


def dedup_predictions(raw_predictions):
    predictions = class_agnostic_nms(raw_predictions, CFG["nms_iou_threshold"])
    predictions = suppress_contained_boxes(predictions, CFG["containment_ratio"])
    predictions = filter_boxes(predictions, CFG["min_box_size"])
    return predictions


def draw_predictions(img, predictions):
    if CFG["draw_roi"]:
        img.draw_rectangle(ROI, color=(60, 60, 60), thickness=1)

    for pred in predictions:
        x1, y1, x2, y2 = pred["bbox"]
        label_id = pred["raw_label_id"]
        label_name = CLASSES[label_id]
        color = CLASS_COLORS.get(label_id, (255, 255, 255))
        text = "%s %.2f" % (label_name, pred["score"])

        if CFG["draw_boxes"]:
            img.draw_rectangle((x1, y1, x2 - x1, y2 - y1), color=color, thickness=3)
        img.draw_string(x1 + 1, y1 + 1, text, color=color, scale=1)


def update_profile():
    PROFILE["loop_count"] += 1
    now = pyb.millis()
    dt = now - PROFILE["last_ms"]
    if dt >= 1000:
        PROFILE["fps"] = PROFILE["loop_count"] * 1000.0 / dt
        PROFILE["loop_count"] = 0
        PROFILE["last_ms"] = now


def draw_profile(img, raw_count, dedup_count):
    img.draw_string(2, 2, "fps=%.1f" % PROFILE["fps"], color=(255, 255, 0), scale=1)
    img.draw_string(2, 16, "raw=%d dedup=%d" % (raw_count, dedup_count), color=(255, 255, 0), scale=1)


def main():

    clock = time.clock()
    net = tf.load(CFG["model_path"])
    gc.collect()

    while True:
        clock.tick()
        img = sensor.snapshot()
        raw_predictions = collect_raw_predictions(net, img)
        dedup_predictions_list = dedup_predictions(raw_predictions)
        draw_predictions(img, dedup_predictions_list)
        update_profile()
        draw_profile(img, len(raw_predictions), len(dedup_predictions_list))
        print("fps=%.2f raw=%d dedup=%d" % (clock.fps(), len(raw_predictions), len(dedup_predictions_list)))
        gc.collect()


main()
