import gc
import time

import pyb
import sensor
import tf


CLASSES = ["car", "sandbag", "teddy", "tennis"]
CLASS_COLORS = {
    0: (255, 80, 80),
    1: (255, 200, 0),
    2: (80, 220, 120),
    3: (80, 160, 255),
}

ROI = (40, 0, 240, 240)

CFG = {
    "model_path": "/sd/yolo3_siou_smartcar_final_with_post_processing.tflite",
    "score_threshold": 0.75,
    "draw_roi": True,
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


def det_to_pixels(det, roi, img):
    rx, ry, rw, rh = roi
    x1, y1, x2, y2, label_id, score = det
    label_id = int(label_id)
    score = float(score)

    if score < CFG["score_threshold"]:
        return None
    if label_id < 0 or label_id >= len(CLASSES):
        return None

    x1 = rx + int(x1 * rw)
    y1 = ry + int(y1 * rh)
    x2 = rx + int(x2 * rw)
    y2 = ry + int(y2 * rh)

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


def collect_detections(net, img, roi):
    roi_img = img.copy(roi=roi)
    detections = []
    for det in tf.detect(net, roi_img):
        parsed = det_to_pixels(det, roi, img)
        if parsed is not None:
            detections.append(parsed)
    return detections


def draw_detections(img, detections):
    if CFG["draw_roi"]:
        img.draw_rectangle(ROI, color=(60, 60, 60), thickness=1)

    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w // 2
        cy = y1 + h // 2
        color = CLASS_COLORS.get(det["label_id"], (255, 255, 255))
        text = "%s %.2f" % (det["label"], det["score"])

        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)
        img.draw_cross(cx, cy, color=color, size=6, thickness=1)
        img.draw_string(x1, max(0, y1 - 12), text, color=color, scale=1)


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


def main():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)

    clock = time.clock()
    net = tf.load(CFG["model_path"])
    gc.collect()

    while True:
        clock.tick()
        img = sensor.snapshot()
        detections = collect_detections(net, img, ROI)
        draw_detections(img, detections)
        update_profile(clock)
        draw_profile(img)
        print("fps=%.2f det=%d" % (clock.fps(), len(detections)))
        gc.collect()


main()
