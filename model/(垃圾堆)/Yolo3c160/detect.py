import gc
import time

import pyb
import sensor
import tf


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)

CLASSES = ["car", "sandbag", "teddy", "tennis"]
CLASS_COLORS = {
    0: (255, 80, 80),
    1: (255, 200, 0),
    2: (80, 220, 120),
    3: (255, 0, 255),
}

CFG = {
    "model_path": "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite",
    "score_threshold": 0.75,
    "center_distance_suppress_pixels": 50,
    "model_input_size": 160
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

    if score < CFG["score_threshold"]:
        return None
    if label_id < 0 or label_id >= len(CLASSES):
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


def apply_center_distance_suppression(detections):
    if len(detections) < 2:
        return detections

    sorted_detections = sorted(detections, key=lambda det: det["score"], reverse=True)
    kept = []
    thresh = CFG["center_distance_suppress_pixels"]
    thresh_sq = thresh * thresh

    for det in sorted_detections:
        x1, y1, x2, y2 = det["bbox"]
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w / 2
        cy = y1 + h / 2
        suppressed = False

        for kept_det in kept:
            if det["label_id"] != kept_det["label_id"]:
                continue

            kx1, ky1, kx2, ky2 = kept_det["bbox"]
            kw = kx2 - kx1
            kh = ky2 - ky1
            kcx = kx1 + kw / 2
            kcy = ky1 + kh / 2

            dx = cx - kcx
            dy = cy - kcy
            dist_sq = dx * dx + dy * dy

            if dist_sq < thresh_sq:
                suppressed = True
                break

        if not suppressed:
            kept.append(det)

    return kept


def draw_detections(img, detections):
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w // 2
        cy = y1 + h // 2
        color = CLASS_COLORS.get(det["label_id"], (255, 255, 255))
        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)
        img.draw_circle(cx, cy, 6, color=color, thickness=2)
        img.draw_cross(cx, cy, color=color, size=5, thickness=2)


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
    clock = time.clock()
    net = tf.load(CFG["model_path"])
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
        detections = apply_center_distance_suppression(detections)
        draw_detections(img, detections)
        update_profile(clock)
        draw_profile(img)
        print("fps=%.2f det=%d" % (clock.fps(), len(detections)))
        gc.collect()


main()
