import gc
import time

import pyb
import seekfree
import sensor
import tf


CLASSES = ["car", "sandbag", "teddy", "tennis"]
CLASS_COLORS = {
    "car": (255, 80, 80),
    "sandbag": (80, 180, 255),
    "teddy": (80, 220, 120),
    "tennis": (255, 190, 80),
}

ROI = (64, 2, 192, 192)

CFG = {
    "model_path": "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite",
    "score_threshold": 0.75,
    "nms_iou_threshold": 0.20,
    "center_distance_ratio": 0.35,
    "track_match_iou": 0.20,
    "track_keep_frames": 1,
    "track_min_hits": 1,
    "smooth_alpha": 0.85,
    "max_tracks": 12,
    "draw_roi": True,
}

PROFILE = {
    "loop_count": 0,
    "last_ms": pyb.millis(),
    "fps": 0.0,
}

tracks = []
next_track_id = 1


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def bbox_iou(box_a, box_b):
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
    area_a = max(1, ax2 - ax1) * max(1, ay2 - ay1)
    area_b = max(1, bx2 - bx1) * max(1, by2 - by1)
    return inter_area / float(area_a + area_b - inter_area)


def smooth_bbox(old_box, new_box, alpha):
    if old_box is None:
        return new_box
    beta = 1.0 - alpha
    return (
        int(old_box[0] * beta + new_box[0] * alpha),
        int(old_box[1] * beta + new_box[1] * alpha),
        int(old_box[2] * beta + new_box[2] * alpha),
        int(old_box[3] * beta + new_box[3] * alpha),
    )


def center_distance_ratio(box_a, box_b):
    acx = (box_a[0] + box_a[2]) / 2.0
    acy = (box_a[1] + box_a[3]) / 2.0
    bcx = (box_b[0] + box_b[2]) / 2.0
    bcy = (box_b[1] + box_b[3]) / 2.0
    dx = acx - bcx
    dy = acy - bcy
    dist = (dx * dx + dy * dy) ** 0.5
    ref = min(max(box_a[2] - box_a[0], box_a[3] - box_a[1]), max(box_b[2] - box_b[0], box_b[3] - box_b[1]))
    if ref <= 0:
        return 999.0
    return dist / float(ref)


def det_to_pixels(det):
    x1, y1, x2, y2, label_id, score = det
    rx, ry, rw, rh = ROI
    px1 = clamp(int(rx + x1 * rw), 0, sensor.width() - 1)
    py1 = clamp(int(ry + y1 * rh), 0, sensor.height() - 1)
    px2 = clamp(int(rx + x2 * rw), 0, sensor.width() - 1)
    py2 = clamp(int(ry + y2 * rh), 0, sensor.height() - 1)
    if px2 <= px1:
        px2 = min(sensor.width() - 1, px1 + 1)
    if py2 <= py1:
        py2 = min(sensor.height() - 1, py1 + 1)
    return {
        "bbox": (px1, py1, px2, py2),
        "label_id": int(label_id),
        "label": CLASSES[int(label_id)],
        "score": float(score),
    }


def nms_per_class(detections, iou_threshold):
    kept = []
    for class_name in CLASSES:
        class_dets = []
        for det in detections:
            if det["label"] == class_name:
                class_dets.append(det)
        class_dets.sort(key=lambda item: item["score"], reverse=True)
        while class_dets:
            best = class_dets.pop(0)
            kept.append(best)
            survivors = []
            for det in class_dets:
                if bbox_iou(best["bbox"], det["bbox"]) < iou_threshold:
                    survivors.append(det)
            class_dets = survivors
    return kept


def suppress_close_center_boxes(detections, ratio_threshold):
    detections = sorted(detections, key=lambda item: item["score"], reverse=True)
    kept = []
    for det in detections:
        suppressed = False
        for best in kept:
            if center_distance_ratio(det["bbox"], best["bbox"]) <= ratio_threshold:
                suppressed = True
                break
        if not suppressed:
            kept.append(det)
    return kept


def collect_detections(net, img):
    roi_img = img.copy(roi=ROI)
    detections = []
    for det in tf.detect(net, roi_img):
        parsed = det_to_pixels(det)
        if parsed["score"] < CFG["score_threshold"]:
            continue
        detections.append(parsed)
    detections = nms_per_class(detections, CFG["nms_iou_threshold"])
    detections = suppress_close_center_boxes(detections, CFG["center_distance_ratio"])
    return detections


def make_track(track_id, det):
    return {
        "id": track_id,
        "label": det["label"],
        "bbox": det["bbox"],
        "score": det["score"],
        "hits": 1,
        "miss": 0,
        "active": True,
    }


def update_tracks(detections):
    global tracks, next_track_id

    detections = sorted(detections, key=lambda item: item["score"], reverse=True)
    unmatched_det_ids = list(range(len(detections)))

    for track in tracks:
        if not track["active"]:
            continue
        best_index = -1
        best_iou = 0.0
        for det_index in unmatched_det_ids:
            det = detections[det_index]
            if det["label"] != track["label"]:
                continue
            iou = bbox_iou(track["bbox"], det["bbox"])
            if iou > best_iou:
                best_iou = iou
                best_index = det_index

        if best_index >= 0 and best_iou >= CFG["track_match_iou"]:
            det = detections[best_index]
            track["bbox"] = smooth_bbox(track["bbox"], det["bbox"], CFG["smooth_alpha"])
            track["score"] = det["score"]
            track["hits"] += 1
            track["miss"] = 0
            unmatched_det_ids.remove(best_index)
        else:
            track["miss"] += 1
            if track["miss"] > CFG["track_keep_frames"]:
                track["active"] = False

    for det_index in unmatched_det_ids:
        tracks.append(make_track(next_track_id, detections[det_index]))
        next_track_id += 1

    active_tracks = []
    for track in tracks:
        if track["active"]:
            active_tracks.append(track)
    active_tracks.sort(key=lambda item: (item["hits"], item["score"]), reverse=True)
    tracks = active_tracks[:CFG["max_tracks"]]


def draw_tracks(img):
    if CFG["draw_roi"]:
        img.draw_rectangle(ROI, color=(60, 60, 60), thickness=1)

    for track in tracks:
        if track["hits"] < CFG["track_min_hits"]:
            continue
        x1, y1, x2, y2 = track["bbox"]
        cx = x1 + (x2 - x1) // 2
        cy = y1 + (y2 - y1) // 2
        color = CLASS_COLORS.get(track["label"], (255, 255, 255))
        text = "%s %.2f" % (track["label"], track["score"])
        img.draw_circle(cx, cy, 8, color=color, thickness=2)
        img.draw_cross(cx, cy, color=color, size=6, thickness=2)
        img.draw_string(cx + 6, max(0, cy - 8), text, color=color, scale=1)


def update_profile():
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
        detections = collect_detections(net, img)
        update_tracks(detections)
        draw_tracks(img)
        update_profile()
        draw_profile(img)
        print("fps=%.2f det=%d tracks=%d" % (clock.fps(), len(detections), len(tracks)))
        gc.collect()


main()
