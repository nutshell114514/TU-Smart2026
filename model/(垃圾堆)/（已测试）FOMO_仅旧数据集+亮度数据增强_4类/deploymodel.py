import gc
import math
import sensor
import tf
import time

MODEL_PATH = "/sd/trained.tflite"
LABEL_PATH = "/sd/labels.txt"

FRAME_SIZE = sensor.QQVGA
ROI = (32, 16, 96, 96)    #最好别改，因为训练模型就是用的这个roi

PROFILES = {
    "fast": {
        "entry_conf": 0.72,
        "keep_conf": 0.58,
        "min_blob_area": 2,
        "min_blob_pixels": 2,
        "merge_distance": 10,   ########
        "max_track_distance": 14,
        "min_confirm_frames": 2,
        "max_missing_frames": 2,
        "ema_alpha": 0.55,
    },
    "balanced": {
        "entry_conf": 0.78,
        "keep_conf": 0.62,
        "min_blob_area": 3,
        "min_blob_pixels": 3,
        "merge_distance": 9,
        "max_track_distance": 12,
        "min_confirm_frames": 2,
        "max_missing_frames": 3,
        "ema_alpha": 0.45,
    },
    "stable": {
        "entry_conf": 0.84,
        "keep_conf": 0.68,
        "min_blob_area": 4,
        "min_blob_pixels": 4,
        "merge_distance": 8,
        "max_track_distance": 10,
        "min_confirm_frames": 3,
        "max_missing_frames": 4,
        "ema_alpha": 0.35,
    },
}

PROFILE_NAME = "fast"        #在这里改配置
CFG = PROFILES[PROFILE_NAME]

DRAW_RADIUS = 5   #标注时画圈的半径
SHOW_DEBUG = False  #展示调试信息，开了之后会在终端显示

LOCK_CAMERA_SETTINGS = False  #True时，锁定当前曝光和白平衡，即不让自动调整
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(FRAME_SIZE)
sensor.skip_frames(time=2000)
if LOCK_CAMERA_SETTINGS:
    sensor.skip_frames(time=1500)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    try:
        sensor.set_auto_exposure(False)
    except Exception:
        pass

labels = [line.strip() for line in open(LABEL_PATH) if line.strip()]
net = tf.load(MODEL_PATH)

#给每个类准备一种颜色，显示结果时用
colors = [
    (255, 0, 0),
    (0, 255, 0),
    (255, 255, 0),
    (0, 0, 255),
    (255, 0, 255),
    (0, 255, 255),
    (255, 255, 255),
]

#按类跟踪；tracks_by_class是个列表列表，每个子列表里将来会存被跟踪的物体；next_track_id是被跟踪物体的编号，这个编号只会一直往下加，不会回滚复用之前被释放的
tracks_by_class = [[] for _ in range(len(labels))]
next_track_id = 1


def clamp_u8(value):
    if value < 0:
        return 0
    if value > 255:
        return 255
    return int(value)


def sqr_distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def merge_close_points(points, merge_distance):
    if not points:
        return []

    merged = []
    used = [False] * len(points)
    merge_distance_sq = merge_distance * merge_distance

    for i in range(len(points)):
        if used[i]:
            continue

        sum_x = points[i][0]
        sum_y = points[i][1]
        score = points[i][2]
        count = 1
        used[i] = True

        for j in range(i + 1, len(points)):
            if used[j]:
                continue
            if sqr_distance(points[i], points[j]) <= merge_distance_sq:
                sum_x += points[j][0]
                sum_y += points[j][1]
                score = max(score, points[j][2])
                count += 1
                used[j] = True

        merged.append((sum_x / count, sum_y / count, score))

    return merged


def fomo_post_process(seg_list, roi, num_classes, config):
    roi_x, roi_y, roi_w, roi_h = roi
    out_w = seg_list[0].width()
    out_h = seg_list[0].height()

    scale_x = roi_w / out_w
    scale_y = roi_h / out_h

    results = [[] for _ in range(num_classes)]
    entry_threshold = clamp_u8(config["entry_conf"] * 255)

    for cls in range(1, num_classes):  # skip background
        seg_img = seg_list[cls]
        blobs = seg_img.find_blobs(
            [(entry_threshold, 255)],
            x_stride=1,
            y_stride=1,
            area_threshold=config["min_blob_area"],
            pixels_threshold=config["min_blob_pixels"],
            merge=True,
            margin=1,
        )

        points = []
        for blob in blobs:
            x, y, w, h = blob.rect()
            cx = roi_x + (x + w * 0.5) * scale_x
            cy = roi_y + (y + h * 0.5) * scale_y
            confidence = blob.density()
            points.append((cx, cy, confidence))

        results[cls] = merge_close_points(points, config["merge_distance"])

    return results


def update_tracks(track_list, detections, config):
    global next_track_id

    max_distance_sq = config["max_track_distance"] * config["max_track_distance"]
    ema_alpha = config["ema_alpha"]

    for track in track_list:
        track["matched"] = False

    for det in detections:
        best_track = None
        best_distance = None

        for track in track_list:
            distance = sqr_distance(det, (track["x"], track["y"]))
            if distance > max_distance_sq:
                continue
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_track = track

        if best_track is None:
            track_list.append({
                "id": next_track_id,
                "x": det[0],
                "y": det[1],
                "score": det[2],
                "hits": 1,
                "misses": 0,
                "matched": True,
            })
            next_track_id += 1
            continue

        best_track["x"] = best_track["x"] * (1.0 - ema_alpha) + det[0] * ema_alpha
        best_track["y"] = best_track["y"] * (1.0 - ema_alpha) + det[1] * ema_alpha
        best_track["score"] = max(best_track["score"] * 0.8, det[2])
        best_track["hits"] += 1
        best_track["misses"] = 0
        best_track["matched"] = True

    keep_threshold = config["keep_conf"]
    survivors = []
    for track in track_list:
        if not track["matched"]:
            track["misses"] += 1
            track["score"] *= 0.85

        if track["misses"] > config["max_missing_frames"]:
            continue
        if track["score"] < keep_threshold and track["hits"] < config["min_confirm_frames"]:
            continue

        survivors.append(track)

    return survivors


def draw_tracks(img, class_index, track_list, config):
    color = colors[class_index % len(colors)]
    label = labels[class_index]
    min_confirm_frames = config["min_confirm_frames"]

    for track in track_list:
        if track["hits"] < min_confirm_frames:
            continue

        x = int(track["x"])
        y = int(track["y"])
        img.draw_circle(x, y, DRAW_RADIUS, color=color, thickness=2)
        img.draw_cross(x, y, color=color, size=8, thickness=1)
        img.draw_string(
            x + 8,
            y - 8,
            "%s %.2f" % (label, track["score"]),
            color=color,
            mono_space=False,
        )


def draw_roi(img):
    img.draw_rectangle(ROI, color=(80, 80, 80), thickness=1)


clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    seg = tf.segment(
        net,
        img,
        roi=ROI,
        scale=1.0,
        offset=0.0,
    )

    raw_detections = fomo_post_process(seg, ROI, len(labels), CFG)

    for class_index in range(1, len(labels)):
        tracks_by_class[class_index] = update_tracks(
            tracks_by_class[class_index],
            raw_detections[class_index],
            CFG,
        )

    draw_roi(img)
    for class_index in range(1, len(labels)):
        draw_tracks(img, class_index, tracks_by_class[class_index], CFG)

        if SHOW_DEBUG and raw_detections[class_index]:
            print("class=%s raw=%s" % (labels[class_index], raw_detections[class_index]))

    print("profile=%s fps=%.2f" % (PROFILE_NAME, clock.fps()))

    del seg
    gc.collect()
