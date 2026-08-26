import gc
import os
import sensor
import tf
import time


MODEL_PATH = "/sd/trained.tflite"
LABEL_PATH = "/sd/labels.txt"
FRAME_SIZE = sensor.QQVGA
ROI = (32, 16, 96, 96)
OUTPUT_DIR = "/sd/segment_debug"


def ensure_dir(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


def load_labels():
    try:
        return [line.strip() for line in open(LABEL_PATH) if line.strip()]
    except Exception:
        return []


def save_segment_outputs(seg_list, labels, prefix):
    for index in range(len(seg_list)):
        seg_img = seg_list[index]
        if index < len(labels):
            name = labels[index]
        else:
            name = "cls_%d" % index
        # Keep the raw segment output as-is so we can inspect the original response map.
        seg_path = "%s/%s_seg_%02d_%s.bmp" % (OUTPUT_DIR, prefix, index, name)
        seg_img.save(seg_path)
        print("saved:", seg_path, "size=", seg_img.width(), "x", seg_img.height())


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(FRAME_SIZE)
sensor.skip_frames(time=2000)

ensure_dir(OUTPUT_DIR)

labels = load_labels()
net = tf.load(MODEL_PATH)
clock = time.clock()

clock.tick()
img = sensor.snapshot()
roi_img = img.copy(roi=ROI)

timestamp = time.ticks_ms()
prefix = "capture_%d" % timestamp

orig_path = "%s/%s_full.bmp" % (OUTPUT_DIR, prefix)
roi_path = "%s/%s_roi.bmp" % (OUTPUT_DIR, prefix)
img.save(orig_path)
roi_img.save(roi_path)

print("saved:", orig_path, "size=", img.width(), "x", img.height())
print("saved:", roi_path, "size=", roi_img.width(), "x", roi_img.height())

seg = tf.segment(
    net,
    img,
    roi=ROI,
    scale=1.0,
    offset=0.0,
)

print("segment channels:", len(seg))
save_segment_outputs(seg, labels, prefix)

del seg
del roi_img
gc.collect()

print("done")
