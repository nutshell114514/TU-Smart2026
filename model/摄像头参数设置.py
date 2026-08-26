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
while(True):
    img = sensor.snapshot()

