#################################################

#白纸贴着推杆放在车前，然后运行程序
#################################################

import sensor, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320x240
sensor.skip_frames(time = 2000)

# 1. 压低曝光，防止“核爆死白”
sensor.set_auto_exposure(False, exposure_us=2500)

# 2. 强行切出中心 60x60 ROI
sensor.set_windowing((60, 60))
sensor.skip_frames(time = 500) # 等待裁剪生效

print("正在计算白平衡，请不要移动白纸...")

# 3. 开启自动白平衡
# ⚠️ 注意：这里的 3000ms (3秒) 绝对不能删！
# 硬件 ISP 算法必须跑够几十帧画面，红蓝增益曲线才能“收敛”到最准确的那个点。
sensor.set_auto_whitebal(True)
time.sleep_ms(3000)

# 4. 提取系数，直接打印！
rgb_gains = sensor.get_rgb_gain_db()
sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gains)

print("\n===========================================")
print("提取成功！请将以下这行代码直接复制到你的主程序中：")
print("FIXED_RGB_GAIN =", rgb_gains)
print("===========================================\n")

# 5. 恢复大视野和正常曝光（方便你直观看一下锁定后的效果）
sensor.set_windowing((320, 240))
# 如果你想看清场地，把这里的曝光调回正常的几千，比如 8000
sensor.set_auto_exposure(False, exposure_us=8000)
sensor.skip_frames(time = 500)

while(True):
    img = sensor.snapshot()
    # IDE 右上角现在显示的，就是完美锁定白平衡后的画面
