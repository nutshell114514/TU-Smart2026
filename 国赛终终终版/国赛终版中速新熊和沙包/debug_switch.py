from micropython import const

# 主车和从车共用的调试开关。
# 1：到 WAIT_READY 后停车，不进入 PUSH_SYNC。
# 0：正常任务流程。
READY_HOLD_ENABLE = const(0)

# 1：初始相机/无线握手完成后，两车跳过启动横移和 SEARCH，直接进入 DONE。
# 0：正常执行启动横移并进入 SEARCH。
BOOT_DIRECT_DONE_ENABLE = const(0)

# 1：强制 OpenART 在所有模式识别球，用于临时测球坐标。
# 0：按任务模式正常开关球识别。
# 开着会让每个视觉帧多跑两遍全图 find_blobs（紫/橙），直接压低 OpenART 帧率，
# 而 Frame2 的有效更新率就等于帧率，所以非标球阶段必须为 0。
# 关闭后从车 APPROACH / WAIT_READY(含 VDOCK) 的球识别仍由 _PUSH_BALL_SYNC_ENABLE
# 单独打开，不受影响；主车侧的球只给从车看，本来就不需要自己识别。
OPENART_BALL_DEBUG_ENABLE = const(1)

# 从车 SEARCH 调参期间冻结主车的最终 RELOCALIZE。
# 1：主车进入 RELOCALIZE 后保持进入时的航向并持续停车，不再等待从车 ready 后起步。
# 0：主车正常执行 RELOCALIZE 的等待、转向、X/Y 修正和 DONE 流程。
SEARCH_TUNE_LEADER_RELOCALIZE_HOLD_ENABLE = const(0)
