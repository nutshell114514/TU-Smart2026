# Split task layer generated from app.py.
import config
import gc
import base
import debug_switch
from math import atan2, cos, degrees, radians, sin, sqrt
from micropython import const
from time import ticks_add, ticks_diff, ticks_ms

# ── 现场常调参数 ──────────────────────────────────────────────────────────
# 以下参数从原定义处（SEARCH 参数区）上移到顶端，方便现场调参。相关的其他
# 跟随参数仍在原处，改动前先看原处的成组注释。

# SEARCH 阶段摄像头到主车识别中心的目标距离，单位为 cm。
# 从车的径向闭环以该值为中心调节，既用于静态对位，也用于动态编队跟随。
# 必须定义在 _search_distance_adjust 之前：它被用作该函数的默认参数值，
# 默认值在 def 执行时求值，定义在后会变成模块导入即 NameError。
# RELOCALIZE 阶段用的是另一个 _FOLLOWER_RELOCALIZE_FOLLOW_DIST，不受本值影响。
_FOLLOWER_FOLLOW_DIST = 26.0

# SEARCH/RECOVER 共用的径向距离死区。RECOVER 的绕行函数位于 SEARCH 参数区之前，
# 因此必须在首次使用前定义，避免 MicroPython 运行时出现 NameError。
_FOLLOWER_SEARCH_DIST_ERR = const(5)

# SEARCH/RECOVER 绕行共用的主车速度前馈增益，同样提前到 RECOVER 之前定义。
_FOLLOWER_SEARCH_ORBIT_FF_LAT_GAIN = 0.5
_FOLLOWER_SEARCH_ORBIT_FF_FWD_GAIN = 1.0

# 主车仍在 SEARCH/S4 横扫时，从车在主车搜索边界基础上提前该距离停止
# 当前横扫轴，等待主车切到换行前进或等待编队后再解除。单位为 cm。
_FOLLOWER_SEARCH_SWEEP_EARLY_STOP_INSET_CM = const(15)

# 从车 RELOCALIZE 的 cone/brick 固定 -Y 斥力参数，与主车完全相同。
# 只作用于跟随主车、前往预设点和自身 X/Y 修正的平移命令；
# 停车、原地找车和转向不叠加斥力。
_RELOCALIZE_AVOID_ENABLE = 1
_RELOCALIZE_REPULSE_TRIGGER_CM = 50.0
_RELOCALIZE_REPULSE_SPEED = 140.0

# 从车收到主车 DONE 后，先锁 270° 前往自己的重定位预设点，再修 X/Y。
# 速度和主车 PREMOVE 完全相同；全程恒速，不做位置环末端减速。
_RELOC_PREMOVE_SPEED = const(190)
# ──────────────────────────────────────────────────────────────────────────

# 不同目标物体在接近和 PUSH 阶段使用的单侧夹角；当前两车相对推送轴均为30°，
# 按现有几何定义对应两根推杆夹角120°。按 obj_id 索引（0不用，1网球，2蓝沙包，3红沙包，4白熊，5棕熊）。
_APPROACH_LOCK_DEG = (0.0, 30.0, 30.0, 30.0, 30.0, 30.0)

# 接近目标时允许的横向参考偏移
_CLOSE_LAT_OFFSET_BY_OBJ = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# 就位阶段从车物体参考：网球13cm，蓝/红沙包9cm，白/棕熊9cm。
_CLOSE_DIST_FOLLOWER_BY_OBJ = (0.0, 13.0, 9.0, 9.0, 9.0, 9.0)

# 推送目标时向场内偏转的角度
_PUSH_INWARD_BIAS_DEG_BY_OBJ = (0.0, 2.0, 3.0, 3.0, 3.0, 3.0)

# 从车按物体独立配置 PUSH PID，按 obj_id 索引：
# 0 不用，1 网球，2 蓝沙包，3 红沙包，4 白熊，5 棕熊。
# 每项依次为 (side_kp, side_kd, fwd_kp, fwd_kd)。
_PUSH_PID_FOLLOWER_BY_OBJ = (
    None,
    (5.0, 10.0, 3.0, 6.0),  # 网球
    (9.0, 18.0, 4.0, 10.0),  # 蓝沙包
    (9.0, 18.0, 4.0, 10.0),  # 红沙包
    (5.0, 10.0, 3.0, 6.0),  # 白熊
    (5.0, 10.0, 3.0, 6.0),  # 棕熊
)

# 从车横向修正限幅，按 obj_id 索引；两个沙包单独提高到 80。
_PUSH_SIDE_MAX_FOLLOWER_BY_OBJ = (0.0, 50.0, 80.0, 80.0, 50.0, 50.0)

_READY_ROUTE_SAFE_HALF = const(18)

_READY_ROUTE_FWD_MIN_DIST = const(6)

_READY_ROUTE_FWD_MAX_DIST = const(30)

_READY_ROUTE_AVOID_ENABLE = bool(getattr(config, "_READY_ROUTE_AVOID_ENABLE", 0))

# OpenART 识别开关
_OPENART_LINE_ENABLE = False

_OPENART_MODEL_ENABLE = True

# 场地尺寸和中心点坐标
_FIELD_W = const(310)

_FIELD_H = const(230)

_OBJ_ID_TENNIS = const(1)

# 与主车 _SLOT_FIXED_TENNIS 保持一致：搬网球时主从车槽位互换，
# 从车去 push+deg 一侧（槽位1）看紫球，避开网球对橙球的色块干扰。
_SLOT_FIXED_TENNIS = const(0)

_FOLLOWER_CAR_RELOCALIZE_ENABLE = const(1)

_FOLLOWER_CAR_RELOCALIZE_INTERVAL_MS = const(100)

_FOLLOWER_CAR_RELOCALIZE_BLEND = const(1)

# ── 从车 PUSH 视觉修正参数总览 ──────────────────────────────
# 从车和主车一样，通过 _request_push_control 把物体视觉送入 base._push_step
# 的物体层；_PUSH_*_FOLLOWER 与 _PUSH_PID_FOLLOWER_BY_OBJ 是 PUSH 唯一的修正源。
# PUSH 阶段的球体修正整层已移除（见 已移除代码备份.md T7）；标记球识别与定位
# 仅保留给 APPROACH / WAIT_READY（含 VDOCK）就位使用。
_PUSH_BALL_SYNC_ENABLE = const(1)

# 上电初值。BOOT_SYNC 是 state_init 直接赋的模式，不走 enter_mode，
# 所以 _set_openart_switches_for_mode 在 BOOT_SYNC 期间不会执行，
# 这里必须自己认一次调试开关，否则"强制所有模式识球"在开机等待期是失效的。
_OPENART_BALL_ENABLE = bool(debug_switch.OPENART_BALL_DEBUG_ENABLE)

_PUSH_BALL_SLOT_SIDE_ENABLE = const(1)

_BALL_L_CLASS_ID = const(6)

_BALL_R_CLASS_ID = const(7)

# ── 现场诊断：单侧标记球屏蔽 ───────────────────────────────────────────────
# 用于判定"某一侧球本身是不是故障源"。被屏蔽的一侧等价于该侧球永远看不见：
#   · _slot_ball_ref 直接返回不可用，误检没有任何机会进入 VDOCK 闭环和就位判据
#   · 该侧同时按 _dock_obj_only 处理，VDOCK 立刻把垂直轴交还物体环，
#     不会每次就位白等 _VDOCK_BALL_WAIT_MS(1000ms)
# 只屏蔽任务层的使用，不改相机识别本身；另一侧行为完全不变，便于 A/B 对比。
# 0 = 都不屏蔽（正常运行）
# 1 = 屏蔽左槽位紫球
# 2 = 屏蔽右槽位橙球
# 3 = 两侧都屏蔽（等价于全局关闭球识别的任务层效果）
_BALL_SIDE_DISABLE = const(0)
# ──────────────────────────────────────────────────────────────────────────

# base 解析层用的球体粗筛距离上限（base._cam_parse 通过 task._BALL_REL_MAX_DIST 读取）。
_BALL_REL_MAX_DIST = 120.0

# _slot_ball_ref 专用的几何合法窗口，比上面那个宽松值紧得多。
# 上面的 120cm 是给 base 解析层用的粗筛，对"槽位球应该出现在哪"来说过于宽松：
# 期望球位约 (±10, 13)、距离仅约 16cm，而这么大的窗口能让画面里任何一块同色
# 误检（橙球与红沙包、网球色相接近）都冒充成"球可见"。误检一旦冒充成功，就会
# 带着错误坐标进入 VDOCK 闭环和就位判据；判成"看不见球"反而是安全的——
# 两边都有物体兜底。
# 取值覆盖 VDOCK 的实际工作范围：环绕半径约 20cm 切入、观察位再后退 7cm，
# 球距最远约 35cm，横向最远约 25cm，因此留到 30 / 45 仍有余量。
_SLOT_BALL_MAX_LAT = const(30)

_SLOT_BALL_MAX_DIST = const(45)

# VDOCK 垂直（V 张开）轴上"过紧方向"的增益/限幅缩放。0.5 = 分离方向权限为
# 过紧方向的 2 倍。两车宁可靠得过紧（物体漏不出多少），也绝不能张开。
# PUSH 球层移除后，本项只被 VDOCK 使用。
_PUSH_BALL_OPEN_RELAX = 0.5

# 1：物体视觉是 PUSH 的【主】修正源，球退为物体不可用时的备选。
# 两者可视条件差一个量级：球参考在 (−8,15)，离画面下边界只有 5cm(车1)，前冲一点就掉出去，
# 且球一丢从车就失去唯一直接反馈；物体虽更近(rel_y 10~16)，但尺寸大一个量级、且是正在推的
# 东西，永远在视野里（主车的物体环一直稳定工作即为证据）。
# 几何上两者等价：任何固定点在从车车体系下的位移都等于 −(从车位移的车体表示)，与该点是球
# 还是物体无关，所以几何投影、不对称收紧和积分项全部原样复用。
# 物体兜底通路的误差放大倍数，含义同 _PUSH_WL_GAIN。
# 物体是绝对基准、不受里程计漂移影响，可以比无线更信任一些。
# 物体实测值与锁存参考的最大允许偏差；超过判为误识别或队形已经崩了，弃用物体改走无线。
# 紧 V 阵列的唯一真值定义：主车球在从车车体系下的期望位置，按目标分 5 组，
# 每组 (左槽位refx, 左槽位refy, 右槽位refx, 右槽位refy)。
# 左槽位看紫球 _cam_ball_rel[0]，右槽位看橙球 _cam_ball_rel[1]。
# 手动把两车推杆摆成期望 V 形后，直接读从车 LCD 上的 BL/BR 实测值填这里。
# 这组球参考原按 V 字角 90° 实测；当前改为120°后暂沿用，需结合实车画面复核。
# 五类统一为左槽位 (9,11)、右槽位 (-11,12)。
# 改前是按目标分两组（网球/白熊/棕熊左槽位 (8,19)，蓝/红沙包左槽位 (9,20)；
# 右槽位均为 (-10,16)），本次重标后不再区分目标类别。
_PUSH_BALL_FIXED_REF_BY_OBJ = (
    # 目标1：网球
    9, 11, -11, 12,
    # 目标2：蓝沙包
    9, 11, -11, 12,
    # 目标3：红沙包
    9, 11, -11, 12,
    # 目标4：白熊
    9, 11, -11, 12,
    # 目标5：棕熊
    9, 11, -11, 12,
)

# V 字贴靠（VDOCK）用：主车推杆上左/右球相对主车车中心（里程计原点）的安装位置，
# 车体系 (横向, 前向) cm。看不到球时用它+主车无线位姿解析出从车该到的车中心目标。
# 无线兜底的目标：紧 V 位时【主车中心】在从车车体系下的位置。
# 原来是存主车球安装位 bL，再由球参考经两次旋转换算过来；现在直接存这个量。
# 去掉 bL 的理由：它是手量的中间量、不可信，而且它把"球参考"和"无线目标"绑死了——
# 球参考一偏，无线目标跟着偏。改后两者解耦，且标定量从三个减到两个（球、物体），
# 这两个在同一姿势下读出来天生自洽。
# 种子值由对称几何直接给出：两车车头锁在 push±45、物体在两车车体系都是 (0,d)，
# 则主车中心 = (±d, d)，间距 d·√2，与球装在哪完全无关（已数值验证）。
# 运行时每次就位成功用 _wl_rel() 直接自学习，比学 bL 少一层换算，误差传播更少。
# 办法A 运行时学习值 [左lat, 左fwd, 右lat, 右fwd]。无线兜底公式直接用它。
# 运行时学习值 [槽1x, 槽1y, 槽2x, 槽2y]；_dock_wl_valid 对应两个槽位。
_dock_wl_ref = [0.0, 0.0, 0.0, 0.0]

_dock_wl_valid = [False, False]
# VDOCK 控制参数。容差/帧数/保持沿用普通接近手感（2cm / 2帧 / 250ms）。
_VDOCK_KP = 4.0
_VDOCK_SPEED = const(55)
_VDOCK_WL_SPEED = const(50)
_VDOCK_POS_EPS = const(3)
_VDOCK_OK_REQUIRE = const(2)
_VDOCK_HOLD_MS = const(100)

# 就位阶段球的垂直修正（只作用于推送方向的垂直分量，即 V 张开轴）。
_VDOCK_OPEN_KP = 4.0

_VDOCK_OPEN_MAX = const(40)

_VDOCK_OPEN_EPS = 1.5

# 物体已就位但看不到球时，最多等这么久就不再要求球，直接收工。
_VDOCK_BALL_WAIT_MS = const(1000)
_VDOCK_TIMEOUT_MS = const(3000)
# VDOCK 找球（无球时的无线趋近扫描）。球可见是就位的硬门槛，无球分支只负责把球捞进视野。
# 观察位会比紧 V 位纵向拉远，使球落到 FOV 中部。
# 进度 u：0=观察位，1=紧 V 位。无球时 u 缓慢爬升 → 从车"横向缓缓贴向主车"，
# 到 _VDOCK_CREEP_U_MAX 后回落到 0 往复扫描，永不在无球状态下判定就位。
_VDOCK_OBS_FWD_CM = 7.0
_VDOCK_CREEP_SPEED = const(20)
_VDOCK_CREEP_RATE = 0.3
_VDOCK_CREEP_U_MAX = 1.1
_VDOCK_CREEP_TRACK_EPS = 3.0

# VTOUCH（方案C 物体贴合微调）参数。
# 只沿车头方向补间隙：从车车头在 push_yaw−45，往前走等于同时朝物体和朝主车靠，
# V 只会更紧不会张开，符合"宁可过紧"的原则。横向一律不动，避免破坏 V 几何。
# 单次微调的最大位移，防止物体视觉出错时把从车顶进主车。
# 单次微调的最大位移。置 0 = VTOUCH 只锁存参考、不做任何位移（moved 恒 >= 0，
# 第一帧就走"走够了"分支）。当前置 0 的原因：VTOUCH 的目标距离取自
# _CLOSE_DIST_FOLLOWER_BY_OBJ，那是已删除的 _AP_CLOSE 留下的表（那时从车正面朝物体），
# 而 V 位是斜 30°，物体实际距离不同，用它会把从车前推 5~6cm 直接把球顶出画面，
# 再被 WAIT_READY 的 target_still_ok 打回 VDOCK，形成"后退-前进-丢球"死循环。
# 等 LCD 的 O 行标定出真实的物体参考、双轴分工上线后，这个位移动作就不需要了。
# 只允许往前贴，不允许后退（贴过头就认了）。

# 与主车基座速度 _PUSH_SPEED_LEADER 对齐（120）。原先高 10 是补从车驱动链稳态亏空，
# 现按需求改为两车同速；若实车出现从车持续落后主车，再单独往上抬这个值。
# 前提是电机没饱和——先看 LCD "MOT d.. e.." 行确认，饱和了提这个数没有任何作用。
_PUSH_SPEED_FOLLOWER = const(140)

# PUSH 黄线绝对坐标只有距任一对应场地边界小于该值时才允许修正里程计。
_PUSH_POS_FIX_EDGE_GATE_CM = const(50)

_CAM_LOST_TOL_MS = const(300)

_READY_OTHER_FRESH_MS = const(250)

_PUSH_RUN = const(0)

_ORBIT_SLOW_ZONE_DEG = const(55)

_MODE_BOOT_SYNC = const(1)

_MODE_SEARCH = const(12)

_MODE_APPROACH = const(13)

_MODE_WAIT_READY = const(14)

_MODE_PUSH_SYNC = const(15)

_MODE_RECOVER = const(16)

_MODE_DONE = const(17)

_MODE_RELOCALIZE = const(19)

_CMD_SUB_NONE = const(0)

_CMD_SUB_ROUTE_RECLOSE_1 = const(1)

_CMD_SUB_ROUTE_RESTORE = const(3)

_CMD_SUB_DIAG_PUSH = const(4)

_CMD_SUB_BOOT_START = const(5)

_CMD_SUB_PUSH_DONE = const(6)

# First-boot SEARCH entry command.
_CMD_SUB_BOOT_SEARCH_SWEEP = const(7)

_BOOT_PHASE_IDLE = const(0)

_BOOT_PHASE_WAIT_FRAME = const(1)

_BOOT_PHASE_LATERAL = const(2)

_task_mode = _MODE_BOOT_SYNC

_next_task_mode = -1

_prev_task_mode = _MODE_SEARCH

_target_edge = 0

_target_obj_id = 0

_target_obj_world_x = 999.0

_target_obj_world_y = 999.0

_target_sel_id_for_cam = 0

_target_rel_x_for_cam = 999.0

_target_rel_y_for_cam = 999.0

_self_ready = 0

_self_mode = 0

_self_sub = 0

# 当前 SEARCH/RELOCALIZE 跟随阶段是否已经通过视觉看到主车；通过无线 flags.bit3 上报。
_wireless_car_seen = False

_follower_reenter_creep = False

_cmd_seq = 0

_master_cmd_sub = 0

_follower_cmd_yaw_dir = 999.9

# 从车不下发第一次横扫阵列保持模式；保留占位供 base.py 主从共用组帧。
_search_first_sweep_mode_to_other = 0

_search_first_boot = True

# 开局朝主车方向的有效标志。首次编队成功（_search_formation_locked）后清除，
# 之后等主车的分支恢复原来的"跟随主车车头角"行为。
_boot_face_active = True

# 从车完成自身开局横移后进入首次 SEARCH；若此时主车仍在 BOOT_SYNC 横移，
# 该标志临时允许 SEARCH 编队继续跟随主车，主车进入 SEARCH 后自动清除。
_search_boot_follow_active = False

_approach_plan_valid = 0

_approach_plan_edge = 0

_approach_plan_obj_id = 0

_approach_self_yaw = 0.0

_approach_self_dir = 0

_approach_cmd_to_other = 999.9

_route_obs_axis_to_other = 999.9

_route_cmd_to_other = 999.9

_push_route_axis = 999.9

_push_route_phase = 0

_push_route_move_yaw = 999.9

_push_route_restore_push = 0

# 从车自身不使用，但 base.state_init 会写它（base.py 主从共用），保留占位避免动态建属性。
_obj_map_count = 0

_obj_done = [0, 0, 0, 0, 0, 0]

_obj_remain = [0, 1, 1, 1, 1, 1]

# 数量模式及全局剩余数由 base.state_init 按 config 重置。
_obj_count_mode = 3

_obj_total_remaining = 5

_obj_group_remain = []

_APPROACH_REQ_NONE = const(0)

_APPROACH_REQ_FORCED_PUSH_YAW = const(1)

_APPROACH_REQ_RECLOSE = const(2)

_APPROACH_REQ_RESTART_CLOSE = const(3)

_approach_req = _APPROACH_REQ_NONE

_approach_push_yaw = 999.9

_approach_spin_dir = 0

_approach_do_back = 0

_approach_from_push_yaw = 999.9

_approach_route_move_yaw = 999.9

_recover_edge = 0

# 最近一次已完成推出动作的方向，供从车在下一轮 SEARCH 独立计算与主车一致的目标航向。
_last_completed_push_yaw = 999.9

# 功能：清空接近阶段的外部请求，恢复为普通接近流程。
# 该函数会把强制推送角、重靠近方向、后退标志等临时指令全部置为无效，避免一次请求被重复执行。
def clear_approach_request():
    global _approach_req, _approach_push_yaw, _approach_spin_dir
    global _approach_do_back, _approach_from_push_yaw, _approach_route_move_yaw
    _approach_req = _APPROACH_REQ_NONE
    _approach_push_yaw = 999.9
    _approach_spin_dir = 0
    _approach_do_back = 0
    _approach_from_push_yaw = 999.9
    _approach_route_move_yaw = 999.9

# 功能：判断两个数值是否在给定误差范围内，主要用于判断到点、到边和重定位是否满足精度。
# 输入参数：a 和 b 为要比较的两个数值，eps 为允许误差。
# 返回值：True 表示 abs(a-b) 不超过 eps。
def _near(a, b, eps):
    return abs(a - b) <= eps

# 功能：计算两个角度之间的最短有符号差值，并把结果规范到 -180 到 180 度。
# 输入参数：a 为当前角或目标角，b 为参考角，单位均为度。
# 返回值：a 相对 b 的最短角度差，正负号表示旋转方向。
def _angle_diff(a, b):
    return (a - b + 180) % 360 - 180

# 功能：把目标边编号转换为沿该边推出场地的基础推送方向。
# 输入参数：edge 为目标要推出的边，1/2/3/4 分别对应不同场地边界。
# 返回值：推送方向 yaw，单位度；未知边默认返回 0 度。
def _push_yaw(edge):
    if edge == 1:
        return 0.0
    if edge == 2:
        return 270.0
    if edge == 3:
        return 90.0
    if edge == 4:
        return 180.0
    return 0.0

# 功能：根据目标边和目标类别计算从车默认接近锁定角。
# 输入参数：edge 为目标边编号。
# 返回值：从车接近时希望保持的车头角，单位度。
def _lock_yaw(edge):
    push = _push_yaw(edge)
    deg = _APPROACH_LOCK_DEG[_target_obj_id] if 1 <= _target_obj_id < len(_APPROACH_LOCK_DEG) else 30.0
    if _SLOT_FIXED_TENNIS and _target_obj_id == _OBJ_ID_TENNIS:
        return (push + deg) % 360.0
    return (push - deg) % 360.0

# 功能：解析无线收到的接近阶段命令，恢复目标 yaw、环绕方向和有效标志。
# 输入参数：cmd 为 _encode_approach_cmd 生成的浮点命令；900 以上或负数视为无效。
# 返回值：三元组，依次为 yaw、spin_dir、是否有效。
def _decode_approach_cmd(cmd):
    if cmd >= 900.0 or cmd < 0.0:
        return (0.0, 0, False)
    yaw_i = int(cmd) % 360
    dir_code = int(round((cmd - int(cmd)) * 10.0))
    if dir_code == 1:
        return (float(yaw_i), 1, True)
    if dir_code == 2:
        return (float(yaw_i), -1, True)
    return (0.0, 0, False)

# 功能：沿指定旋转方向计算从 start 到 end 需要走过的角度。
# 输入参数：start 为起始角，end 为终止角，spin_dir 为方向，非负表示一个方向，负数表示反方向。
# 返回值：按指定方向累计的角度差，范围为 0 到 360 度。
def _delta_dir(start, end, spin_dir):
    if spin_dir >= 0:
        return (start - end) % 360.0
    return (end - start) % 360.0

# 功能：判断某个角度点是否位于从 start 到 end 的指定方向圆弧上。
# 输入参数：start/end 为圆弧起止角，spin_dir 为圆弧方向，point 为待判断角度。
# 返回值：True 表示 point 在这段圆弧进度内。
def _point_on_arc(start, end, spin_dir, point):
    return _delta_dir(start, point, spin_dir) <= _delta_dir(start, end, spin_dir)

# 功能：查找当前摄像头普通目标缓存中是否存在正在处理的目标。
# 返回值：目标槽位下标；-1 表示当前帧没有看到目标。
def _find_target_in_cam():
    if base._cam_obj_count <= 0 or base._cam_obj_id[0] != _target_obj_id:
        return -1
    return 0

# 功能：判断摄像头是否看到了另一辆车。
# 返回值：0 表示看到了对方车辆槽位；-1 表示没有可用的对方车辆视觉数据。
def _find_car_in_cam():
    return 0 if base._cam_car_x < 900.0 else -1

# 功能：用当前摄像头相对坐标和本车位姿更新目标的世界坐标。
# 目标坐标第一次出现时直接写入，之后使用低通融合，减小视觉抖动对接近和推送路线的影响。
# 输入参数：cam_idx 为目标在 base 摄像头缓存中的槽位下标。
# 返回值：二元组 (cos(yaw), sin(yaw))，供调用者继续做车体系到世界系转换。
def _update_target_world_from_cam(cam_idx):
    global _target_obj_world_x, _target_obj_world_y
    rel_x = base._cam_obj_rel_x[cam_idx]
    rel_y = base._cam_obj_rel_y[cam_idx]
    yaw_rad = radians(base._car.current_angle)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    wx = base._car.Position_X + c * rel_x + s * rel_y
    wy = base._car.Position_Y - s * rel_x + c * rel_y
    if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        _target_obj_world_x = wx
        _target_obj_world_y = wy
    else:
        w = 0.4
        _target_obj_world_x = _target_obj_world_x * (1 - w) + wx * w
        _target_obj_world_y = _target_obj_world_y * (1 - w) + wy * w
    return (c, s)

# 功能：从主车无线共享信息中同步当前搜索目标。
# 从车在搜索/跟随阶段没有自主选目标，主要接收主车发送的目标边、目标 ID 和目标世界坐标。
# 返回值：True 表示成功同步到有效目标；False 表示主车目标信息无效或尚未更新。
def sync_follower_search_target():
    global _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y, _target_sel_id_for_cam
    if base._Other_Target_X >= 900.0 or base._Other_Target_Y >= 900.0:
        return False
    if base._Other_Target_Edge <= 0 or base._Other_Target_ObjId <= 0:
        return False
    _target_edge = base._Other_Target_Edge
    _target_obj_id = base._Other_Target_ObjId
    _target_obj_world_x = base._Other_Target_X
    _target_obj_world_y = base._Other_Target_Y
    _target_sel_id_for_cam = _target_obj_id
    return True

# 功能：根据目标边得到球体辅助推送使用的基础推送方向。
# 输入参数：edge 为目标边编号。
# 返回值：推送 yaw，单位度；未知边返回 0。
def _ball_push_yaw_for_edge(edge):
    if edge == 1:
        return 0.0
    if edge == 2:
        return 270.0
    if edge == 3:
        return 90.0
    if edge == 4:
        return 180.0
    return 0.0

# VDOCK 双轴分工——横向听球（管 V 紧不紧），前向听物体（管推杆贴不贴住）。
# 依据：从车在车体系平移 δ 时，任何固定点的车体坐标恰好变化 −δ，雅可比是 −I，
# 两轴零交叉耦合，所以可以各用最直接的传感器，且两个约束【同时】满足，
# 不需要像原来 VDOCK→VTOUCH 那样先后做（那个顺序正是丢球的根源）。
# 当前固定球参考：五类统一为左球(9,11)、右球(-11,12)；
# 就位物体目标按类别读取_CLOSE_DIST_FOLLOWER_BY_OBJ。
# 网球两个轴都听物体：网球黄绿色会在色块粗筛里把橙球顶掉（它就在两杆之间、
# 离镜头最近、画面里比 2.5cm 的标记球大一个量级，而 detect_balls 按像素数取最大）。
# 全程不碰球识别，也就不需要为它单独固定就位槽位。
_DOCK_OBJ_ONLY_TENNIS = const(1)

# 功能：取物体在从车车体系下的实测值与紧 V 位参考值。
# 参考直接用现成的两张表，已由实车标定确认。
# 返回值：五元组 (是否有效, 实测x, 实测y, 参考x, 参考y)。
def _dock_obj_ref():
    obj_id = _target_obj_id
    if not 0 <= obj_id < len(_CLOSE_DIST_FOLLOWER_BY_OBJ):
        return (False, 0.0, 0.0, 0.0, 0.0)
    cam_rel = _find_target_in_cam()
    if cam_rel < 0:
        return (False, 0.0, 0.0, 0.0, 0.0)
    return (True, base._cam_obj_rel_x[cam_rel], base._cam_obj_rel_y[cam_rel],
            _CLOSE_LAT_OFFSET_BY_OBJ[obj_id], _CLOSE_DIST_FOLLOWER_BY_OBJ[obj_id])

# 功能：判断当前目标是否全程不使用球识别就位。
# 输入参数：lock_yaw 为当前锁定车头角，用于判定所在槽位；None 表示由本函数自取。
# 除网球外，被 _BALL_SIDE_DISABLE 屏蔽的那一侧也走纯物体就位，
# 这样屏蔽后 VDOCK 会立刻把垂直轴交还物体环，而不是每次白等 _VDOCK_BALL_WAIT_MS。
def _dock_obj_only(lock_yaw=None):
    if _DOCK_OBJ_ONLY_TENNIS and _target_obj_id == _OBJ_ID_TENNIS:
        return True
    if _BALL_SIDE_DISABLE:
        if lock_yaw is None:
            lock_yaw = _wait_lock_yaw if _wait_lock_valid else _lock_yaw(_target_edge)
        return _ball_side_disabled(_expected_side(lock_yaw))
    return False

# 功能：根据锁定车头角判断当前更应该使用左球还是右球作为参考侧。
# 输入参数：lock_yaw 为推送阶段锁定车头角。
# 返回值：1 或 2，表示期望参考侧。
def _expected_side(lock_yaw):
    return 2 if _angle_diff(lock_yaw, _ball_push_yaw_for_edge(_target_edge)) < 0.0 else 1

# 功能：判断指定槽位的标记球是否被 _BALL_SIDE_DISABLE 现场屏蔽。
# 输入参数：side 为槽位（1=左/紫，2=右/橙）。
# 返回值：True 表示该侧球当作"永远看不见"处理。
def _ball_side_disabled(side):
    if side == 1:
        return _BALL_SIDE_DISABLE == 1 or _BALL_SIDE_DISABLE == 3
    return _BALL_SIDE_DISABLE == 2 or _BALL_SIDE_DISABLE == 3

def _slot_ball_ref(lock_yaw):
    obj_id = _target_obj_id
    if not _PUSH_BALL_SLOT_SIDE_ENABLE or obj_id < 1 or obj_id > 5:
        return (False, 0.0, 0.0, 0.0, 0.0)
    side = _expected_side(lock_yaw)
    # 现场诊断屏蔽：该侧球直接判成不可见，误检不再有任何机会进入闭环。
    if _ball_side_disabled(side):
        return (False, 0.0, 0.0, 0.0, 0.0)
    idx = (obj_id - 1) * 4
    if side == 1:
        rel_x = base._cam_ball_rel_x[0]
        rel_y = base._cam_ball_rel_y[0]
        ref_x = _PUSH_BALL_FIXED_REF_BY_OBJ[idx]
        ref_y = _PUSH_BALL_FIXED_REF_BY_OBJ[idx + 1]
    else:
        rel_x = base._cam_ball_rel_x[1]
        rel_y = base._cam_ball_rel_y[1]
        ref_x = _PUSH_BALL_FIXED_REF_BY_OBJ[idx + 2]
        ref_y = _PUSH_BALL_FIXED_REF_BY_OBJ[idx + 3]
    if rel_x >= 900.0 or rel_y >= 900.0:
        return (False, 0.0, 0.0, 0.0, 0.0)
    # 几何合法窗口收紧到槽位球的真实工作范围，把同色误检挡在闭环之外。
    # 判成"看不见球"是安全的：VDOCK 会把该轴交还物体环，就位判据也有物体兜底。
    if abs(rel_x) > _SLOT_BALL_MAX_LAT:
        return (False, 0.0, 0.0, 0.0, 0.0)
    if rel_x * rel_x + rel_y * rel_y > _SLOT_BALL_MAX_DIST * _SLOT_BALL_MAX_DIST:
        return (False, 0.0, 0.0, 0.0, 0.0)
    return (True, rel_x, rel_y, ref_x, ref_y)

# 功能：取"紧 V 位时主车中心应在从车车体系的什么位置"。
# 已学习过就用学习值；否则用对称几何种子 (±d, d)，d 取该目标的推杆贴合距离。
# 输入参数：side 为期望槽位（1/2）。
# 返回值：二元组 (x, y)。
def _dock_wl_target(side):
    i = 0 if side == 1 else 2
    if _dock_wl_valid[0 if side == 1 else 1]:
        return (_dock_wl_ref[i], _dock_wl_ref[i + 1])
    obj_id = _target_obj_id
    d = _CLOSE_DIST_FOLLOWER_BY_OBJ[obj_id] if 0 <= obj_id < len(_CLOSE_DIST_FOLLOWER_BY_OBJ) else 10.0
    return (d if side == 1 else -d, d)

# 功能：就位成功瞬间记录主车中心的实测相对位置，作为后续无线兜底的目标。
# 和原来学 bL 是同一个信息源，但少一层换算。
# 输入参数：side 为当前槽位。
def _dock_wl_learn(side):
    ok, wx, wy = _wl_rel()
    if not ok:
        return
    # 合理性检查：两车中心间距应在 10~30cm，超出视为里程计/无线异常，不采用。
    d2 = wx * wx + wy * wy
    if d2 < 100.0 or d2 > 900.0:
        return
    i = 0 if side == 1 else 2
    if _dock_wl_valid[0 if side == 1 else 1]:
        _dock_wl_ref[i] = _dock_wl_ref[i] * 0.5 + wx * 0.5
        _dock_wl_ref[i + 1] = _dock_wl_ref[i + 1] * 0.5 + wy * 0.5
    else:
        _dock_wl_ref[i] = wx
        _dock_wl_ref[i + 1] = wy
        _dock_wl_valid[0 if side == 1 else 1] = True

# 功能：把主车无线位置换算成从车车体系下的相对位置。
# 该相对位置可在球体暂时丢失时作为备选参考，辅助保持双车间相对几何关系。
# 返回值：三元组，依次为是否有效、相对 x、相对 y。
def _wl_rel():
    ox = base._Other_Car_X
    oy = base._Other_Car_Y
    if ox >= 900.0 or oy >= 900.0:
        return (False, 0.0, 0.0)
    dx = ox - base._car.Position_X
    dy = oy - base._car.Position_Y
    yaw_rad = radians(base._car.current_angle)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    return (True, c * dx - s * dy, s * dx + c * dy)

_BOOT_OTHER_FRESH_MS = const(800)

_BOOT_BEEP_CAM_MS = const(120)

_BOOT_BEEP_WIRELESS_MS = const(400)

_BOOT_BEEP_GAP_MS = const(250)

_boot_phase = _BOOT_PHASE_IDLE

_boot_lateral_y0 = 0.0

_beep_cam_done = False

_beep_cam_t0 = 0

_beep_wireless_done = False

# 功能：判断主车在开机同步阶段是否已经在线且 ready 状态有效。
# 这里同时检查无线时间戳新鲜度，避免使用很久之前残留的主车状态。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示主车最近回包且 ready 值达到开机同步要求。
def _other_boot_ready(now_ms):
    if base._Other_Car_Ready_Ts == 0:
        return False
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _BOOT_OTHER_FRESH_MS:
        return False
    return base._Other_Car_Ready >= 1

# 功能：重置开机同步状态。
# 会清空蜂鸣提示状态和本车 ready 状态，确保重新进入 BOOT_SYNC 时从干净状态开始。
def boot_sync_reset():
    global _master_cmd_sub, _self_ready
    global _boot_phase, _boot_lateral_y0
    global _search_boot_follow_active
    global _beep_cam_done, _beep_cam_t0, _beep_wireless_done
    _boot_phase = _BOOT_PHASE_IDLE
    _boot_lateral_y0 = 0.0
    _search_boot_follow_active = False
    _beep_cam_done = False
    _beep_cam_t0 = 0
    _beep_wireless_done = False
    _self_ready = 0
    _master_cmd_sub = _CMD_SUB_NONE

# 功能：执行从车开机同步流程。
# 摄像头链路正常即 ready=1，从车保持当前航向停车；收到主车启动信号后
# 记录自身里程计起点，以与主车相同的航向、速度和距离独立横移；
# 自身横移完成后才进入首次 SEARCH 跟随主车。
# 输入参数：now_ms 为当前时间戳。
# 开机阶段与横移期间锁定的车头角。
_BOOT_FACE_YAW = 90.0

def boot_sync_update(now_ms):
    global _master_cmd_sub, _next_task_mode, _self_ready
    global _boot_phase, _boot_lateral_y0
    global _search_boot_follow_active
    global _beep_cam_done, _beep_cam_t0, _beep_wireless_done
    cam_ok = base._cam_rx_last_ms != 0
    _self_ready = 1 if cam_ok else 0
    # 开机阶段锁定 _BOOT_FACE_YAW，等待主车发出启动命令。
    if cam_ok and (not _beep_cam_done):
        _beep_cam_done = True
        _beep_cam_t0 = now_ms
        base._fix_beep_active = 1
        base._fix_beep_until_ms = ticks_add(now_ms, _BOOT_BEEP_CAM_MS)
    if not _beep_wireless_done and _beep_cam_done and (ticks_diff(now_ms, _beep_cam_t0) >= _BOOT_BEEP_GAP_MS) and _other_boot_ready(now_ms):
        _beep_wireless_done = True
        base._fix_beep_active = 1
        base._fix_beep_until_ms = ticks_add(now_ms, _BOOT_BEEP_WIRELESS_MS)
    if not cam_ok:
        _master_cmd_sub = _CMD_SUB_NONE
        _boot_phase = _BOOT_PHASE_IDLE
        _boot_lateral_y0 = 0.0
        _search_boot_follow_active = False
        base.request_hold(_BOOT_FACE_YAW)
        return

    peer_fresh = (
        base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
    )

    if _boot_phase == _BOOT_PHASE_IDLE:
        start_requested = (
            base._Other_Master_Cmd_Sub == _CMD_SUB_BOOT_START
            or (
                peer_fresh
                and (
                    base._Other_Car_Mode == _MODE_SEARCH
                    or base._Other_Master_Cmd_Sub == _CMD_SUB_BOOT_SEARCH_SWEEP
                )
            )
        )
        base.request_hold(_BOOT_FACE_YAW)
        if not start_requested:
            return
        if not _beep_wireless_done:
            _beep_wireless_done = True
            base._fix_beep_active = 1
            base._fix_beep_until_ms = ticks_add(now_ms, _BOOT_BEEP_WIRELESS_MS)
        if debug_switch.BOOT_DIRECT_DONE_ENABLE:
            _boot_phase = _BOOT_PHASE_IDLE
            _search_boot_follow_active = False
            _next_task_mode = _MODE_DONE
            return
        _boot_phase = _BOOT_PHASE_WAIT_FRAME
        return

    if _boot_phase == _BOOT_PHASE_WAIT_FRAME:
        _boot_phase = _BOOT_PHASE_LATERAL
        _boot_lateral_y0 = base._car.Position_Y
        base.request_world(-float(base._BOOT_LONGITUDINAL_SPEED), float(base._BOOT_LATERAL_SPEED), _BOOT_FACE_YAW)
        return

    if _boot_phase == _BOOT_PHASE_LATERAL:
        base.request_world(-float(base._BOOT_LONGITUDINAL_SPEED), float(base._BOOT_LATERAL_SPEED), _BOOT_FACE_YAW)
        if base._car.Position_Y - _boot_lateral_y0 >= float(base._BOOT_LATERAL_DISTANCE_FOLLOWER_CM):
            _boot_phase = _BOOT_PHASE_IDLE
            _search_boot_follow_active = True
            _next_task_mode = _MODE_SEARCH
        return

_KP_FACE_OBJ = const(60)

_KD_FACE_OBJ = 1.0

_FACE_GYRO_MAX = const(200)

# 主车已经开始 APPROACH ORBIT 且两车角度差达到安全值后，从车先按主车
# 共享的物体坐标转向并锁存航向；暂时看不到目标时以该车体前向速度继续寻找。
_APPROACH_WAIT_TARGET_FWD_SPEED = 40.0

_APPROACH_READY_SUB = const(20)

_AP_FACE = const(0)

_AP_ORBIT = const(1)

_AP_VDOCK = const(3)

# V 字贴靠完成后的物体贴合微调（方案C）。球闭环只保证两车【相对】几何正确，
# 两车 IMU 的差分漂移会让整个 V 相对物体转过一个角度，表现为一侧推杆贴住、
# 另一侧没贴住。这一步让从车用自己的物体视觉沿车头方向补上这段间隙，
# 补完把当时的球位置锁存成 PUSH 的球参考——于是球参考每个物体周期都被
# 物体这个绝对基准重新校准一次，IMU 漂多少都不再累积到推杆上。
_AP_ROUTE_BACK = const(10)

_AP_PRE_ORBIT = const(11)

_APPROACH_YAW_EPS = const(6)

# 侧向阵型进 APPROACH 时，从车原地朝向目标物体的预转向容差，与 AP_ORBIT
# 绕行完成判定的 _APPROACH_YAW_EPS 分开，避免互相牵连。
_APPROACH_FACE_YAW_EPS = const(3)

_LOST_RECOVER_MAX_MS = const(2000)

# FACE / PRE_ORBIT 看不到目标时更快退回 SEARCH；其他接近阶段仍沿用
# _LOST_RECOVER_MAX_MS，保留较长的缓存目标恢复窗口。
_FACE_PRE_ORBIT_LOST_SEARCH_MS = const(1000)

_FACE_DEADBAND_DEG = const(0)

_ORBIT_RADIAL_KP = 4.0

_ORBIT_RUN_RADIAL_KP = 4.5

# 与主车一致：普通 APPROACH/ORBIT 的径向向内前馈系数。
# v_ff = gain * orb_spd^2 / rw；重靠近 ORBIT 不使用该前馈。
_ORBIT_INWARD_FF_GAIN = 0.03

_ORBIT_SPEED_MAX = const(100)

_ORBIT_SPEED_MIN = const(20)

_ORBIT_OVERSHOOT_MARGIN_DEG = const(3)

_ORBIT_RADIAL_MAX = const(35)

_ORBIT_FACE_LEAD_GAIN = 1.5

_ORBIT_FACE_LEAD_MAX_DEG = 12.0

_PRE_ORBIT_RADIAL_MAX = const(80)

_PRE_ORBIT_R_EPS = const(5)

_PRE_ORBIT_FACE_EPS = const(8)

_FACE_ORBIT_X_EPS = const(8)

_FACE_ORBIT_R_EPS = const(5)

_ORBIT_RELOCALIZE_ENABLE = const(1)

_ORBIT_RELOCALIZE_MAX_CORR = const(45)

_ORBIT_RELOCALIZE_BLEND = const(1)

_ARC_ORBIT_R = const(20)

# FACE 阶段横向居中比例系数。
_FACE_LAT_KP = const(5)

# FACE 阶段横向速度最大值。
_FACE_LAT_MAX = const(60)

_ORBIT_LEADER_CLEAR_DEG = const(8)

_ORBIT_ANCHOR_BLEND = 0.12

_ORBIT_ANCHOR_MAX_STEP = 1.5

_ROUTE_RECLOSE_YAW_EPS = const(10)

_ROUTE_RECLOSE_ORBIT_SPEED = const(35)

_ROUTE_RECLOSE_ORBIT_R = const(18)

_ROUTE_RECLOSE_ORBIT_RADIAL_KP = const(4)

_ROUTE_RECLOSE_ORBIT_RADIAL_MAX = const(50)


_READY_ROUTE_BACK_DIST = const(6)

_READY_ROUTE_BACK_SPEED = const(35)

_READY_ROUTE_BACK_TIMEOUT_MS = const(900)

_READY_ROUTE_MIN_YAW_SEP_DEG = const(60)

_ORBIT_YAW_SEP_DEG = const(10)

_prelock_sub = 0

_mode_sub = _AP_FACE

_mode_hold_ms = 0

_vdock_ok_cnt = 0

# VDOCK 找球扫描状态：进度 u（0=观察位，1=紧 V 位）、扫描方向、上次推进时间戳。
_vdock_creep_u = 0.0

_vdock_creep_dir = 1.0

_vdock_creep_ts = 0

_vdock_ball_lost_t0 = 0

# 就位阶段球不可见的起始时刻。
_vdock_ball_lost_t0 = 0

# VTOUCH 状态：起点坐标（用于限制单次微调位移）、达标计数。
# 方案C 的产物：物体贴合完成瞬间实测的球位置，作为本轮 PUSH 的球参考。
# 这是"被物体校准过"的紧 V 真值，优先级高于 _PUSH_BALL_FIXED_REF_BY_OBJ 手标值。
_vtouch_ref_valid = False

_vtouch_ref_x = 0.0

_vtouch_ref_y = 0.0

_approach_t0 = 0

# 主车退回 SEARCH 时，从车需连续观察一小段时间后再退出 APPROACH，避免单帧丢包误回退。
_approach_peer_search_t0 = 0

_ap_lost_lock_yaw = 0.0

_route_back_x0 = 0.0

_route_back_y0 = 0.0

_orbit_start_yaw = 0.0

_orbit_target_yaw = 0.0

_orbit_dir = 0

_orbit_plan_valid = False

# 功能：请求接近状态机从 CLOSE 子状态重新开始。
# 该函数通常用于等待阶段发现靠近姿态变差后，不重新做完整环绕，只重新做最后贴近。
def restart_close():
    global _reset_first_sub
    _reset_first_sub = _AP_VDOCK
    _reset_approach_phase(_AP_VDOCK)

_reset_first_sub = _AP_FACE

_ap_anchor_valid = False

_ap_anchor_x = 999.0

_ap_anchor_y = 999.0

# 功能：重置接近阶段状态机。
# 从车进入 APPROACH 时会优先处理主车发来的重靠近/强制推送角请求，否则按普通流程对目标进行面向、环绕、居中和靠近。
# 同时会清空面向目标控制、环绕锚点、计时/计数状态，并配置从车的面向目标角速度控制参数。
def approach_reset():
    global _approach_cmd_to_other, _approach_plan_valid, _self_sub
    global _prelock_sub, _reset_first_sub
    global _mode_sub, _mode_hold_ms, _vdock_ok_cnt, _approach_t0, _ap_lost_lock_yaw
    global _vdock_creep_u, _vdock_creep_dir, _vdock_creep_ts, _vdock_ball_lost_t0
    global _vtouch_ref_valid
    global _approach_peer_search_t0
    global _route_back_x0, _route_back_y0
    global _orbit_plan_valid
    if _approach_req == _APPROACH_REQ_RECLOSE:
        if not start_reclose_plan(_approach_route_move_yaw):
            _approach_plan_valid = 0
            _approach_cmd_to_other = 999.9
        clear_approach_request()
    elif _approach_req == _APPROACH_REQ_RESTART_CLOSE:
        restart_close()
        clear_approach_request()
    elif _approach_req == _APPROACH_REQ_FORCED_PUSH_YAW:
        if not start_forced_push_yaw_plan(_approach_push_yaw, _approach_spin_dir, bool(_approach_do_back), _approach_from_push_yaw):
            _approach_plan_valid = 0
            _approach_cmd_to_other = 999.9
        clear_approach_request()
    _prelock_sub = 0
    first_sub = _reset_first_sub
    _mode_sub = first_sub
    _self_sub = _mode_sub
    _reset_first_sub = _AP_FACE
    _mode_hold_ms = 0
    _vdock_ok_cnt = 0
    _vdock_creep_u = 0.0
    _vdock_creep_dir = 1.0
    _vdock_creep_ts = 0
    _vdock_ball_lost_t0 = 0
    # 换目标就作废上一轮的物体校准值：网球走紫球、其他目标走橙球，
    # 两者差一个球间距，跨目标沿用会引入 2.5cm 的固定偏差。
    _vtouch_ref_valid = False
    _approach_t0 = 0
    _approach_peer_search_t0 = 0
    _ap_lost_lock_yaw = base._car.current_angle
    _route_back_x0 = 0.0
    _route_back_y0 = 0.0
    _orbit_plan_valid = False
    if first_sub == _AP_FACE:
        _approach_plan_valid = 0
    base.clear_face()
    _clear_orbit_anchor()
    base.configure_face(_KP_FACE_OBJ, _KD_FACE_OBJ, _FACE_GYRO_MAX)

# 功能：清除环绕阶段使用的目标锚点。
# 锚点用于摄像头短暂丢目标时仍能围绕最近一次目标世界坐标运动。
def _clear_orbit_anchor():
    global _ap_anchor_valid, _ap_anchor_x, _ap_anchor_y
    _ap_anchor_valid = False
    _ap_anchor_x = 999.0
    _ap_anchor_y = 999.0

# 功能：在接近阶段记录当前视觉目标世界坐标作为环绕锚点。
# 副车进入 PRE_ORBIT/ORBIT 前会调用本函数，用本车当前帧亲眼看到的目标位置修正一次环绕中心。
# 这样可以避免锚点继续沿用主车共享坐标或融合坐标，减小后续绕偏心圆的风险。
# 输入参数：cam_obj 为目标在摄像头缓存中的槽位；槽位无效、目标类别不匹配或视觉坐标无效时会清空锚点。
def _set_orbit_anchor(cam_obj):
    global _ap_anchor_valid, _ap_anchor_x, _ap_anchor_y
    if cam_obj < 0 or _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        _clear_orbit_anchor()
        return
    _ap_anchor_valid = True
    _ap_anchor_x = _target_obj_world_x
    _ap_anchor_y = _target_obj_world_y

# 功能：在 ORBIT 阶段用当前视觉目标坐标缓慢修正环绕锚点。
# 该函数只在目标类别匹配、视觉世界坐标有效时工作；每次修正会先乘以小权重，再限制最大步长，避免视觉抖动让圆心突然跳变。
# 输入参数：cam_obj 为目标在摄像头缓存中的槽位。
# 返回值：True 表示本次对锚点进行了有效检查或修正；False 表示条件不足，锚点保持不变。
def _update_orbit_anchor_from_cam(cam_obj):
    global _ap_anchor_x, _ap_anchor_y
    if not _ap_anchor_valid:
        return False
    if cam_obj < 0 or cam_obj >= base._cam_obj_count:
        return False
    if base._cam_obj_id[cam_obj] != _target_obj_id:
        return False
    cam_x = base._cam_obj_x[cam_obj]
    cam_y = base._cam_obj_y[cam_obj]
    if cam_x >= 900.0 or cam_y >= 900.0:
        return False
    dx = cam_x - _ap_anchor_x
    dy = cam_y - _ap_anchor_y
    dist = sqrt(dx * dx + dy * dy)
    if dist < 0.1:
        return True
    step = dist * _ORBIT_ANCHOR_BLEND
    if step > _ORBIT_ANCHOR_MAX_STEP:
        step = _ORBIT_ANCHOR_MAX_STEP
    _ap_anchor_x += dx * step / dist
    _ap_anchor_y += dy * step / dist
    return True

# 功能：把环绕锚点从世界坐标转换到当前车体系相对坐标。
# 返回值：二元组 (rel_x, rel_y)，表示锚点在车体坐标系中的相对位置。
def _orbit_anchor_rel():
    dx = _ap_anchor_x - base._car.Position_X
    dy = _ap_anchor_y - base._car.Position_Y
    yaw_rad = radians(base._car.current_angle)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    return (c * dx - s * dy, s * dx + c * dy)

# 功能：环绕结束后利用锚点和当前视觉相对坐标修正本车位置。
# 输入参数：cam_obj 为目标在摄像头缓存中的槽位。
# 返回值：True 表示发起了位置修正请求，False 表示条件不足或修正量不可信。
def _relocalize_after_orbit(cam_obj):
    if not _ORBIT_RELOCALIZE_ENABLE:
        return False
    if not _ap_anchor_valid or cam_obj < 0:
        return False
    if base._cam_obj_id[cam_obj] != _target_obj_id:
        return False
    rel_x = base._cam_obj_rel_x[cam_obj]
    rel_y = base._cam_obj_rel_y[cam_obj]
    if rel_x >= 900.0 or rel_y >= 900.0:
        return False
    yaw_rad = radians(base._car.current_angle)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    new_x = _ap_anchor_x - c * rel_x - s * rel_y
    new_y = _ap_anchor_y + s * rel_x - c * rel_y
    dx = new_x - base._car.Position_X
    dy = new_y - base._car.Position_Y
    if sqrt(dx * dx + dy * dy) > _ORBIT_RELOCALIZE_MAX_CORR:
        return False
    w = _ORBIT_RELOCALIZE_BLEND
    if w < 0.0:
        w = 0.0
    if w > 1.0:
        w = 1.0
    fix_x = base._car.Position_X * (1.0 - w) + new_x * w
    fix_y = base._car.Position_Y * (1.0 - w) + new_y * w
    if fix_x < 900.0:
        base._pos_fix_x = fix_x
        base._pos_fix_x_valid = True
    if fix_y < 900.0:
        base._pos_fix_y = fix_y
        base._pos_fix_y_valid = True
    base._pos_fix_req = True
    return True

# 功能：重置接近阶段的子状态和协同命令。
# 用于普通进入接近、绕障重靠近、推送丢失恢复等场景；在路径绕障阶段会保留部分主车路线命令。
# 输入参数：first_sub 为接近阶段初始子状态，默认从 FACE 开始。
def _reset_approach_phase(first_sub=_AP_FACE):
    global _follower_cmd_yaw_dir, _master_cmd_sub, _route_cmd_to_other, _route_obs_axis_to_other
    global _mode_sub, _mode_hold_ms, _vdock_ok_cnt, _approach_t0, _ap_lost_lock_yaw
    global _vdock_creep_u, _vdock_creep_dir, _vdock_creep_ts, _vdock_ball_lost_t0
    global _route_back_x0, _route_back_y0
    global _orbit_plan_valid
    _mode_sub = first_sub
    _mode_hold_ms = 0
    _vdock_ok_cnt = 0
    _vdock_creep_u = 0.0
    _vdock_creep_dir = 1.0
    _vdock_creep_ts = 0
    _vdock_ball_lost_t0 = 0
    _approach_t0 = 0
    _ap_lost_lock_yaw = base._car.current_angle
    _route_back_x0 = 0.0
    _route_back_y0 = 0.0
    _orbit_plan_valid = False
    _route_obs_axis_to_other = 999.9
    if _push_route_phase not in (1, 3):
        _route_cmd_to_other = 999.9
        _follower_cmd_yaw_dir = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
    base.clear_face()
    _clear_orbit_anchor()

# 功能：方案C 的收尾。把"推杆已贴住物体"这一刻实测的球位置锁存成本轮 PUSH 的球参考。
# 这样球参考每个物体周期都被物体这个绝对基准重新校准一次，两车 IMU 的差分漂移
# 不再累积到推杆上。看不到球时保持上一轮的锁存值（或退回手标固定表）。
# 输入参数：lock_yaw 为当前锁定车头角，用于选期望侧球。
def _vtouch_latch_ref(lock_yaw):
    global _vtouch_ref_valid, _vtouch_ref_x, _vtouch_ref_y
    ok, bx, by, unused_rx, unused_ry = _slot_ball_ref(lock_yaw)
    if not ok:
        return
    _vtouch_ref_valid = True
    _vtouch_ref_x = bx
    _vtouch_ref_y = by

# 功能：VDOCK 结束前的主车时序门。只有主车已经进入 WAIT_READY，且无线新鲜、
# 目标一致时，从车才允许离开 VDOCK 进入 WAIT_READY；否则继续在 VDOCK 内保持/闭环。
def _leader_wait_ready_for_vdock(now_ms):
    if base._Other_Car_Mode != _MODE_WAIT_READY:
        return False
    if base._Other_Car_Ready_Ts == 0:
        return False
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _READY_OTHER_FRESH_MS:
        return False
    if base._Other_Target_Edge != _target_edge or base._Other_Target_ObjId != _target_obj_id:
        return False
    return True

# 功能：运行从车接近阶段的核心子状态机。
# 从车会按照主车共享的目标和接近命令完成面向目标、预环绕、环绕到另一侧夹推角、横向居中和最终靠近。
# 输入参数：now_ms 为当前时间戳；target_yaw 为从车最终应达到的接近角；spin_dir 为指定环绕方向，None 表示自动选择。
# 返回值：True 表示接近已经完成，可以进入等待主车就位；False 表示仍在接近或已请求切换到搜索。
def _run_approach_phase(now_ms, target_yaw, spin_dir=None):
    global _follower_reenter_creep, _next_task_mode
    global _mode_sub, _mode_hold_ms, _vdock_ok_cnt, _approach_t0, _ap_lost_lock_yaw
    global _vdock_creep_u, _vdock_creep_dir, _vdock_creep_ts, _vdock_ball_lost_t0
    global _vtouch_ref_valid, _vtouch_ref_x, _vtouch_ref_y
    global _route_back_x0, _route_back_y0
    global _orbit_start_yaw, _orbit_target_yaw, _orbit_dir, _orbit_plan_valid
    ap_sub = _mode_sub
    if ap_sub == _AP_ROUTE_BACK:
        base.clear_face()
        if _approach_t0 == 0:
            _approach_t0 = now_ms
            _route_back_x0 = base._car.Position_X
            _route_back_y0 = base._car.Position_Y
        dx = base._car.Position_X - _route_back_x0
        dy = base._car.Position_Y - _route_back_y0
        if sqrt(dx * dx + dy * dy) >= _READY_ROUTE_BACK_DIST or ticks_diff(now_ms, _approach_t0) > _READY_ROUTE_BACK_TIMEOUT_MS:
            base.request_hold(target_yaw)
            _approach_t0 = 0
            _mode_sub = _AP_FACE
            _mode_hold_ms = 0
            return False
        yaw_rad = radians(target_yaw)
        speed = _READY_ROUTE_BACK_SPEED
        base.request_world(-sin(yaw_rad) * speed, -cos(yaw_rad) * speed, target_yaw)
        return False
    if ap_sub == _AP_VDOCK:
        # 就位物体目标按物体类别读取。
        # 球修正只在就位阶段作用于推送垂直方向，把 V 收紧。
        # 车头全程保持 target_yaw，只做平移。
        base.clear_face()
        yaw_rad = radians(target_yaw)
        cy = cos(yaw_rad)
        sy = sin(yaw_rad)
        ok_o, ox, oy, rox, roy = _dock_obj_ref()
        if ok_o:
            _approach_t0 = 0
            _vdock_creep_dir = -1.0
            _vdock_creep_ts = 0
            # 按轴彻底分工，两个自由度对应两个独立约束，互不相抢：
            #   沿推送方向  <- 物体环（我离物体多远）
            #   垂直推送方向 <- 球（V 紧不紧）；球不可用时这一轴交还物体环
            # 之前是物体环管两个轴、球又来管垂直轴，两个约束抢同一个自由度，
            # 主车位置稍有偏差就无解；再加上"物体到位才允许球修正"的门控，
            # 球一动就把物体误差顶出容差(垂直移动在车体两轴各投影 0.707，
            # 移 3cm 就超 2cm 容差)、门控关闭、物体环再拉回来 —— 必然极限环。
            d_rad = radians(_angle_diff(_ball_push_yaw_for_edge(_target_edge), target_yaw))
            sd = sin(d_rad)
            cd = cos(d_rad)
            ex = ox - rox
            ey = oy - roy
            e_fwd = ex * sd + ey * cd
            e_obj_open = -ex * cd + ey * sd
            obj_ok = abs(e_fwd) < _VDOCK_POS_EPS
            v_perp = 0.0
            ball_ok = True
            if _dock_obj_only(target_yaw):
                obj_ok = obj_ok and abs(e_obj_open) < _VDOCK_POS_EPS
            else:
                ok_b, bx, by, rbx, rby = _slot_ball_ref(target_yaw)
                if ok_b:
                    _vdock_ball_lost_t0 = 0
                    e_open = -(bx - rbx) * cd + (by - rby) * sd
                    # e_open 符号跟槽位走；不对称判据用与槽位无关的 e_sep（正=分离）。
                    e_sep = e_open if sd >= 0.0 else -e_open
                    ball_ok = abs(e_open) < _VDOCK_OPEN_EPS
                    kp = _VDOCK_OPEN_KP if e_sep >= 0.0 else _VDOCK_OPEN_KP * _PUSH_BALL_OPEN_RELAX
                    mx = _VDOCK_OPEN_MAX if e_sep >= 0.0 else _VDOCK_OPEN_MAX * _PUSH_BALL_OPEN_RELAX
                    v_perp = kp * e_open
                    if v_perp > mx:
                        v_perp = mx
                    elif v_perp < -mx:
                        v_perp = -mx
                    e_obj_open = 0.0
                else:
                    # 看不到球：垂直轴交还物体环；等够 _VDOCK_BALL_WAIT_MS 就不再要求球。
                    if _vdock_ball_lost_t0 == 0:
                        _vdock_ball_lost_t0 = now_ms
                    ball_ok = ticks_diff(now_ms, _vdock_ball_lost_t0) > _VDOCK_BALL_WAIT_MS
                    obj_ok = obj_ok and abs(e_obj_open) < _VDOCK_POS_EPS
            if obj_ok and ball_ok:
                base.request_hold(target_yaw)
                _vdock_ok_cnt += 1
                if _vdock_ok_cnt < _VDOCK_OK_REQUIRE:
                    _mode_hold_ms = 0
                    return False
                if _mode_hold_ms == 0:
                    _mode_hold_ms = now_ms
                if ticks_diff(now_ms, _mode_hold_ms) > _VDOCK_HOLD_MS:
                    if not _leader_wait_ready_for_vdock(now_ms):
                        base.request_hold(target_yaw)
                        return False
                    _dock_wl_learn(_expected_side(target_yaw))
                    _vtouch_latch_ref(target_yaw)
                    _mode_hold_ms = 0
                    _approach_t0 = 0
                    return True
                return False
            _vdock_ok_cnt = 0
            _mode_hold_ms = 0
            # 沿推送方向：物体环。垂直方向：球；球不可用时 e_obj_open 非零，由物体环接管。
            v_fwd = _VDOCK_KP * e_fwd
            if v_fwd > _VDOCK_SPEED:
                v_fwd = _VDOCK_SPEED
            elif v_fwd < -_VDOCK_SPEED:
                v_fwd = -_VDOCK_SPEED
            v_obj_perp = _VDOCK_KP * e_obj_open
            if v_obj_perp > _VDOCK_SPEED:
                v_obj_perp = _VDOCK_SPEED
            elif v_obj_perp < -_VDOCK_SPEED:
                v_obj_perp = -_VDOCK_SPEED
            # 推送系->世界系。沿推送方向单位向量 (sin p, cos p)。
            # 垂直方向必须用 e_open/e_obj_open 的【投影方向】u = (-cos p, sin p)——
            # 它由 u_body = (-cd, sd) 转到世界系得到，两个槽位都指向同一侧（已数值验证）。
            # 用反了会变成正反馈，垂直误差发散。两项都是 +u 方向：物体环和球层的
            # 命令映射一致（base._push_step 的球层也是 +ch_open 沿 u_body）。
            pr = radians(_ball_push_yaw_for_edge(_target_edge))
            cp = cos(pr)
            sp = sin(pr)
            perp = v_obj_perp + v_perp
            base.request_world(sp * v_fwd - cp * perp, cp * v_fwd + sp * perp, target_yaw)
            return False
        # 看不到物体：不允许判定就位（球可见是紧 V 的唯一真值定义），只做"找球"。
        # 先用主车无线位姿解析出观察位（球落在 FOV 中部），到位后仍无球就沿
        # 观察位→紧 V 位这条线缓缓贴向主车（车头不动，动作以横移为主），
        # 到头再回落往复扫描，直到球进视野后由上面的球闭环接管。
        obj_id = _target_obj_id
        if base._Other_Car_X >= 900.0 or base._Other_Car_Y >= 900.0 or obj_id < 1 or obj_id > 5:
            base.request_hold(target_yaw)
            _vdock_ok_cnt = 0
            _mode_hold_ms = 0
            if _approach_t0 == 0:
                _approach_t0 = now_ms
            elif ticks_diff(now_ms, _approach_t0) > _VDOCK_TIMEOUT_MS:
                _approach_t0 = 0
                _follower_reenter_creep = True
                _next_task_mode = _MODE_SEARCH
            return False
        _approach_t0 = 0
        _vdock_ok_cnt = 0
        _mode_hold_ms = 0
        # 目标：主车中心该出现在从车车体系的 (gx, gy)。按扫描进度 u 在观察位(u=0)
        # 和紧 V 位(u=1)之间插值——u=0 时从车后退 _VDOCK_OBS_FWD_CM，主车中心的
        # rel_y 相应增大，球就从画面下边界回到中部。
        wx, wy = _dock_wl_target(_expected_side(target_yaw))
        u = _vdock_creep_u
        gx = wx
        gy = wy + _VDOCK_OBS_FWD_CM * (1.0 - u)
        # 由"主车中心该在从车车体系哪里"反推从车中心世界坐标：F = L − Rb(θF)·(gx,gy)
        fx = base._Other_Car_X - (cy * gx + sy * gy)
        fy = base._Other_Car_Y - (-sy * gx + cy * gy)
        dx = fx - base._car.Position_X
        dy = fy - base._car.Position_Y
        # 只有跟上了当前扫描点才推进进度，保证是"缓缓移动"而不是一步冲到底。
        if _vdock_creep_ts == 0:
            _vdock_creep_ts = now_ms
        dt = ticks_diff(now_ms, _vdock_creep_ts)
        _vdock_creep_ts = now_ms
        if dt < 0 or dt > 500:
            dt = 0
        if dx * dx + dy * dy < _VDOCK_CREEP_TRACK_EPS * _VDOCK_CREEP_TRACK_EPS:
            u += _vdock_creep_dir * _VDOCK_CREEP_RATE * dt * 0.001
            if u >= _VDOCK_CREEP_U_MAX:
                u = _VDOCK_CREEP_U_MAX
                _vdock_creep_dir = -1.0
            elif u <= 0.0:
                u = 0.0
                _vdock_creep_dir = 1.0
            _vdock_creep_u = u
        base.request_pos(fx, fy, _VDOCK_WL_SPEED if u <= 0.0 else _VDOCK_CREEP_SPEED, target_yaw)
        return False
    cam_rel = _find_target_in_cam()
    anchor_drive = (ap_sub == _AP_PRE_ORBIT or ap_sub == _AP_ORBIT) and _ap_anchor_valid
    if cam_rel < 0 and (not anchor_drive):
        if ap_sub == _AP_FACE or ap_sub == _AP_PRE_ORBIT:
            base.clear_face()
            if _approach_t0 == 0:
                _approach_t0 = now_ms
            if ticks_diff(now_ms, _approach_t0) > _FACE_PRE_ORBIT_LOST_SEARCH_MS:
                _follower_reenter_creep = True
                _next_task_mode = _MODE_SEARCH
            return False
        if _approach_t0 == 0:
            _approach_t0 = now_ms
            _ap_lost_lock_yaw = base._car.current_angle
        lost_ms = ticks_diff(now_ms, _approach_t0)
        if _target_obj_world_x < 900.0 and _target_obj_world_y < 900.0:
            if _ap_anchor_valid and ap_sub == _AP_ORBIT:
                ref_x, ref_y = (_ap_anchor_x, _ap_anchor_y)
            else:
                ref_x, ref_y = (_target_obj_world_x, _target_obj_world_y)
            recover_yaw = degrees(atan2(ref_x - base._car.Position_X, ref_y - base._car.Position_Y)) % 360.0
            base.clear_face()
            base.request_world(0.0, 0.0, recover_yaw)
            if lost_ms > _LOST_RECOVER_MAX_MS:
                base.request_hold(recover_yaw)
                _follower_reenter_creep = True
                _next_task_mode = _MODE_SEARCH
        elif lost_ms > _CAM_LOST_TOL_MS:
            base.clear_face()
            base.request_hold(_ap_lost_lock_yaw)
            _follower_reenter_creep = True
            _next_task_mode = _MODE_SEARCH
        return False
    _approach_t0 = 0
    if anchor_drive:
        rel_x, rel_y = _orbit_anchor_rel()
        d = sqrt(rel_x * rel_x + rel_y * rel_y)
        if d < 0.1:
            d = 0.1
        face_err = degrees(atan2(rel_x, rel_y))
    else:
        rel_x = base._cam_obj_rel_x[cam_rel]
        rel_y = base._cam_obj_rel_y[cam_rel]
        d = sqrt(rel_x * rel_x + rel_y * rel_y)
        if d < 0.1:
            d = 0.1
        c, s = _update_target_world_from_cam(cam_rel)
        face_err = degrees(atan2(rel_x, rel_y))
    if abs(face_err) < _FACE_DEADBAND_DEG:
        face_err = 0.0
    if ap_sub == _AP_FACE:
        base.clear_face()
        orb_r = _ARC_ORBIT_R
        vx_b = _FACE_LAT_KP * rel_x
        if vx_b > _FACE_LAT_MAX:
            vx_b = _FACE_LAT_MAX
        if vx_b < -_FACE_LAT_MAX:
            vx_b = -_FACE_LAT_MAX
        vy_b = _ORBIT_RADIAL_KP * (rel_y - orb_r)
        if vy_b > _PRE_ORBIT_RADIAL_MAX:
            vy_b = _PRE_ORBIT_RADIAL_MAX
        if vy_b < -_PRE_ORBIT_RADIAL_MAX:
            vy_b = -_PRE_ORBIT_RADIAL_MAX
        lock_yaw = _ap_lost_lock_yaw
        if abs(rel_x) < _FACE_ORBIT_X_EPS and abs(rel_y - orb_r) < _FACE_ORBIT_R_EPS:
            # FACE 进入 ORBIT 不再停车保持。进入 ORBIT 后由视觉朝向环和
            # 径向环继续修正剩余的横向、半径误差。
            # 进入 AP_ORBIT 只表示“已经面向物体并到达环绕起点”，不等于立即运动。
            # 普通规划可在下一周期解析；侧向同侧分支则用该状态通知主车按本车此刻
            # 的实际方位补算方向。真正运动前由 approach_update 做完整规划门控。
            _set_orbit_anchor(cam_rel)
            _mode_sub = _AP_ORBIT
            _orbit_plan_valid = False
            _mode_hold_ms = 0
        else:
            _mode_hold_ms = 0
            base.request_world(c * vx_b + s * vy_b, -s * vx_b + c * vy_b, lock_yaw)
        return False
    if ap_sub == _AP_PRE_ORBIT:
        if not _ap_anchor_valid and cam_rel >= 0:
            _set_orbit_anchor(cam_rel)
        rc = _push_route_phase == 1
        orb_r = _ROUTE_RECLOSE_ORBIT_R if rc else _ARC_ORBIT_R
        pre_rkp = _ORBIT_RADIAL_KP
        pre_rmax = _PRE_ORBIT_RADIAL_MAX
        pre_r_eps = _PRE_ORBIT_R_EPS
        pre_face_eps = _PRE_ORBIT_FACE_EPS
        base._face_req_err = face_err
        base._face_req_active = 1
        base._face_req_seq += 1
        obj_x = _ap_anchor_x if _ap_anchor_valid else _target_obj_world_x
        obj_y = _ap_anchor_y if _ap_anchor_valid else _target_obj_world_y
        dx = base._car.Position_X - obj_x
        dy = base._car.Position_Y - obj_y
        rw = sqrt(dx * dx + dy * dy)
        if rw < 1.0:
            rw = 1.0
        rx_u = dx / rw
        ry_u = dy / rw
        v_rad = pre_rkp * (rw - orb_r)
        if v_rad > pre_rmax:
            v_rad = pre_rmax
        if v_rad < -pre_rmax:
            v_rad = -pre_rmax
        base.request_world(-rx_u * v_rad, -ry_u * v_rad, base._car.current_angle)
        if abs(rw - orb_r) < pre_r_eps and abs(face_err) < pre_face_eps:
            # PRE_ORBIT 进入捕获范围后由 ORBIT 立即接管，不再停车保持。
            _mode_sub = _AP_ORBIT
            _orbit_plan_valid = False
            _mode_hold_ms = 0
        else:
            _mode_hold_ms = 0
        return False
    if ap_sub == _AP_ORBIT:
        rc = _push_route_phase == 1
        yaw_eps = _ROUTE_RECLOSE_YAW_EPS if rc else _APPROACH_YAW_EPS
        orb_spd = _ROUTE_RECLOSE_ORBIT_SPEED if rc else _ORBIT_SPEED_MAX
        orb_r = _ROUTE_RECLOSE_ORBIT_R if rc else _ARC_ORBIT_R
        orb_rkp = _ROUTE_RECLOSE_ORBIT_RADIAL_KP if rc else _ORBIT_RUN_RADIAL_KP
        orb_rmax = _ROUTE_RECLOSE_ORBIT_RADIAL_MAX if rc else _ORBIT_RADIAL_MAX
        yaw_err = _angle_diff(target_yaw, base._car.current_angle)
        if not _orbit_plan_valid or abs(_angle_diff(target_yaw, _orbit_target_yaw)) > 1.0 or spin_dir != _orbit_dir:
            _orbit_start_yaw = base._car.current_angle
            _orbit_target_yaw = target_yaw
            _orbit_dir = spin_dir if spin_dir is not None else 0
            _orbit_plan_valid = True
        slow_zone = _ORBIT_SLOW_ZONE_DEG
        if slow_zone > 0.0 and abs(yaw_err) < slow_zone:
            base._t = abs(yaw_err) / slow_zone
            orb_spd = _ORBIT_SPEED_MIN + (orb_spd - _ORBIT_SPEED_MIN) * base._t
        if _route_form_yaw_too_close():
            base.clear_face()
            base.request_hold(base._car.current_angle)
            _mode_hold_ms = 0
            return False
        if abs(yaw_err) < yaw_eps:
            base.clear_face()
            base.request_hold(target_yaw)
            # ORBIT 到达规划角后立即交给 VDOCK，不再停车保持 300 ms。
            _relocalize_after_orbit(cam_rel)
            base._fix_beep_active = 1
            base._fix_beep_until_ms = ticks_add(now_ms, 50)
            _mode_hold_ms = 0
            # 普通接近与绕障重靠近统一走 V 字贴靠。
            _mode_sub = _AP_VDOCK
            return False
        _mode_hold_ms = 0
        _update_orbit_anchor_from_cam(cam_rel)
        obj_x = _ap_anchor_x if _ap_anchor_valid else _target_obj_world_x
        obj_y = _ap_anchor_y if _ap_anchor_valid else _target_obj_world_y
        dx = base._car.Position_X - obj_x
        dy = base._car.Position_Y - obj_y
        rw = sqrt(dx * dx + dy * dy)
        if rw < 1.0:
            rw = 1.0
        rx_u = dx / rw
        ry_u = dy / rw
        if spin_dir is None:
            dir_sign = -1.0 if yaw_err >= 0.0 else 1.0
        else:
            total = _delta_dir(_orbit_start_yaw, target_yaw, spin_dir)
            progress = _delta_dir(_orbit_start_yaw, base._car.current_angle, spin_dir)
            if progress > total + _ORBIT_OVERSHOOT_MARGIN_DEG:
                dir_sign = -1.0 if yaw_err >= 0.0 else 1.0
            else:
                dir_sign = float(spin_dir)
        face_lead = _ORBIT_FACE_LEAD_GAIN * orb_spd / rw
        if face_lead > _ORBIT_FACE_LEAD_MAX_DEG:
            face_lead = _ORBIT_FACE_LEAD_MAX_DEG
        base._face_req_err = face_err - dir_sign * face_lead
        base._face_req_active = 1
        base._face_req_seq += 1
        tx_u = -dir_sign * ry_u
        ty_u = dir_sign * rx_u
        v_rad_fb = orb_rkp * (rw - orb_r)
        v_rad_ff = 0.0
        if not rc and _ORBIT_INWARD_FF_GAIN > 0.0:
            v_rad_ff = _ORBIT_INWARD_FF_GAIN * orb_spd * orb_spd / rw
        v_rad = v_rad_fb + v_rad_ff
        if v_rad > orb_rmax:
            v_rad = orb_rmax
        if v_rad < -orb_rmax:
            v_rad = -orb_rmax
        base.request_world(tx_u * orb_spd - rx_u * v_rad, ty_u * orb_spd - ry_u * v_rad, base._car.current_angle)
        return False
    return False

# 功能：计算某辆车相对当前目标的方位角。
# 输入参数：car_x/car_y 为车辆世界坐标。
# 返回值：从车辆指向目标的 yaw，单位度；目标或车辆坐标无效时返回 0。
def _orbit_heading_of_car(car_x, car_y):
    if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        return 0.0
    if car_x >= 900.0 or car_y >= 900.0:
        return 0.0
    return degrees(atan2(_target_obj_world_x - car_x, _target_obj_world_y - car_y)) % 360.0

# 功能：给当前目标边生成左右两侧可选的接近角。
# 输入参数：edge 为目标边编号。
# 返回值：二元组，分别为推送方向两侧的候选 yaw。
def _candidate_yaws(edge):
    push = _push_yaw(edge)
    deg = _APPROACH_LOCK_DEG[_target_obj_id] if 1 <= _target_obj_id < len(_APPROACH_LOCK_DEG) else 30.0
    return ((push + deg) % 360.0, (push - deg) % 360.0)

# 功能：选择从当前角到目标角更短的环绕方向。
# 输入参数：now_yaw 为当前角，target_yaw 为目标角，单位度。
# 返回值：1 或 -1，表示沿哪个方向到达目标角代价更小。
def _best_dir(now_yaw, target_yaw):
    pos_cost = _delta_dir(now_yaw, target_yaw, 1)
    neg_cost = _delta_dir(now_yaw, target_yaw, -1)
    if pos_cost <= neg_cost:
        return 1
    return -1


# 功能：为从车选择避开主车的环绕方向。
# 从车需要到达与主车相对的另一侧夹推角，本函数会评估正反两个方向，尽量避开主车当前角度和主车规划路径，减少环绕过程中的相遇风险。
# 输入参数：leader_yaw 为主车目标接近角；leader_dir 为主车环绕方向；follower_yaw 为从车目标接近角。
# 返回值：从车应采用的环绕方向，1 或 -1。
def _choose_follower_dir(leader_yaw, leader_dir, follower_yaw):
    leader_now = _orbit_heading_of_car(base._Other_Car_X, base._Other_Car_Y)
    leader_now_valid = base._Other_Car_X < 900.0 and base._Other_Car_Y < 900.0
    if not leader_now_valid:
        leader_now = leader_yaw
    follower_now = _orbit_heading_of_car(base._car.Position_X, base._car.Position_Y)
    best = None
    for follower_dir in (1, -1):
        cost = _delta_dir(follower_now, follower_yaw, follower_dir)
        score = cost
        if _point_on_arc(follower_now, follower_yaw, follower_dir, leader_yaw) or _point_on_arc(follower_now, follower_yaw, follower_dir, (leader_yaw + _ORBIT_LEADER_CLEAR_DEG) % 360.0) or _point_on_arc(follower_now, follower_yaw, follower_dir, (leader_yaw - _ORBIT_LEADER_CLEAR_DEG) % 360.0):
            score += 10000.0
        if leader_now_valid and (_point_on_arc(follower_now, follower_yaw, follower_dir, leader_now) or _point_on_arc(follower_now, follower_yaw, follower_dir, (leader_now + _ORBIT_LEADER_CLEAR_DEG) % 360.0) or _point_on_arc(follower_now, follower_yaw, follower_dir, (leader_now - _ORBIT_LEADER_CLEAR_DEG) % 360.0)):
            score += 6000.0
        if _point_on_arc(leader_now, leader_yaw, leader_dir, follower_now) or _point_on_arc(leader_now, leader_yaw, leader_dir, follower_yaw) or _point_on_arc(follower_now, follower_yaw, follower_dir, leader_now) or _point_on_arc(follower_now, follower_yaw, follower_dir, leader_yaw):
            score += 3000.0
        item = (score, cost, follower_dir)
        if best is None or item < best:
            best = item
    return best[2]

# 功能：根据主车发来的接近命令生成从车自己的接近规划。
# 从车解析主车目标 yaw 后，会选择另一侧候选 yaw 作为自己的站位，并计算一个尽量不穿过主车路径的环绕方向。
# 返回值：True 表示成功生成从车接近规划；False 表示主车接近命令无效。
def _make_follower_face_plan():
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw
    follower_yaw, follower_dir, ok = _decode_approach_cmd(base._Other_Follower_Cmd_Yaw_Dir)
    if ok:
        _approach_plan_valid = 1
        _approach_plan_edge = _target_edge
        _approach_plan_obj_id = _target_obj_id
        _approach_self_yaw = follower_yaw
        _approach_self_dir = follower_dir
        _approach_cmd_to_other = 999.9
        return True
    leader_yaw, leader_dir, ok = _decode_approach_cmd(base._Other_Approach_Cmd)
    if not ok:
        return False
    left_yaw, right_yaw = _candidate_yaws(_target_edge)
    if abs(_angle_diff(leader_yaw, left_yaw)) <= abs(_angle_diff(leader_yaw, right_yaw)):
        follower_yaw = right_yaw
    else:
        follower_yaw = left_yaw
    follower_dir = _choose_follower_dir(leader_yaw, leader_dir, follower_yaw)
    _approach_plan_valid = 1
    _approach_plan_edge = _target_edge
    _approach_plan_obj_id = _target_obj_id
    _approach_self_yaw = follower_yaw
    _approach_self_dir = follower_dir
    _approach_cmd_to_other = 999.9
    return True

# 功能：根据主车恢复/绕障命令生成强制推送方向对应的接近规划。
# 从车不主动决定强制方向，而是读取主车通过 _Other_Follower_Cmd_Yaw_Dir 下发的从车目标角，保证两车恢复到互补站位。
# 输入参数：push_yaw 为期望推送方向；spin_dir 为指定环绕方向；do_back 表示是否先后退；from_push_yaw 为上一段推送方向。
# 返回值：True 表示成功生成计划；False 表示命令或目标信息无效。
def start_forced_push_yaw_plan(push_yaw, spin_dir=0, do_back=True, from_push_yaw=999.9):
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw, _cmd_seq, _follower_cmd_yaw_dir, _master_cmd_sub
    global _reset_first_sub
    edge = _target_edge
    if edge <= 0 or push_yaw >= 900.0:
        return False
    deg = _APPROACH_LOCK_DEG[_target_obj_id] if 1 <= _target_obj_id < len(_APPROACH_LOCK_DEG) else 30.0
    left_yaw = (push_yaw + deg) % 360.0
    right_yaw = (push_yaw - deg) % 360.0
    now_heading = _orbit_heading_of_car(base._car.Position_X, base._car.Position_Y)
    if from_push_yaw < 900.0:
        if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
            return False
    elif spin_dir == 0:
        if base._Other_Master_Cmd_Sub == _CMD_SUB_ROUTE_RESTORE:
            target_yaw, spin_dir, ok = _decode_approach_cmd(base._Other_Follower_Cmd_Yaw_Dir)
            if not ok:
                return False
        else:
            return False
    else:
        target_yaw = right_yaw
        if spin_dir == 0:
            spin_dir = _best_dir(now_heading, target_yaw)
    _approach_plan_valid = 1
    _approach_plan_edge = edge
    _approach_plan_obj_id = _target_obj_id
    _approach_self_yaw = target_yaw
    _approach_self_dir = spin_dir
    _reset_first_sub = _AP_ROUTE_BACK if do_back else _AP_FACE
    return True

# 功能：根据主车绕障重靠近命令生成从车接近规划。
# 主车发现推送走廊有障碍时会发出路线重靠近命令，从车读取主车给自己的 yaw 和方向，配合主车从另一侧重新夹住目标。
# 输入参数：route_move_yaw 为目标侧向绕障移动方向。
# 返回值：True 表示成功接受主车重靠近计划；False 表示命令或目标坐标无效。
def start_reclose_plan(route_move_yaw):
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw, _cmd_seq, _follower_cmd_yaw_dir, _master_cmd_sub
    global _reset_first_sub
    edge = _target_edge
    if edge <= 0 or route_move_yaw >= 900.0:
        return False
    if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        return False
    pos_push = (route_move_yaw + 180.0) % 360.0
    pos_guide = route_move_yaw % 360.0
    now_heading = _orbit_heading_of_car(base._car.Position_X, base._car.Position_Y)
    if base._Other_Master_Cmd_Sub != _CMD_SUB_ROUTE_RECLOSE_1:
        return False
    target_yaw, spin_dir, ok = _decode_approach_cmd(base._Other_Follower_Cmd_Yaw_Dir)
    if not ok:
        return False
    _approach_cmd_to_other = 999.9
    _approach_plan_valid = 1
    _approach_plan_edge = edge
    _approach_plan_obj_id = _target_obj_id
    _approach_self_yaw = target_yaw
    _approach_self_dir = spin_dir
    _reset_first_sub = _AP_PRE_ORBIT
    return True

# 功能：判断绕障接近阶段从车是否与主车角度过近。
# 如果两车绕目标站位过近，从车会暂停或等待，避免双方在目标附近相互挤压。
# 返回值：True 表示需要避让或暂停；False 表示当前角度间隔安全。
def _route_form_yaw_too_close():
    if _push_route_phase not in (1, 3):
        return False
    if abs(_angle_diff(base._car.current_angle, base._Other_Car_Angle)) >= _READY_ROUTE_MIN_YAW_SEP_DEG:
        return False
    spin_dir = _approach_self_dir
    if spin_dir == 0:
        return True
    self_to_other = _delta_dir(base._car.current_angle, base._Other_Car_Angle, spin_dir)
    other_to_self = _delta_dir(base._Other_Car_Angle, base._car.current_angle, spin_dir)
    if abs(self_to_other - other_to_self) < 1.0:
        return True
    return self_to_other < other_to_self

# 功能：检测主车是否在 APPROACH 中切换了新的目标，并让从车同步切换。
# 主车可能在推送失败或重新搜索后改变目标；从车发现目标 ID/边变化后会清空旧规划并回到 FACE 子状态。
# 返回值：True 表示已经同步到新目标并重置接近状态；False 表示目标没有变化。
def _follower_sync_changed_leader_target():
    global _approach_cmd_to_other, _approach_plan_valid, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y, _target_sel_id_for_cam
    global _prelock_sub, _mode_sub, _mode_hold_ms, _approach_t0
    global _vdock_ok_cnt, _vdock_creep_u, _vdock_creep_dir, _vdock_creep_ts, _vdock_ball_lost_t0
    if base._Other_Target_X >= 900.0 or base._Other_Target_Y >= 900.0:
        return False
    if base._Other_Target_Edge <= 0 or base._Other_Target_ObjId <= 0:
        return False
    if _target_edge == base._Other_Target_Edge and _target_obj_id == base._Other_Target_ObjId:
        return False
    _target_edge = base._Other_Target_Edge
    _target_obj_id = base._Other_Target_ObjId
    _target_obj_world_x = base._Other_Target_X
    _target_obj_world_y = base._Other_Target_Y
    _target_sel_id_for_cam = _target_obj_id
    _approach_plan_valid = 0
    _approach_cmd_to_other = 999.9
    _prelock_sub = 0
    _mode_sub = _AP_FACE
    _mode_hold_ms = 0
    _vdock_ok_cnt = 0
    _vdock_creep_u = 0.0
    _vdock_creep_dir = 1.0
    _vdock_creep_ts = 0
    _vdock_ball_lost_t0 = 0
    _approach_t0 = 0
    base.clear_face()
    _clear_orbit_anchor()
    return True

# 功能：确保从车接近规划与主车最新接近命令一致。
# 如果主车命令变化，会废弃旧规划并重新生成；否则复用已有规划，避免每周期反复改变路线。
# 返回值：True 表示当前已有或已成功生成接近规划。
def _make_face_plan_once():
    global _approach_plan_valid
    if _approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id):
        return True
    return _make_follower_face_plan()

# 功能：更新从车 APPROACH 模式。
# 从车会同步主车目标，先预锁定目标，再根据主车接近命令生成自己的另一侧站位；接近完成后进入 WAIT_READY 等待主车开始推送。
# 输入参数：now_ms 为当前时间戳。
# 对方状态新鲜度阈值，单位 ms。approach_update 内也会使用，必须声明在该
# 函数之前，下划线 const 在函数编译后才声明会退化成不存在的全局变量导致 NameError。
_PUSH_PEER_FRESH_MS = const(700)
_APPROACH_PEER_SEARCH_CONFIRM_MS = const(120)
_APPROACH_PUSH_LOST_SPIN_DPS = 90.0

def approach_update(now_ms):
    global _approach_cmd_to_other, _approach_plan_valid, _follower_cmd_yaw_dir, _master_cmd_sub, _next_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _self_sub, _self_ready, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y, _target_sel_id_for_cam
    global _prelock_sub
    global _search_form_stable_t0, _fs_orbit_valid
    global _search_bearing_filt_valid, _search_face_yaw_cmd_valid
    global _approach_peer_search_t0, _ap_lost_lock_yaw
    global _mode_sub, _ap_anchor_valid, _ap_anchor_x, _ap_anchor_y, _orbit_plan_valid, _mode_hold_ms
    if _push_route_phase in (1, 3):
        base.avoid_beep(2)
    if _prev_task_mode == _MODE_PUSH_SYNC and base._Other_Car_Mode == _MODE_SEARCH:
        _self_ready = 0
        _target_sel_id_for_cam = 0
        _approach_plan_valid = 0
        _approach_cmd_to_other = 999.9
        _target_edge = 0
        _target_obj_id = 0
        _target_obj_world_x = 999.0
        _target_obj_world_y = 999.0
        _push_route_axis = 999.9
        _push_route_phase = 0
        _push_route_move_yaw = 999.9
        _push_route_restore_push = 0
        _master_cmd_sub = _CMD_SUB_NONE
        _follower_cmd_yaw_dir = 999.9
        base._push_world_side_cmd = 0.0
        base.request_hold(base._car.current_angle)
        _next_task_mode = _MODE_SEARCH
        return
    # 主车已经回到 SEARCH 时，从车不能继续停留在 APPROACH，否则主车会在
    # SEARCH 的队形等待中永久等不到从车的 SEARCH ready。要求主车 SEARCH
    # 帧连续新鲜 120 ms 后再回退，过滤单帧旧包或短暂通信抖动。
    peer_search_fresh = (
        base._Other_Car_Mode == _MODE_SEARCH
        and base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS
    )
    if peer_search_fresh:
        if _approach_peer_search_t0 == 0:
            _approach_peer_search_t0 = now_ms
        elif ticks_diff(now_ms, _approach_peer_search_t0) >= _APPROACH_PEER_SEARCH_CONFIRM_MS:
            _clear_push_target_lock()
            _approach_peer_search_t0 = 0
            base.clear_face()
            base.request_hold(base._car.current_angle)
            _next_task_mode = _MODE_SEARCH
            return
    else:
        _approach_peer_search_t0 = 0
    if _follower_sync_changed_leader_target():
        return
    if _prev_task_mode != _MODE_SEARCH:
        _self_ready = 0
        _self_sub = _mode_sub
        if _find_target_in_cam() < 0:
            base.clear_face()
            if _prev_task_mode == _MODE_PUSH_SYNC:
                spin_dir = _approach_self_dir
                if spin_dir != 1 and spin_dir != -1:
                    spin_dir = 1 if _angle_diff(_ap_lost_lock_yaw, _push_yaw(_target_edge)) >= 0.0 else -1
                base.request_yaw_rate(spin_dir * _APPROACH_PUSH_LOST_SPIN_DPS)
            return
        peer_approach_ok = base._Other_Car_Mode == _MODE_APPROACH or base._Other_Car_Mode == _MODE_WAIT_READY
        if not peer_approach_ok or base._Other_Car_Ready_Ts == 0 or ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _PUSH_PEER_FRESH_MS or base._Other_Target_Edge != _target_edge or base._Other_Target_ObjId != _target_obj_id:
            base.clear_face()
            base.request_hold(_ap_lost_lock_yaw)
            return
        if _mode_sub == _AP_ORBIT and not (_approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id)):
            if not _make_face_plan_once():
                base.clear_face()
                base.request_hold(_ap_lost_lock_yaw)
                return
    if _prev_task_mode == _MODE_SEARCH and base._Other_Car_Mode == _MODE_APPROACH and base._Other_Car_Push_Sub < _AP_ORBIT:
        _self_ready = 0
        _self_sub = _AP_FACE
        car_idx = _find_car_in_cam()
        peer_fresh = (
            base._Other_Car_Ready_Ts != 0
            and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _FOLLOWER_SEARCH_FF_FRESH_MS
            and base._Other_Car_X < 900.0
            and base._Other_Car_Y < 900.0
        )
        if car_idx < 0 or not peer_fresh:
            _search_form_stable_t0 = 0
            _fs_orbit_valid = False
            _search_bearing_filt_valid = False
            _search_face_yaw_cmd_valid = False
            base.request_hold(base._car.current_angle)
            return
        rel_x = base._cam_car_rel_x
        rel_y = base._cam_car_rel_y
        _search_form_stable_t0 = 0
        _fs_orbit_valid = False
        radial_vx, radial_vy, unused = _search_distance_adjust(
            rel_x, rel_y, _FOLLOWER_SEARCH_LONG_KP, _FOLLOWER_SEARCH_LONG_MAX
        )
        leader_rad = radians(base._Other_Car_Angle)
        right_x = cos(leader_rad)
        right_y = -sin(leader_rad)
        center_err = rel_x
        if abs(center_err) <= _FOLLOWER_SEARCH_CENTER_DB:
            center_err = 0.0
        side_speed = _FOLLOWER_SEARCH_CENTER_KP * center_err
        if side_speed > _FOLLOWER_SEARCH_CENTER_MAX:
            side_speed = _FOLLOWER_SEARCH_CENTER_MAX
        elif side_speed < -_FOLLOWER_SEARCH_CENTER_MAX:
            side_speed = -_FOLLOWER_SEARCH_CENTER_MAX
        ff_x, ff_y = _search_leader_feedforward()
        cmd_x = ff_x + radial_vx + right_x * side_speed
        cmd_y = ff_y + radial_vy + right_y * side_speed
        cmd_mag = sqrt(cmd_x * cmd_x + cmd_y * cmd_y)
        if cmd_mag > _FOLLOWER_SEARCH_TOTAL_MAX:
            cmd_scale = _FOLLOWER_SEARCH_TOTAL_MAX / cmd_mag
            cmd_x *= cmd_scale
            cmd_y *= cmd_scale
        # 只有同向（跟主车航向差 <20°）阵型才需要车头继续贴主车实时航向；
        # 侧向阵型车头本来就跟主车差着约90°，这里如果也跟着主车转，主车
        # 到达 _AP_ORBIT 后下面那段"转向锁定物体"又会把车头转回物体方向，
        # 变成先转一次、再转回来的多余动作。侧向阵型这里保持当前车头即可，
        # 等主车到 _AP_ORBIT 后直接由下面的转向锁定物体逻辑一次转到位。
        if abs(_angle_diff(base._Other_Car_Angle, base._car.current_angle)) < 20.0:
            face_yaw = base._Other_Car_Angle
        else:
            face_yaw = base._car.current_angle
        base.request_world(cmd_x, cmd_y, face_yaw)
        return
    if _prev_task_mode == _MODE_SEARCH and _mode_sub == _AP_FACE:
        if base._Other_Car_Mode != _MODE_APPROACH and base._Other_Car_Mode != _MODE_WAIT_READY:
            base.request_hold(base._car.current_angle)
            return
        if base._Other_Car_Mode == _MODE_APPROACH and base._Other_Car_Push_Sub < _AP_ORBIT:
            base.request_hold(base._car.current_angle)
            return
        # FACE 只负责面向物体并径向靠近环绕起点，不需要提前知道最终槽位和
        # 绕行方向。侧向同侧分支中，主车会故意暂缓下发从车规划；这里必须
        # 允许从车先到达 AP_ORBIT，再由下方 AP_ORBIT 硬门停车等待规划，避免
        # “主车等从车上报 AP_ORBIT、从车等主车规划才肯离开 FACE”的循环等待。
        if abs(_angle_diff(base._Other_Car_Angle, base._car.current_angle)) < 20.0:
            base.request_hold(base._car.current_angle)
            return
        # 侧向 SEARCH 阵型进入 APPROACH 时，先用主车共享的物体世界坐标完成一次定向。
        # 定向完成后把该角度锁存为普通 FACE 的起始航向，避免 FACE 重新使用进入
        # APPROACH 时保存的侧向 search_yaw，把已经面向物体的从车又转回去。
        # _prelock_sub 复用为一次性完成标记，目标改变时已有同步逻辑会把它清零。
        if _prelock_sub != _APPROACH_READY_SUB:
            if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
                base.request_hold(base._car.current_angle)
                return
            object_yaw = _orbit_heading_of_car(base._car.Position_X, base._car.Position_Y)
            if abs(_angle_diff(object_yaw, base._car.current_angle)) > _APPROACH_FACE_YAW_EPS:
                base.request_hold(object_yaw)
                return
            _ap_lost_lock_yaw = object_yaw
            _prelock_sub = _APPROACH_READY_SUB
        if _find_target_in_cam() < 0:
            # 摄像头持续看不到目标（比如主车转向挡住了从车视线）时，用主车共享的
            # 物体世界坐标兜底：航向已经锁定朝向目标，只需比较本车到目标的直线
            # 距离是否收敛到绕行半径附近，等效于视觉版 FACE->ORBIT 的入圆判据。
            # 满足就直接切 ORBIT，锚点先用世界坐标占位，摄像头重新看到后
            # _update_orbit_anchor_from_cam 会自动把锚点修正回视觉坐标。
            ready_by_world = False
            if _target_obj_world_x < 900.0 and _target_obj_world_y < 900.0:
                dx = _target_obj_world_x - base._car.Position_X
                dy = _target_obj_world_y - base._car.Position_Y
                world_dist = sqrt(dx * dx + dy * dy)
                ready_by_world = abs(world_dist - _ARC_ORBIT_R) < _FACE_ORBIT_R_EPS
            if not ready_by_world:
                _set_body(0.0, _APPROACH_WAIT_TARGET_FWD_SPEED, _ap_lost_lock_yaw)
                return
            _ap_anchor_valid = True
            _ap_anchor_x = _target_obj_world_x
            _ap_anchor_y = _target_obj_world_y
            _mode_sub = _AP_ORBIT
            _orbit_plan_valid = False
            _mode_hold_ms = 0
    edge = _target_edge
    approach_plan_ready = _approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id)
    if _mode_sub == _AP_ORBIT and not approach_plan_ready:
        # AP_ORBIT 起点硬门：主车帧必须新鲜、目标必须一致，而且“槽位+方向”必须
        # 同时解码成功。从车先上报 AP_ORBIT，再保持当前姿态等待主车补齐规划。
        peer_plan_source_ok = (
            (base._Other_Car_Mode == _MODE_APPROACH or base._Other_Car_Mode == _MODE_WAIT_READY)
            and base._Other_Car_Ready_Ts != 0
            and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS
            and base._Other_Target_Edge == _target_edge
            and base._Other_Target_ObjId == _target_obj_id
        )
        if peer_plan_source_ok:
            approach_plan_ready = _make_face_plan_once()
        if not approach_plan_ready:
            # 明确保持 AP_ORBIT 上报，供主车完成延迟的从车方向规划。
            _self_sub = _AP_ORBIT
            base.clear_face()
            base.request_hold(base._car.current_angle)
            return
    if (_mode_sub == _AP_ORBIT
            and (not 0.0 <= _approach_self_yaw < 360.0
                 or (_approach_self_dir != 1 and _approach_self_dir != -1))):
        base.clear_face()
        base.request_hold(base._car.current_angle)
        return
    if approach_plan_ready:
        target_yaw = _approach_self_yaw
        spin_dir = _approach_self_dir
    else:
        target_yaw = _lock_yaw(edge)
        spin_dir = None
    if _mode_sub == _AP_ORBIT and abs(_angle_diff(base._car.current_angle, base._Other_Car_Angle)) < _ORBIT_YAW_SEP_DEG:
        base.request_hold(base._car.current_angle)
        return
    if _prelock_sub != _APPROACH_READY_SUB:
        _prelock_sub = _APPROACH_READY_SUB
    if _run_approach_phase(now_ms, target_yaw, spin_dir):
        _self_ready = 0
        _next_task_mode = _MODE_WAIT_READY
    _self_sub = _mode_sub

_PUSH_LOST_SPIN_MS = const(3200)

_PUSH_LOST_SPIN_EPS = const(8)

_PUSH_PEER_LOST_CONFIRM_MS = const(2000)

_PUSH_RECOVER_FOUND_READY = const(5)


_PUSH_GO_TIMEOUT_MS = const(1500)

# 与主车保持一致；物体环横向修正把物体往本车中心拉，权限不宜过大。
_PUSH_SIDE_KP_FOLLOWER = const(5)


_PUSH_SIDE_KD_FOLLOWER = const(10)


_PUSH_FWD_KP_FOLLOWER = const(0)


# 与主车使用相同的前向 D 项。
_PUSH_FWD_KD_FOLLOWER = const(0)


_PUSH_SIDE_MAX_FOLLOWER = const(50)


# 与主车 _PUSH_FWD_MAX_LEADER 对齐；两车基础推送速度均为120。
_PUSH_FWD_MAX_FOLLOWER = const(30)


_PUSH_SIDE_SLEW_FOLLOWER = const(15)


_PUSH_FWD_SLEW_FOLLOWER = const(15)


_PUSH_D_LPF_FOLLOWER = 0.35

_PUSH_SIDE_DEADBAND = const(0)

_PUSH_FWD_DEADBAND = const(0)

_PUSH_SIDE_MIN = const(0)

_PUSH_FWD_MIN = const(0)


_PUSH_VISUAL_LOOP_FOLLOWER_ENABLE = const(1)

_PUSH_WORLD_SIDE_LOCK_ENABLE = const(1)


# 普通 PUSH 的 y 目标使用物体紧贴推杆标定值；x 目标按物体和槽位标定：
# 网球/两熊在 push_yaw±30° 槽位使用±2cm；两个沙包都使用0cm。
_PUSH_REF_SLOT_X_CM_BY_OBJ = (0.0, 2.0, 0.0, 0.0, 2.0, 2.0)
_PUSH_REF_REL_FOLLOWER_BY_OBJ = (None, (0.0, 13.0), (0.0, 9.0), (0.0, 9.0), (0.0, 9.0), (0.0, 9.0))

_push_lock_yaw = 0.0

_push_ref_valid = False

_push_ref_rel_x = 0.0

_push_ref_rel_y = 0.0

_push_done = False

_push_started = False

_mode_sub = 0

_push_sub_t0 = 0

_push_spin_start_yaw = 0.0

_push_peer_lost_t0 = 0

_push_recover_found = False

_push_recover_hold_yaw = 0.0

_push_route_axis = 999.9

_push_route_yaw = 999.9

# 横推起步的低速 SETTLE 段起点时间戳；0 表示尚未开始或已经切到巡航速度。
_push_route_settle_t0 = 0

# 本车在当前横推中的角色：True=引导/后退车，False=推进车。由 push_start
# 在进入 _PUSH_ROUTE 时按车头与 route_yaw 的夹角判定，push_update 用它
# 决定 SETTLE 结束后切换到哪一档巡航配置。
_push_route_is_guide = False

# SETTLE 段是否已经切到巡航速度，避免每帧重复调用 configure_push。
_push_route_cruise_applied = False

_push_start_t0 = 0

_push_world_side_axis = 0

_push_world_side_ref = 999.9

_push_world_side_valid = False

_push_diag_active = False

_PUSH_SPIN = const(2)

_PUSH_ROUTE = const(3)

_PUSH_ROUTE_EXTRA_DIST = const(15)

# 从车本地兜底：主车下发的 _CMD_SUB_ROUTE_RESTORE 才是正常退出信号；
# 本超时只在无线丢包导致从车收不到退出信号时兜底，避免顶死不退。
_PUSH_ROUTE_TIMEOUT_MS = const(4000)

# 横推（对推）阶段两车共用的基座标称速度。两车必须相等：速度差会在纯 P
# 距离环下留下 Δ/kp 的稳态间距偏差，把修正权限提前吃满；相等时视觉丢失
# 退化为等速平移，间距冻结不变，退化行为是安全的。
_PUSH_ROUTE_SPEED = const(120)

# 推进车（车头与推送方向夹角<=90°）的距离参考值，单位 cm。推进车主要靠
# 基座速度顶物体前进，距离环只用于脱开后重新接触，权限很小。
_PUSH_ROUTE_PUSH_REF_Y = 12.0

# 引导/后退车（车头与推送方向相反）的距离参考值，单位 cm。比接近阶段
# 贴近距(14)略小，给 2cm 预压；预压由参考距离提供，不再依赖速度差。
_PUSH_ROUTE_CONTACT_DIST = 12.0

_PUSH_ROUTE_PUSH_FWD_MAX = const(12)

_PUSH_ROUTE_GUIDE_FWD_MAX = const(30)

# 横推起步先用更低速度跑一小段，让距离环先收敛、双方接触先建立，
# 再切到 _PUSH_ROUTE_SPEED 正式输运；避免起步瞬间纯开环冲撞。
_PUSH_ROUTE_SETTLE_SPEED = const(25)

_PUSH_ROUTE_SETTLE_MS = const(250)

# 功能：查询当前目标的推送内偏角。
# 返回值：目标配置表中的内偏角，缺省为 0 度。
def _push_inward_bias():
    table = _PUSH_INWARD_BIAS_DEG_BY_OBJ
    obj_id = _target_obj_id
    if table is not None and 0 <= obj_id < len(table):
        return table[obj_id]
    return 0.0

# 功能：查询当前目标的从车推送视觉闭环参数。
# 返回值：四元组 side_kp、side_kd、fwd_kp、fwd_kd；目标没有专用参数时返回默认参数。
def _push_pid_params():
    obj_id = _target_obj_id
    table = _PUSH_PID_FOLLOWER_BY_OBJ
    if table is not None and 0 < obj_id < len(table) and (table[obj_id] is not None):
        return table[obj_id]
    return (_PUSH_SIDE_KP_FOLLOWER, _PUSH_SIDE_KD_FOLLOWER, _PUSH_FWD_KP_FOLLOWER, _PUSH_FWD_KD_FOLLOWER)

# 功能：查询当前目标的从车横向修正限幅。
def _push_side_max():
    obj_id = _target_obj_id
    table = _PUSH_SIDE_MAX_FOLLOWER_BY_OBJ
    if table is not None and 0 < obj_id < len(table):
        return table[obj_id]
    return _PUSH_SIDE_MAX_FOLLOWER

# 功能：查询当前目标的固定视觉相对参考点。
# 固定参考点用于推送时让目标保持在车体坐标的期望位置，而不是每次以刚看到的位置为参考。
# 返回值：三元组，依次为是否有效、参考 rel_x、参考 rel_y。
def _fixed_push_ref():
    obj_id = _target_obj_id
    table = _PUSH_REF_REL_FOLLOWER_BY_OBJ
    if table is None or obj_id <= 0 or obj_id >= len(table):
        return (False, 0.0, 0.0)
    ref = table[obj_id]
    if ref is None:
        return (False, 0.0, 0.0)
    push_yaw = _push_yaw_for_edge(_target_edge)
    slot_side = _angle_diff(_push_lock_yaw, push_yaw)
    ref_x = ref[0]
    slot_x = _PUSH_REF_SLOT_X_CM_BY_OBJ[obj_id]
    if slot_side > 1.0:
        ref_x = slot_x
    elif slot_side < -1.0:
        ref_x = -slot_x
    return (True, ref_x, ref[1])

# 功能：把目标边转换为基础推送方向。
# 输入参数：edge 为目标边编号。
# 返回值：推送 yaw，单位度；未知边返回 0。
def _push_yaw_for_edge(edge):
    if edge == 1:
        return 0.0
    if edge == 2:
        return 270.0
    if edge == 3:
        return 90.0
    if edge == 4:
        return 180.0
    return 0.0

# 功能：取出某个点在推送完成判定轴上的坐标。
# 输入参数：x/y 为点坐标，edge 为目标边。
# 返回值：该点在当前边对应轴上的值。
def _push_axis_value(x, y, edge):
    return x if edge == 1 or edge == 4 else y

# 功能：在推送阶段观测目标前方走廊中的其他物体。
# 从车会把自己看到的障碍轴坐标共享给主车，用于诊断推送或绕障判断。
def _update_push_obs():
    global _route_cmd_to_other, _route_obs_axis_to_other
    if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        _route_obs_axis_to_other = 999.9
        return
    if base._cam_obj_count <= 1:
        _route_obs_axis_to_other = 999.9
        _route_cmd_to_other = 999.9
        return
    obj_type = base._cam_obj_id[1]
    if obj_type == _target_obj_id or not 1 <= obj_type < len(_obj_remain) or _obj_remain[obj_type] <= 0:
        _route_obs_axis_to_other = 999.9
        _route_cmd_to_other = 999.9
        return
    ox = base._cam_obj_x[1]
    oy = base._cam_obj_y[1]
    if ox >= 900.0 or oy >= 900.0:
        _route_obs_axis_to_other = 999.9
        _route_cmd_to_other = 999.9
        return
    edge = _target_edge
    target_axis = _push_axis_value(_target_obj_world_x, _target_obj_world_y, edge)
    push_rad = radians(_push_yaw_for_edge(edge))
    px = sin(push_rad)
    py = cos(push_rad)
    sx = -py
    sy = px
    dx = ox - _target_obj_world_x
    dy = oy - _target_obj_world_y
    proj_fwd = dx * px + dy * py
    if proj_fwd <= 0.0 or proj_fwd < _READY_ROUTE_FWD_MIN_DIST or proj_fwd > _READY_ROUTE_FWD_MAX_DIST or abs(dx * sx + dy * sy) > _READY_ROUTE_SAFE_HALF:
        _route_obs_axis_to_other = 999.9
    else:
        _route_obs_axis_to_other = _push_axis_value(ox, oy, edge)
    _route_cmd_to_other = 999.9

# 功能：获取普通推送时的从车锁定角。
# 输入参数：edge 为目标边。
# 返回值：锁定 yaw，单位度。
def _normal_lock_yaw(edge):
    if _approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id):
        return _approach_self_yaw
    return _lock_yaw(edge)

# 功能：清空当前推送目标和与主车共享的推送/绕障协同状态。
# 该函数用于推送丢失后回到搜索，确保旧目标 ID、目标坐标、路线命令和主车命令不会污染下一轮任务。
def _clear_push_target_lock():
    global _approach_cmd_to_other, _approach_plan_valid, _follower_cmd_yaw_dir, _master_cmd_sub, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _self_ready, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y, _target_sel_id_for_cam
    _self_ready = 0
    _target_sel_id_for_cam = 0
    _approach_plan_valid = 0
    _approach_cmd_to_other = 999.9
    _target_edge = 0
    _target_obj_id = 0
    _target_obj_world_x = 999.0
    _target_obj_world_y = 999.0
    _push_route_axis = 999.9
    _push_route_phase = 0
    _push_route_move_yaw = 999.9
    _push_route_restore_push = 0
    _master_cmd_sub = _CMD_SUB_NONE
    _follower_cmd_yaw_dir = 999.9
    base._push_world_side_cmd = 0.0


# 功能：把当前目标记为已完成，并更新剩余目标数量。
def _mark_target_done():
    global _obj_total_remaining
    obj_id = _target_obj_id
    if not 1 <= obj_id < len(_obj_done):
        return
    if _obj_count_mode == 1:
        if _obj_total_remaining <= 0:
            return
        _obj_done[obj_id] += 1
        _obj_total_remaining -= 1
        enabled = 1 if _obj_total_remaining > 0 else 0
        for i in range(1, len(_obj_remain)):
            _obj_remain[i] = enabled
        return
    if _obj_count_mode == 3:
        # 五类分别计数：只扣本次真正推出的类别，其他类别不能替代。
        if _obj_remain[obj_id] <= 0:
            return
        _obj_done[obj_id] += 1
        _obj_remain[obj_id] -= 1
        _obj_total_remaining = max(0, _obj_total_remaining - 1)
        return
    groups = getattr(config, "_OBJ_GROUPS", ())
    for group_idx in range(len(groups)):
        group = groups[group_idx]
        if obj_id not in group:
            continue
        if group_idx >= len(_obj_group_remain) or _obj_group_remain[group_idx] <= 0:
            return
        _obj_done[obj_id] += 1
        _obj_group_remain[group_idx] -= 1
        _obj_total_remaining = max(0, _obj_total_remaining - 1)
        enabled = 1 if _obj_group_remain[group_idx] > 0 else 0
        for group_obj_id in group:
            if 1 <= group_obj_id < len(_obj_remain):
                _obj_remain[group_obj_id] = enabled
        return


# 功能：判断所有需要搬运的物体是否已经完成。
def _all_objects_done():
    return _obj_total_remaining <= 0

# 功能：重置推送阶段状态机并配置 base.py 中的推送控制器。
# 从车会额外重置球体辅助状态，并按当前目标加载从车推送 PID 参数。
def push_reset():
    global _push_lock_yaw, _push_ref_valid, _push_ref_rel_x, _push_ref_rel_y
    global _push_done, _push_started, _mode_sub
    global _push_sub_t0
    global _push_spin_start_yaw, _push_peer_lost_t0
    global _push_recover_found, _push_recover_hold_yaw
    global _push_route_axis, _push_route_yaw, _push_start_t0
    global _push_world_side_axis, _push_world_side_ref
    global _push_world_side_valid
    global _push_diag_active
    _push_lock_yaw = 0.0
    _push_ref_valid = False
    _push_ref_rel_x = 0.0
    _push_ref_rel_y = 0.0
    _push_done = False
    _push_started = False
    _mode_sub = _PUSH_RUN
    _push_sub_t0 = 0
    _push_spin_start_yaw = 0.0
    _push_peer_lost_t0 = 0
    _push_recover_found = False
    _push_recover_hold_yaw = 0.0
    if _push_route_phase != 2:
        _push_route_axis = 999.9
    _push_route_yaw = 999.9
    _push_start_t0 = 0
    _push_world_side_axis = 0
    _push_world_side_ref = 999.9
    _push_world_side_valid = False
    _push_diag_active = False
    base._push_world_side_cmd = 0.0
    base.reset_push()
    pid = _push_pid_params()
    base.configure_push(speed=_PUSH_SPEED_FOLLOWER, inward_bias=_push_inward_bias(), cam_lost_tol_ms=_CAM_LOST_TOL_MS, side_kp=pid[0], side_kd=pid[1], fwd_kp=pid[2], fwd_kd=pid[3], side_max=_push_side_max(), fwd_max=_PUSH_FWD_MAX_FOLLOWER, side_slew=_PUSH_SIDE_SLEW_FOLLOWER, fwd_slew=_PUSH_FWD_SLEW_FOLLOWER, d_lpf=_PUSH_D_LPF_FOLLOWER, side_deadband=_PUSH_SIDE_DEADBAND, fwd_deadband=_PUSH_FWD_DEADBAND, side_min=_PUSH_SIDE_MIN, fwd_min=_PUSH_FWD_MIN)

# 功能：开始一次从车推送目标动作。
# 根据目标边设置锁定角、世界横向保持轴和视觉参考点；如果主车规划了绕障路线，则先进入 ROUTE 子状态配合侧向带离。
# 输入参数：edge 为目标边；cam_obj 为目标摄像头槽位，-1 表示当前没有视觉目标。
def push_start(edge, cam_obj):
    global _push_route_phase
    global _push_lock_yaw, _push_ref_valid, _push_ref_rel_x, _push_ref_rel_y
    global _push_started, _mode_sub, _push_route_axis, _push_route_yaw, _push_sub_t0
    global _push_world_side_axis, _push_world_side_ref
    global _push_world_side_valid
    global _push_route_settle_t0, _push_route_is_guide, _push_route_cruise_applied
    push_reset()
    _push_lock_yaw = _normal_lock_yaw(edge)
    if edge == 1 or edge == 4:
        _push_world_side_axis = 1
        _push_world_side_ref = base._car.Position_X
        _push_world_side_valid = True
    elif edge == 2 or edge == 3:
        _push_world_side_axis = 2
        _push_world_side_ref = base._car.Position_Y
        _push_world_side_valid = True
    ref_ok, ref_x, ref_y = _fixed_push_ref()
    if ref_ok:
        _push_ref_rel_x = ref_x
        _push_ref_rel_y = ref_y
        _push_ref_valid = True
    elif cam_obj >= 0:
        _push_ref_rel_x = base._cam_obj_rel_x[cam_obj]
        _push_ref_rel_y = base._cam_obj_rel_y[cam_obj]
        _push_ref_valid = True
    route_axis = _push_route_axis
    route_yaw = _push_route_move_yaw
    if route_yaw >= 900.0:
        route_obj_axis = _push_axis_value(_target_obj_world_x, _target_obj_world_y, edge)
        if route_obj_axis >= 900.0 or route_axis >= 900.0:
            route_yaw = 999.9
        elif edge == 1 or edge == 4:
            route_yaw = 90.0 if route_axis >= route_obj_axis else 270.0
        else:
            route_yaw = 0.0 if route_axis >= route_obj_axis else 180.0
    route_phase = _push_route_phase
    if route_phase == 0 and route_axis < 900.0 and (route_yaw < 900.0) and (_push_route_restore_push == 0):
        route_phase = 2
        _push_route_phase = 2
    if route_phase == 2 and route_axis < 900.0 and (route_yaw < 900.0):
        _mode_sub = _PUSH_ROUTE
        _push_route_axis = route_axis + (1.0 if route_yaw == 90.0 or route_yaw == 0.0 else -1.0) * _PUSH_ROUTE_EXTRA_DIST
        _push_route_yaw = route_yaw
        _push_sub_t0 = 0
        _push_route_settle_t0 = 0
        _push_route_cruise_applied = False
        approach_yaw = _approach_self_yaw
        if approach_yaw < 900.0:
            _push_lock_yaw = approach_yaw
        else:
            deg = _APPROACH_LOCK_DEG[_target_obj_id] if 1 <= _target_obj_id < len(_APPROACH_LOCK_DEG) else 30.0
            if _SLOT_FIXED_TENNIS and _target_obj_id == _OBJ_ID_TENNIS:
                _push_lock_yaw = (route_yaw + deg) % 360.0
            else:
                _push_lock_yaw = (route_yaw - deg) % 360.0
        _push_world_side_valid = False
        base.reset_push()
        # 引导/后退车（车头与 route_yaw 大致相反，退让给推进车让路）：距离参考
        # 用固定预压值，不再取起推瞬间的随机第一帧；推进车（车头与 route_yaw
        # 同向）同样给固定参考，距离环权限很小，只负责脱开后重新接触。
        # 两车基座速度相同（_PUSH_ROUTE_SPEED），起步先走 SETTLE 低速让距离环
        # 收敛、双方接触先建立，_push_route_settle_t0 记录起点供 push_update 判断。
        is_guide = abs(_angle_diff(_push_lock_yaw, (_push_route_yaw + 180.0) % 360.0)) <= 90.0
        _push_route_is_guide = is_guide
        pid = _push_pid_params()
        if is_guide:
            _push_ref_rel_x = 0.0
            _push_ref_rel_y = _PUSH_ROUTE_CONTACT_DIST
            fwd_max = _PUSH_ROUTE_GUIDE_FWD_MAX
        else:
            _push_ref_rel_x = 0.0
            _push_ref_rel_y = _PUSH_ROUTE_PUSH_REF_Y
            fwd_max = _PUSH_ROUTE_PUSH_FWD_MAX
        _push_ref_valid = True
        base.configure_push(speed=_PUSH_ROUTE_SETTLE_SPEED, inward_bias=_push_inward_bias(), cam_lost_tol_ms=_CAM_LOST_TOL_MS, side_kp=pid[0], side_kd=pid[1], fwd_kp=pid[2], fwd_kd=pid[3], side_max=_push_side_max(), fwd_max=fwd_max, side_slew=_PUSH_SIDE_SLEW_FOLLOWER, fwd_slew=_PUSH_FWD_SLEW_FOLLOWER, d_lpf=_PUSH_D_LPF_FOLLOWER, side_deadband=_PUSH_SIDE_DEADBAND, fwd_deadband=_PUSH_FWD_DEADBAND, side_min=_PUSH_SIDE_MIN, fwd_min=_PUSH_FWD_MIN)
    _push_started = True

# 功能：进入目标视觉丢失后的原地旋转搜索子状态。
# 输入参数：now_ms 为当前时间戳。
def _enter_lost_spin(now_ms):
    global _self_ready
    global _mode_sub, _push_sub_t0, _push_spin_start_yaw
    global _push_recover_found, _push_recover_hold_yaw
    _self_ready = 0
    _push_recover_found = False
    _push_recover_hold_yaw = 0.0
    _mode_sub = _PUSH_SPIN
    _push_sub_t0 = now_ms
    _push_spin_start_yaw = base._car.current_angle

# 功能：完成绕障路线推送后，恢复到普通推送方向的接近流程。
# 从车会清掉路线轴和路线 yaw，然后请求 APPROACH，等待主车重新组织普通夹推角。
# 输入参数：cam_obj 为当前目标摄像头槽位，-1 表示没有视觉目标。
def _finish_route_push(cam_obj):
    global _approach_do_back, _approach_from_push_yaw, _approach_push_yaw, _approach_req, _approach_spin_dir
    global _follower_cmd_yaw_dir, _master_cmd_sub, _next_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _route_cmd_to_other, _self_ready
    global _mode_sub, _push_lock_yaw, _push_ref_valid, _push_ref_rel_x, _push_ref_rel_y
    global _push_route_axis, _push_route_yaw, _push_sub_t0
    edge = _target_edge
    _push_lock_yaw = _normal_lock_yaw(edge)
    ref_ok, ref_x, ref_y = _fixed_push_ref()
    if ref_ok:
        _push_ref_rel_x = ref_x
        _push_ref_rel_y = ref_y
        _push_ref_valid = True
    elif cam_obj >= 0:
        _push_ref_rel_x = base._cam_obj_rel_x[cam_obj]
        _push_ref_rel_y = base._cam_obj_rel_y[cam_obj]
        _push_ref_valid = True
    else:
        _push_ref_valid = False
    _push_route_axis = 999.9
    _push_route_yaw = 999.9
    _push_sub_t0 = 0
    _push_route_axis = 999.9
    _push_route_move_yaw = 999.9
    _route_cmd_to_other = 999.9
    _master_cmd_sub = _CMD_SUB_NONE
    _follower_cmd_yaw_dir = 999.9
    _push_route_phase = 3
    _self_ready = 0
    base.reset_push()
    _mode_sub = _PUSH_RUN
    normal_yaw = _push_yaw_for_edge(edge)
    _approach_req = _APPROACH_REQ_FORCED_PUSH_YAW
    _approach_push_yaw = normal_yaw
    _approach_spin_dir = 0
    _approach_do_back = 1 if True else 0
    _approach_from_push_yaw = 999.9
    _next_task_mode = _MODE_APPROACH

# 功能：把当前从车推送控制请求转交给 base.request_push。
# 该函数负责决定是否使用普通目标视觉闭环，以及是否启用世界横向锁定。
# 输入参数：edge 为目标边；lock_yaw 为锁定角；move_yaw 为指定移动方向，999.9 表示普通推送；world_side_enable 表示是否允许横向世界坐标锁定。
def _request_push_control(edge, lock_yaw, move_yaw=999.9, world_side_enable=True):
    cam_obj = _find_target_in_cam()
    # 从车和主车一样，用自己的物体视觉环贴住物体（5.28 方案）。
    # PUSH 阶段的球层已整层移除，物体视觉环是唯一的修正源。
    visual_enable = _PUSH_VISUAL_LOOP_FOLLOWER_ENABLE
    world_side_valid = _PUSH_WORLD_SIDE_LOCK_ENABLE and world_side_enable and (_mode_sub != _PUSH_ROUTE) and _push_world_side_valid
    if visual_enable and cam_obj >= 0:
        base.request_push(edge, lock_yaw, True, base._cam_obj_rel_x[cam_obj], base._cam_obj_rel_y[cam_obj], _push_ref_valid, _push_ref_rel_x, _push_ref_rel_y, move_yaw, world_side_valid, _push_world_side_axis, _push_world_side_ref, False, 0.0, 0.0)
    else:
        base.request_push(edge, lock_yaw, False, 0.0, 0.0, False, 0.0, 0.0, move_yaw, world_side_valid, _push_world_side_axis, _push_world_side_ref, False, 0.0, 0.0)

# 功能：等待主车确认推送起步。
# 从车进入 PUSH_SYNC 后会先保持锁定角，直到收到新鲜的主车 ready=4 或等待超时，避免从车抢先推送。
# 输入参数：now_ms 为当前时间戳；lock_yaw 为等待期间保持的车头角。
# 返回值：True 表示可以真正推送；False 表示仍需保持等待。
def _push_start_delay_elapsed(now_ms, lock_yaw):
    global _self_ready
    global _push_start_t0
    if _push_start_t0 == 0:
        _push_start_t0 = now_ms
    elapsed = ticks_diff(now_ms, _push_start_t0)
    if base._Other_Car_Ready >= 4 or elapsed >= _PUSH_GO_TIMEOUT_MS:
        return True
    base.request_hold(lock_yaw)
    return False

# 功能：判断主车推送状态是否在线且足够新鲜。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示主车 ready 有效且无线时间戳未超时。
def _peer_push_ok(now_ms):
    if base._Other_Car_Ready < 1:
        return False
    if base._Other_Car_Ready_Ts == 0:
        return False
    return ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS

# 功能：推送过程中主车掉线或协同异常时，停止推送并回到接近阶段重新组织队形。
def _abort_push_to_approach(lock_yaw):
    global _approach_cmd_to_other, _approach_plan_valid, _next_task_mode, _self_ready
    base.request_hold(lock_yaw)
    _self_ready = 0
    _approach_plan_valid = 0
    _approach_cmd_to_other = 999.9
    _next_task_mode = _MODE_APPROACH

# 功能：把世界横向锁定参考值重设为当前车辆位置。
# 诊断斜推结束后调用它，避免斜推造成的横向偏移继续被旧参考拉回。
def _reset_world_side_ref_to_current():
    global _push_world_side_ref
    if _push_world_side_axis == 1:
        _push_world_side_ref = base._car.Position_X
    elif _push_world_side_axis == 2:
        _push_world_side_ref = base._car.Position_Y

# 功能：清除诊断斜推命令。
# 输入参数：rebase 表示如果刚结束诊断斜推，是否把横向锁定参考重设到当前车辆位置。
def _clear_diag_cmd(rebase=False):
    global _follower_cmd_yaw_dir, _master_cmd_sub
    global _push_diag_active
    was_active = _push_diag_active
    _push_diag_active = False
    base._push_world_side_cmd = 0.0
    if rebase and was_active:
        _reset_world_side_ref_to_current()
# 功能：从车接收并执行主车下发的诊断斜推命令。
# 主车发现推送走廊单侧有障碍时，会通过无线发送诊断 yaw；从车只负责跟随该方向，不主动决定诊断方向。
# 输入参数：now_ms 为当前时间戳。
# 返回值：诊断推送 yaw；999.9 表示当前没有有效诊断命令。
def _follower_diag_update(now_ms):
    global _push_diag_active
    fresh = base._Other_Car_Ready_Ts != 0 and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS
    if fresh and (base._Other_Car_Mode == _MODE_PUSH_SYNC and base._Other_Target_Edge == _target_edge and (base._Other_Target_ObjId == _target_obj_id)) and (base._Other_Master_Cmd_Sub == _CMD_SUB_DIAG_PUSH):
        yaw = base._Other_Follower_Cmd_Yaw_Dir
        if yaw < 900.0:
            _push_diag_active = True
            base._push_world_side_cmd = 0.0
            base.avoid_beep(1)
            return yaw
    _clear_diag_cmd(True)
    return 999.9

# 功能：更新从车的推送障碍观测并读取主车诊断命令。
# 输入参数：now_ms 为当前时间戳；edge 为目标边；cam_obj 为当前目标摄像头槽位。
# 返回值：诊断推送 yaw；999.9 表示不启用诊断方向。
def _push_diag_update(now_ms, edge, cam_obj):
    try:
        _update_push_obs()
    except Exception:
        pass
    return _follower_diag_update(now_ms)

# 功能：更新从车 PUSH_SYNC 模式。
# 该函数处理主车起步等待、主车掉线保护、目标视觉丢失后退/旋转搜索、路线推送、诊断斜推、球体辅助推送、速度前馈和主车完成信号。
# 输入参数：now_ms 为当前时间戳；为 None 时函数内部读取 ticks_ms()。
def push_update(now_ms=None):
    global _recover_edge
    global _follower_cmd_yaw_dir, _master_cmd_sub, _next_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _route_cmd_to_other, _self_sub, _self_ready, _target_obj_world_x, _target_obj_world_y
    global _push_done, _push_started, _push_peer_lost_t0
    global _push_sub_t0, _push_start_t0
    global _push_recover_found, _push_recover_hold_yaw
    global _push_route_settle_t0, _push_route_cruise_applied
    if now_ms is None:
        now_ms = ticks_ms()
    _self_sub = _mode_sub
    if not _push_started:
        push_start(_target_edge, -1)
    edge = _target_edge
    lock_yaw = _push_lock_yaw
    base.clear_face()
    cam_obj = _find_target_in_cam()
    if _push_done:
        base.request_hold(lock_yaw)
        return
    if not _push_start_delay_elapsed(now_ms, lock_yaw):
        return
    in_lost_recover = _mode_sub == _PUSH_SPIN
    peer_fresh = base._Other_Car_Ready_Ts != 0 and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS
    # PUSH_SPIN 只由主车决定是否进入。从车不再依据自己的物体视觉丢失计时
    # 独立切换；只要收到新鲜的主车 PUSH_SPIN 状态，当周期立即同步进入。
    if (
        not in_lost_recover
        and peer_fresh
        and base._Other_Car_Mode == _MODE_PUSH_SYNC
        and base._Other_Car_Push_Sub == _PUSH_SPIN
    ):
        _enter_lost_spin(now_ms)
        _self_sub = _mode_sub
        in_lost_recover = True
    if in_lost_recover and peer_fresh:
        if base._Other_Car_Mode == _MODE_SEARCH:
            base.request_hold(lock_yaw)
            _clear_push_target_lock()
            _next_task_mode = _MODE_SEARCH
            return
        if base._Other_Car_Mode == _MODE_APPROACH:
            if _push_recover_found:
                base.request_hold(lock_yaw)
                _abort_push_to_approach(lock_yaw)
            else:
                _clear_push_target_lock()
                _next_task_mode = _MODE_SEARCH
            return
    if not in_lost_recover:
        if _peer_push_ok(now_ms):
            _push_peer_lost_t0 = 0
        elif _push_peer_lost_t0 == 0:
            _push_peer_lost_t0 = now_ms
        elif ticks_diff(now_ms, _push_peer_lost_t0) > _PUSH_PEER_LOST_CONFIRM_MS:
            _abort_push_to_approach(lock_yaw)
            return
    if _mode_sub == _PUSH_SPIN:
        if _push_recover_found or cam_obj >= 0:
            if not _push_recover_found:
                _push_recover_found = True
                _push_recover_hold_yaw = base._car.current_angle
            _self_ready = _PUSH_RECOVER_FOUND_READY
            base.request_hold(_push_recover_hold_yaw)
            return
        elapsed = ticks_diff(now_ms, _push_sub_t0)
        if elapsed >= _PUSH_LOST_SPIN_MS:
            base.request_hold(base._car.current_angle)
            _self_ready = 0
            return
        spin_dir = 1 if _angle_diff(lock_yaw, _push_yaw_for_edge(edge)) >= 0.0 else -1
        target_yaw = (_push_spin_start_yaw + spin_dir * 360.0 * (elapsed / _PUSH_LOST_SPIN_MS)) % 360.0
        if abs(_angle_diff(target_yaw, base._car.current_angle)) < _PUSH_LOST_SPIN_EPS:
            target_yaw = (target_yaw + spin_dir * _PUSH_LOST_SPIN_EPS) % 360.0
        base.request_world(0.0, 0.0, target_yaw)
        return
    if _mode_sub == _PUSH_ROUTE:
        if _push_sub_t0 == 0:
            _push_sub_t0 = now_ms
        elif cam_obj >= 0:
            _target_obj_world_x = base._cam_obj_x[cam_obj]
            _target_obj_world_y = base._cam_obj_y[cam_obj]
        if not _push_route_cruise_applied:
            if _push_route_settle_t0 == 0:
                _push_route_settle_t0 = now_ms
            elif ticks_diff(now_ms, _push_route_settle_t0) >= _PUSH_ROUTE_SETTLE_MS:
                _push_route_cruise_applied = True
                pid = _push_pid_params()
                fwd_max = _PUSH_ROUTE_GUIDE_FWD_MAX if _push_route_is_guide else _PUSH_ROUTE_PUSH_FWD_MAX
                base.configure_push(speed=_PUSH_ROUTE_SPEED, inward_bias=_push_inward_bias(), cam_lost_tol_ms=_CAM_LOST_TOL_MS, side_kp=pid[0], side_kd=pid[1], fwd_kp=pid[2], fwd_kd=pid[3], side_max=_push_side_max(), fwd_max=fwd_max, side_slew=_PUSH_SIDE_SLEW_FOLLOWER, fwd_slew=_PUSH_FWD_SLEW_FOLLOWER, d_lpf=_PUSH_D_LPF_FOLLOWER, side_deadband=_PUSH_SIDE_DEADBAND, fwd_deadband=_PUSH_FWD_DEADBAND, side_min=_PUSH_SIDE_MIN, fwd_min=_PUSH_FWD_MIN)
        if base._Other_Master_Cmd_Sub == _CMD_SUB_ROUTE_RESTORE:
            base.request_hold(_push_lock_yaw)
            _finish_route_push(cam_obj)
            return
        # 本地兜底：正常退出信号是主车下发的 _CMD_SUB_ROUTE_RESTORE；这里只在
        # 无线丢包导致收不到退出信号时兜底，避免顶死不退。
        if cam_obj >= 0:
            obj_axis = _push_axis_value(base._cam_obj_x[cam_obj], base._cam_obj_y[cam_obj], edge)
        else:
            obj_axis = _push_axis_value(_target_obj_world_x, _target_obj_world_y, edge)
        route_reached = obj_axis >= _push_route_axis if _push_route_yaw == 90.0 or _push_route_yaw == 0.0 else obj_axis <= _push_route_axis
        if obj_axis < 900.0 and route_reached:
            base.request_hold(_push_lock_yaw)
            _finish_route_push(cam_obj)
            return
        if ticks_diff(now_ms, _push_sub_t0) > _PUSH_ROUTE_TIMEOUT_MS:
            base.request_hold(_push_lock_yaw)
            _finish_route_push(cam_obj)
            return
        _request_push_control(edge, _push_lock_yaw, _push_route_yaw, False)
        return
    # 从车自身是否看到物体只影响视觉修正，不再决定 PUSH_SPIN 的进入。
    # 主车仍为 PUSH_RUN 时，从车继续当前推送；主车切 SPIN 时由上方无线同步。
    diag_yaw = _push_diag_update(now_ms, edge, cam_obj)
    if diag_yaw < 900.0:
        _request_push_control(edge, lock_yaw, diag_yaw, True)
    else:
        _request_push_control(edge, lock_yaw)
    peer_mode = base._Other_Car_Mode
    if base._Other_Master_Cmd_Sub == _CMD_SUB_PUSH_DONE or peer_mode == _MODE_RECOVER or peer_mode == _MODE_RELOCALIZE or (peer_mode == _MODE_DONE):
        # 收到主车 PUSH 完成后立即清掉本车向外推送残速，再进入 RECOVER BACK。
        base.request_hard_hold(lock_yaw)
        recover_edge = _target_edge
        _mark_target_done()
        _self_ready = 0
        base._push_world_side_cmd = 0.0
        set_recover_target(recover_edge)
        _next_task_mode = _MODE_RECOVER
        return
    if peer_mode == _MODE_SEARCH:
        base.request_hold(lock_yaw)
        _clear_push_target_lock()
        _next_task_mode = _MODE_SEARCH
        return
    return

_READY_EXIT_POS_EPS = const(12)

_READY_EXIT_LAT_EPS = const(12)

_route_avoid_cmd_last = 999.9

_route_avoid_obs_prev = 999.9

# 功能：确认从车在 WAIT_READY 阶段是否仍保持在适合开始推送的位置。
# 只有目标仍在视觉中，且相对前后距离和横向偏差都接近预期，才允许进入同步推送。
# 返回值：True 表示当前靠近姿态仍可用；False 表示需要重新接近。
#
# 判据优先级：物体在前、球在后，两者是【或】的关系。
# 物体是绝对基准：体积大、就是正在推的东西、永远在视野里，且"物体相对从车的位置
# 正确"本身就等价于"从车站位正确"，足以作为发车判据。
# 球只保留救场作用（物体短暂看不见时顶上），【不再有否决权】：一个位置离谱的误检球
# 曾经可以直接否掉一个完好的物体读数，导致 _self_ready 掉 0、主车干等，
# 严重时连续 2 秒后退回 APPROACH 重做 VDOCK，表现为从车起步明显滞后。
# 橙球与红沙包/网球色相接近、最易误检，该否决权正是橙球侧延迟远大于紫球侧的原因。
def target_still_ok():
    obj_id = _target_obj_id
    cam_rel = _find_target_in_cam()
    if cam_rel >= 0:
        rel_x = base._cam_obj_rel_x[cam_rel]
        rel_y = base._cam_obj_rel_y[cam_rel]
        if 0 <= obj_id < len(_CLOSE_LAT_OFFSET_BY_OBJ):
            lat_offset = _CLOSE_LAT_OFFSET_BY_OBJ[obj_id]
        else:
            lat_offset = 0.0
        close_dist = _CLOSE_DIST_FOLLOWER_BY_OBJ[obj_id] if 0 <= obj_id < len(_CLOSE_DIST_FOLLOWER_BY_OBJ) else 9.0
        if abs(rel_y - close_dist) < _READY_EXIT_POS_EPS and abs(rel_x - lat_offset) < _READY_EXIT_LAT_EPS:
            return True
    # 与 VDOCK 保持一致：_dock_obj_only 的目标（网球）全程不碰球识别，
    # 因为网球黄绿色会在色块粗筛里把橙球顶掉。原来这里漏了这个判断，
    # 网球在 WAIT_READY 反而照样用球做判据，正好是最易误检的组合。
    if _dock_obj_only():
        return False
    ok, rel_x, rel_y, ref_x, ref_y = _slot_ball_ref(_wait_lock_yaw if _wait_lock_valid else _lock_yaw(_target_edge))
    if not ok:
        return False
    return abs(rel_y - ref_y) < _READY_EXIT_POS_EPS and abs(rel_x - ref_x) < _READY_EXIT_LAT_EPS

# 功能：判断某个目标边在绕障规划中使用 X 轴还是 Y 轴作为横向清障轴。
# 输入参数：edge 为目标边编号。
# 返回值：True 表示使用 X 轴，False 表示使用 Y 轴。
def _route_axis_is_x(edge):
    return edge == 1 or edge == 4

# 功能：根据轴选择从坐标中取出 X 或 Y。
# 输入参数：x/y 为坐标值，axis_is_x 表示是否取 X。
# 返回值：选中的轴坐标。
def _route_axis_value(x, y, axis_is_x):
    return x if axis_is_x else y

# 功能：根据主车给出的清障轴坐标计算目标需要横向移动的方向。
# 输入参数：clear_axis 为希望目标移动到的清障轴坐标。
# 返回值：路线移动 yaw；目标坐标或清障轴无效时返回 999.9。
def route_move_yaw(clear_axis):
    edge = _target_edge
    ref_x = base._Other_Target_X
    ref_y = base._Other_Target_Y
    obj_axis = _route_axis_value(ref_x, ref_y, _route_axis_is_x(edge))
    if obj_axis >= 900.0 or clear_axis >= 900.0:
        return 999.9
    if edge == 1 or edge == 4:
        return 90.0 if clear_axis >= obj_axis else 270.0
    if edge == 2 or edge == 3:
        return 0.0 if clear_axis >= obj_axis else 180.0
    return 999.9

# 功能：更新从车等待阶段对目标前方推送走廊的障碍观测。
# 从车主要使用摄像头第二个普通目标槽位作为障碍候选，并把障碍轴坐标共享给主车。
# 输入参数：now_ms 为当前时间戳。
def _route_update_observation(now_ms):
    global _route_cmd_to_other, _route_obs_axis_to_other
    if not _READY_ROUTE_AVOID_ENABLE:
        _route_obs_axis_to_other = 999.9
        if _push_route_phase not in (1, 3):
            _route_cmd_to_other = 999.9
        return
    if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        _route_obs_axis_to_other = 999.9
        if _push_route_phase not in (1, 3):
            _route_cmd_to_other = 999.9
        return
    global _route_avoid_obs_prev
    if base._cam_obj_count <= 1:
        _route_avoid_obs_prev = 999.9
        _route_obs_axis_to_other = 999.9
        if _push_route_phase not in (1, 3):
            _route_cmd_to_other = 999.9
        return
    obj_type = base._cam_obj_id[1]
    if obj_type == _target_obj_id or not 1 <= obj_type < len(_obj_remain) or _obj_remain[obj_type] <= 0:
        _route_avoid_obs_prev = 999.9
        _route_obs_axis_to_other = 999.9
        if _push_route_phase not in (1, 3):
            _route_cmd_to_other = 999.9
        return
    ox = base._cam_obj_x[1]
    oy = base._cam_obj_y[1]
    if ox >= 900.0 or oy >= 900.0:
        _route_avoid_obs_prev = 999.9
        _route_obs_axis_to_other = 999.9
        if _push_route_phase not in (1, 3):
            _route_cmd_to_other = 999.9
        return
    axis_is_x = _route_axis_is_x(_target_edge)
    push_rad = radians(_push_yaw(_target_edge))
    px = sin(push_rad)
    py = cos(push_rad)
    sx = -py
    sy = px
    dx = ox - _target_obj_world_x
    dy = oy - _target_obj_world_y
    proj_fwd = dx * px + dy * py
    if proj_fwd <= 0.0 or proj_fwd < _READY_ROUTE_FWD_MIN_DIST or proj_fwd > _READY_ROUTE_FWD_MAX_DIST or abs(dx * sx + dy * sy) > _READY_ROUTE_SAFE_HALF:
        _route_avoid_obs_prev = 999.9
        _route_obs_axis_to_other = 999.9
        if _push_route_phase not in (1, 3):
            _route_cmd_to_other = 999.9
        return
    best_axis = _route_axis_value(ox, oy, axis_is_x)
    if abs(best_axis - _route_avoid_obs_prev) > 2.0:
        _route_avoid_obs_prev = best_axis
    _route_obs_axis_to_other = best_axis
    _route_cmd_to_other = proj_fwd


# 功能：WAIT_READY 中更新从车的绕障路线信息。
# 从车主要观测障碍并等待主车命令；若主车给出清障轴，则记录路线轴和路线移动方向。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示路线检查可继续；False 表示目标姿态不合格或主车命令不足。
def update_ready_route(now_ms):
    global _push_route_axis, _push_route_move_yaw
    if not _READY_ROUTE_AVOID_ENABLE:
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        return target_still_ok()
    if not target_still_ok():
        if _push_route_phase not in (1, 3):
            _push_route_axis = 999.9
            _push_route_move_yaw = 999.9
        return False
    _route_update_observation(now_ms)
    global _route_avoid_cmd_last
    cmd = base._Other_Route_Cmd
    if cmd >= 900.0:
        return False
    if cmd < 0.0:
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        return True
    if cmd != _route_avoid_cmd_last:
        _route_avoid_cmd_last = cmd
    _push_route_axis = cmd
    _push_route_move_yaw = route_move_yaw(cmd)
    return True

# 功能：只更新等待阶段的障碍观测，不主动生成新的路线计划。
# 输入参数：now_ms 为当前时间戳。
def update_ready_route_observation(now_ms):
    if not _READY_ROUTE_AVOID_ENABLE:
        return
    if not target_still_ok():
        return
    _route_update_observation(now_ms)

_READY_RELOCALIZE_ENABLE = const(1)

_READY_RELOCALIZE_MAX_CORR = const(100)

_READY_RELOCALIZE_BLEND = 0.8

_READY_CLOSE_BAD_MS = const(2000)

_ready_close_bad_t0 = 0

_wait_lock_yaw = 0.0

_wait_lock_valid = False

# 功能：重置等待主车就位阶段的状态。
# 会清空靠近姿态异常计时，并锁定等待阶段使用的车头角。
def ready_reset():
    global _ready_close_bad_t0
    global _wait_lock_yaw, _wait_lock_valid
    _ready_close_bad_t0 = 0
    _wait_lock_yaw = 0.0
    _wait_lock_valid = False

# 功能：WAIT_READY 刚进入时利用主车共享目标坐标和本车视觉目标反算从车位置。
# 这能在开始同步推送前修正从车里程计误差，避免两车对同一目标的世界坐标理解不一致。
# 返回值：True 表示发起了位置修正请求；False 表示目标或视觉条件不足。
def _relocalize_follower_at_ready():
    if not _READY_RELOCALIZE_ENABLE:
        return False
    if base._Other_Target_X >= 900.0 or base._Other_Target_Y >= 900.0:
        return False
    cam_obj = _find_target_in_cam()
    if cam_obj < 0:
        return False
    rel_x = base._cam_obj_rel_x[cam_obj]
    rel_y = base._cam_obj_rel_y[cam_obj]
    if rel_x >= 900.0 or rel_y >= 900.0:
        return False
    yaw_rad = radians(base._car.current_angle)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    new_x = base._Other_Target_X - c * rel_x - s * rel_y
    new_y = base._Other_Target_Y + s * rel_x - c * rel_y
    dx = new_x - base._car.Position_X
    dy = new_y - base._car.Position_Y
    if sqrt(dx * dx + dy * dy) > _READY_RELOCALIZE_MAX_CORR:
        return False
    w = _READY_RELOCALIZE_BLEND
    if w < 0.0:
        w = 0.0
    if w > 1.0:
        w = 1.0
    fix_x = base._car.Position_X * (1.0 - w) + new_x * w
    fix_y = base._car.Position_Y * (1.0 - w) + new_y * w
    if fix_x < 900.0:
        base._pos_fix_x = fix_x
        base._pos_fix_x_valid = True
    if fix_y < 900.0:
        base._pos_fix_y = fix_y
        base._pos_fix_y_valid = True
    base._pos_fix_req = True
    return True

# 功能：确认可以开始推送，把本车 ready 置为推送准备状态并请求进入 PUSH_SYNC。
def _do_push_start():
    global _next_task_mode, _self_ready
    _self_ready = 3
    _next_task_mode = _MODE_PUSH_SYNC

# 功能：从等待阶段触发绕障重靠近。
# 输入参数：route_move_yaw 为目标需要侧向带离障碍的移动方向。
# 返回值：固定返回 True，表示已设置 APPROACH 请求。
def _start_route_reclose(route_move_yaw):
    global _approach_req, _approach_route_move_yaw
    global _next_task_mode, _self_ready
    _approach_req = _APPROACH_REQ_RECLOSE
    _approach_route_move_yaw = route_move_yaw
    _self_ready = 0
    _next_task_mode = _MODE_APPROACH
    return True

# 功能：从车在 WAIT_READY 中跟随主车路线/推送入口命令。
# 若主车已经进入 PUSH_SYNC，从车立即同步进入推送；若主车下发绕障轴，则从车先进入重靠近流程。
# 返回值：True 表示已经响应主车命令并切换流程；False 表示继续普通等待。
def _follower_follow_ready_entry():
    global _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _target_obj_world_x, _target_obj_world_y
    if base._Other_Car_Mode == _MODE_PUSH_SYNC:
        cmd = base._Other_Route_Cmd
        if _READY_ROUTE_AVOID_ENABLE and 0.0 <= cmd < 900.0 and _push_route_phase != 2:
            ox_t = base._Other_Target_X
            oy_t = base._Other_Target_Y
            if ox_t < 900.0 and oy_t < 900.0:
                _target_obj_world_x = ox_t
                _target_obj_world_y = oy_t
            _push_route_axis = cmd
            _push_route_move_yaw = route_move_yaw(cmd)
            _push_route_phase = 2
        _do_push_start()
        return True
    if not _READY_ROUTE_AVOID_ENABLE:
        return False
    cmd = base._Other_Route_Cmd
    if 0.0 <= cmd < 900.0 and _push_route_phase == 0:
        ox_t = base._Other_Target_X
        oy_t = base._Other_Target_Y
        if ox_t < 900.0 and oy_t < 900.0:
            _target_obj_world_x = ox_t
            _target_obj_world_y = oy_t
        _push_route_axis = cmd
        _push_route_move_yaw = route_move_yaw(cmd)
        _push_route_phase = 1
        if _start_route_reclose(_push_route_move_yaw):
            return True
        _push_route_phase = 0
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        _push_route_restore_push = 0
    return False

# 功能：更新从车 WAIT_READY 模式。
# 从车在这里保持接近完成时的角度，校验目标是否仍在合适位置，观测推送走廊障碍，并跟随主车的路线绕障或推送启动命令。
# 输入参数：now_ms 为当前时间戳。
def ready_update(now_ms):
    global _approach_req
    global _follower_cmd_yaw_dir, _master_cmd_sub, _next_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _route_cmd_to_other, _self_ready, _target_obj_world_x, _target_obj_world_y
    global _wait_lock_yaw, _wait_lock_valid
    global _ready_close_bad_t0
    if not _wait_lock_valid:
        _relocalize_follower_at_ready()
        _self_ready = 1
        _wait_lock_yaw = base._car.current_angle
        _wait_lock_valid = True
    if debug_switch.READY_HOLD_ENABLE:
        _self_ready = 1
        base.request_hold(_wait_lock_yaw)
        return
    if _follower_follow_ready_entry():
        return
    if not target_still_ok():
        _self_ready = 0
        if _ready_close_bad_t0 == 0:
            _ready_close_bad_t0 = now_ms
        if ticks_diff(now_ms, _ready_close_bad_t0) > _READY_CLOSE_BAD_MS:
            _ready_close_bad_t0 = 0
            _approach_req = _APPROACH_REQ_RESTART_CLOSE
            _next_task_mode = _MODE_APPROACH
            return
    else:
        _ready_close_bad_t0 = 0
        _self_ready = 1
    base.request_hold(_wait_lock_yaw)
    update_ready_route_observation(now_ms)
    if _READY_ROUTE_AVOID_ENABLE and base._Other_Car_Mode == _MODE_PUSH_SYNC:
        cmd = base._Other_Route_Cmd
        if 0.0 <= cmd < 900.0 and _push_route_phase != 2:
            ox_t = base._Other_Target_X
            oy_t = base._Other_Target_Y
            if ox_t < 900.0 and oy_t < 900.0:
                _target_obj_world_x = ox_t
                _target_obj_world_y = oy_t
            _push_route_axis = cmd
            _push_route_move_yaw = route_move_yaw(cmd)
            _push_route_phase = 2
        _do_push_start()
        return
    cmd = base._Other_Route_Cmd
    if _READY_ROUTE_AVOID_ENABLE and 0.0 <= cmd < 900.0 and _push_route_phase == 0 and (_push_route_restore_push == 0):
        ox_t = base._Other_Target_X
        oy_t = base._Other_Target_Y
        if ox_t < 900.0 and oy_t < 900.0:
            _target_obj_world_x = ox_t
            _target_obj_world_y = oy_t
        _push_route_axis = cmd
        _push_route_move_yaw = route_move_yaw(cmd)
        _push_route_phase = 1
        if _start_route_reclose(_push_route_move_yaw):
            return
        _push_route_phase = 0
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        _push_route_restore_push = 0
    route_phase = _push_route_phase
    if route_phase in (1, 2, 3, 4):
        route_ok = True
    else:
        route_ok = update_ready_route(now_ms)
    if not route_ok:
        return
    route_phase = _push_route_phase
    route_axis = _push_route_axis
    if route_phase == 4:
        _push_route_restore_push = 1
        _push_route_phase = 0
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        _route_cmd_to_other = 999.9
        _follower_cmd_yaw_dir = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
    elif base._Other_Car_Mode == _MODE_PUSH_SYNC:
        _do_push_start()

_SUB_BACK = const(0)

_SUB_RECOVER_TURN = const(1)

_SUB_RECOVER_TRANSLATE = const(2)

_SUB_RECOVER_ORBIT_YAW_ALIGN = const(3)

_SUB_RECOVER_ORBIT = const(4)

_RECOVER_BACK_SPEED = const(120)

# 与主车一致：后退位移达到 8 cm 正常退出；1500 ms 只作为超时保护。
_RECOVER_BACK_DIST = 8.0

_RECOVER_BACK_TIMEOUT_MS = const(1500)

_RECOVER_TURN_DPS = const(200)

_RECOVER_TURN_EPS_DEG = const(7)

# 航向对齐完成后使用更宽的重入阈值，避免误差在 7 度边界附近反复切换状态。
_RECOVER_YAW_REENTER_EPS_DEG = const(10)

_RECOVER_TRANSLATE_SPEED = const(120)

_RECOVER_FIND_CAR_CENTER_X = const(10)

_RECOVER_FIND_CAR_FRESH_MS = const(250)

_RECOVER_TO_SEARCH_BEEP_MS = const(120)

# 绕行速度/减速/半径参数与 SEARCH 里的 _FOLLOWER_ORBIT_* 保持同一份数值来源，
# 因此在这里提前定义、RECOVER 直接引用，而不是各自写一份数字。SEARCH 小节里
# 同名变量已经不再重复定义（MicroPython const 折叠要求先定义、后使用，
# 所以这组常量必须放在这个更早使用它们的 RECOVER 小节之前）。
_FOLLOWER_ORBIT_LEADER_SPD = const(80)

# SEARCH/RECOVER 共用的绕行减速：剩余角度大于 35 度时保持 80 正常绕行；
# 进入 35 度后线性降速，到 15 度边界时降到 75。若越过目标，也固定使用 75 反向精调。
_FOLLOWER_ORBIT_SLOW_START_DEG = const(35)
_FOLLOWER_ORBIT_FINE_SPD = 75.0

_FOLLOWER_ORBIT_RADIAL_MAX = const(80)

_FOLLOWER_ORBIT_RADIAL_KP = const(4)

# SEARCH 的 ORBIT_FORM 绕主车就位专用径向增益。RECOVER 也采用这个已经验证过
# 的增益，避免绕行时切向速度把半径持续压小。
_FOLLOWER_SEARCH_ORBIT_RADIAL_KP = const(7)

_FOLLOWER_ORBIT_PHI_EPS = const(10)

# 仅在本轮 PUSH 由 cone/brick 触发过斜避时使用。先粗绕到目标圆周角，
# 再原地锁角，最后通过平移形成 28 cm 阵列。
_RECOVER_DIAG_ORBIT_SPEED = const(140)
_RECOVER_DIAG_ORBIT_FINE_SPEED = _FOLLOWER_ORBIT_FINE_SPD
_RECOVER_DIAG_ORBIT_SLOW_START_DEG = const(30)
_RECOVER_DIAG_ORBIT_RADIAL_KP = const(_FOLLOWER_SEARCH_ORBIT_RADIAL_KP)
_RECOVER_DIAG_ORBIT_RADIAL_MAX = const(_FOLLOWER_ORBIT_RADIAL_MAX)
_RECOVER_DIAG_ORBIT_TARGET_DIST = _FOLLOWER_FOLLOW_DIST
# 过近时允许更强的向外脱离；过远时仍使用 80，避免加速冲向主车。
_RECOVER_DIAG_ORBIT_RADIAL_OUT_MAX = const(120)
_RECOVER_DIAG_ORBIT_TOTAL_MAX = const(200)
_RECOVER_DIAG_ORBIT_PHI_EPS = const(_FOLLOWER_ORBIT_PHI_EPS)
_RECOVER_DIAG_FORM_DIST_EPS = const(7)
_RECOVER_DIAG_FORM_STABLE_MS = const(200)
_mode_sub = _SUB_BACK

_mode_hold_ms = 0

_recover_back_yaw = 0.0

_recover_sub_t0 = 0

_recover_back_x0 = 0.0

_recover_back_y0 = 0.0

_recover_edge = 0

_recover_diag_obstacle_yaw = 999.9
_recover_side_formation_yaw = 999.9
_recover_side_formation_ready = False
_recover_orbit_dir = 1.0
_recover_orbit_started = False
_recover_orbit_prev_remaining = 360.0
_recover_orbit_fine_correcting = False
_recover_orbit_stable_t0 = 0
_recover_translate_visual_active = False
_recover_translate_side_speed = 0.0
_recover_translate_fwd_speed = 0.0

# 功能：关闭恢复阶段使用的视觉位置修正开关。
def _clear_recover_fix_flags():
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False

# 功能：按车体系速度发出世界速度请求。
# 输入参数：vx/vy 为车体系速度，angle 为目标车头角。
def _set_body(vx, vy, angle):
    yaw = radians(base._car.current_angle)
    c = cos(yaw)
    s = sin(yaw)
    base.request_world(c * vx + s * vy, -s * vx + c * vy, angle)

# 功能：根据刚才的推送方向和从车所在槽位计算 RECOVER 固定观察航向。
# 左槽朝 push+90°，右槽朝 push-90°，使摄像头横向面对另一辆车所在侧。
def _recover_turn_target_yaw():
    push_yaw = _push_yaw(_recover_edge)
    side = _angle_diff(_approach_self_yaw, push_yaw)
    if side >= 0.0:
        return (push_yaw + 90.0) % 360.0
    return (push_yaw - 90.0) % 360.0


# 功能：计算从车 RECOVER 找主车时的世界平移方向。
# 始终沿刚才推送方向的反方向返回场内，与左右槽位无关。
def _recover_translate_yaw():
    return (_push_yaw(_recover_edge) + 180.0) % 360.0


def _recover_latch_diag_obstacle(now_ms):
    """锁存主车在 RECOVER 中广播的本轮斜避障碍侧向。"""
    global _recover_diag_obstacle_yaw
    fresh = (
        base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
    )
    cmd_yaw = base._Other_Follower_Cmd_Yaw_Dir
    if (not _all_objects_done()
            and fresh
            and base._Other_Car_Mode == _MODE_RECOVER
            and base._Other_Master_Cmd_Sub == _CMD_SUB_DIAG_PUSH
            and 0.0 <= cmd_yaw < 900.0):
        _recover_diag_obstacle_yaw = (int((cmd_yaw + 45.0) // 90.0) * 90.0) % 360.0
 

# RECOVER 斜避绕行的视觉方位低通滤波，与 SEARCH 的 _search_bearing_filt 同一
# 用途、同一混合系数，但状态独立，避免两条路径互相干扰。原来这里直接用
# 每周期的瞬时 atan2(rel_x, rel_y) 当车头目标下发 request_world，噪声或车身
# 转动扰动一放大就会形成"测量抖动->车头摆->测量更抖"的自激摆动（8.16
# 实测单次斜避绕行耗时约4秒、剩余角度一度冲到208度），滤波后车头目标改为
# 跟踪平滑后的方位，不再逐帧硬跟原始读数。
_RECOVER_DIAG_BEARING_BLEND = 0.35

_recover_bearing_filt = 0.0
_recover_bearing_filt_valid = False

def _recover_bearing_filter(raw_bearing_err):
    global _recover_bearing_filt, _recover_bearing_filt_valid
    if not _recover_bearing_filt_valid:
        _recover_bearing_filt = raw_bearing_err
        _recover_bearing_filt_valid = True
    else:
        _recover_bearing_filt += _RECOVER_DIAG_BEARING_BLEND * _angle_diff(
            raw_bearing_err, _recover_bearing_filt
        )
        _recover_bearing_filt = (_recover_bearing_filt + 180.0) % 360.0 - 180.0
    return _recover_bearing_filt


def _recover_diag_orbit_update(now_ms):
    """保持看向主车，以主车为圆心绕到障碍物反侧。"""
    global _mode_sub, _recover_sub_t0, _self_ready
    global _recover_orbit_dir, _recover_orbit_started
    global _recover_orbit_prev_remaining, _recover_orbit_fine_correcting
    global _recover_orbit_stable_t0
    global _recover_diag_obstacle_yaw
    global _recover_translate_visual_active, _recover_translate_side_speed, _recover_translate_fwd_speed
    _clear_recover_fix_flags()
    _self_ready = 0
    target_yaw = _recover_diag_obstacle_yaw
    car_fresh = (
        base._cam_rx_last_ms != 0
        and ticks_diff(now_ms, base._cam_rx_last_ms) <= _RECOVER_FIND_CAR_FRESH_MS
        and _find_car_in_cam() >= 0
    )
    if not car_fresh:
        _recover_orbit_stable_t0 = 0
        base.clear_face()
        base.request_hold(base._car.current_angle)
        return

    rel_x = base._cam_car_rel_x
    rel_y = base._cam_car_rel_y
    rel_dist = sqrt(rel_x * rel_x + rel_y * rel_y)
    raw_bearing_err = degrees(atan2(rel_x, rel_y))
    bearing_err = _recover_bearing_filter(raw_bearing_err)
    measured_face_yaw = (base._car.current_angle + bearing_err) % 360.0
    phi_now = (measured_face_yaw + 180.0) % 360.0
    target_phi = (target_yaw + 180.0) % 360.0
    phi_err = _angle_diff(target_phi, phi_now)

    if not _recover_orbit_started:
        if abs(phi_err) >= 179.0:
            _recover_orbit_dir = 1.0
        else:
            _recover_orbit_dir = 1.0 if phi_err >= 0.0 else -1.0
        _recover_orbit_prev_remaining = abs(phi_err)
        _recover_orbit_fine_correcting = False
        _recover_orbit_started = True

    # 和 SEARCH 的粗绕交接一致：圆周位置到达后立即取消切向速度，
    # 航向、居中和距离误差交给后续两个独立状态收敛。
    if abs(phi_err) <= _RECOVER_DIAG_ORBIT_PHI_EPS:
        base.clear_face()
        base.request_hard_hold(base._car.current_angle)
        _recover_sub_t0 = now_ms
        _recover_orbit_stable_t0 = 0
        _recover_translate_visual_active = False
        _recover_translate_side_speed = 0.0
        _recover_translate_fwd_speed = 0.0
        _mode_sub = _SUB_RECOVER_ORBIT_YAW_ALIGN
        return

    if _recover_orbit_dir > 0.0:
        orbit_remaining = (target_phi - phi_now + 360.0) % 360.0
    else:
        orbit_remaining = (phi_now - target_phi + 360.0) % 360.0
    passed_target = (
        _recover_orbit_prev_remaining <= _RECOVER_DIAG_ORBIT_SLOW_START_DEG
        and orbit_remaining > _recover_orbit_prev_remaining + _RECOVER_DIAG_ORBIT_PHI_EPS
    )
    if passed_target:
        _recover_orbit_dir = 1.0 if phi_err >= 0.0 else -1.0
        _recover_orbit_fine_correcting = True
        if _recover_orbit_dir > 0.0:
            orbit_remaining = (target_phi - phi_now + 360.0) % 360.0
        else:
            orbit_remaining = (phi_now - target_phi + 360.0) % 360.0
    _recover_orbit_prev_remaining = orbit_remaining

    yaw_rad = radians(base._car.current_angle)
    toward_x = cos(yaw_rad) * rel_x + sin(yaw_rad) * rel_y
    toward_y = -sin(yaw_rad) * rel_x + cos(yaw_rad) * rel_y
    radius = max(rel_dist, 1.0)
    rx_u = -toward_x / radius
    ry_u = -toward_y / radius
    tx_u = _recover_orbit_dir * ry_u
    ty_u = -_recover_orbit_dir * rx_u
    if _recover_orbit_fine_correcting:
        orbit_speed = _RECOVER_DIAG_ORBIT_FINE_SPEED
    elif orbit_remaining > _RECOVER_DIAG_ORBIT_SLOW_START_DEG:
        orbit_speed = _RECOVER_DIAG_ORBIT_SPEED
    else:
        slow_span = _RECOVER_DIAG_ORBIT_SLOW_START_DEG - _RECOVER_DIAG_ORBIT_PHI_EPS
        orbit_speed = _RECOVER_DIAG_ORBIT_FINE_SPEED + (
            (_RECOVER_DIAG_ORBIT_SPEED - _RECOVER_DIAG_ORBIT_FINE_SPEED)
            * (orbit_remaining - _RECOVER_DIAG_ORBIT_PHI_EPS) / max(slow_span, 1.0)
        )
    radial_max = (
        _RECOVER_DIAG_ORBIT_RADIAL_OUT_MAX
        if rel_dist < _RECOVER_DIAG_ORBIT_TARGET_DIST
        else _RECOVER_DIAG_ORBIT_RADIAL_MAX
    )
    radial_x, radial_y, dist_in_band = _search_distance_adjust(
        rel_x, rel_y, _RECOVER_DIAG_ORBIT_RADIAL_KP,
        radial_max, _RECOVER_DIAG_ORBIT_TARGET_DIST
    )
    peer_fresh = (
        base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
    )
    ff_x, ff_y = _search_leader_feedforward(False, True) if peer_fresh else (0.0, 0.0)
    raw_cmd_x = ff_x + tx_u * orbit_speed + radial_x
    raw_cmd_y = ff_y + ty_u * orbit_speed + radial_y
    raw_cmd_mag = sqrt(raw_cmd_x * raw_cmd_x + raw_cmd_y * raw_cmd_y)
    cmd_x = raw_cmd_x
    cmd_y = raw_cmd_y
    saturated = raw_cmd_mag > _RECOVER_DIAG_ORBIT_TOTAL_MAX
    if saturated:
        scale = _RECOVER_DIAG_ORBIT_TOTAL_MAX / raw_cmd_mag
        cmd_x *= scale
        cmd_y *= scale
    # 与 SEARCH 绕圆使用同一车头控制链：视觉方位误差直接进入带陀螺阻尼的 face PD。
    # request_world 的航向锁在当前角，避免每帧重建绝对目标角后再经过普通双环追赶。
    base._face_req_err = bearing_err
    base._face_req_active = 1
    base._face_req_seq += 1
    base.request_world(cmd_x, cmd_y, base._car.current_angle)

# 功能：清空当前目标信息。
# 恢复结束或任务重新搜索前调用，避免旧目标继续参与后续规划。
def _clear_target():
    global _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y
    _clear_recover_fix_flags()
    _target_obj_world_x = 999.0
    _target_obj_world_y = 999.0
    _target_edge = 0
    _target_obj_id = 0

# 功能：主车视觉居中后直接结束 RECOVER。
# 只依据本车剩余目标计数选择 SEARCH/RELOCALIZE，不依赖主车当前模式，
# 也不在状态切换前插入停车命令。
def _finish_recover():
    global _next_task_mode
    _clear_target()
    if _all_objects_done():
        _next_task_mode = _MODE_RELOCALIZE
    else:
        base._fix_beep_active = 1
        base._fix_beep_until_ms = ticks_add(ticks_ms(), _RECOVER_TO_SEARCH_BEEP_MS)
        _next_task_mode = _MODE_SEARCH

# 功能：重置从车恢复状态机。
# 从车先后退并转到槽位观察角；有斜避时按记录方向决定是否换边，随后锁存侧阵列。
def recover_reset():
    global _recover_force_car_reloc_pending, _self_ready
    global _mode_sub, _recover_sub_t0
    global _recover_back_x0, _recover_back_y0
    global _recover_diag_obstacle_yaw
    global _recover_side_formation_yaw, _recover_side_formation_ready
    global _recover_orbit_dir, _recover_orbit_started
    global _recover_orbit_prev_remaining, _recover_orbit_fine_correcting
    global _recover_orbit_stable_t0
    global _recover_translate_visual_active, _recover_translate_side_speed, _recover_translate_fwd_speed
    global _recover_bearing_filt_valid
    _mode_sub = _SUB_BACK
    _recover_sub_t0 = 0
    _recover_back_x0 = 0.0
    _recover_back_y0 = 0.0
    _recover_diag_obstacle_yaw = 999.9
    _recover_side_formation_yaw = 999.9
    _recover_side_formation_ready = False
    _recover_orbit_dir = 1.0
    _recover_orbit_started = False
    _recover_orbit_prev_remaining = 360.0
    _recover_orbit_fine_correcting = False
    _recover_orbit_stable_t0 = 0
    _recover_translate_visual_active = False
    _recover_translate_side_speed = 0.0
    _recover_translate_fwd_speed = 0.0
    _recover_bearing_filt_valid = False
    base.clear_face()
    base.configure_face(_KP_FACE_OBJ, _KD_FACE_OBJ, _FACE_GYRO_MAX)
    _recover_force_car_reloc_pending = False
    _self_ready = 0
    _clear_recover_fix_flags()
# 功能：记录刚完成推送的目标边，供后退后的槽位转向和下一轮 SEARCH 航向计算。
# 输入参数：edge 为刚完成推送的目标边。
def set_recover_target(edge):
    global _recover_edge, _last_completed_push_yaw
    global _recover_back_yaw, _recover_sub_t0, _recover_back_x0, _recover_back_y0
    _recover_edge = edge
    _last_completed_push_yaw = _push_yaw(edge)
    _recover_back_yaw = 0.0
    _recover_sub_t0 = 0
    _recover_back_x0 = 0.0
    _recover_back_y0 = 0.0

# 功能：更新从车 RECOVER 模式。
# 从车沿车体后方退出目标区域后，先转到槽位对应观察航向；仅当主车本轮实际下发斜避命令时绕主车换边。
# 看到主车并满足阵列条件后锁存该侧阵列，
# 供第一次 SWEEP 原样使用；未满足阵列条件时持续纠偏，不再超时停车。
# 输入参数：now_ms 为当前时间戳。
def recover_update(now_ms):
    global _next_task_mode, _recover_force_car_reloc_pending, _self_ready
    global _mode_sub
    global _recover_back_yaw, _recover_sub_t0, _recover_back_x0, _recover_back_y0
    global _recover_diag_obstacle_yaw
    global _recover_side_formation_yaw, _recover_side_formation_ready
    global _recover_orbit_started, _recover_orbit_stable_t0
    global _recover_orbit_prev_remaining, _recover_orbit_fine_correcting
    global _recover_translate_visual_active, _recover_translate_side_speed, _recover_translate_fwd_speed
    _recover_latch_diag_obstacle(now_ms)
    if _mode_sub == _SUB_BACK:
        if _recover_sub_t0 == 0:
            _recover_sub_t0 = now_ms
            _recover_back_yaw = base._car.current_angle
            _recover_back_x0 = base._car.Position_X
            _recover_back_y0 = base._car.Position_Y
        _set_body(0.0, -_RECOVER_BACK_SPEED, _recover_back_yaw)
        back_dx = base._car.Position_X - _recover_back_x0
        back_dy = base._car.Position_Y - _recover_back_y0
        back_rad = radians(_recover_back_yaw)
        # 只累计沿锁定车尾方向的有向位移，向外惯性滑行和横向移动不能完成 BACK。
        back_progress = back_dx * (-sin(back_rad)) + back_dy * (-cos(back_rad))
        back_done = back_progress >= _RECOVER_BACK_DIST
        back_elapsed_ms = ticks_diff(now_ms, _recover_sub_t0)
        back_timeout = back_elapsed_ms > _RECOVER_BACK_TIMEOUT_MS
        if back_done or back_timeout:
            # BACK结束先急停，下一周期再按恒定角速度转到槽位观察角。
            base.request_hard_hold(_recover_back_yaw)
            _recover_sub_t0 = 0
            _mode_sub = _SUB_RECOVER_TURN
        return
    if _mode_sub == _SUB_RECOVER_TURN:
        _clear_recover_fix_flags()
        _self_ready = 0
        current_yaw = base._car.current_angle
        target_yaw = _recover_turn_target_yaw()
        yaw_error = _angle_diff(target_yaw, current_yaw)
        if abs(yaw_error) <= _RECOVER_TURN_EPS_DEG:
            base.request_hold(target_yaw)
            _recover_sub_t0 = now_ms
            enter_orbit = (
                _recover_diag_obstacle_yaw < 900.0
                and abs(_angle_diff(_recover_diag_obstacle_yaw, current_yaw)) > _RECOVER_TURN_EPS_DEG
            )
            if enter_orbit:
                _recover_orbit_started = False
                _recover_orbit_prev_remaining = 360.0
                _recover_orbit_fine_correcting = False
                _recover_orbit_stable_t0 = 0
                _mode_sub = _SUB_RECOVER_ORBIT
            else:
                _recover_orbit_stable_t0 = 0
                _recover_translate_visual_active = False
                _recover_translate_side_speed = 0.0
                _recover_translate_fwd_speed = 0.0
                _mode_sub = _SUB_RECOVER_TRANSLATE
            return
        base.request_yaw_rate(_RECOVER_TURN_DPS if yaw_error > 0.0 else -_RECOVER_TURN_DPS)
        return
    if _mode_sub == _SUB_RECOVER_ORBIT:
        _recover_diag_orbit_update(now_ms)
        return
    if _mode_sub == _SUB_RECOVER_ORBIT_YAW_ALIGN:
        _clear_recover_fix_flags()
        _self_ready = 0
        target_yaw = _recover_diag_obstacle_yaw
        signed_heading_err = _angle_diff(target_yaw, base._car.current_angle)
        heading_err = abs(signed_heading_err)
        if heading_err <= _RECOVER_TURN_EPS_DEG:
            base.request_hard_hold(target_yaw)
            _recover_sub_t0 = now_ms
            _recover_orbit_stable_t0 = 0
            _recover_translate_visual_active = False
            _recover_translate_side_speed = 0.0
            _recover_translate_fwd_speed = 0.0
            _mode_sub = _SUB_RECOVER_TRANSLATE
        else:
            base.request_world(0.0, 0.0, target_yaw)
        return
    if _mode_sub == _SUB_RECOVER_TRANSLATE:
        _clear_recover_fix_flags()
        _self_ready = 0
        target_yaw = (
            _recover_diag_obstacle_yaw
            if _recover_diag_obstacle_yaw < 900.0
            else _recover_turn_target_yaw()
        )
        if (_recover_diag_obstacle_yaw < 900.0
                and abs(_angle_diff(target_yaw, base._car.current_angle)) > _RECOVER_YAW_REENTER_EPS_DEG):
            _recover_sub_t0 = now_ms
            _recover_orbit_stable_t0 = 0
            _mode_sub = _SUB_RECOVER_ORBIT_YAW_ALIGN
            return
        car_fresh = (
            base._cam_rx_last_ms != 0
            and ticks_diff(now_ms, base._cam_rx_last_ms) <= _RECOVER_FIND_CAR_FRESH_MS
            and _find_car_in_cam() >= 0
        )
        # 前后距离条件不再只在触发斜避时检查：不管本轮是否斜避，都要求
        # 摄像头测得的直线距离收敛到跟斜避同一个目标值和容差
        # （_FOLLOWER_FOLLOW_DIST ± _RECOVER_DIAG_FORM_DIST_EPS）。
        rel_x = 999.0
        rel_y = 999.0
        rel_dist = 999.0
        dist_ok = False
        if car_fresh:
            rel_x = base._cam_car_rel_x
            rel_y = base._cam_car_rel_y
            rel_dist = sqrt(rel_x * rel_x + rel_y * rel_y)
            dist_ok = abs(rel_dist - _FOLLOWER_FOLLOW_DIST) <= _RECOVER_DIAG_FORM_DIST_EPS
        diag_formation_ok = dist_ok
        if _recover_diag_obstacle_yaw < 900.0:
            diag_formation_ok = (
                dist_ok
                and abs(_angle_diff(target_yaw, base._car.current_angle)) <= _RECOVER_TURN_EPS_DEG
            )
        formation_centered = (
            car_fresh
            and abs(rel_x) <= _RECOVER_FIND_CAR_CENTER_X
            and diag_formation_ok
        )
        if formation_centered:
            if _recover_diag_obstacle_yaw >= 900.0:
                # 没有斜避：当前位置就是本轮最终侧阵列，不再为 SEARCH 后方槽位换边。
                _recover_side_formation_yaw = target_yaw
                _recover_side_formation_ready = True
                _recover_force_car_reloc_pending = True
                _finish_recover()
                return
            if _recover_orbit_stable_t0 == 0:
                _recover_orbit_stable_t0 = now_ms
            elif ticks_diff(now_ms, _recover_orbit_stable_t0) >= _RECOVER_DIAG_FORM_STABLE_MS:
                _recover_side_formation_yaw = target_yaw
                _recover_side_formation_ready = True
                _recover_force_car_reloc_pending = True
                _finish_recover()
                return
            base.request_hard_hold(target_yaw)
            return
        else:
            _recover_orbit_stable_t0 = 0
        if car_fresh:
            if rel_x > _RECOVER_FIND_CAR_CENTER_X:
                _recover_translate_side_speed = _RECOVER_TRANSLATE_SPEED
            elif rel_x < -_RECOVER_FIND_CAR_CENTER_X:
                _recover_translate_side_speed = -_RECOVER_TRANSLATE_SPEED
            else:
                _recover_translate_side_speed = 0.0
            # 前后方向同样按 bang-bang 修正到 _FOLLOWER_FOLLOW_DIST±_RECOVER_DIAG_FORM_DIST_EPS，
            # 跟左右修正用同一套阈值风格，避免只居中左右、前后距离却没人管。
            fwd_err = rel_y - _FOLLOWER_FOLLOW_DIST
            if fwd_err > _RECOVER_DIAG_FORM_DIST_EPS:
                _recover_translate_fwd_speed = _RECOVER_TRANSLATE_SPEED
            elif fwd_err < -_RECOVER_DIAG_FORM_DIST_EPS:
                _recover_translate_fwd_speed = -_RECOVER_TRANSLATE_SPEED
            else:
                _recover_translate_fwd_speed = 0.0
            _recover_translate_visual_active = True
        if _recover_translate_visual_active:
            _set_body(_recover_translate_side_speed, _recover_translate_fwd_speed, target_yaw)
            return
        # Before the leader is visible, retain the original fixed field direction.
        move_yaw = _recover_translate_yaw()
        move_rad = radians(move_yaw)
        base.request_world(
            sin(move_rad) * _RECOVER_TRANSLATE_SPEED,
            cos(move_rad) * _RECOVER_TRANSLATE_SPEED,
            target_yaw,
        )
        return

_RELOC_PREMOVE = const(0)

_RELOC_X_TURN = const(1)

_RELOC_X_DRIVE = const(2)

_RELOC_Y_TURN = const(3)

_RELOC_Y_DRIVE = const(4)

_RELOC_FOLLOW_LEADER = const(10)

_RELOC_DONE_ESCAPE = const(11)

# These SEARCH/RELOCALIZE constants are also used by the RELOCALIZE functions
# below.  Keep them before the first function that references them: private
# MicroPython const names are folded at compile time and are not available as
# normal module globals for an earlier function body to look up at runtime.
_FSEARCH_VISUAL_FOLLOW = const(0)

_FOLLOWER_RELOCALIZE_ORBIT_RADIAL_MAX = const(120)

_FOLLOWER_RELOCALIZE_FORM_STABLE_MS = const(50)

_FOLLOWER_RELOCALIZE_LOCK_DIST_ERR = const(15)

# _FOLLOWER_ORBIT_RADIAL_KP 已上移到 RECOVER 小节之前统一定义，这里不再重复。

_mode_sub = _RELOC_FOLLOW_LEADER

_mode_hold_ms = 0

_relocalize_sub_t0 = 0

_premove_start_x = 0.0

_premove_start_y = 0.0

_premove_start_valid = False

_done_escape_axis = 0

_done_escape_start = 0.0

_done_escape_yaw = 0.0

_reloc_x_snap = 0

_reloc_y_snap = 0

_RELOC_DRIVE_SETTLE_MS = const(90)

_RELOC_DRIVE_TIMEOUT_MS = const(5000)

# 黄线 Y 坐标修正允许更长的搜索距离；X 修正仍使用上面的通用超时。
_RELOC_Y_DRIVE_TIMEOUT_MS = const(20000)

_RELOC_FIX_THRESH = const(60)

_RELOC_FIX_REQUIRE = const(2)

_RELOC_SELF_FIX_REQUIRE = const(2)

_x_ok_cnt = 0

_y_ok_cnt = 0

_RELOC_SPEED = const(140)

# X 方向边走边修：读数进入 _RELOC_FIX_THRESH（60）视为触发黄线识别，
# 减速到该速度继续行驶，不停车，等下一次读数确认进入更严的 _RELOC_X_FIX_THRESH_TIGHT
# （40）后直接转向进入 Y 修正。
_RELOC_X_SLOW_SPEED = const(90)

_RELOC_X_FIX_THRESH_TIGHT = const(40)

# Y 视觉值必须先接近当前里程坐标，才允许底层直接写入，防止错误首帧造成坐标瞬移。
_RELOC_Y_SLOW_SPEED = const(90)

_RELOC_Y_FIX_THRESH_TIGHT = const(40)

_RELOC_Y_FIX_REQUIRE = const(3)

_x_slow_armed = False

_y_slow_armed = False

_RELOC_PREMOVE_EPS_X = const(15)

_RELOC_PREMOVE_EPS_Y = const(15)

_RELOC_PREMOVE_TIMEOUT_MS = const(4000)

# 从车收到主车 DONE 时的防碰脱离：0~30°沿世界+X走20cm，>30~90°沿世界+Y走40cm。
# 平移速度与 PREMOVE 相同；1500ms 只用于里程计或底盘异常时防止状态卡死。
_RELOC_DONE_ESCAPE_X_CM = 20.0
_RELOC_DONE_ESCAPE_Y_CM = 40.0
_RELOC_DONE_ESCAPE_TIMEOUT_MS = const(1500)

_TASK_RELOCALIZE_DEPLOY_TURN_EPS_DEG = const(6)

# 与主车使用同一张预设点表：主车取 [0]，从车取 [1]，避免两车驶向同一点。
_RELOCALIZE_PRESET_POS = ((50.0, 50.0), (80.0, 80.0))

# 从车 RELOCALIZE 障碍记忆参数，与主车相同：cone/brick 共用四槽，
# 40 cm 内视为同一障碍，离开视野后继续保留 3000 ms。
_RELOCALIZE_OBS_MEMORY_MS = const(3000)
_RELOCALIZE_OBS_SAME_DIST_CM = 40.0
_RELOCALIZE_OBS_SLOT_MAX = const(4)
_relocalize_obs_x = [999.0] * _RELOCALIZE_OBS_SLOT_MAX
_relocalize_obs_y = [999.0] * _RELOCALIZE_OBS_SLOT_MAX
_relocalize_obs_ts = [0] * _RELOCALIZE_OBS_SLOT_MAX

# RELOCALIZE 跟随段同时失去主车视觉和有效无线坐标后，先等待短暂恢复。
_FOLLOWER_RELOCALIZE_SELF_FALLBACK_MS = const(1500)

_relocalize_peer_lost_t0 = 0

_relocalize_self_active = False

def _relocalize_obstacle_add(x, y, ts, now_ms):
    if x >= 900.0 or y >= 900.0 or ts == 0:
        return
    if ticks_diff(now_ms, ts) > _RELOCALIZE_OBS_MEMORY_MS:
        return
    same_i = -1
    empty_i = -1
    old_i = 0
    old_ts = 0
    same_d2 = _RELOCALIZE_OBS_SAME_DIST_CM * _RELOCALIZE_OBS_SAME_DIST_CM
    for i in range(_RELOCALIZE_OBS_SLOT_MAX):
        slot_ts = _relocalize_obs_ts[i]
        if slot_ts != 0 and ticks_diff(now_ms, slot_ts) > _RELOCALIZE_OBS_MEMORY_MS:
            _relocalize_obs_x[i] = 999.0
            _relocalize_obs_y[i] = 999.0
            _relocalize_obs_ts[i] = 0
            slot_ts = 0
        if slot_ts == 0:
            if empty_i < 0:
                empty_i = i
            continue
        dx = x - _relocalize_obs_x[i]
        dy = y - _relocalize_obs_y[i]
        if dx * dx + dy * dy <= same_d2:
            same_i = i
            break
        if old_ts == 0 or ticks_diff(slot_ts, old_ts) < 0:
            old_i = i
            old_ts = slot_ts
    if same_i >= 0:
        i = same_i
    elif empty_i >= 0:
        i = empty_i
    else:
        i = old_i
    _relocalize_obs_x[i] = x
    _relocalize_obs_y[i] = y
    _relocalize_obs_ts[i] = ts


# 与主车相同：保持原平移速度，固定沿世界坐标 -Y 方向叠加避障速度。
def _relocalize_request_with_repulse(vx, vy, yaw, now_ms):
    if not _RELOCALIZE_AVOID_ENABLE:
        base.avoid_beep(0)
        base.request_world(vx, vy, yaw)
        return
    _relocalize_obstacle_add(base._cam_cone_x, base._cam_cone_y, base._cam_cone_ts, now_ms)
    _relocalize_obstacle_add(base._cam_brick_x, base._cam_brick_y, base._cam_brick_ts, now_ms)
    if vx * vx + vy * vy <= 0.0001:
        base.avoid_beep(0)
        base.request_world(vx, vy, yaw)
        return
    nearest_d2 = _RELOCALIZE_REPULSE_TRIGGER_CM * _RELOCALIZE_REPULSE_TRIGGER_CM
    found = False
    car_x = base._car.Position_X
    car_y = base._car.Position_Y
    for i in range(_RELOCALIZE_OBS_SLOT_MAX):
        ts = _relocalize_obs_ts[i]
        if ts == 0:
            continue
        if ticks_diff(now_ms, ts) > _RELOCALIZE_OBS_MEMORY_MS:
            _relocalize_obs_x[i] = 999.0
            _relocalize_obs_y[i] = 999.0
            _relocalize_obs_ts[i] = 0
            continue
        dx = _relocalize_obs_x[i] - car_x
        dy = _relocalize_obs_y[i] - car_y
        d2 = dx * dx + dy * dy
        if d2 < nearest_d2:
            nearest_d2 = d2
            found = True
    if not found:
        base.avoid_beep(0)
        base.request_world(vx, vy, yaw)
        return
    base.avoid_beep(3)
    base.request_world(vx, vy - _RELOCALIZE_REPULSE_SPEED, yaw)

# 功能：重置从车全局重定位状态机。
# 跟随阶段只用视觉把主车保持在画面中心并维持距离，不锁绝对航向、不绕主车；
# 主车进入 DONE 后，从车解除跟随；危险航向先执行固定方向脱离，再前往自己的预设点，
# 最后依次完成 X/Y 黄线坐标修正。
def relocalize_reset():
    global _mode_sub, _mode_hold_ms, _reloc_x_snap, _reloc_y_snap, _relocalize_sub_t0
    global _premove_start_x, _premove_start_y, _premove_start_valid
    global _done_escape_axis, _done_escape_start, _done_escape_yaw
    global _fsearch_state
    global _x_ok_cnt, _y_ok_cnt, _x_slow_armed, _y_slow_armed
    global _relocalize_peer_lost_t0, _relocalize_self_active
    base.avoid_beep(0)
    _mode_sub = _RELOC_FOLLOW_LEADER
    _mode_hold_ms = 0
    _relocalize_sub_t0 = 0
    _premove_start_x = 0.0
    _premove_start_y = 0.0
    _premove_start_valid = False
    _done_escape_axis = 0
    _done_escape_start = 0.0
    _done_escape_yaw = 0.0
    _reloc_x_snap = 0
    _reloc_y_snap = 0
    _x_ok_cnt = 0
    _y_ok_cnt = 0
    _x_slow_armed = False
    _y_slow_armed = False
    _relocalize_peer_lost_t0 = 0
    _relocalize_self_active = False
    for i in range(_RELOCALIZE_OBS_SLOT_MAX):
        _relocalize_obs_x[i] = 999.0
        _relocalize_obs_y[i] = 999.0
        _relocalize_obs_ts[i] = 0
    base._vis_x_fix_cnt = 0
    base._vis_y_fix_cnt = 0
    # 复用 SEARCH 的视觉滤波和 face 参数初始化，但 RELOCALIZE 不进入 ORBIT，
    # 也不使用 _search_yaw 锁定绝对航向。
    _reset_search()
    _fsearch_state = _FSEARCH_VISUAL_FOLLOW

# 功能：结束全局重定位。
# 会关闭视觉修正开关、清空当前目标，并请求进入 DONE 模式。
def _finish_relocalize():
    global _next_task_mode, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y
    base.avoid_beep(0)
    base._vis_yaw_fix_en = False
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False
    _target_obj_world_x = 999.0
    _target_obj_world_y = 999.0
    _target_edge = 0
    _target_obj_id = 0
    gc.collect()
    _next_task_mode = _MODE_DONE


def _relocalize_car_visual_available(now_ms):
    return (
        base._cam_rx_last_ms != 0
        and ticks_diff(now_ms, base._cam_rx_last_ms) <= _CAM_LOST_TOL_MS
        and _find_car_in_cam() >= 0
    )


def _relocalize_peer_reference_available(now_ms):
    if _relocalize_car_visual_available(now_ms):
        return True
    return (
        base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _FOLLOWER_SEARCH_FF_FRESH_MS
        and base._Other_Car_X < 900.0
        and base._Other_Car_Y < 900.0
    )


# 功能：RELOCALIZE 前半段的简化跟车控制。
# 有视觉时用 face 把主车保持在画面中心，并用径向平移保持 30 cm 距离；
# 无线数据新鲜时叠加主车速度前馈。没有视觉但无线坐标仍新鲜时，用两车
# 世界坐标估算方位和距离继续跟随、争取重新看到主车。两种参考都失效时原地找车。
# ready=2 只要求视觉主车居中、距离合格且无线状态新鲜，不检查绝对航向。
def _relocalize_follow_leader(now_ms):
    global _search_form_stable_t0, _self_ready, _wireless_car_seen
    visual_ok = _relocalize_car_visual_available(now_ms)
    peer_fresh = (
        base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _FOLLOWER_SEARCH_FF_FRESH_MS
        and base._Other_Car_X < 900.0
        and base._Other_Car_Y < 900.0
    )

    if visual_ok:
        _wireless_car_seen = True
        rel_x = base._cam_car_rel_x
        rel_y = base._cam_car_rel_y
    elif peer_fresh:
        dx = base._Other_Car_X - base._car.Position_X
        dy = base._Other_Car_Y - base._car.Position_Y
        yaw_rad = radians(base._car.current_angle)
        c = cos(yaw_rad)
        s = sin(yaw_rad)
        # 世界向量 -> 从车车体系，用无线位置估算主车在画面中的方向。
        rel_x = c * dx - s * dy
        rel_y = s * dx + c * dy
    else:
        base.clear_face()
        _self_ready = 0
        _search_form_stable_t0 = 0
        base.request_yaw_rate(_FOLLOWER_SEARCH_LINK_LOST_SPIN_DPS)
        return

    rel_dist = sqrt(rel_x * rel_x + rel_y * rel_y)
    bearing_err = degrees(atan2(rel_x, rel_y))
    _search_face_yaw_from_bearing(bearing_err)
    base._face_req_err = _search_bearing_filt if _search_face_correcting else 0.0
    base._face_req_active = 1
    base._face_req_seq += 1

    radial_vx, radial_vy, unused = _search_distance_adjust(
        rel_x,
        rel_y,
        _FOLLOWER_ORBIT_RADIAL_KP,
        _FOLLOWER_RELOCALIZE_ORBIT_RADIAL_MAX,
        _FOLLOWER_RELOCALIZE_FOLLOW_DIST,
    )
    if peer_fresh:
        ff_x, ff_y = _search_leader_feedforward(True)
    else:
        ff_x = 0.0
        ff_y = 0.0
    cmd_x = ff_x + radial_vx
    cmd_y = ff_y + radial_vy
    cmd_mag = sqrt(cmd_x * cmd_x + cmd_y * cmd_y)
    if cmd_mag > _FOLLOWER_RELOCALIZE_TOTAL_MAX:
        scale = _FOLLOWER_RELOCALIZE_TOTAL_MAX / cmd_mag
        cmd_x *= scale
        cmd_y *= scale
    # face 控制直接接管 Z 轴；普通角度环只保持当前角，不设固定世界航向。
    _relocalize_request_with_repulse(cmd_x, cmd_y, base._car.current_angle, now_ms)

    stable = (
        visual_ok
        and peer_fresh
        and abs(_search_bearing_filt) <= _FOLLOWER_SEARCH_FACE_EXIT
        and abs(rel_dist - _FOLLOWER_RELOCALIZE_FOLLOW_DIST) <= _FOLLOWER_RELOCALIZE_LOCK_DIST_ERR
    )
    if not stable:
        _self_ready = 0
        _search_form_stable_t0 = 0
        return
    if _search_form_stable_t0 == 0:
        _search_form_stable_t0 = now_ms
        _self_ready = 0
        return
    if ticks_diff(now_ms, _search_form_stable_t0) >= _FOLLOWER_RELOCALIZE_FORM_STABLE_MS:
        _self_ready = 2
    else:
        _self_ready = 0


def _relocalize_begin_self(now_ms, check_done_escape=False):
    global _mode_sub, _relocalize_sub_t0, _relocalize_peer_lost_t0
    global _premove_start_x, _premove_start_y, _premove_start_valid
    global _done_escape_axis, _done_escape_start, _done_escape_yaw
    global _reloc_x_snap, _reloc_y_snap
    global _x_ok_cnt, _y_ok_cnt, _relocalize_self_active, _self_ready, _self_sub
    global _x_slow_armed, _y_slow_armed
    _relocalize_self_active = True
    _relocalize_peer_lost_t0 = 0
    _relocalize_sub_t0 = 0
    _premove_start_x = 0.0
    _premove_start_y = 0.0
    _premove_start_valid = False
    _done_escape_axis = 0
    _done_escape_start = 0.0
    _done_escape_yaw = base._car.current_angle
    _reloc_x_snap = base._vis_x_fix_cnt
    _reloc_y_snap = base._vis_y_fix_cnt
    _x_ok_cnt = 0
    _y_ok_cnt = 0
    _x_slow_armed = False
    _y_slow_armed = False
    _self_ready = 0
    base.clear_face()
    base._vis_yaw_fix_en = False
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False
    if check_done_escape:
        yaw_360 = _done_escape_yaw % 360.0
        if 0.0 <= yaw_360 <= 30.0:
            _done_escape_axis = 1
            _done_escape_start = base._car.Position_X
            _relocalize_sub_t0 = now_ms
            _mode_sub = _RELOC_DONE_ESCAPE
            _self_sub = _mode_sub
            base.request_world(_RELOC_PREMOVE_SPEED, 0.0, _done_escape_yaw)
            return
        if 30.0 < yaw_360 <= 90.0:
            _done_escape_axis = 2
            _done_escape_start = base._car.Position_Y
            _relocalize_sub_t0 = now_ms
            _mode_sub = _RELOC_DONE_ESCAPE
            _self_sub = _mode_sub
            base.request_world(0.0, _RELOC_PREMOVE_SPEED, _done_escape_yaw)
            return
    _mode_sub = _RELOC_X_TURN
    _self_sub = _mode_sub
    base.request_world(0.0, 0.0, 270.0)


def _relocalize_begin_self_x_drive(now_ms):
    global _mode_sub, _relocalize_sub_t0, _reloc_x_snap, _x_ok_cnt, _self_sub
    global _premove_start_valid, _x_slow_armed
    _relocalize_sub_t0 = now_ms
    _premove_start_valid = False
    _reloc_x_snap = base._vis_x_fix_cnt
    _x_ok_cnt = 0
    _x_slow_armed = False
    base._vis_x_fix_en = True
    base._vis_y_fix_en = False
    _mode_sub = _RELOC_X_DRIVE
    _self_sub = _mode_sub
    _relocalize_request_with_repulse(-_RELOC_SPEED, 0.0, 270.0, now_ms)

# 功能：更新从车 RELOCALIZE 模式。
# 主车整个 RELOCALIZE 期间，从车只保持主车居中和距离，不锁角、不绕行；收到
# 主车进入 DONE 后才解除跟随；航向0~90°的危险区间先按分段规则脱离，之后
# 前往自己的预设点，再依次执行自身 X/Y 黄线修正。
def relocalize_update():
    global _self_sub
    global _mode_sub, _mode_hold_ms, _reloc_x_snap, _reloc_y_snap, _relocalize_sub_t0
    global _self_ready, _x_ok_cnt, _y_ok_cnt, _x_slow_armed, _y_slow_armed
    global _premove_start_x, _premove_start_y, _premove_start_valid
    global _recover_force_car_reloc_pending
    global _relocalize_peer_lost_t0
    now = ticks_ms()
    base.avoid_beep(0)
    _self_sub = _mode_sub
    if _mode_sub == _RELOC_FOLLOW_LEADER:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        peer_done = (
            base._Other_Car_Mode == _MODE_DONE
        ) and (
            base._Other_Car_Ready_Ts != 0
            and ticks_diff(now, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
        )
        if peer_done:
            base.clear_face()
            _self_ready = 0
            _recover_force_car_reloc_pending = False
            _relocalize_begin_self(now, True)
            return
        if _relocalize_peer_reference_available(now):
            _relocalize_peer_lost_t0 = 0
        else:
            if _relocalize_peer_lost_t0 == 0:
                _relocalize_peer_lost_t0 = now
            elif ticks_diff(now, _relocalize_peer_lost_t0) >= _FOLLOWER_RELOCALIZE_SELF_FALLBACK_MS:
                _recover_force_car_reloc_pending = False
                _relocalize_begin_self(now)
                return
        _relocalize_follow_leader(now)
        return
    elif _mode_sub == _RELOC_DONE_ESCAPE:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        elapsed = ticks_diff(now, _relocalize_sub_t0)
        if _done_escape_axis == 1:
            moved = base._car.Position_X - _done_escape_start
            done = moved >= _RELOC_DONE_ESCAPE_X_CM
        elif _done_escape_axis == 2:
            moved = base._car.Position_Y - _done_escape_start
            done = moved >= _RELOC_DONE_ESCAPE_Y_CM
        else:
            done = True
        if done or elapsed >= _RELOC_DONE_ESCAPE_TIMEOUT_MS:
            _relocalize_sub_t0 = 0
            _mode_sub = _RELOC_X_TURN
            _self_sub = _mode_sub
            base.request_world(0.0, 0.0, 270.0)
            return
        if _done_escape_axis == 1:
            base.request_world(_RELOC_PREMOVE_SPEED, 0.0, _done_escape_yaw)
        else:
            base.request_world(0.0, _RELOC_PREMOVE_SPEED, _done_escape_yaw)
        return
    elif _mode_sub == _RELOC_PREMOVE:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        tx, ty = _RELOCALIZE_PRESET_POS[1]
        if _relocalize_sub_t0 == 0:
            _relocalize_sub_t0 = now
        if not _premove_start_valid:
            _premove_start_x = base._car.Position_X
            _premove_start_y = base._car.Position_Y
            _premove_start_valid = True

        dx = tx - base._car.Position_X
        dy = ty - base._car.Position_Y
        distance = sqrt(dx * dx + dy * dy)
        arrived = _near(base._car.Position_X, tx, _RELOC_PREMOVE_EPS_X) and _near(
            base._car.Position_Y, ty, _RELOC_PREMOVE_EPS_Y
        )

        # 与主车相同：即使编码器量化让车辆跳过矩形到点框，只要已经穿过
        # 初始目标方向的目标垂线，也视为完成 PREMOVE，避免继续反向追点。
        start_dx = tx - _premove_start_x
        start_dy = ty - _premove_start_y
        start_distance = sqrt(start_dx * start_dx + start_dy * start_dy)
        crossed_target = False
        if start_distance > 1.0:
            progress = (
                (base._car.Position_X - _premove_start_x) * start_dx
                + (base._car.Position_Y - _premove_start_y) * start_dy
            ) / start_distance
            crossed_target = progress >= start_distance

        timed_out = ticks_diff(now, _relocalize_sub_t0) > _RELOC_PREMOVE_TIMEOUT_MS
        if arrived or crossed_target or timed_out or distance <= 1.0:
            _relocalize_begin_self_x_drive(now)
            return

        _relocalize_request_with_repulse(
            _RELOC_PREMOVE_SPEED * dx / distance,
            _RELOC_PREMOVE_SPEED * dy / distance,
            270.0,
            now,
        )
    elif _mode_sub == _RELOC_X_TURN:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        base.request_world(0.0, 0.0, 270.0)
        err = _angle_diff(base._car.current_angle, 270.0)
        if abs(err) < _TASK_RELOCALIZE_DEPLOY_TURN_EPS_DEG:
            tx, ty = _RELOCALIZE_PRESET_POS[1]
            dx = tx - base._car.Position_X
            dy = ty - base._car.Position_Y
            distance = sqrt(dx * dx + dy * dy)
            arrived = _near(base._car.Position_X, tx, _RELOC_PREMOVE_EPS_X) and _near(
                base._car.Position_Y, ty, _RELOC_PREMOVE_EPS_Y
            )
            if arrived or distance <= 1.0:
                _relocalize_begin_self_x_drive(now)
            else:
                _relocalize_sub_t0 = now
                _premove_start_x = base._car.Position_X
                _premove_start_y = base._car.Position_Y
                _premove_start_valid = True
                _mode_sub = _RELOC_PREMOVE
                _self_sub = _mode_sub
                # 转向进入 6° 范围后不停车，当周期直接开始锁 270° PREMOVE。
                _relocalize_request_with_repulse(
                    _RELOC_PREMOVE_SPEED * dx / distance,
                    _RELOC_PREMOVE_SPEED * dy / distance,
                    270.0,
                    now,
                )
    elif _mode_sub == _RELOC_X_DRIVE:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = True
        base._vis_y_fix_en = False
        drive_speed = _RELOC_X_SLOW_SPEED if _x_slow_armed else _RELOC_SPEED
        _relocalize_request_with_repulse(-drive_speed, 0.0, 270.0, now)
        if _relocalize_sub_t0 == 0:
            _relocalize_sub_t0 = now
        elapsed = ticks_diff(now, _relocalize_sub_t0)
        if elapsed >= _RELOC_DRIVE_TIMEOUT_MS:
            base._vis_x_fix_en = False
            _relocalize_sub_t0 = 0
            _x_ok_cnt = 0
            _x_slow_armed = False
            _finish_relocalize()
            return
        if elapsed >= _RELOC_DRIVE_SETTLE_MS:
            if base._vis_x_fix_cnt != _reloc_x_snap:
                _reloc_x_snap = base._vis_x_fix_cnt
                if not _x_slow_armed:
                    # 首次读数进入黄线阈值：触发减速，边走边等下一次读数
                    # 做更严的确认，不停车。
                    if base._Position_X_fix < _RELOC_FIX_THRESH:
                        _x_slow_armed = True
                elif base._Position_X_fix < _RELOC_X_FIX_THRESH_TIGHT:
                    _x_ok_cnt += 1
                else:
                    _x_ok_cnt = 0
            if _x_slow_armed and _x_ok_cnt >= _RELOC_SELF_FIX_REQUIRE:
                # 减速后确认黄线识别在 40cm 以内，边走边修已经完成一次坐标
                # 修正，不停车、不做静止态复核，直接转向进入 Y 修正。
                _relocalize_sub_t0 = 0
                _x_ok_cnt = 0
                _x_slow_armed = False
                base._vis_x_fix_en = False
                _mode_sub = _RELOC_Y_TURN
                _self_sub = _mode_sub
                base.request_world(0.0, 0.0, 180.0)
    elif _mode_sub == _RELOC_Y_TURN:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        base.request_world(0.0, 0.0, 180.0)
        err = _angle_diff(base._car.current_angle, 180.0)
        if abs(err) < _TASK_RELOCALIZE_DEPLOY_TURN_EPS_DEG:
            _relocalize_sub_t0 = now
            _reloc_y_snap = base._vis_y_fix_cnt
            _y_ok_cnt = 0
            _y_slow_armed = False
            base._vis_y_fix_en = False
            _mode_sub = _RELOC_Y_DRIVE
            _self_sub = _mode_sub
            # 进入 6 度范围后不停车，当周期直接向低 Y 前进。
            _relocalize_request_with_repulse(0.0, -_RELOC_SPEED, 180.0, now)
    elif _mode_sub == _RELOC_Y_DRIVE:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        y_fix = base._Position_Y_fix
        base._vis_y_fix_en = y_fix < _RELOC_FIX_THRESH
        drive_speed = _RELOC_Y_SLOW_SPEED if _y_slow_armed else _RELOC_SPEED
        _relocalize_request_with_repulse(0.0, -drive_speed, 180.0, now)
        if _relocalize_sub_t0 == 0:
            _relocalize_sub_t0 = now
        elapsed = ticks_diff(now, _relocalize_sub_t0)
        if elapsed >= _RELOC_Y_DRIVE_TIMEOUT_MS:
            _relocalize_sub_t0 = 0
            _y_ok_cnt = 0
            _y_slow_armed = False
            _finish_relocalize()
            return
        if elapsed < _RELOC_DRIVE_SETTLE_MS:
            return
        if base._vis_y_fix_cnt != _reloc_y_snap:
            _reloc_y_snap = base._vis_y_fix_cnt
            if not _y_slow_armed:
                if base._Position_Y_fix < _RELOC_FIX_THRESH:
                    _y_slow_armed = True
            elif base._Position_Y_fix < _RELOC_Y_FIX_THRESH_TIGHT:
                _y_ok_cnt += 1
            else:
                _y_ok_cnt = 0
        if not _y_slow_armed or _y_ok_cnt < _RELOC_Y_FIX_REQUIRE:
            return
        # Y 行进中的视觉修正已经确认合格；后面不需要再转向，因此不停车、
        # 不做静止态复核，直接切入 DONE 回库运动。
        _relocalize_sub_t0 = 0
        _y_ok_cnt = 0
        _y_slow_armed = False
        _finish_relocalize()
        return

_DONE_EPS_X = const(2)

_DONE_EPS_Y = const(2)

_DONE_TRANSLATE = const(0)

_DONE_TRACK = const(1)

_DONE_ORBIT = const(2)

_DONE_HOLD = const(3)

_DONE_TURN_EPS_DEG = const(6)
_DONE_SEARCH_DPS = const(180.0)
_DONE_CAR_FRESH_MS = const(250)
_DONE_FORM_STABLE_MS = const(200)
_DONE_FORM_CENTER_X = const(10)
_DONE_FORM_DIST_EPS = const(7)
_DONE_FOLLOW_DIST = const(26.0)
_DONE_RADIAL_DEADBAND = const(5)
_DONE_RADIAL_KP = const(4)
_DONE_RADIAL_MAX = const(60)
_DONE_RADIAL_OUT_MAX = const(60)
_DONE_TOTAL_MAX = const(120)
_DONE_ORBIT_SPEED = const(40.0)
_DONE_ORBIT_DIR = const(1.0)
_DONE_ORBIT_TOTAL_DEG = const(390.0)
_DONE_READY = const(6)
_DONE_FINISHED_READY = const(7)
_DONE_BEARING_BLEND = const(0.35)

_mode_sub = _DONE_TRANSLATE

_mode_hold_ms = 0

_done_form_stable_t0 = 0
_done_bearing_filt = 0.0
_done_bearing_filt_valid = False
_done_orbit_accum = 0.0
_done_orbit_prev_yaw = 0.0
_done_hold_yaw = 0.0


def _done_reset_relative_state():
    global _done_form_stable_t0
    global _done_bearing_filt, _done_bearing_filt_valid
    _done_form_stable_t0 = 0
    _done_bearing_filt = 0.0
    _done_bearing_filt_valid = False


def _done_car_fresh(now_ms):
    return (
        base._cam_rx_last_ms != 0
        and ticks_diff(now_ms, base._cam_rx_last_ms) <= _DONE_CAR_FRESH_MS
        and _find_car_in_cam() >= 0
    )


def _done_peer_fresh(now_ms):
    return (
        base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
        and base._Other_Car_Mode == _MODE_DONE
    )


def _done_request_relative(rel_x, rel_y, orbit):
    global _done_bearing_filt, _done_bearing_filt_valid
    distance = sqrt(rel_x * rel_x + rel_y * rel_y)
    raw_bearing = degrees(atan2(rel_x, rel_y))
    if not _done_bearing_filt_valid:
        _done_bearing_filt = raw_bearing
        _done_bearing_filt_valid = True
    else:
        _done_bearing_filt += _DONE_BEARING_BLEND * _angle_diff(
            raw_bearing, _done_bearing_filt
        )
        _done_bearing_filt = (_done_bearing_filt + 180.0) % 360.0 - 180.0
    bearing = _done_bearing_filt
    base._face_req_err = bearing
    base._face_req_active = 1
    base._face_req_seq += 1

    error = distance - _DONE_FOLLOW_DIST
    if abs(error) <= _DONE_RADIAL_DEADBAND:
        radial_speed = 0.0
    else:
        radial_speed = _DONE_RADIAL_KP * error
        radial_max = _DONE_RADIAL_OUT_MAX if error < 0.0 else _DONE_RADIAL_MAX
        if radial_speed > radial_max:
            radial_speed = radial_max
        elif radial_speed < -radial_max:
            radial_speed = -radial_max

    yaw = radians(base._car.current_angle)
    c = cos(yaw)
    s = sin(yaw)
    toward_x = c * rel_x + s * rel_y
    toward_y = -s * rel_x + c * rel_y
    radius = max(distance, 1.0)
    unit_x = toward_x / radius
    unit_y = toward_y / radius
    cmd_x = unit_x * radial_speed
    cmd_y = unit_y * radial_speed
    if orbit:
        cmd_x -= _DONE_ORBIT_DIR * unit_y * _DONE_ORBIT_SPEED
        cmd_y += _DONE_ORBIT_DIR * unit_x * _DONE_ORBIT_SPEED
    cmd_mag = sqrt(cmd_x * cmd_x + cmd_y * cmd_y)
    if cmd_mag > _DONE_TOTAL_MAX:
        scale = _DONE_TOTAL_MAX / cmd_mag
        cmd_x *= scale
        cmd_y *= scale
    base.request_world(cmd_x, cmd_y, base._car.current_angle)
    return distance, bearing


def _done_enter_track():
    global _mode_sub, _self_ready
    global _done_orbit_accum, _done_orbit_prev_yaw
    base.clear_face()
    _mode_sub = _DONE_TRACK
    _self_ready = 0
    _done_orbit_accum = 0.0
    _done_orbit_prev_yaw = base._car.current_angle
    _done_reset_relative_state()


# 功能：重置 DONE 模式状态。
# DONE 阶段从车回到自己的起始点并保持当前锁定角，作为任务结束后的停靠状态。
def done_reset():
    global _mode_sub, _mode_hold_ms, _self_ready
    global _done_orbit_accum, _done_orbit_prev_yaw, _done_hold_yaw
    _mode_sub = _DONE_TRANSLATE
    _mode_hold_ms = 0
    _self_ready = 0
    _done_orbit_accum = 0.0
    _done_orbit_prev_yaw = 0.0
    _done_hold_yaw = base._car.current_angle
    _done_reset_relative_state()
    base.clear_face()
    base.configure_face(_KP_FACE_OBJ, _KD_FACE_OBJ, _FACE_GYRO_MAX)

# 功能：更新 DONE 模式。
# 从车移动回自己的起始点附近，到达后保持锁定角不再执行新的任务目标。
# 输入参数：now_ms 为当前时间戳。
def done_update(now_ms):
    global _mode_sub, _mode_hold_ms, _self_ready
    global _done_form_stable_t0
    global _done_orbit_accum, _done_orbit_prev_yaw, _done_hold_yaw
    tx, ty = config._DONE_TARGET_POS[1]
    if _mode_sub == _DONE_TRANSLATE:
        base.request_pos(tx, ty, 200.0, _done_hold_yaw)
        arrived = _near(base._car.Position_X, tx, _DONE_EPS_X) and _near(base._car.Position_Y, ty, _DONE_EPS_Y)
        if arrived:
            base.request_hold(_done_hold_yaw)
            if _mode_hold_ms == 0:
                _mode_hold_ms = now_ms
            if ticks_diff(now_ms, _mode_hold_ms) > 300:
                _done_enter_track()
        else:
            _mode_hold_ms = 0
        return
    if _mode_sub == _DONE_TRACK:
        if not _done_car_fresh(now_ms):
            _done_enter_track()
            base.request_yaw_rate(_DONE_SEARCH_DPS)
            return

        rel_x = base._cam_car_rel_x
        rel_y = base._cam_car_rel_y
        distance, bearing = _done_request_relative(rel_x, rel_y, False)
        stable = (
            _done_peer_fresh(now_ms)
            and abs(rel_x) <= _DONE_FORM_CENTER_X
            and abs(distance - _DONE_FOLLOW_DIST) <= _DONE_FORM_DIST_EPS
            and abs(bearing) <= _DONE_TURN_EPS_DEG
        )
        if not stable:
            _done_form_stable_t0 = 0
            _self_ready = 0
            return
        if _done_form_stable_t0 == 0:
            _done_form_stable_t0 = now_ms
            _self_ready = 0
            return
        if ticks_diff(now_ms, _done_form_stable_t0) < _DONE_FORM_STABLE_MS:
            _self_ready = 0
            return

        _self_ready = _DONE_READY
        if base._Other_Car_Ready >= _DONE_READY:
            _mode_sub = _DONE_ORBIT
            _done_orbit_accum = 0.0
            _done_orbit_prev_yaw = base._car.current_angle
        return
    if _mode_sub == _DONE_ORBIT:
        peer_ok = _done_peer_fresh(now_ms) and base._Other_Car_Ready >= _DONE_READY
        if not _done_car_fresh(now_ms) or not peer_ok:
            _done_enter_track()
            base.request_yaw_rate(_DONE_SEARCH_DPS)
            return

        _self_ready = _DONE_READY
        _done_request_relative(base._cam_car_rel_x, base._cam_car_rel_y, True)
        yaw_step = _angle_diff(base._car.current_angle, _done_orbit_prev_yaw)
        _done_orbit_prev_yaw = base._car.current_angle
        if _DONE_ORBIT_DIR > 0.0:
            if yaw_step > 0.0:
                _done_orbit_accum += yaw_step
        elif yaw_step < 0.0:
            _done_orbit_accum -= yaw_step
        if _done_orbit_accum >= _DONE_ORBIT_TOTAL_DEG:
            _done_hold_yaw = base._car.current_angle
            base.clear_face()
            base.request_hard_hold(_done_hold_yaw)
            _self_ready = _DONE_FINISHED_READY
            _mode_sub = _DONE_HOLD
        return
    if _mode_sub == _DONE_HOLD:
        base.clear_face()
        base.request_hard_hold(_done_hold_yaw)

_SEARCH_FACE_CENTER = const(0)



# SEARCH 阶段摄像头到主车识别中心的目标距离 _FOLLOWER_FOLLOW_DIST
# 已上移到文件顶端的"现场常调参数"区。

_FOLLOWER_FOLLOW_SPEED = const(160)

# RELOCALIZE 简化跟车只保留下列距离、径向限幅、合速度、稳定时间和到位误差参数；
# 原先复用 SEARCH ORBIT 的轨道速度、精调速度和无线位置跟随速度已经删除。
_FOLLOWER_RELOCALIZE_FOLLOW_DIST = 30.0
_FOLLOWER_RELOCALIZE_TOTAL_MAX = 240.0

_FOLLOWER_VISUAL_FOLLOW_DIST_KP = const(7)

_FOLLOWER_VISUAL_FOLLOW_FWD_MAX = const(100)

# SEARCH 队形的纵向位置反馈。误差乘以 3.0 得到修正速度，并限制在 45；
# 主车无线世界速度先转换到从车车体系。ORBIT 与 S5 平移跟随分别配置前馈增益。
_FOLLOWER_SEARCH_LONG_KP = 3.0
_FOLLOWER_SEARCH_LONG_MAX = 45.0
# S1/ORBIT 的移动圆心前馈已上移到文件顶端，供 SEARCH/RECOVER 共用。
# S5 平移跟随（主车 Sweep/Forward）：左右 1.3，前后 1.0。
_FOLLOWER_SEARCH_FF_LAT_GAIN = 1.15
_FOLLOWER_SEARCH_FF_FWD_GAIN = 1.0
# S4 横扫的前后耦合方向稳定但幅度不对称：正向横扫会落后，反向横扫时
# 普通距离反馈已经足够。只给正向一个较小的 10% 初始补偿，反向保持 0，
# 避免重现第七次对称 20% 补偿在反向横扫中造成的丢视野。
_FOLLOWER_SEARCH_SWEEP_CROSS_FWD_POS_GAIN = -0.1
_FOLLOWER_SEARCH_SWEEP_CROSS_FWD_NEG_GAIN = 0.00
# RELOCALIZE 保留独立统一增益，后续调 SEARCH 两轴时不会连带改变重定位跟随。
_FOLLOWER_RELOCALIZE_FF_GAIN = 1.1
_FOLLOWER_SEARCH_TOTAL_MAX = 240.0

# 主车无线速度超过 500 ms 未更新即停止使用前馈，避免沿用陈旧速度指令。
_FOLLOWER_SEARCH_FF_FRESH_MS = 500

# SEARCH/RELOCALIZE 同时丢失主车视觉和有效无线坐标时的原地搜索角速度。
_FOLLOWER_SEARCH_LINK_LOST_SPIN_DPS = 90.0

# 从车看向主车时当前使用 6/6 度进入/退出阈值，相当于单一 6 度门限。
_FOLLOWER_SEARCH_FACE_ENTER = 6.0
_FOLLOWER_SEARCH_FACE_EXIT = 6.0
# SEARCH/RECOVER 实际径向距离控制死区已上移到文件顶端。
# 动态跟随时的画面横向闭环参数。主车识别中心偏差在 2 cm 内不修正；超出后
# 按 3.0 倍增益生成横移速度，并限制在 55，避免横向纠偏过猛。
_FOLLOWER_SEARCH_CENTER_DB = 2.0
_FOLLOWER_SEARCH_CENTER_KP = 3.0
_FOLLOWER_SEARCH_CENTER_MAX = 55.0

_FOLLOWER_SEARCH_FORM_YAW_EPS = 6.0

# 普通 SEARCH 首次编队先用主从车世界坐标判断是否到达主车正后方。
# search_yaw 为 0/180 度时比较 X，90/270 度时比较 Y。
_FOLLOWER_SEARCH_WORLD_REAR_LAT_EPS = 7.0

# 进入 REAR_YAW_ALIGN 前，主车沿 search_yaw 正前方必须至少领先从车该距离。
# 该门只防止两车前后距离太近时提前原地转向；最终 25 cm 跟随距离仍由 S5 收敛。
_FOLLOWER_SEARCH_WORLD_REAR_FWD_MIN = const(15)

# 横向世界坐标进入 6 cm 后先急停，再由独立 ALIGN 状态原地修正 search_yaw。
# 死区淡入区间倍数：|err| 从 eps 到 eps*该值之间，增益由 0 线性升到 Kp。
_SEARCH_DEADBAND_FADE = 1.5

_FOLLOWER_SEARCH_TRANSLATE_LAT_KP = 6.0
# 前后距离反馈分成两套增益：rel_y 小于目标距离表示从车离主车太近，
# 使用 TOO_CLOSE_KP 后退拉开；rel_y 大于目标距离时使用 TOO_FAR_KP 前进追赶。
# 太近方向使用更强的增益快速后退拉开，太远方向保持较柔和的前进追赶。
_FOLLOWER_SEARCH_TRANSLATE_FWD_TOO_CLOSE_KP = 7.0
_FOLLOWER_SEARCH_TRANSLATE_FWD_TOO_FAR_KP = 7.0
_FOLLOWER_SEARCH_TRANSLATE_LAT_MAX = 70.0
_FOLLOWER_SEARCH_TRANSLATE_FWD_MAX = 80.0
_FOLLOWER_SEARCH_TRANSLATE_LAT_EPS = 8.0
_FOLLOWER_SEARCH_TRANSLATE_FWD_EPS = 8.0
# ready=2 进入使用上面的严格阈值；已经 ready=2 后使用稍宽的释放阈值，
# 避免动态跟随中的单帧视觉抖动立即撤销 ready。
_FOLLOWER_SEARCH_READY_RELEASE_LAT_EPS = 10.0
_FOLLOWER_SEARCH_READY_RELEASE_FWD_EPS = 10.0
_FOLLOWER_SEARCH_READY_RELEASE_YAW_EPS = 8.0
_FOLLOWER_SEARCH_TRANSLATE_STABLE_MS = const(200)
_FOLLOWER_SEARCH_REORBIT_BEARING = 60.0
_FOLLOWER_SEARCH_REORBIT_HOLD_MS = const(200)

# 绕行速度/减速/半径参数（_FOLLOWER_ORBIT_LEADER_SPD 等）已上移到 RECOVER
# 小节之前统一定义，RECOVER 直接引用同一份常量，这里不再重复。

# 7.14 SEARCH 抑振修改：视觉方位采用0.35低通权重，并把每个SEARCH周期
# 的目标航向变化限制为3度，避免检测框轻微横跳直接变成车头左右摆动。
_FOLLOWER_SEARCH_BEARING_BLEND = 0.35
_FOLLOWER_SEARCH_FACE_STEP_MAX_DEG = 3.0

_fs_orbit_valid = False

_fs_orbit_target_phi = 0.0

_mode_sub = _SEARCH_FACE_CENTER

# SEARCH 航向迟滞状态。False 表示主车已在画面中央容差内；True 表示正在
# 主动修正朝向。该状态跨控制周期保存，才能形成 ENTER/EXIT 两级迟滞。
_search_face_correcting = False
_search_bearing_filt_valid = False
_search_bearing_filt = 0.0
_search_face_yaw_cmd_valid = False
_search_face_yaw_cmd = 0.0

# 从车普通 SEARCH 的显式编队子状态。RELOCALIZE 仅在初始化视觉滤波时把
# 状态置为 VISUAL_FOLLOW，运行控制由独立的 _relocalize_follow_leader 完成。
_FSEARCH_ORBIT_FORM = const(1)
_FSEARCH_REAR_YAW_ALIGN = const(2)
_FSEARCH_LINK_LOST = const(3)
_FSEARCH_TRANSLATE_FORM = const(5)
_LEADER_SEARCH_SWEEP_SUB = const(4)
_fsearch_state = _FSEARCH_VISUAL_FOLLOW

_search_form_stable_t0 = 0
_search_bad_geometry_t0 = 0
# 首次形成主车正后方阵列后锁存；本轮后续的位置纠偏只做平移，车头始终锁定 _search_yaw。
_search_formation_locked = False
_search_orbit_dir = 1.0
_search_orbit_prev_remaining = 360.0
_search_orbit_fine_correcting = False
_search_approach_seen_cnt = 0

# S5 横扫提前停车锁存。锁存轴使用世界坐标：0=未锁，1=X，2=Y；方向为 -1/+1。
# 搜索矩形与主车 task_leader.py 的 _SEARCH_AREA_* 保持一致。
_SEARCH_SWEEP_STOP_AXIS_NONE = const(0)
_SEARCH_SWEEP_STOP_AXIS_X = const(1)
_SEARCH_SWEEP_STOP_AXIS_Y = const(2)
_FOLLOWER_SEARCH_AREA_X_MIN = const(100)
_FOLLOWER_SEARCH_AREA_X_MAX = const(220)
_FOLLOWER_SEARCH_AREA_Y_MIN = const(60)
_FOLLOWER_SEARCH_AREA_Y_MAX = const(180)
_search_sweep_stop_axis = _SEARCH_SWEEP_STOP_AXIS_NONE
_search_sweep_stop_dir = 0

# 本轮 SEARCH 的统一目标航向。该值由从车在进入 SEARCH 时按与主车相同的规则独立计算。
_search_yaw = 0.0

# SEARCH 跟随主车切入 APPROACH 时锁存本车搜索航向。后续 APPROACH
# 因目标丢失退回 SEARCH 时恢复本值，保留主从车原有的相对航向关系。
_search_resume_yaw = 0.0
_search_resume_yaw_valid = False

# RECOVER 后整轮 SEARCH 直接继承 RECOVER 最终侧阵列。Frame3 flags.bit4~5
# 传递“持续保持该阵列”，不再传 LEFT/RIGHT，也不再按横扫方向重新选边。
_search_normal_yaw = 0.0

# 从车 SEARCH 中看到主车已进入 APPROACH 后，连续确认几帧再切入 APPROACH。
# 这能过滤无线/显示时序里的单帧旧模式，避免从车提前跳 13 让主车仍卡在 12。
_SEARCH_APPROACH_CONFIRM_FRAMES = const(1)

# 功能：重置从车连续搜索编队控制。
# 正常 RECOVER 后继承已检查的侧阵列；其他入口才绕到普通搜索后方槽位。
def _reset_search():
    global _search_first_boot, _search_yaw, _search_normal_yaw
    global _search_resume_yaw_valid
    global _fs_orbit_valid
    global _fs_orbit_target_phi
    global _search_face_correcting
    global _search_bearing_filt_valid, _search_bearing_filt
    global _search_face_yaw_cmd_valid, _search_face_yaw_cmd
    global _fsearch_state, _search_form_stable_t0, _search_bad_geometry_t0
    global _search_formation_locked, _self_ready, _boot_face_active, _wireless_car_seen
    global _search_orbit_dir
    global _search_orbit_prev_remaining, _search_orbit_fine_correcting
    global _search_approach_seen_cnt
    global _search_sweep_stop_axis, _search_sweep_stop_dir

    side_preformed = (
        _task_mode == _MODE_SEARCH
        and _prev_task_mode == _MODE_RECOVER
        and _recover_side_formation_ready
        and _recover_side_formation_yaw < 900.0
    )

    # 与主车使用相同的 SEARCH 航向来源：首次固定为网球直搜的 0 度；
    # APPROACH 丢失则恢复本车离开 SEARCH 时锁存的航向；正常搬运恢复后使用
    # 最近一次推出方向的反方向；其他异常回退才按主车无线坐标面向场地中心。
    if _search_first_boot:
        raw_yaw = 0.0
        _search_first_boot = False
    elif _prev_task_mode == _MODE_APPROACH and _search_resume_yaw_valid:
        raw_yaw = _search_resume_yaw
        _search_resume_yaw_valid = False
    elif _prev_task_mode == _MODE_RECOVER and _last_completed_push_yaw < 900.0:
        raw_yaw = _last_completed_push_yaw + 180.0
    else:
        if base._Other_Car_X < 900.0 and base._Other_Car_Y < 900.0:
            leader_x = base._Other_Car_X
            leader_y = base._Other_Car_Y
        else:
            leader_x = base._car.Position_X
            leader_y = base._car.Position_Y
        dx_center = leader_x - _FIELD_W * 0.5
        dy_center = leader_y - _FIELD_H * 0.5
        if abs(dx_center) >= abs(dy_center):
            raw_yaw = 270.0 if dx_center >= 0.0 else 90.0
        else:
            raw_yaw = 180.0 if dy_center >= 0.0 else 0.0
    _search_normal_yaw = (int((raw_yaw + 45.0) // 90.0) * 90.0) % 360.0
    # RECOVER 已锁存最终侧阵列：无斜避保持原侧，有斜避仅在需要时换到另一侧。
    # 第一次 SWEEP 直接沿用该航向，与主车实际横扫方向无关。
    _search_yaw = _recover_side_formation_yaw if side_preformed else _search_normal_yaw

    _fs_orbit_valid = False
    _fs_orbit_target_phi = (_search_yaw + 180.0) % 360.0
    _search_face_correcting = False
    _search_bearing_filt_valid = False
    _search_bearing_filt = 0.0
    _search_face_yaw_cmd_valid = False
    _search_face_yaw_cmd = base._car.current_angle
    # RECOVER 已检查过侧向阵列，不再从普通后方槽位重新绕一次；其他入口保持原流程。
    _fsearch_state = _FSEARCH_TRANSLATE_FORM if side_preformed else _FSEARCH_ORBIT_FORM
    _search_form_stable_t0 = 0
    _search_bad_geometry_t0 = 0
    _search_formation_locked = side_preformed
    _wireless_car_seen = False
    _search_orbit_dir = 1.0
    _search_orbit_prev_remaining = 360.0
    _search_orbit_fine_correcting = False
    _search_approach_seen_cnt = 0
    _search_sweep_stop_axis = _SEARCH_SWEEP_STOP_AXIS_NONE
    _search_sweep_stop_dir = 0
    # 7.14 SEARCH VISUAL_FOLLOW face修改：SEARCH可能在从未进入APPROACH时
    # 直接启动，因此必须在这里配置face PD；否则base默认kp/kd/限幅均为0，
    # 即使置位_face_req_active，车头也不会响应视觉误差。
    base.configure_face(_KP_FACE_OBJ, _KD_FACE_OBJ, _FACE_GYRO_MAX)
    # 从车完成开局横移后才进入 SEARCH；每次均从独立绕行编队且 ready=0 开始。
    _self_ready = 0


# 统一的前后/径向距离动态调整函数。
# rel_x/rel_y 是摄像头坐标系下主车相对从车的位置；函数按两车直线距离判断，
# 只有 abs(distance - target_dist) <= 5 cm 才认为进入误差带。返回的 vx/vy 是世界
# 坐标系径向修正量，可单独用于视觉跟随，也可与绕圆切向速度或主车前馈叠加。
def _search_distance_adjust(rel_x, rel_y, kp, max_speed, target_dist=_FOLLOWER_FOLLOW_DIST):
    distance = sqrt(rel_x * rel_x + rel_y * rel_y)
    error = distance - target_dist
    if abs(error) <= _FOLLOWER_SEARCH_DIST_ERR:
        return (0.0, 0.0, True)
    speed = kp * error
    if speed > max_speed:
        speed = max_speed
    elif speed < -max_speed:
        speed = -max_speed
    yaw_rad = radians(base._car.current_angle)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    toward_x = c * rel_x + s * rel_y
    toward_y = -s * rel_x + c * rel_y
    scale = speed / max(distance, 1.0)
    return (toward_x * scale, toward_y * scale, False)


def _search_leader_feedforward(relocalize=False, orbit=False):
    """把主车世界速度按从车车体系左右/前后轴分别缩放，再转换回世界速度。"""
    if base._Other_World_Vx < 900.0 and base._Other_World_Vy < 900.0:
        yaw_rad = radians(base._car.current_angle)
        c_yaw = cos(yaw_rad)
        s_yaw = sin(yaw_rad)
        # 世界速度 -> 从车车体系：lat 为左右横移轴，fwd 为车头前后轴。
        leader_lat = c_yaw * base._Other_World_Vx - s_yaw * base._Other_World_Vy
        leader_fwd = s_yaw * base._Other_World_Vx + c_yaw * base._Other_World_Vy
        sweep_cross_fwd = 0.0
        if relocalize:
            lat_gain = _FOLLOWER_RELOCALIZE_FF_GAIN
            fwd_gain = _FOLLOWER_RELOCALIZE_FF_GAIN
        elif orbit:
            lat_gain = _FOLLOWER_SEARCH_ORBIT_FF_LAT_GAIN
            fwd_gain = _FOLLOWER_SEARCH_ORBIT_FF_FWD_GAIN
        else:
            lat_gain = _FOLLOWER_SEARCH_FF_LAT_GAIN
            fwd_gain = _FOLLOWER_SEARCH_FF_FWD_GAIN
            # 横扫方向与前后漂移方向在连续实测中稳定同号。使用缩放前的主车
            # 横向速度生成 S4 专用交叉前馈，避免等误差扩大后再靠位置反馈追赶。
            if (base._Other_Car_Mode == _MODE_SEARCH
                    and base._Other_Car_Push_Sub == _LEADER_SEARCH_SWEEP_SUB):
                if leader_lat > 0.0:
                    sweep_cross_fwd = leader_lat * _FOLLOWER_SEARCH_SWEEP_CROSS_FWD_POS_GAIN
                elif leader_lat < 0.0:
                    sweep_cross_fwd = leader_lat * _FOLLOWER_SEARCH_SWEEP_CROSS_FWD_NEG_GAIN
        leader_lat *= lat_gain
        leader_fwd = leader_fwd * fwd_gain + sweep_cross_fwd
        # 从车车体系 -> 世界速度。
        return (
            c_yaw * leader_lat + s_yaw * leader_fwd,
            -s_yaw * leader_lat + c_yaw * leader_fwd,
        )
    return 0.0, 0.0


# 7.14 SEARCH 抑振修改：把原始视觉方位误差转换成平滑、有限变化率的目标航向。
# 1. 用环形角度差做低通，正确处理179/-179度跨界；2. 复用config已有的
# 6度进入、4度退出迟滞；3. 每周期最多改变3度，避免目标航向阶跃。
# 功能：死区线性淡入系数。|err| <= eps 返回 0；>= eps*_SEARCH_DEADBAND_FADE 返回 1；
# 中间线性过渡。用它乘在 Kp 输出上，消除死区边缘的输出跳变。
# 输入参数：err 为误差；eps 为死区半宽。
# 返回值：0~1 的系数。
def _search_deadband_fade(err, eps):
    a = err if err >= 0.0 else -err
    if a <= eps:
        return 0.0
    hi = eps * _SEARCH_DEADBAND_FADE
    if a >= hi:
        return 1.0
    return (a - eps) / (hi - eps)


def _search_world_rear_aligned():
    """按整 90 度 SEARCH 航向判断从车是否已在主车正后方且间距安全。"""
    leader_x = base._Other_Car_X
    leader_y = base._Other_Car_Y
    if leader_x >= 900.0 or leader_y >= 900.0:
        return False

    dx = leader_x - base._car.Position_X
    dy = leader_y - base._car.Position_Y
    if _search_yaw == 0.0:
        return abs(dx) <= _FOLLOWER_SEARCH_WORLD_REAR_LAT_EPS and dy > _FOLLOWER_SEARCH_WORLD_REAR_FWD_MIN
    if _search_yaw == 180.0:
        return abs(dx) <= _FOLLOWER_SEARCH_WORLD_REAR_LAT_EPS and dy < -_FOLLOWER_SEARCH_WORLD_REAR_FWD_MIN
    if _search_yaw == 90.0:
        return abs(dy) <= _FOLLOWER_SEARCH_WORLD_REAR_LAT_EPS and dx > _FOLLOWER_SEARCH_WORLD_REAR_FWD_MIN
    return abs(dy) <= _FOLLOWER_SEARCH_WORLD_REAR_LAT_EPS and dx < -_FOLLOWER_SEARCH_WORLD_REAR_FWD_MIN


# 功能：在普通 SEARCH/S5 横扫接近边界时，提前锁住从车当前横扫轴。
# 仅在主车新鲜状态明确为 SEARCH/S4 时允许锁存；主车进入 S6、退出 SEARCH、
# 无线过期或从车离开 S5 时立即释放。最后一行直接反向 SWEEP 时，则根据
# 主车横扫速度换向解除旧锁存，避免取消 S3 后把从车永久锁住。
# 返回值：0=不锁，1=把最终世界 X 速度清零，2=把最终世界 Y 速度清零。
def _search_sweep_early_stop_axis(now_ms):
    global _search_sweep_stop_axis, _search_sweep_stop_dir
    active = (
        _fsearch_state == _FSEARCH_TRANSLATE_FORM
        and base._Other_Car_Mode == _MODE_SEARCH
        and base._Other_Car_Push_Sub == _LEADER_SEARCH_SWEEP_SUB
        and base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _FOLLOWER_SEARCH_FF_FRESH_MS
    )
    if not active:
        _search_sweep_stop_axis = _SEARCH_SWEEP_STOP_AXIS_NONE
        _search_sweep_stop_dir = 0
        return _SEARCH_SWEEP_STOP_AXIS_NONE

    # 横扫轴始终由主车的正常 SEARCH 航向决定。第一次 Sweep/Forward 中从车
    # 临时把自身 _search_yaw 转了 90°，这里仍必须按 _search_normal_yaw 判断。
    if _search_normal_yaw == 0.0 or _search_normal_yaw == 180.0:
        axis = _SEARCH_SWEEP_STOP_AXIS_X
        position = base._car.Position_X
        world_speed = base._Other_World_Vx
        stop_min = _FOLLOWER_SEARCH_AREA_X_MIN + _FOLLOWER_SEARCH_SWEEP_EARLY_STOP_INSET_CM
        stop_max = _FOLLOWER_SEARCH_AREA_X_MAX - _FOLLOWER_SEARCH_SWEEP_EARLY_STOP_INSET_CM
    else:
        axis = _SEARCH_SWEEP_STOP_AXIS_Y
        position = base._car.Position_Y
        world_speed = base._Other_World_Vy
        stop_min = _FOLLOWER_SEARCH_AREA_Y_MIN + _FOLLOWER_SEARCH_SWEEP_EARLY_STOP_INSET_CM
        stop_max = _FOLLOWER_SEARCH_AREA_Y_MAX - _FOLLOWER_SEARCH_SWEEP_EARLY_STOP_INSET_CM

    current_dir = 0
    if world_speed < 900.0:
        if world_speed > 1.0:
            current_dir = 1
        elif world_speed < -1.0:
            current_dir = -1

    if _search_sweep_stop_axis != _SEARCH_SWEEP_STOP_AXIS_NONE:
        same_axis = _search_sweep_stop_axis == axis
        direction_reversed = (
            current_dir != 0
            and _search_sweep_stop_dir != 0
            and current_dir != _search_sweep_stop_dir
        )
        if same_axis and not direction_reversed:
            return _search_sweep_stop_axis
        _search_sweep_stop_axis = _SEARCH_SWEEP_STOP_AXIS_NONE
        _search_sweep_stop_dir = 0

    # 方向只从主车当前横扫轴速度取得。速度暂时为0时保留上一方向；收到反向
    # 速度后，上面的 direction_reversed 会解除上一条边界的提前停车锁存。
    if current_dir != 0:
        _search_sweep_stop_dir = current_dir

    if ((_search_sweep_stop_dir > 0 and position >= stop_max)
            or (_search_sweep_stop_dir < 0 and position <= stop_min)):
        _search_sweep_stop_axis = axis
    return _search_sweep_stop_axis


def _search_face_yaw_from_bearing(raw_bearing_err):
    global _search_face_correcting
    global _search_bearing_filt_valid, _search_bearing_filt
    global _search_face_yaw_cmd_valid, _search_face_yaw_cmd
    if not _search_bearing_filt_valid:
        _search_bearing_filt = raw_bearing_err
        _search_bearing_filt_valid = True
    else:
        _search_bearing_filt += _FOLLOWER_SEARCH_BEARING_BLEND * _angle_diff(raw_bearing_err, _search_bearing_filt)
        _search_bearing_filt = (_search_bearing_filt + 180.0) % 360.0 - 180.0

    abs_err = abs(_search_bearing_filt)
    if _search_face_correcting:
        if abs_err <= _FOLLOWER_SEARCH_FACE_EXIT:
            _search_face_correcting = False
    elif abs_err >= _FOLLOWER_SEARCH_FACE_ENTER:
        _search_face_correcting = True

    desired_yaw = (base._car.current_angle + _search_bearing_filt) % 360.0 if _search_face_correcting else base._car.current_angle
    if not _search_face_yaw_cmd_valid:
        _search_face_yaw_cmd = base._car.current_angle
        _search_face_yaw_cmd_valid = True
    yaw_step = _angle_diff(desired_yaw, _search_face_yaw_cmd)
    if yaw_step > _FOLLOWER_SEARCH_FACE_STEP_MAX_DEG:
        yaw_step = _FOLLOWER_SEARCH_FACE_STEP_MAX_DEG
    elif yaw_step < -_FOLLOWER_SEARCH_FACE_STEP_MAX_DEG:
        yaw_step = -_FOLLOWER_SEARCH_FACE_STEP_MAX_DEG
    _search_face_yaw_cmd = (_search_face_yaw_cmd + yaw_step) % 360.0
    return _search_face_yaw_cmd



# 功能：更新从车 SEARCH 模式。
# 从车默认跟随主车；当主车进入接近/等待阶段时，从车会同步目标，尝试重定位自身、面向目标、靠近目标或绕主车侧移，直到本车也看到目标并进入 APPROACH。
# 输入参数：now_ms 为当前时间戳。
def _update_search(now_ms):
    global _fs_orbit_target_phi, _fs_orbit_valid
    global _OPENART_MODEL_ENABLE, _next_task_mode, _self_sub
    global _search_face_correcting
    global _search_bearing_filt_valid, _search_face_yaw_cmd_valid
    global _fsearch_state, _search_form_stable_t0, _search_bad_geometry_t0
    global _search_formation_locked, _self_ready, _boot_face_active, _wireless_car_seen
    global _search_boot_follow_active
    global _search_orbit_dir
    global _search_orbit_prev_remaining, _search_orbit_fine_correcting
    global _search_approach_seen_cnt
    _self_sub = _fsearch_state
    sweep_stop_axis = _search_sweep_early_stop_axis(now_ms)
    # 首次启动时若从车先完成自身横移并进入 SEARCH，临时允许继续跟随仍在
    # BOOT 横移的主车；主车离开 BOOT 后恢复正常的 SEARCH 对端模式限制。
    if _search_boot_follow_active and base._Other_Car_Mode != _MODE_BOOT_SYNC:
        _search_boot_follow_active = False
    control_mode = _MODE_SEARCH
    follow_dist = _FOLLOWER_FOLLOW_DIST
    wireless_speed = _FOLLOWER_FOLLOW_SPEED
    orbit_leader_spd = _FOLLOWER_ORBIT_LEADER_SPD
    orbit_fine_spd = _FOLLOWER_ORBIT_FINE_SPD
    orbit_radial_max = _FOLLOWER_ORBIT_RADIAL_MAX
    total_max = _FOLLOWER_SEARCH_TOTAL_MAX
    visual_follow_fwd_max = _FOLLOWER_VISUAL_FOLLOW_FWD_MAX
    # 从车在 SEARCH 中不读取、筛选或选择场上目标。主车一旦通过稳定检测进入
    # APPROACH（或已经进入 WAIT_READY），从车只复制主车通过无线发送的目标，
    # 然后切入 approach_reset/approach_update 协作接近流程。
    if base._Other_Car_Mode == _MODE_APPROACH or base._Other_Car_Mode == _MODE_WAIT_READY:
        peer_fresh = (
            base._Other_Car_Ready_Ts != 0
            and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
        )
        target_ok = sync_follower_search_target()
        if peer_fresh and target_ok:
            _search_approach_seen_cnt += 1
        else:
            _search_approach_seen_cnt = 0
        # 离开SEARCH视觉跟随前释放face控制权，后续APPROACH由自身状态重新配置。
        base.clear_face()
        _self_ready = 0
        if _search_approach_seen_cnt >= _SEARCH_APPROACH_CONFIRM_FRAMES:
            _next_task_mode = _MODE_APPROACH
        else:
            base.request_hold(base._Other_Car_Angle)
        return
    _search_approach_seen_cnt = 0

    # SEARCH 通常允许主车处于 SEARCH/RECOVER；首次启动直切 SEARCH 时额外
    # 临时接受 BOOT_SYNC，其余模式保持停车等待。
    peer_mode_ok = (
        base._Other_Car_Mode == _MODE_RECOVER
        or base._Other_Car_Mode == _MODE_SEARCH
        or (_search_boot_follow_active and base._Other_Car_Mode == _MODE_BOOT_SYNC)
    )
    if not peer_mode_ok:
        base.clear_face()
        _self_ready = 0
        # 主车还没进 SEARCH（通常是开机后它还在 BOOT_SYNC）。这段等待期保持
        # 开机横移航向，等主车进入 SEARCH 后再由编队流程接管。
        # 编队真正开始后由下面的流程统一到 _search_yaw，那时已用视觉锁住主车。
        base.request_hold(_BOOT_FACE_YAW if _boot_face_active else base._Other_Car_Angle)
        return

    # 正常 RECOVER 继承的侧阵型贯穿整轮 SEARCH；主车连续扫描，从车全程动态修正。

    car_idx = _find_car_in_cam()
    if car_idx >= 0:
        # 只要在本轮 SEARCH/RELOCALIZE 跟随阶段看到主车一次便锁存，供主车
        # RECOVER 完成航向修正后判断从车已经具备视觉跟随条件。
        _wireless_car_seen = True

    # 世界横向坐标到位后，位置阶段已经锁存。这里不再使用视觉 face，
    # 也不再允许退回绕行；先原地转到统一 search_yaw，再发布阵型完成。
    # 位置条件一旦锁存，后续不再依赖视觉或无线坐标，短暂丢帧不会退回绕行。
    if _fsearch_state == _FSEARCH_REAR_YAW_ALIGN:
        base.clear_face()
        _self_ready = 0
        _search_form_stable_t0 = 0
        _search_bad_geometry_t0 = 0
        _fs_orbit_valid = False
        heading_err = abs(_angle_diff(_search_yaw, base._car.current_angle))
        if heading_err <= _FOLLOWER_SEARCH_FORM_YAW_EPS:
            _fsearch_state = _FSEARCH_TRANSLATE_FORM
            _self_sub = _fsearch_state
            _search_formation_locked = True
            _boot_face_active = False
            _self_ready = 2
            base.request_hold(_search_yaw)
        else:
            base.request_world(0.0, 0.0, _search_yaw)
        return

    if _recover_force_car_reloc_pending and car_idx < 0:
        # RECOVER 转向完成后的首次 SEARCH 必须先重新看到主车。即使无线位姿仍然
        # 新鲜，也禁止按无线坐标盲追；保持原地以固定角速度持续找车。视觉恢复后
        # 下一周期退出本分支，继续原有视觉编队，并在视觉+无线同时有效时完成修坐标。
        _fsearch_state = _FSEARCH_LINK_LOST
        _self_sub = _fsearch_state
        _self_ready = 0
        _search_form_stable_t0 = 0
        _search_bad_geometry_t0 = 0
        _search_formation_locked = False
        _search_bearing_filt_valid = False
        _search_face_yaw_cmd_valid = False
        _fs_orbit_valid = False
        base.clear_face()
        base.request_yaw_rate(_FOLLOWER_SEARCH_LINK_LOST_SPIN_DPS)
        return
    peer_fresh = (
        base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _FOLLOWER_SEARCH_FF_FRESH_MS
        and base._Other_Car_X < 900.0
        and base._Other_Car_Y < 900.0
    )

    if car_idx < 0 and not peer_fresh:
        # 摄像头与无线坐标同时失效时进入 LINK_LOST。禁止使用陈旧位置盲追，
        # 改为以固定 90 deg/s 原地自转搜索；重新看到主车后下一周期立即退出。
        _fsearch_state = _FSEARCH_LINK_LOST
        _self_sub = _fsearch_state
        _self_ready = 0
        _search_form_stable_t0 = 0
        _search_bad_geometry_t0 = 0
        _search_formation_locked = False
        _search_bearing_filt_valid = False
        _search_face_yaw_cmd_valid = False
        _fs_orbit_valid = False
        base.clear_face()
        base.request_yaw_rate(_FOLLOWER_SEARCH_LINK_LOST_SPIN_DPS)
        return

    if car_idx < 0:
        # 摄像头暂时看不到主车但无线坐标仍新鲜时，按主车航向计算其正后方目标距离点并做位置闭环。
        # 无线坐标不能用于视觉编队完成判定，因此此时始终 ready=0，重新看到主车后再恢复视觉距离、绕圆和连续两帧检查。
        _fsearch_state = _FSEARCH_VISUAL_FOLLOW
        _self_sub = _fsearch_state
        _self_ready = 0
        _search_form_stable_t0 = 0
        _search_bad_geometry_t0 = 0
        _search_formation_locked = False
        _search_bearing_filt_valid = False
        _search_face_yaw_cmd_valid = False
        _fs_orbit_valid = False
        # 视觉丢失后不能继续使用最后一帧误差旋转；关闭face，再按无线坐标
        # 执行主车正后方目标距离位置闭环。视觉恢复后重新从 ORBIT 建立几何，
        # 因此此处保持当前航向，禁止在看不到主车时盲转到最终角。
        base.clear_face()
        follow_yaw = _search_yaw if _search_formation_locked else base._car.current_angle
        leader_rad = radians(_search_yaw)
        target_x = base._Other_Car_X - sin(leader_rad) * follow_dist
        target_y = base._Other_Car_Y - cos(leader_rad) * follow_dist
        base.request_pos(target_x, target_y, wireless_speed, follow_yaw)
        return

    if not peer_fresh:
        # LINK_LOST 自转期间重新看到主车后立即退出自转。无线坐标尚未恢复时，
        # 先仅用视觉相对坐标修正朝向和跟随距离；无线恢复后再回到完整编队控制。
        _fsearch_state = _FSEARCH_VISUAL_FOLLOW
        _self_sub = _fsearch_state
        _self_ready = 0
        _search_form_stable_t0 = 0
        _search_bad_geometry_t0 = 0
        _search_formation_locked = False
        _search_bearing_filt_valid = False
        _search_face_yaw_cmd_valid = False
        _fs_orbit_valid = False
        rel_x = base._cam_car_rel_x
        rel_y = base._cam_car_rel_y
        rel_dist = sqrt(rel_x * rel_x + rel_y * rel_y)
        bearing_err = degrees(atan2(rel_x, rel_y))
        _search_face_yaw_from_bearing(bearing_err)
        radial_vx, radial_vy, unused = _search_distance_adjust(
            rel_x, rel_y, _FOLLOWER_VISUAL_FOLLOW_DIST_KP, visual_follow_fwd_max, follow_dist
        )
        base._face_req_err = _search_bearing_filt if _search_face_correcting else 0.0
        base._face_req_active = 1
        base._face_req_seq += 1
        base.request_world(radial_vx, radial_vy, base._car.current_angle)
        return

    rel_x = base._cam_car_rel_x
    rel_y = base._cam_car_rel_y
    rel_dist = sqrt(rel_x * rel_x + rel_y * rel_y)
    bearing_err = degrees(atan2(rel_x, rel_y))
    # 7.14 SEARCH 抑振修改：视觉跟随和绕圆共同使用低通、6/4度迟滞及
    # 每周期3度变化限制后的目标航向，不再把原始检测方位直接送入角度环。
    _search_face_yaw_from_bearing(bearing_err)

    # 普通 SEARCH 粗绕行完成后只允许锁存 TRANSLATE_FORM，普通视觉抖动
    # 不能把它重置回其他中间状态。
    valid_staged_state = (
        _fsearch_state == _FSEARCH_ORBIT_FORM
        or _fsearch_state == _FSEARCH_REAR_YAW_ALIGN
        or _fsearch_state == _FSEARCH_TRANSLATE_FORM
    )
    if not valid_staged_state:
        base.clear_face()
        _fsearch_state = _FSEARCH_ORBIT_FORM
        _search_orbit_fine_correcting = False
        _search_form_stable_t0 = 0
        _search_bad_geometry_t0 = 0
        _fs_orbit_valid = False
        _self_ready = 0
    _self_sub = _fsearch_state

    # SEARCH 状态一：从车视觉跟随主车。平移和旋转完全解耦。径向速度仍由
    # request_world控制；车头旋转改由base的face PD直接使用滤波后的视觉误差。
    # 6度进入/4度退出迟滞未处于纠偏时发送0误差，使D项平滑制动而不切换控制权。
    if _fsearch_state == _FSEARCH_VISUAL_FOLLOW:
        _self_ready = 0
        _search_form_stable_t0 = 0
        _fs_orbit_valid = False
        radial_vx, radial_vy, unused = _search_distance_adjust(
            rel_x, rel_y, _FOLLOWER_VISUAL_FOLLOW_DIST_KP, visual_follow_fwd_max, follow_dist
        )
        base._face_req_err = _search_bearing_filt if _search_face_correcting else 0.0
        base._face_req_active = 1
        base._face_req_seq += 1
        # face开启后会直接覆盖普通角度环的Z轴命令；第三个参数使用当前航向，
        # 避免普通角度环在后台积累另一个目标角，退出face时发生车头跳变。
        base.request_world(radial_vx, radial_vy, base._car.current_angle)
        return

    # SEARCH 连续编队：以主车实时位置为圆心，绕到统一目标航向对应的正后方。
    # 主车运动时圆心随无线坐标持续更新，整个过程不依赖主车 SEARCH 子状态。
    if _fsearch_state == _FSEARCH_ORBIT_FORM:
        _fs_orbit_target_phi = (_search_yaw + 180.0) % 360.0

        # 摄像头给出的主车相对向量负责确定真实绕行方位；无线坐标只提供移动
        # 圆心速度前馈。这样即使两车世界坐标有误差，也不会提前判定已经到槽位。
        yaw_rad = radians(base._car.current_angle)
        c_yaw = cos(yaw_rad)
        s_yaw = sin(yaw_rad)
        toward_x = c_yaw * rel_x + s_yaw * rel_y
        toward_y = -s_yaw * rel_x + c_yaw * rel_y
        radius = rel_dist
        if radius < 1.0:
            radius = 1.0
        # rx/ry 是从主车指向从车的世界坐标单位向量，用于生成绕主车的切向速度。
        rx_u = -toward_x / radius
        ry_u = -toward_y / radius
        phi_now = (base._car.current_angle + _search_bearing_filt + 180.0) % 360.0

        if not _fs_orbit_valid:
            phi_err = _angle_diff(_fs_orbit_target_phi, phi_now)
            # 180 度等长时固定选正方向；其他情况按带符号最短角锁定劣弧。
            if abs(phi_err) >= 179:
                _search_orbit_dir = 1.0
            else:
                _search_orbit_dir = 1.0 if phi_err >= 0.0 else -1.0
            _search_orbit_prev_remaining = abs(phi_err)
            _search_orbit_fine_correcting = False
            _fs_orbit_valid = True

        phi_err = _angle_diff(_fs_orbit_target_phi, phi_now)

        # 主车世界速度作为移动圆心的前馈；绕行和距离反馈只负责消除相对误差。
        ff_x, ff_y = _search_leader_feedforward(False, True)

        # 按整 90 度航向对应的世界横向坐标判断是否到达主车正后方；另一轴
        # 同时要求主车沿 search_yaw 正前方领先超过 15 cm，避免距离太近时转向。
        # RECOVER 刚找到主车时，先让循环尾部完成一次“主车无线坐标 + 本车视觉”
        # 的强制坐标修正；下一周期再使用修正后的世界坐标做 6 cm 判定。
        handoff_ready = (
            not _recover_force_car_reloc_pending
            and _search_world_rear_aligned()
        )
        if handoff_ready:
            base.clear_face()
            _fsearch_state = _FSEARCH_REAR_YAW_ALIGN
            _self_sub = _fsearch_state
            _self_ready = 0
            _search_form_stable_t0 = 0
            _search_bad_geometry_t0 = 0
            _search_orbit_fine_correcting = False
            _fs_orbit_valid = False
            base.request_hard_hold(base._car.current_angle)
            return

        # 横向未到位时继续用视觉看住主车并平移绕行，不允许最终航向控制提前接管。
        if not handoff_ready:
            # 尚未到槽位时保持进入绕圆时选定的方向。只有在减速区内越过目标，
            # 才按当前带符号方位误差重新选择回调方向。
            if _search_orbit_dir > 0.0:
                orbit_remaining = (_fs_orbit_target_phi - phi_now + 360.0) % 360.0
            else:
                orbit_remaining = (phi_now - _fs_orbit_target_phi + 360.0) % 360.0
            passed_target = (
                _search_orbit_prev_remaining <= _FOLLOWER_ORBIT_SLOW_START_DEG
                and orbit_remaining > _search_orbit_prev_remaining + _FOLLOWER_ORBIT_PHI_EPS
            )
            if passed_target:
                _search_orbit_dir = 1.0 if phi_err >= 0.0 else -1.0
                _search_orbit_fine_correcting = True
                if _search_orbit_dir > 0.0:
                    orbit_remaining = (_fs_orbit_target_phi - phi_now + 360.0) % 360.0
                else:
                    orbit_remaining = (phi_now - _fs_orbit_target_phi + 360.0) % 360.0
            _search_orbit_prev_remaining = orbit_remaining

            if _search_formation_locked:
                # 阵列一旦形成，后续即使需要重新修正槽位，也只允许麦轮平移；
                # 不再让视觉 face 覆盖角度环，避免跟车时车头重新转向主车。
                base.clear_face()
            else:
                base._face_req_err = _search_bearing_filt
                base._face_req_active = 1
                base._face_req_seq += 1
            tx_u = _search_orbit_dir * ry_u
            ty_u = -_search_orbit_dir * rx_u
            if _search_orbit_fine_correcting:
                orbit_speed = orbit_fine_spd
            elif orbit_remaining > _FOLLOWER_ORBIT_SLOW_START_DEG:
                orbit_speed = orbit_leader_spd
            else:
                slow_span = _FOLLOWER_ORBIT_SLOW_START_DEG - _FOLLOWER_ORBIT_PHI_EPS
                orbit_speed = orbit_fine_spd + (
                    (orbit_leader_spd - orbit_fine_spd)
                    * (orbit_remaining - _FOLLOWER_ORBIT_PHI_EPS) / max(slow_span, 1.0)
                )
            radial_vx, radial_vy, unused = _search_distance_adjust(
                rel_x, rel_y, _FOLLOWER_SEARCH_ORBIT_RADIAL_KP, orbit_radial_max, follow_dist
            )
            cmd_x = ff_x + tx_u * orbit_speed + radial_vx
            cmd_y = ff_y + ty_u * orbit_speed + radial_vy
            cmd_mag = sqrt(cmd_x * cmd_x + cmd_y * cmd_y)
            saturated = cmd_mag > total_max
            if saturated:
                cmd_scale = total_max / cmd_mag
                cmd_x *= cmd_scale
                cmd_y *= cmd_scale
                cmd_mag = total_max
            base.request_world(
                cmd_x,
                cmd_y,
                _search_yaw if _search_formation_locked else base._car.current_angle,
            )
            _self_ready = 0
            _search_form_stable_t0 = 0
            return

    if _fsearch_state == _FSEARCH_TRANSLATE_FORM:
        # 普通 SEARCH 的唯一最终控制器：锁住统一航向，独立收敛横向/前向
        # 视觉误差并跟随主车移动；这里没有切向绕圆命令。
        base.clear_face()
        _fs_orbit_valid = False
        heading_err = abs(_angle_diff(_search_yaw, base._car.current_angle))
        lat_err = rel_x
        fwd_err = rel_y - follow_dist

        # 普通横向和前后槽位误差始终交给平移反馈修正，不再触发回绕。
        # 仅当主车跑到本车后方，或视觉方位严重偏离并持续成立时，才重新 ORBIT。
        bad_geometry = (
            abs(_search_bearing_filt) > _FOLLOWER_SEARCH_REORBIT_BEARING
            or rel_y <= 0.0
        )
        if bad_geometry:
            if _search_bad_geometry_t0 == 0:
                _search_bad_geometry_t0 = now_ms
            elif ticks_diff(now_ms, _search_bad_geometry_t0) >= _FOLLOWER_SEARCH_REORBIT_HOLD_MS:
                _fsearch_state = _FSEARCH_ORBIT_FORM
                _self_sub = _fsearch_state
                _self_ready = 0
                _search_form_stable_t0 = 0
                _search_bad_geometry_t0 = 0
                _fs_orbit_valid = False
                _search_formation_locked = False
                # 异常几何成立后直接交回 ORBIT；不在状态交接处额外停车。
                return
        else:
            _search_bad_geometry_t0 = 0

        # ready 现在只表示从车当前是否位于阵型误差带；主车不再等待该握手。
        ready_latched = _self_ready == 2
        lat_ready_eps = (
            _FOLLOWER_SEARCH_READY_RELEASE_LAT_EPS
            if ready_latched else _FOLLOWER_SEARCH_TRANSLATE_LAT_EPS
        )
        fwd_ready_eps = (
            _FOLLOWER_SEARCH_READY_RELEASE_FWD_EPS
            if ready_latched else _FOLLOWER_SEARCH_TRANSLATE_FWD_EPS
        )
        yaw_ready_eps = (
            _FOLLOWER_SEARCH_READY_RELEASE_YAW_EPS
            if ready_latched else _FOLLOWER_SEARCH_FORM_YAW_EPS
        )
        lat_in_band = abs(lat_err) <= lat_ready_eps
        fwd_in_band = abs(fwd_err) <= fwd_ready_eps
        # 死区线性淡入。原来是硬切：误差跨过 eps 那一刻输出从 0 直接跳到 Kp*eps
        # 到 Kp*eps，没有斜率限制也没有 D 项，误差在死区边缘来回
        # 穿越就是持续抖动。改成 eps~fade*eps 之间增益由 0 线性升到 Kp，
        # "到位就不动"的行为不变，只把跳变磨平。
        vx_body = _FOLLOWER_SEARCH_TRANSLATE_LAT_KP * lat_err * _search_deadband_fade(lat_err, _FOLLOWER_SEARCH_TRANSLATE_LAT_EPS)
        fwd_kp = (
            _FOLLOWER_SEARCH_TRANSLATE_FWD_TOO_CLOSE_KP
            if fwd_err < 0.0
            else _FOLLOWER_SEARCH_TRANSLATE_FWD_TOO_FAR_KP
        )
        vy_body = fwd_kp * fwd_err * _search_deadband_fade(fwd_err, _FOLLOWER_SEARCH_TRANSLATE_FWD_EPS)
        if vx_body > _FOLLOWER_SEARCH_TRANSLATE_LAT_MAX:
            vx_body = _FOLLOWER_SEARCH_TRANSLATE_LAT_MAX
        elif vx_body < -_FOLLOWER_SEARCH_TRANSLATE_LAT_MAX:
            vx_body = -_FOLLOWER_SEARCH_TRANSLATE_LAT_MAX
        if vy_body > _FOLLOWER_SEARCH_TRANSLATE_FWD_MAX:
            vy_body = _FOLLOWER_SEARCH_TRANSLATE_FWD_MAX
        elif vy_body < -_FOLLOWER_SEARCH_TRANSLATE_FWD_MAX:
            vy_body = -_FOLLOWER_SEARCH_TRANSLATE_FWD_MAX

        yaw_rad = radians(base._car.current_angle)
        c_yaw = cos(yaw_rad)
        s_yaw = sin(yaw_rad)
        fb_x = c_yaw * vx_body + s_yaw * vy_body
        fb_y = -s_yaw * vx_body + c_yaw * vy_body

        # TRANSLATE 在 ready=2 前后都叠加主车速度前馈。这样从车在开局 BOOT
        # 横移或首次斜移中切入平移后不会丢掉移动圆心，也不需要主车停车等待。
        ff_x, ff_y = _search_leader_feedforward()
        cmd_x = ff_x + fb_x
        cmd_y = ff_y + fb_y
        # 提前停车作用在“前馈+反馈”合成后的横扫轴上；只清当前横扫轴，
        # 另一轴的前后距离修正以及 _search_yaw 航向锁定继续正常工作。
        if sweep_stop_axis == _SEARCH_SWEEP_STOP_AXIS_X:
            cmd_x = 0.0
        elif sweep_stop_axis == _SEARCH_SWEEP_STOP_AXIS_Y:
            cmd_y = 0.0
        cmd_mag = sqrt(cmd_x * cmd_x + cmd_y * cmd_y)
        saturated = cmd_mag > total_max
        if saturated:
            cmd_scale = total_max / cmd_mag
            cmd_x *= cmd_scale
            cmd_y *= cmd_scale
            cmd_mag = total_max

        stable = (
            lat_in_band
            and fwd_in_band
            and heading_err <= yaw_ready_eps
        )
        base.request_world(cmd_x, cmd_y, _search_yaw)

        # 新编队刚完成而主车仍处于 RECOVER 等待时持续发布 ready=2，
        # 直到主车收到 S5/ready 后离开等待，避免一次无线周期恰好漏掉握手。
        if _search_formation_locked and base._Other_Car_Mode == _MODE_RECOVER:
            _search_form_stable_t0 = 0
            _self_ready = 2
            return
        # ready 表示“当前正在到位带内”，不再是整轮 SEARCH 的永久锁存。
        if not stable:
            _search_form_stable_t0 = 0
            _self_ready = 0
            return
        if _search_form_stable_t0 == 0:
            _search_form_stable_t0 = now_ms
            _self_ready = 0
            return
        if ticks_diff(now_ms, _search_form_stable_t0) >= _FOLLOWER_SEARCH_TRANSLATE_STABLE_MS:
            _search_formation_locked = True
            _boot_face_active = False
            _self_ready = 2
        else:
            _self_ready = 0
        return

    # 未识别的内部状态保持停车；正常状态会在上方合法性检查中重置为 ORBIT。
    _self_ready = 0
    base.request_hold(base._Other_Car_Angle)
    return


_follower_car_reloc_last_ms = 0

# RECOVER 找到并居中主车后的一次性强制修正请求；条件不足时保留到首次成功，不阻塞状态切换。
_recover_force_car_reloc_pending = False


# 功能：利用本车视觉看到的主车位置修正从车自身位置。
# 主车无线坐标和本车摄像头看到的主车相对坐标同时有效时，可以反算从车世界坐标并融合到里程计。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示发起了位置修正请求；False 表示条件不足或未到修正间隔。
def _relocalize_follower_from_seen_car(now_ms):
    global _follower_car_reloc_last_ms, _recover_force_car_reloc_pending
    if not _FOLLOWER_CAR_RELOCALIZE_ENABLE:
        return False
    # RELOCALIZE 解除跟随后由 Y 边线独占坐标修正，避免主车视觉反算覆盖 Y 修正结果。
    if _task_mode == _MODE_DONE or (_task_mode == _MODE_RELOCALIZE and _mode_sub != _RELOC_FOLLOW_LEADER):
        _recover_force_car_reloc_pending = False
        return False
    interval = _FOLLOWER_CAR_RELOCALIZE_INTERVAL_MS
    if interval < 0:
        interval = 0
    if (not _recover_force_car_reloc_pending) and _follower_car_reloc_last_ms and ticks_diff(now_ms, _follower_car_reloc_last_ms) < interval:
        return False
    if base._Other_Car_X >= 900.0 or base._Other_Car_Y >= 900.0:
        return False
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > 500:
        return False
    if not _relocalize_car_visual_available(now_ms):
        return False
    rel_x = base._cam_car_rel_x
    rel_y = base._cam_car_rel_y
    if rel_x >= 900.0 or rel_y >= 900.0:
        return False
    yaw_rad = radians(base._car.current_angle)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    new_x = base._Other_Car_X - c * rel_x - s * rel_y
    new_y = base._Other_Car_Y + s * rel_x - c * rel_y
    w = _FOLLOWER_CAR_RELOCALIZE_BLEND
    if w < 0.0:
        w = 0.0
    if w > 1.0:
        w = 1.0
    fix_x = base._car.Position_X * (1.0 - w) + new_x * w
    fix_y = base._car.Position_Y * (1.0 - w) + new_y * w
    if fix_x < 900.0:
        base._pos_fix_x = fix_x
        base._pos_fix_x_valid = True
    if fix_y < 900.0:
        base._pos_fix_y = fix_y
        base._pos_fix_y_valid = True
    base._pos_fix_req = True
    _follower_car_reloc_last_ms = now_ms
    _recover_force_car_reloc_pending = False
    return True

# 功能：根据当前任务模式设置 OpenART 识别开关。
# 输入参数：mode 为即将进入或正在运行的任务模式。
def _set_openart_switches_for_mode(mode):
    global _OPENART_BALL_ENABLE, _OPENART_LINE_ENABLE, _OPENART_MODEL_ENABLE
    _OPENART_LINE_ENABLE = mode == _MODE_RELOCALIZE or mode == _MODE_PUSH_SYNC
    _OPENART_MODEL_ENABLE = mode == _MODE_SEARCH or mode == _MODE_APPROACH or mode == _MODE_WAIT_READY or mode == _MODE_RECOVER or mode == _MODE_RELOCALIZE or mode == _MODE_DONE or (mode == _MODE_PUSH_SYNC)
    # PUSH_SYNC 已不再使用球体修正，因此只在 APPROACH / WAIT_READY（含 VDOCK）识球。
    _OPENART_BALL_ENABLE = debug_switch.OPENART_BALL_DEBUG_ENABLE or ((mode == _MODE_APPROACH or mode == _MODE_WAIT_READY) and _PUSH_BALL_SYNC_ENABLE)

# 功能：进入推送模式时根据推送方向打开对应轴的视觉位置修正。
# 普通推送按目标边选择修正轴；路线推送则按路线 yaw 判断主要移动轴。
def _enable_push_pos_fix():
    yaw = _push_route_move_yaw
    if _push_route_phase == 2 and yaw < 900.0:
        axis_x = abs(_angle_diff(yaw, 90.0)) < 45.0 or abs(_angle_diff(yaw, 270.0)) < 45.0
    else:
        axis_x = _target_edge == 2 or _target_edge == 3
    base._vis_x_fix_en = axis_x
    base._vis_y_fix_en = not axis_x

# 功能：判断主车是否已经完成当前流程并进入重定位或完成状态。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示主车状态新鲜且已经进入 RELOCALIZE 或 DONE。
def _flow_peer_all_done(now_ms):
    if base._Other_Car_Ready_Ts == 0:
        return False
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _READY_OTHER_FRESH_MS:
        return False
    return base._Other_Car_Mode == _MODE_RELOCALIZE or base._Other_Car_Mode == _MODE_DONE

# 功能：从车任务层每周期更新入口。
# base.loop 会周期性调用它；本函数根据当前模式分发到对应 update，处理模式切换请求、从主车结束状态同步以及目标相对坐标回传。
# 返回值：当前任务模式编号。
def update():
    global _next_task_mode, _self_mode, _target_rel_x_for_cam, _target_rel_y_for_cam
    now = ticks_ms()
    mode = _task_mode
    _self_mode = mode
    # RECOVER 必须先通过原地自转把主车放回视野中央；即使主车已经完成，
    # 也由 recover_update 在视觉 x 进入 ±10 cm 后再切入 RELOCALIZE。
    if _flow_peer_all_done(now) and mode != _MODE_RECOVER and mode != _MODE_RELOCALIZE and (mode != _MODE_DONE):
        enter_mode(_MODE_RELOCALIZE)
        return _task_mode
    if mode == _MODE_BOOT_SYNC:
        boot_sync_update(now)
    elif mode == _MODE_SEARCH:
        _update_search(now)
    elif mode == _MODE_WAIT_READY:
        ready_update(now)
    elif mode == _MODE_DONE:
        done_update(now)
    elif mode == _MODE_APPROACH:
        approach_update(now)
    elif mode == _MODE_PUSH_SYNC:
        push_update(now)
    elif mode == _MODE_RECOVER:
        recover_update(now)
    elif mode == _MODE_RELOCALIZE:
        relocalize_update()
    if _next_task_mode >= 0:
        next_mode = _next_task_mode
        _next_task_mode = -1
        if next_mode != _task_mode:
            enter_mode(next_mode)
    _relocalize_follower_from_seen_car(now)
    if mode == _MODE_SEARCH:
        _target_rel_x_for_cam = 999.0
        _target_rel_y_for_cam = 999.0
    elif base._Other_Target_X < 900.0 and base._Other_Target_Y < 900.0:
        dx = base._Other_Target_X - base._car.Position_X
        dy = base._Other_Target_Y - base._car.Position_Y
        yaw_rad = radians(base._car.current_angle)
        c = cos(yaw_rad)
        s = sin(yaw_rad)
        _target_rel_x_for_cam = c * dx - s * dy
        _target_rel_y_for_cam = s * dx + c * dy
    else:
        _target_rel_x_for_cam = 999.0
        _target_rel_y_for_cam = 999.0
    return _task_mode


# 功能：进入新的从车任务模式。
# 该函数统一处理模式切换时的清理和初始化：更新 OpenART 开关、停止面向目标控制、调用各模式 reset、设置视觉修正开关、清理旧目标和协同命令。
# 输入参数：new_mode 为目标任务模式编号。
def enter_mode(new_mode):
    global _approach_cmd_to_other, _approach_plan_valid, _follower_car_reloc_last_ms, _follower_cmd_yaw_dir, _follower_reenter_creep, _master_cmd_sub, _next_task_mode, _prev_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _route_cmd_to_other, _route_obs_axis_to_other, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y, _target_sel_id_for_cam, _task_mode
    global _search_resume_yaw, _search_resume_yaw_valid
    gc.collect()
    if new_mode == _MODE_APPROACH:
        if _task_mode == _MODE_SEARCH:
            _search_resume_yaw = _search_yaw
            _search_resume_yaw_valid = True
    if new_mode != _task_mode:
        _prev_task_mode = _task_mode
    _task_mode = new_mode
    _next_task_mode = -1
    _set_openart_switches_for_mode(new_mode)
    base.clear_face()
    if new_mode == _MODE_BOOT_SYNC:
        boot_sync_reset()
    elif new_mode == _MODE_SEARCH:
        _reset_search()
        if _follower_reenter_creep:
            _follower_reenter_creep = False
    elif new_mode == _MODE_WAIT_READY:
        if _push_route_phase == 1:
            _push_route_phase = 2
        elif _push_route_phase == 3:
            _push_route_phase = 4
        ready_reset()
    elif new_mode == _MODE_DONE:
        done_reset()
    elif new_mode == _MODE_APPROACH:
        approach_reset()
    elif new_mode == _MODE_PUSH_SYNC:
        push_reset()
    elif new_mode == _MODE_RECOVER:
        recover_reset()
    elif new_mode == _MODE_RELOCALIZE:
        relocalize_reset()
    base._vis_yaw_fix_en = False
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False
    base._vis_pos_fix_edge_gate_cm = 0.0
    if new_mode == _MODE_PUSH_SYNC:
        base._vis_pos_fix_edge_gate_cm = _PUSH_POS_FIX_EDGE_GATE_CM
        _enable_push_pos_fix()
    if new_mode == _MODE_APPROACH:
        _target_sel_id_for_cam = _target_obj_id
    elif new_mode == _MODE_SEARCH:
        _follower_car_reloc_last_ms = 0
        _target_sel_id_for_cam = 0
        _approach_plan_valid = 0
        _approach_cmd_to_other = 999.9
        _follower_cmd_yaw_dir = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
        _route_obs_axis_to_other = 999.9
        _route_cmd_to_other = 999.9
        _push_route_axis = 999.9
        _push_route_phase = 0
        _push_route_move_yaw = 999.9
        _push_route_restore_push = 0
        _target_edge = 0
        _target_obj_id = 0
        _target_obj_world_x = 999.0
        _target_obj_world_y = 999.0
