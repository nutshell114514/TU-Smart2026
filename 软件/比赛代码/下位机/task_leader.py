import config
import gc
import base
import debug_switch
from math import atan2, cos, degrees, radians, sin, sqrt
from micropython import const
from time import ticks_add, ticks_diff, ticks_ms

# 主车任务层参数。
# 本文件只负责任务状态机和双车协同决策，底盘速度、姿态融合、串口收发都由 base.py 执行。
# 999/999.9 在本文件中通常表示无效坐标、无效角度或没有可执行协同命令。

# 主车全局重定位期间对 cone/brick 的固定 -Y 斥力避障总开关。只影响主车 RELOCALIZE，
# 不影响 SEARCH 视觉边界避障和 PUSH 阶段的避障。
# 1 = 开启：靠近障碍时固定叠加世界坐标 -Y 方向速度，
#     参数见 _RELOCALIZE_REPULSE_TRIGGER_CM / _RELOCALIZE_REPULSE_SPEED。
# 0 = 关闭：重定位按原速直线行驶，完全不做避障，也不记录障碍。
_RELOCALIZE_AVOID_ENABLE = 1

# RELOCALIZE 固定 -Y 斥力参数，与上面的总开关配套。
# 障碍进入 TRIGGER_CM 后立即叠加 REPULSE_SPEED，不随距离变化。
# 斥力只叠加到世界 Y 速度，不减速也不停车；SEARCH 用的是另一组参数。
_RELOCALIZE_REPULSE_TRIGGER_CM = 50.0
_RELOCALIZE_REPULSE_SPEED = 140.0

# RELOCALIZE 的 PREMOVE 段速度：主车锁 270° 车头直奔预设点 _RELOCALIZE_PRESET_POS[0]。
# 本值是合速度，按到目标点的单位方向分解成世界 vx/vy；走的是 request_world 速度
# 模式而不是位置模式，全程恒速、没有接近减速，靠 ±15cm 到点框、越过目标垂线或
# _RELOC_PREMOVE_TIMEOUT_MS 超时三选一退出，所以调太大容易冲过头。
# 后续 X_DRIVE / Y_DRIVE 段用的是另一个 _RELOC_SPEED（120，仍在原处），不受本值影响。
_RELOC_PREMOVE_SPEED = const(120)

# PUSH 阶段 cone/brick 斜推避障总开关。
_PUSH_DIAG_ENABLE = const(1)

# ── 现场常调参数 ──────────────────────────────────────────────────────────
# 以下参数从原定义处（SEARCH 参数区）上移到顶端，方便现场调参。相关的其他
# SEARCH 参数仍在原处，改动前先看原处的成组注释。

# SEARCH 蛇形横扫基准速度。换行前移速度直接引用本值，最大避障速度为本值的1.4倍，
# 后续只需修改这里即可保持三者 1:1:1.4；换行完成后直接开始反向横扫。
_SEARCH_SWEEP_SPEED = 70.0

# 蛇形搜索到达横向边界后，沿车头方向前进的单次步长，单位 cm。
# 实际步长取 min(本值, 到搜索区前边界的剩余距离)。
_SEARCH_FORWARD_STEP_CM = 30.0

# 开局首次斜移（_SEARCH_FIRST_DIAG）的目标点和世界合速度，单位分别为 cm、速度单位。
# 进入首次 SEARCH 时根据当前位置到目标点的向量只计算一次固定 vx/vy，之后不追点；
# X/Y 任一有效轴沿初始方向越过目标值即结束，避免固定45°斜移造成另一轴偏差过大。
_SEARCH_FIRST_DIAG_END_X = 120.0
_SEARCH_FIRST_DIAG_END_Y = 55.0
_SEARCH_FIRST_DIAG_SPEED = 80.0

# PUSH 阶段撞上 cone/brick 时的斜推偏转角，单位度。
# 斜避分两档：本值用于 cone/brick，普通斜避用原处的 _PUSH_DIAG_BIAS_DEG（15）。
_CONE_DIAG_BIAS_DEG = const(40)

# 障碍与被推物体的横向间距阈值，单位 cm。达到该间距即视为已绕开、不再斜避；
# 未达到时斜推距离取 max(0, 本值 - 实际间距)，间距越小斜走得越远。
_PUSH_OBS_CLEAR_DIST = const(60)

# 普通斜避（视觉实际识别到障碍触发）的横移距离，单位 cm。
# 预避障（按 config._OBSTACLE_EDGE_LAYOUT 提前预判方向，偏角见 _PUSH_PRE_DIAG_BIAS_DEG，
# 原处已上移到本区）也复用本值。原处仅保留常调参数说明，实际定义移到此处。
_PUSH_DIAG_DIST = const(20)

# 推完一个物体后，下一轮 SEARCH 起点在"推送轴"上的回撤坐标，单位 cm。
# 名字里的角度是上一次的推出航向，四选一（容差 ±1°）；车头取其反方向。
# 起点另一轴沿用刚进入 SEARCH 时的当前坐标，本组只覆盖推送轴这一个坐标：
#   推 270°（向低 X）-> X 回撤到 100    推 90°（向高 X）-> X 回撤到 220
#   推 0°（向高 Y）  -> Y 回撤到 180    推 180°（向低 Y）-> Y 回撤到 60
# 车把物体顶到边线后停在场地边缘，这组值负责把它拉回搜索区再开始扫描，
# 因此默认值与搜索矩形边界 _SEARCH_AREA_X_MIN/MAX、_SEARCH_AREA_Y_MIN/MAX
# 完全相同；改搜索区范围时这组一般要同步改，否则起点会落在搜索区外。
# 仅在"从 RECOVER 正常进入 SEARCH 且推出航向有效"时生效；首次搜索用
# 首次搜索走固定斜移，异常重入则保持当前位置，都不走这组。
_SEARCH_LINE_X_AFTER_PUSH_270 = 100.0
_SEARCH_LINE_Y_AFTER_PUSH_0 = 185.0
_SEARCH_LINE_X_AFTER_PUSH_90 = 230.0
_SEARCH_LINE_Y_AFTER_PUSH_180 = 50.0
# 数量模式 2/3 下，对应物品组已经搬完后使用的缩短回场坐标。
# 模式 1 只有总数，无法可靠判断物品组是否完成，因此不会使用这两个值。
_SEARCH_LINE_X_AFTER_PUSH_270_SANDBAGS_DONE = 120.0
_SEARCH_LINE_X_AFTER_PUSH_90_TEDDIES_DONE = 190.0
# ──────────────────────────────────────────────────────────────────────────

# 不同目标物体在接近和 PUSH 阶段使用的单侧夹角；当前两车相对推送轴均为30°，
# 按现有几何定义对应两根推杆夹角120°。
_APPROACH_LOCK_DEG = (0.0, 30.0, 30.0, 30.0, 30.0, 30.0)

# 接近目标时允许的横向参考偏移
_CLOSE_LAT_OFFSET_BY_OBJ = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# 就位阶段物体参考：网球16cm，蓝/红沙包13cm，白/棕熊13cm。
_CLOSE_DIST_LEADER_BY_OBJ = (0.0, 16.0, 13.0, 13.0, 13.0, 13.0)


# 推送目标时向场内偏转的角度
_PUSH_INWARD_BIAS_DEG_BY_OBJ = (0.0, 2.0, 4.0, 4.0, 4.0, 4.0)

# 主车按物体独立配置 PUSH PID，按 obj_id 索引：
# 0 不用，1 网球，2 蓝沙包，3 红沙包，4 白熊，5 棕熊。
# 每项依次为 (side_kp, side_kd, fwd_kp, fwd_kd)。
_PUSH_PID_LEADER_BY_OBJ = (
    None,
    (5.0, 10.0, 3.0, 6.0),  # 网球
    (9.0, 18.0, 4.0, 10.0),  # 蓝沙包
    (9.0, 18.0, 4.0, 10.0),  # 红沙包
    (5.0, 10.0, 3.0, 6.0),  # 白熊
    (5.0, 10.0, 3.0, 6.0),  # 棕熊
)

# 主车横向修正限幅，按 obj_id 索引；两个沙包单独提高到 80。
_PUSH_SIDE_MAX_LEADER_BY_OBJ = (0.0, 50.0, 80.0, 80.0, 50.0, 50.0)


# 等待推送前的路径检查参数。用于判断目标前方走廊是否存在其他未完成物体，需要先绕开。
_READY_ROUTE_SAFE_HALF = const(18)

_READY_ROUTE_FWD_MIN_DIST = const(6)

_READY_ROUTE_FWD_MAX_DIST = const(30)

_READY_ROUTE_MARGIN = const(10)

_READY_ROUTE_OBJ_DIR = (0, -1, -1, -1, 1, 1)

_READY_ROUTE_AVOID_ENABLE = bool(getattr(config, "_READY_ROUTE_AVOID_ENABLE", 0))

# OpenART 识别开关
_OPENART_LINE_ENABLE = True

_OPENART_MODEL_ENABLE = True

# 场地尺寸和中心点坐标
_FIELD_W = const(310)

_FIELD_H = const(230)

_CENTER_X = const(155)

_CENTER_Y = const(115)

# SEARCH / PUSH 共用的场边障碍先验。关闭时所有类型都放行，完全保持旧视觉逻辑。
_OBSTACLE_LAYOUT_FILTER_ENABLE = bool(getattr(config, "_OBSTACLE_LAYOUT_FILTER_ENABLE", 0))
_OBSTACLE_EDGE_LAYOUT = getattr(config, "_OBSTACLE_EDGE_LAYOUT", ((0, 0, 0),) * 4)
_OBSTACLE_CORNER_LAYOUT = getattr(config, "_OBSTACLE_CORNER_LAYOUT", (0, 0, 0, 0))
_OBSTACLE_CONE = const(1)
_OBSTACLE_BRICK = const(2)

# SEARCH 用世界坐标反推 cone/brick 归属边时的判定余量：离某条边界线在
# 该距离以内即归为那条边。边1/4 用 Y 判定，边2/3 用 X 判定。
_OBSTACLE_EDGE_MATCH_MARGIN_CM = 100.0

# SEARCH 新目标专用：如果 OpenART 把固定红砖误报成红沙包，则按布局表中
# brick(2) 所在边段的二维矩形区域过滤。垂直边线方向取 50 cm，沿边方向
# 在三等分区间两端各扩 20 cm；参数留在 config，现场改布局时无需改 OpenART。
_RED_SANDBAG_BRICK_ZONE_EDGE_DEPTH_CM = float(getattr(config, "_RED_SANDBAG_BRICK_ZONE_EDGE_DEPTH_CM", 50.0))
_RED_SANDBAG_BRICK_ZONE_ALONG_TOL_CM = float(getattr(config, "_RED_SANDBAG_BRICK_ZONE_ALONG_TOL_CM", 20.0))


def _obstacle_edge_corners(edge):
    # 返回沿本边坐标从小到大对应的两个角点。
    if edge == 1:
        return 1, 2
    if edge == 2:
        return 0, 1
    if edge == 3:
        return 3, 2
    return 0, 3


def _obstacle_layout_allowed(edge, obstacle_type, along_pos=None):
    if not _OBSTACLE_LAYOUT_FILTER_ENABLE:
        return True
    if edge < 1 or edge > 4:
        return False
    edge_layout = _OBSTACLE_EDGE_LAYOUT[edge - 1]
    corner_low, corner_high = _obstacle_edge_corners(edge)
    if along_pos is None:
        # PUSH 已知目标边，但不依赖车在边上的当前位置：整条边和两端角点都要考虑。
        if (_OBSTACLE_CORNER_LAYOUT[corner_low] == obstacle_type
                or _OBSTACLE_CORNER_LAYOUT[corner_high] == obstacle_type):
            return True
        return (edge_layout[0] == obstacle_type
                or edge_layout[1] == obstacle_type
                or edge_layout[2] == obstacle_type)
    # SEARCH 的前后方向坐标刚由 RECOVER 修正过，按三等分选择当前边段。
    edge_len = _FIELD_W if edge == 1 or edge == 4 else _FIELD_H
    if along_pos < edge_len / 3.0:
        return (edge_layout[0] == obstacle_type
                or _OBSTACLE_CORNER_LAYOUT[corner_low] == obstacle_type)
    if along_pos < edge_len * 2.0 / 3.0:
        return edge_layout[1] == obstacle_type
    return (edge_layout[2] == obstacle_type
            or _OBSTACLE_CORNER_LAYOUT[corner_high] == obstacle_type)


def _search_red_sandbag_in_brick_zone(x, y):
    # 只由主车 SEARCH 锁新目标时调用；APPROACH / PUSH 中的真实红沙包不受影响。
    if not _OBSTACLE_LAYOUT_FILTER_ENABLE or x >= 900.0 or y >= 900.0:
        return False
    for edge_idx in range(4):
        edge_layout = _OBSTACLE_EDGE_LAYOUT[edge_idx]
        if edge_idx == 0:      # 边1：高 Y，沿边坐标为 X
            edge_dist = abs(y - _FIELD_H)
            along_pos = x
            edge_len = _FIELD_W
        elif edge_idx == 1:    # 边2：低 X，沿边坐标为 Y
            edge_dist = abs(x)
            along_pos = y
            edge_len = _FIELD_H
        elif edge_idx == 2:    # 边3：高 X，沿边坐标为 Y
            edge_dist = abs(x - _FIELD_W)
            along_pos = y
            edge_len = _FIELD_H
        else:                  # 边4：低 Y，沿边坐标为 X
            edge_dist = abs(y)
            along_pos = x
            edge_len = _FIELD_W
        if edge_dist > _RED_SANDBAG_BRICK_ZONE_EDGE_DEPTH_CM:
            continue
        section_len = edge_len / 3.0
        for section in range(3):
            if edge_layout[section] != _OBSTACLE_BRICK:
                continue
            section_min = section * section_len - _RED_SANDBAG_BRICK_ZONE_ALONG_TOL_CM
            section_max = (section + 1) * section_len + _RED_SANDBAG_BRICK_ZONE_ALONG_TOL_CM
            if section_min <= along_pos <= section_max:
                return True
    return False


def _opposite_edge(edge):
    # 边1<->边4（Y方向一对），边2<->边3（X方向一对）。
    if edge == 1:
        return 4
    if edge == 4:
        return 1
    if edge == 2:
        return 3
    if edge == 3:
        return 2
    return 0


def _obstacle_world_edge(x, y, ref_edge):
    # 按 ref_edge 所在的轴（1/4 用 Y，2/3 用 X），根据障碍物自身世界坐标
    # 离哪条边界线更近（在 _OBSTACLE_EDGE_MATCH_MARGIN_CM 以内）判断它实际
    # 归属边1/边4还是边2/边3。不在任一边界余量内时返回 0（无法判定）。
    if ref_edge == 1 or ref_edge == 4:
        if y >= _FIELD_H - _OBSTACLE_EDGE_MATCH_MARGIN_CM:
            return 1
        if y <= _OBSTACLE_EDGE_MATCH_MARGIN_CM:
            return 4
        return 0
    if ref_edge == 2 or ref_edge == 3:
        if x <= _OBSTACLE_EDGE_MATCH_MARGIN_CM:
            return 2
        if x >= _FIELD_W - _OBSTACLE_EDGE_MATCH_MARGIN_CM:
            return 3
        return 0
    return 0


def _search_sweep_edge_and_along():
    # sweep_sign 是世界坐标轴方向；由它得到当前正在接近的外围边。
    if _search_yaw == 0.0 or _search_yaw == 180.0:
        return (3 if _search_sweep_sign > 0.0 else 2), base._car.Position_Y
    return (1 if _search_sweep_sign > 0.0 else 4), base._car.Position_X

_OBJ_ID_CAR = const(0)

_OBJ_ID_TENNIS = const(1)

_OBJ_ID_RED_SANDBAG = const(3)

# 1：搬网球时把两车槽位固定下来，不再按"谁离得近谁先挑"动态分配。
# 必须定义在 _lock_yaw 之前：MicroPython 里 _NAME = const(...) 是编译期常量，
# 不会生成模块全局变量，在定义之前引用会变成运行期 NameError。
_SLOT_FIXED_TENNIS = const(0)

# 普通 SEARCH 进入 APPROACH 时，如果从车相对主车仍保持约 90° 的侧向车头，
# 使用侧向入场专用的三分支槽位规划。只有落在 90°±该容差内才启用；其余姿态
# 继续使用原规划器，避免通信抖动或转向过渡态被误判成稳定侧向编队。
_APPROACH_SIDE_ENTRY_TOL_DEG = const(30)

# 从车到达 ORBIT 起点后理论上已经面向物体。主车会同时用从车世界坐标和目标
# 坐标反算环绕角；若它与从车实时报出的车头角相差不超过该值，则采用几何角，
# 否则采用车头角，避免里程计累计误差把贪心方向算反。
_APPROACH_FOLLOWER_HEADING_CHECK_DEG = const(30)

# 重定位时先移动到的预设安全位置。
_RELOCALIZE_PRESET_POS = ((50.0, 50.0), (70.0, 90.0))


# 上电初值。BOOT_SYNC 不走 enter_mode，这里必须自己认一次调试开关，否则
# "强制所有模式识球"在开机等待期是失效的。
_OPENART_BALL_ENABLE = bool(debug_switch.OPENART_BALL_DEBUG_ENABLE)

_BALL_REL_MAX_DIST = 120.0


_PUSH_SPEED_LEADER = const(120)

_CAM_LOST_TOL_MS = const(300)

_READY_OTHER_FRESH_MS = const(250)

_PUSH_RUN = const(0)

_ORBIT_SLOW_ZONE_DEG = const(55)

# 主任务模式编号
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

_CMD_SUB_ROUTE_PUSH = const(2)

_CMD_SUB_ROUTE_RESTORE = const(3)

_CMD_SUB_DIAG_PUSH = const(4)

_CMD_SUB_BOOT_START = const(5)

_CMD_SUB_PUSH_DONE = const(6)

# First-boot SEARCH entry/synchronization signal.
_CMD_SUB_BOOT_SEARCH_SWEEP = const(7)
_BOOT_SEARCH_SIGNAL_MS = const(300)

_BOOT_PHASE_IDLE = const(0)
_BOOT_PHASE_WAIT_FRAME = const(1)
_BOOT_PHASE_LATERAL = const(2)

# 主任务状态机当前模式、下一模式和上一模式。
_task_mode = _MODE_BOOT_SYNC

_next_task_mode = -1

_prev_task_mode = _MODE_SEARCH

# 当前选中的目标物体、目标边、目标世界坐标，以及发给摄像头的目标选择信息。
_target_edge = 0

_target_obj_id = 0

_target_obj_world_x = 999.0

_target_obj_world_y = 999.0

_target_sel_id_for_cam = 0

_target_rel_x_for_cam = 999.0

_target_rel_y_for_cam = 999.0

# 本车通过无线发给从车的就绪状态、模式和子状态。
_self_ready = 0

_self_mode = 0

_self_sub = 0

# 主车不使用该发送标志；保留同名字段供 base.py 统一组帧。
_wireless_car_seen = False


# 主车发给从车的协同命令序号和命令内容。
_cmd_seq = 0

_master_cmd_sub = 0

_follower_cmd_yaw_dir = 999.9

# RECOVER 后整轮 SEARCH 的阵列保持模式，经 Frame3 flags.bit4~5 发送。
# 0=无效，3=持续保持 RECOVER 最终侧阵列；旧的左右方向编码已删除。
_search_first_sweep_mode_to_other = 0

# 接近阶段的协同规划结果：双方绕目标形成夹角时，主车自己的目标 yaw 和旋转方向。
_search_first_boot = True

_approach_plan_valid = 0

_approach_plan_edge = 0

_approach_plan_obj_id = 0

_approach_self_yaw = 0.0

_approach_self_dir = 0

_approach_cmd_to_other = 999.9

# 侧向入场规划中给从车预分配的槽位，以及“等从车到达 ORBIT 起点后再由主车
# 计算贪心方向”的挂起标志。挂起期间两个无线规划字段都保持无效，从车只允许
# 完成 FACE/半径调整并在 ORBIT 起点停车等待。
_approach_follower_yaw = 999.9

_approach_follower_dir_pending = 0

_route_obs_axis_to_other = 999.9

_route_cmd_to_other = 999.9

_push_route_axis = 999.9

_push_route_phase = 0

_push_route_move_yaw = 999.9

_push_route_restore_push = 0

# 目标完成统计。_obj_remain 会由 base.state_init 根据当前数量模式重置。
_obj_done = [0, 0, 0, 0, 0, 0]

_obj_remain = [0, 1, 1, 1, 1, 1]

# 数量模式及全局剩余数由 base.state_init 按 config 重置。
_obj_count_mode = 3

_obj_total_remaining = 5

_obj_group_remain = []

# 主车维护的小型目标世界坐标地图，用于等待阶段判断推送走廊障碍。
_OBJ_MAP_MAX = const(4)

_obj_map_count = 0

_obj_map_type = [0] * _OBJ_MAP_MAX

_obj_map_x = [999.0] * _OBJ_MAP_MAX

_obj_map_y = [999.0] * _OBJ_MAP_MAX

_obj_map_ms = [0] * _OBJ_MAP_MAX

# 接近阶段的外部请求类型。推送失败、绕障结束或需要重新靠近时会用它改变 approach_reset 的起始动作。
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

# 仅在 PUSH 丢目标后双方重新发现同一目标时置位。
# 本轮 APPROACH 将使用左右阵列快速恢复专用的槽位和绕向分配。
_approach_from_push_lost = False

_recover_edge = 0

# 最近一次已经进入恢复流程的推出航向。目标信息在 RECOVER 结束时会被清除，
# 因此必须单独保存该值，供下一次 SEARCH 选择反方向车头航向。
_last_completed_push_yaw = 999.9

# 功能：清空接近阶段的外部请求，恢复为普通接近流程。
# 这个函数会把强制推送角、重靠近方向、后退标志等临时指令全部置为无效，避免一次请求被重复执行。
def clear_approach_request():
    global _approach_req, _approach_push_yaw, _approach_spin_dir
    global _approach_do_back, _approach_from_push_yaw, _approach_route_move_yaw
    _approach_req = _APPROACH_REQ_NONE
    _approach_push_yaw = 999.9
    _approach_spin_dir = 0
    _approach_do_back = 0
    _approach_from_push_yaw = 999.9
    _approach_route_move_yaw = 999.9

_FORM_BACK_DIST = const(12)

# 功能：判断两个数值是否在给定误差范围内，主要用于判断到点、到边和重定位是否满足精度。
def _near(a, b, eps):
    return abs(a - b) <= eps

# 功能：计算两个角度之间的最短有符号差值，并把结果规范到 -180 到 180 度。
def _angle_diff(a, b):
    return (a - b + 180) % 360 - 180

# 功能：计算两个世界坐标点之间的欧氏距离。
def _dist2(ax, ay, bx, by):
    dx, dy = (bx - ax, by - ay)
    return sqrt(dx * dx + dy * dy)

# 功能：把目标边编号转换为沿该边推出场地的基础推送方向。
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

# 物体类别到推送边的固定映射：0项保留；网球->边1，沙包->边2，玩偶->边3。
_OBJ_PUSH_EDGE = (0, 1, 2, 2, 3, 3)


# 功能：按物体类别的固定映射选择当前目标应该被推向哪条边。
def _obj_to_edge(obj_id):
    if not 1 <= obj_id < len(_obj_remain):
        return 0
    if obj_id < len(_OBJ_PUSH_EDGE):
        return _OBJ_PUSH_EDGE[obj_id]
    return 0

# 功能：根据目标边和目标类别计算主车默认接近锁定角。
# 输入参数：edge 为目标边编号。
# 返回值：主车接近时希望保持的车头角，单位度。
def _lock_yaw(edge):
    push = _push_yaw(edge)
    deg = _APPROACH_LOCK_DEG[_target_obj_id] if 1 <= _target_obj_id < len(_APPROACH_LOCK_DEG) else 30.0
    # 网球时主从车槽位互换，与 _apply_tennis_slot 保持一致。
    if _SLOT_FIXED_TENNIS and _target_obj_id == _OBJ_ID_TENNIS:
        return (push - deg) % 360.0
    return (push + deg) % 360.0

# 功能：把接近阶段的目标 yaw 和环绕方向压缩到一个浮点数里，便于无线协议发送给从车。
# 输入参数：target_yaw 为目标车头角，spin_dir 为环绕方向，非负编码为 1，负方向编码为 2。
# 返回值：编码后的命令值，整数部分是 yaw，小数部分是方向标记。
def _encode_approach_cmd(target_yaw, spin_dir):
    return int(round(target_yaw)) % 360 + (1 if spin_dir >= 0 else 2) * 0.1

# 功能：沿指定旋转方向计算从 start 到 end 需要走过的角度。
# 输入参数：start 为起始角，end 为终止角，spin_dir 为方向，非负表示一个方向，负数表示反方向。
# 返回值：按指定方向累计的角度差，范围为 0 到 360 度。
def _delta_dir(start, end, spin_dir):
    if spin_dir >= 0:
        return (start - end) % 360.0
    return (end - start) % 360.0


# 功能：判断 point 是否位于从 start 到 end、沿指定方向走过的圆弧上。
def _point_on_arc(start, end, spin_dir, point):
    return _delta_dir(start, point, spin_dir) <= _delta_dir(start, end, spin_dir)

# 功能：查找当前摄像头普通目标缓存中是否存在正在处理的目标。
# 返回值：目标槽位下标；-1 表示当前帧没有看到目标。
def _find_target_in_cam():
    if base._cam_obj_count <= 0 or base._cam_obj_id[0] != _target_obj_id:
        return -1
    return 0

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

_OBJ_MAP_TIMEOUT_MS = const(3000)

# 功能：清除当前接近协同规划，使主车下一次重新计算自己和从车的接近角。
def _clear_approach_plan():
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw
    _approach_plan_valid = 0
    _approach_plan_edge = 0
    _approach_plan_obj_id = 0
    _approach_self_yaw = 0.0
    _approach_self_dir = 0
    _approach_cmd_to_other = 999.9

# 功能：把当前摄像头看到的普通目标融合进主车目标地图。
# 地图会忽略已完成目标和无效坐标，同类别且距离近的观测会合并，容量满时替换最旧条目。
# 输入参数：now_ms 为当前时间戳，用于记录目标最后更新时间。
def _update_obj_world(now_ms):
    global _obj_map_count
    obj_count = _obj_map_count
    for i in range(base._cam_obj_count):
        ox = base._cam_obj_x[i]
        oy = base._cam_obj_y[i]
        oid = base._cam_obj_id[i]
        if ox >= 900 or oy >= 900:
            continue
        if not 1 <= oid < len(_obj_remain) or _obj_remain[oid] <= 0:
            continue
        ox = max(-20.0, min(_FIELD_W + 20.0, ox))
        oy = max(-20.0, min(_FIELD_H + 20.0, oy))
        merged_idx = -1
        for j in range(obj_count):
            if _obj_map_type[j] == oid and _dist2(_obj_map_x[j], _obj_map_y[j], ox, oy) < 20.0:
                merged_idx = j
                break
        if merged_idx >= 0:
            j = merged_idx
            w = 0.5
            _obj_map_x[j] = _obj_map_x[j] * (1 - w) + ox * w
            _obj_map_y[j] = _obj_map_y[j] * (1 - w) + oy * w
            _obj_map_ms[j] = now_ms
        else:
            if obj_count < _OBJ_MAP_MAX:
                j = obj_count
                obj_count += 1
            else:
                j = 0
                oldest_ms = _obj_map_ms[0]
                for k in range(1, _OBJ_MAP_MAX):
                    if _obj_map_ms[k] < oldest_ms:
                        oldest_ms = _obj_map_ms[k]
                        j = k
            _obj_map_type[j] = oid
            _obj_map_x[j] = ox
            _obj_map_y[j] = oy
            _obj_map_ms[j] = now_ms
    _obj_map_count = obj_count

# 功能：从目标地图中删除一个条目，并用最后一个条目补位，保持数组前段连续。
# 输入参数：idx 为要删除的地图槽位下标。
def _obj_map_delete(idx):
    global _obj_map_count
    last = _obj_map_count - 1
    if idx < 0 or idx > last:
        return
    if idx != last:
        _obj_map_type[idx] = _obj_map_type[last]
        _obj_map_x[idx] = _obj_map_x[last]
        _obj_map_y[idx] = _obj_map_y[last]
        _obj_map_ms[idx] = _obj_map_ms[last]
    _obj_map_type[last] = 0
    _obj_map_x[last] = 999.0
    _obj_map_y[last] = 999.0
    _obj_map_ms[last] = 0
    _obj_map_count = last

# 功能：清理过期或已经完成的目标地图条目，避免主车根据旧观测继续规划。
# 输入参数：now_ms 为当前时间戳。
def _prune_obj_map(now_ms):
    cutoff = now_ms - _OBJ_MAP_TIMEOUT_MS
    i = _obj_map_count - 1
    while i >= 0:
        if _obj_map_ms[i] <= cutoff or _obj_remain[_obj_map_type[i]] <= 0:
            _obj_map_delete(i)
        i -= 1

_BOOT_OTHER_FRESH_MS = const(800)

_BOOT_BEEP_CAM_MS = const(120)

_BOOT_BEEP_WIRELESS_MS = const(400)

_BOOT_BEEP_GAP_MS = const(250)

_boot_start_sent_t0 = 0

_boot_phase = _BOOT_PHASE_IDLE

_boot_lateral_y0 = 0.0

_boot_search_signal_t0 = 0

_beep_cam_done = False

_beep_cam_t0 = 0

_beep_wireless_done = False

# 功能：判断从车在开机同步阶段是否已经在线且 ready 状态有效。
# 这里同时检查无线时间戳新鲜度，避免使用很久之前残留的从车状态。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示从车最近回包且 ready 值达到开机同步要求。
def _other_boot_ready(now_ms):
    if base._Other_Car_Ready_Ts == 0:
        return False
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _BOOT_OTHER_FRESH_MS:
        return False
    return base._Other_Car_Ready >= 1

# 功能：重置开机同步状态。
# 会清空开始命令、蜂鸣提示状态和本车 ready 状态，确保重新进入 BOOT_SYNC 时从干净状态开始。
def boot_sync_reset():
    global _master_cmd_sub, _self_ready
    global _boot_start_sent_t0
    global _boot_phase, _boot_lateral_y0, _boot_search_signal_t0
    global _beep_cam_done, _beep_cam_t0, _beep_wireless_done
    _boot_start_sent_t0 = 0
    _boot_phase = _BOOT_PHASE_IDLE
    _boot_lateral_y0 = 0.0
    _boot_search_signal_t0 = 0
    _beep_cam_done = False
    _beep_cam_t0 = 0
    _beep_wireless_done = False
    _self_ready = 0
    _master_cmd_sub = _CMD_SUB_NONE

# 功能：执行主车开机同步流程。
# 主车先等待摄像头链路正常，再等待从车无线 ready；两条链路分别用短蜂鸣和长蜂鸣提示。
# 两车都准备好后，主车通过 _CMD_SUB_BOOT_START 持续通知从车开始。主车本车
# 横移达到主车配置的 60 cm 后立即进入 SEARCH；开局斜移由首次 SEARCH 子状态执行。
# 输入参数：now_ms 为当前时间戳。
def boot_sync_update(now_ms):
    global _master_cmd_sub, _next_task_mode, _self_ready
    global _boot_start_sent_t0
    global _boot_phase, _boot_lateral_y0, _boot_search_signal_t0
    global _beep_cam_done, _beep_cam_t0, _beep_wireless_done
    cam_ok = base._cam_rx_last_ms != 0
    _self_ready = 1 if cam_ok else 0
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
        _boot_start_sent_t0 = 0
        _boot_phase = _BOOT_PHASE_IDLE
        _boot_lateral_y0 = 0.0
        base.request_hold(base._car.current_angle)
        return
    # ready 只用于首次建立握手。新 SEARCH 会在从车切换模式后立即把 ready
    # 清零；如果这里每周期重新检查 ready，主车会撤销已经发出的 BOOT_START，
    # 从而永久停在 BOOT_SYNC。握手开始后必须锁存命令直到完成或超时。
    if _boot_phase == _BOOT_PHASE_IDLE:
        if not _other_boot_ready(now_ms):
            _master_cmd_sub = _CMD_SUB_NONE
            base.request_hold(base._car.current_angle)
            return
        _boot_start_sent_t0 = now_ms
        _boot_phase = _BOOT_PHASE_WAIT_FRAME
        _master_cmd_sub = _CMD_SUB_BOOT_START
        return

    if _boot_phase == _BOOT_PHASE_WAIT_FRAME:
        _master_cmd_sub = _CMD_SUB_BOOT_START
        if debug_switch.BOOT_DIRECT_DONE_ENABLE:
            base.request_hold(base._car.current_angle)
            peer_done = (
                base._Other_Car_Mode == _MODE_DONE
                and base._Other_Car_Ready_Ts != 0
                and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _BOOT_OTHER_FRESH_MS
            )
            if peer_done:
                _boot_phase = _BOOT_PHASE_IDLE
                _next_task_mode = _MODE_DONE
            return
        _boot_phase = _BOOT_PHASE_LATERAL
        _boot_lateral_y0 = base._car.Position_Y
        base.request_world(float(base._BOOT_LONGITUDINAL_SPEED), float(base._BOOT_LATERAL_SPEED), 90.0)
        return

    if _boot_phase == _BOOT_PHASE_LATERAL:
        _master_cmd_sub = _CMD_SUB_BOOT_START
        base.request_world(float(base._BOOT_LONGITUDINAL_SPEED), float(base._BOOT_LATERAL_SPEED), 90.0)
        if base._car.Position_Y - _boot_lateral_y0 >= float(base._BOOT_LATERAL_DISTANCE_LEADER_CM):
            _boot_phase = _BOOT_PHASE_IDLE
            _boot_search_signal_t0 = now_ms
            _master_cmd_sub = _CMD_SUB_BOOT_SEARCH_SWEEP
            _next_task_mode = _MODE_SEARCH
        return

_KP_FACE_OBJ = const(60)

_KD_FACE_OBJ = 1.0

# 限制面向目标时允许输出的最大角速度命令，防止原地转向过猛。
_FACE_GYRO_MAX = const(120)

# 接近完成后发给从车的 ready 子状态编号。
# 主车在 APPROACH 阶段到达目标两侧站位后，会进入 WAIT_READY 并等待从车也准备好。
_APPROACH_READY_SUB = const(20)

# 接近阶段子状态编号。
_AP_FACE = const(0)

_AP_ORBIT = const(1)

_AP_CLOSE = const(2)

_AP_ROUTE_BACK = const(10)

_AP_PRE_ORBIT = const(11)

_APPROACH_POS_EPS = const(4)

_APPROACH_YAW_EPS = const(6)

_LOST_RECOVER_MAX_MS = const(2000)

# FACE / PRE_ORBIT 看不到目标时更快退回 SEARCH；其他接近阶段仍沿用
# _LOST_RECOVER_MAX_MS，保留较长的缓存目标恢复窗口。
_FACE_PRE_ORBIT_LOST_SEARCH_MS = const(1000)

_FACE_EPS = const(4)

_FACE_DEADBAND_DEG = const(0)

_ORBIT_RADIAL_KP = 4.0

_ORBIT_RUN_RADIAL_KP = 4.5

# 普通 APPROACH/ORBIT 的径向向内前馈系数。前馈使用当前规划切向速度和
# 实时半径：v_ff = gain * orb_spd^2 / rw。0.0 表示保留实现但暂不启用。
_ORBIT_INWARD_FF_GAIN = 0.03

_ORBIT_SPEED_MAX = const(80)

_ORBIT_SPEED_MIN = const(20)

_ORBIT_OVERSHOOT_MARGIN_DEG = const(3)

_ORBIT_RADIAL_MAX = const(35)

_ORBIT_FACE_LEAD_GAIN = 2.0

_ORBIT_FACE_LEAD_MAX_DEG = 12.0

_PRE_ORBIT_RADIAL_MAX = const(80)

_PRE_ORBIT_R_EPS = const(5)

_PRE_ORBIT_FACE_EPS = const(8)

_FACE_ORBIT_X_EPS = const(8)

_FACE_ORBIT_R_EPS = const(5)

# 是否允许 ORBIT 结束后用视觉目标锚点反算本车位置并修正里程计。
# 1 表示启用，0 表示关闭。
_ORBIT_RELOCALIZE_ENABLE = const(1)

# ORBIT 后视觉反算位置允许的最大修正量。
# 如果反算位置与当前里程计差距超过该值，认为不可信并放弃修正。
_ORBIT_RELOCALIZE_MAX_CORR = const(45)

# ORBIT 后位置修正融合比例。
# 1 表示完全采用视觉反算位置，0 表示不改变当前位置。
_ORBIT_RELOCALIZE_BLEND = const(1)

# 普通接近阶段目标环绕半径。
# 主车在 PRE_ORBIT/ORBIT 中尽量保持离目标约这个距离绕行。
_ARC_ORBIT_R = const(20)

# 主车环绕时给从车预留的角度安全量，单位度。
# 用于避免两车在目标附近的夹角过近。
_ORBIT_LEADER_CLEAR_DEG = const(8)

# FACE 阶段横向居中比例系数。
# 根据目标横向角误差生成车体系横向速度，使目标回到车体中线附近。
# ORBIT 阶段持续修正锚点时使用的视觉融合比例。
# 数值越大，锚点越快靠近视觉目标坐标，但也越容易受视觉抖动影响。
_ORBIT_ANCHOR_BLEND = 0.12

# ORBIT 阶段每帧允许锚点移动的最大距离。
# 用于限制视觉坐标跳变对环绕圆心的影响。
_ORBIT_ANCHOR_MAX_STEP = 1.5

_FACE_LAT_KP = const(5)

# FACE 阶段横向速度最大值。
# 限制横向居中动作的速度，避免平移过猛导致目标丢失。
_FACE_LAT_MAX = const(60)

# CLOSE 阶段前后距离控制比例系数。
# 根据目标相对 y 距离与期望贴近距离的误差生成前后速度。
_CLOSE_RADIAL_KP = const(4)

# CLOSE 阶段前后移动速度上限。
# 限制最后靠近目标时的速度，保护目标和两车站位。
_CLOSE_SPEED = const(62)

# CLOSE 阶段横向位置控制比例系数。
# 根据目标相对 x 与期望横向偏移的误差生成横向速度。
_CLOSE_LAT_KP = const(4)

_CLOSE_LAT_MAX = const(42)

_CLOSE_OK_REQUIRE = const(2)

_CLOSE_HOLD_MS = const(70)

_CLOSE_LOST_SEARCH_MS = const(1000)

_ROUTE_RECLOSE_YAW_EPS = const(10)

_ROUTE_RECLOSE_ORBIT_SPEED = const(35)

_ROUTE_RECLOSE_ORBIT_R = const(18)

_ROUTE_RECLOSE_ORBIT_RADIAL_KP = const(4)

# 绕障重靠近 ORBIT 阶段径向速度上限。
_ROUTE_RECLOSE_ORBIT_RADIAL_MAX = const(50)

# 绕障重靠近 CLOSE 阶段前后移动速度上限。
_ROUTE_RECLOSE_CLOSE_SPEED = const(50)

# 绕障重靠近 CLOSE 阶段前后距离控制比例系数。
_ROUTE_RECLOSE_CLOSE_RADIAL_KP = 2.5

# 绕障重靠近 CLOSE 阶段横向控制比例系数。
_ROUTE_RECLOSE_CLOSE_LAT_KP = const(3)

# 绕障重靠近 CLOSE 阶段横向速度上限。
_ROUTE_RECLOSE_CLOSE_LAT_MAX = const(30)

# 绕障重靠近 CLOSE 阶段连续满足条件次数要求。
_ROUTE_RECLOSE_CLOSE_OK_REQUIRE = const(2)

# 绕障重靠近 CLOSE 阶段前后距离误差阈值。
_ROUTE_RECLOSE_CLOSE_POS_EPS = const(4)

# 绕障重靠近 CLOSE 阶段横向/朝向误差阈值。
_ROUTE_RECLOSE_CLOSE_FACE_EPS = const(3)

# 绕障重靠近时，不同目标类别使用的贴近距离表。
# 下标为目标类别 ID，第 0 项保留不用。
_ROUTE_RECLOSE_CLOSE_DIST_LEADER_BY_OBJ = (0.0, 16.0, 13.0, 13.0, 13.0, 13.0)

# ROUTE_BACK 子状态后退距离。
# 进入绕障或恢复普通推送方向前，主车会先沿当前目标角反方向退出这段距离。
_READY_ROUTE_BACK_DIST = const(6)

# ROUTE_BACK 子状态后退速度。
_READY_ROUTE_BACK_SPEED = const(35)

# ROUTE_BACK 子状态最长执行时间，单位 ms。
# 即使距离没有达到，也会在超时后继续后续接近流程，避免卡死。
_READY_ROUTE_BACK_TIMEOUT_MS = const(900)

# 绕障接近时主车与从车最小 yaw 间隔，单位度。
# 两车环绕目标时如果角度间隔小于该值，主车会暂停让从车先通过。
_READY_ROUTE_MIN_YAW_SEP_DEG = const(60)

_NON_SEARCH_ORBIT_YAW_SEP_DEG = const(30)

# 接近阶段内部子状态。
# FACE 先对准目标，PRE_ORBIT 贴到环绕半径，ORBIT 绕到规划角度，CLOSE 最终靠近。
_prelock_sub = 0

_mode_sub = _AP_FACE

_mode_hold_ms = 0

_close_ok_cnt = 0

_approach_t0 = 0

_ap_lost_lock_yaw = 0.0

_route_back_x0 = 0.0

_route_back_y0 = 0.0

_orbit_start_yaw = 0.0

_orbit_target_yaw = 0.0

_orbit_dir = 0

_orbit_plan_valid = False


# 功能：查询普通接近阶段主车对指定目标的期望贴近距离。
# 输入参数：obj_id 为目标类别编号。
# 返回值：距离标定值；表中没有该目标时返回编队默认后退距离。
def _approach_close_dist_for_obj(obj_id):
    table = _CLOSE_DIST_LEADER_BY_OBJ
    default = _FORM_BACK_DIST
    if 0 <= obj_id < len(table):
        return table[obj_id]
    return default

# 功能：查询绕障后重新靠近阶段使用的期望贴近距离。
# 输入参数：obj_id 为目标类别编号。
# 返回值：重靠近距离标定值；表中没有该目标时退回普通接近距离。
def _close_dist_for_reclose(obj_id):
    table = _ROUTE_RECLOSE_CLOSE_DIST_LEADER_BY_OBJ
    if 0 <= obj_id < len(table):
        return table[obj_id]
    return _approach_close_dist_for_obj(obj_id)

# 功能：请求接近状态机从 CLOSE 子状态重新开始。
# 该函数通常用于等待阶段发现靠近姿态变差后，不重新做完整环绕，只重新做最后贴近。
def restart_close():
    global _reset_first_sub
    _reset_first_sub = _AP_CLOSE
    _reset_approach_phase(_AP_CLOSE)

_reset_first_sub = _AP_FACE

_ap_anchor_valid = False

_ap_anchor_x = 999.0

_ap_anchor_y = 999.0

# 功能：重置接近阶段状态机。
# 如果之前有外部接近请求，会优先生成强制推送角规划、绕障重靠近规划或重启 CLOSE；否则回到普通 FACE 起点。
# 同时清空面向目标控制、环绕锚点和计时/计数状态，并配置面向目标角速度控制参数。
def approach_reset():
    global _approach_cmd_to_other, _approach_plan_valid, _self_sub
    global _approach_follower_yaw, _approach_follower_dir_pending
    global _prelock_sub, _reset_first_sub
    global _mode_sub, _mode_hold_ms, _close_ok_cnt, _approach_t0, _ap_lost_lock_yaw
    global _route_back_x0, _route_back_y0
    global _follower_last_plan_cmd
    global _orbit_plan_valid
    _approach_follower_yaw = 999.9
    _approach_follower_dir_pending = 0
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
    _close_ok_cnt = 0
    _approach_t0 = 0
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

# 功能：在接近阶段记录当前目标世界坐标作为环绕锚点。
# 输入参数：cam_obj 为目标在摄像头缓存中的槽位；无效槽位或目标坐标无效时会清空锚点。
def _set_orbit_anchor(cam_obj):
    global _ap_anchor_valid, _ap_anchor_x, _ap_anchor_y
    if cam_obj < 0 or _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        _clear_orbit_anchor()
        return
    _ap_anchor_valid = True
    _ap_anchor_x = _target_obj_world_x
    _ap_anchor_y = _target_obj_world_y

# 功能：在 ORBIT 阶段用当前视觉目标坐标缓慢修正环绕锚点。
# 只在目标类别匹配、视觉世界坐标有效时工作；每次修正会乘以小权重并限制最大步长，避免视觉抖动让圆心突然跳变。
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
# 用于普通进入接近、绕障重靠近、推送丢失恢复等场景；在路径绕障阶段会保留部分给从车的路线命令。
# 输入参数：first_sub 为接近阶段初始子状态，默认从 FACE 开始。
def _reset_approach_phase(first_sub=_AP_FACE):
    global _follower_cmd_yaw_dir, _master_cmd_sub, _route_cmd_to_other, _route_obs_axis_to_other
    global _approach_follower_yaw, _approach_follower_dir_pending
    global _mode_sub, _mode_hold_ms, _close_ok_cnt, _approach_t0, _ap_lost_lock_yaw
    global _route_back_x0, _route_back_y0
    global _orbit_plan_valid
    _mode_sub = first_sub
    _mode_hold_ms = 0
    _close_ok_cnt = 0
    _approach_t0 = 0
    _ap_lost_lock_yaw = base._car.current_angle
    _route_back_x0 = 0.0
    _route_back_y0 = 0.0
    _orbit_plan_valid = False
    _approach_follower_yaw = 999.9
    _approach_follower_dir_pending = 0
    _route_obs_axis_to_other = 999.9
    if _push_route_phase not in (1, 3):
        _route_cmd_to_other = 999.9
        _follower_cmd_yaw_dir = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
    base.clear_face()
    _clear_orbit_anchor()

# 功能：运行接近阶段的核心子状态机。
# 该函数完成从“看到目标并转向”到“环绕到规划角度”再到“靠近目标”的连续动作，是主车进入 WAIT_READY 前的关键流程。
# 运行过程中会持续更新目标世界坐标，处理短暂丢失目标、环绕避让从车、视觉居中和靠近距离控制。
# 输入参数：now_ms 为当前时间戳；target_yaw 为本车最终应达到的接近角；spin_dir 为指定环绕方向，None 表示自动选择。
# 返回值：True 表示接近已经完成，可以进入等待从车就位；False 表示仍在接近或已请求切换到搜索。
def _run_approach_phase(now_ms, target_yaw, spin_dir=None):
    global _follower_reenter_creep, _next_task_mode
    global _mode_sub, _mode_hold_ms, _close_ok_cnt, _approach_t0, _ap_lost_lock_yaw
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
    cam_rel = _find_target_in_cam()
    anchor_drive = (ap_sub == _AP_PRE_ORBIT or ap_sub == _AP_ORBIT) and _ap_anchor_valid
    if cam_rel < 0 and (not anchor_drive):
        if ap_sub == _AP_FACE or ap_sub == _AP_PRE_ORBIT:
            base.clear_face()
            if _approach_t0 == 0:
                _approach_t0 = now_ms
            if ticks_diff(now_ms, _approach_t0) > _FACE_PRE_ORBIT_LOST_SEARCH_MS:
                _next_task_mode = _MODE_SEARCH
            return False
        if ap_sub == _AP_CLOSE:
            base.clear_face()
            if _approach_t0 == 0:
                _approach_t0 = now_ms
                _ap_lost_lock_yaw = base._car.current_angle
            lost_ms = ticks_diff(now_ms, _approach_t0)
            _close_ok_cnt = 0
            _mode_hold_ms = 0
            if _target_obj_world_x < 900.0 and _target_obj_world_y < 900.0:
                recover_yaw = degrees(atan2(_target_obj_world_x - base._car.Position_X, _target_obj_world_y - base._car.Position_Y)) % 360.0
                base.request_world(0.0, 0.0, recover_yaw)
            if lost_ms > _CLOSE_LOST_SEARCH_MS:
                base.request_hold(base._car.current_angle)
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
                _next_task_mode = _MODE_SEARCH
        elif lost_ms > _CAM_LOST_TOL_MS:
            base.clear_face()
            base.request_hold(_ap_lost_lock_yaw)
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
            if not _make_face_plan_once():
                return False
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
            # CLOSE 同时控制横向和前后误差，ORBIT 到位后直接交给 CLOSE。
            _relocalize_after_orbit(cam_rel)
            base._fix_beep_active = 1
            base._fix_beep_until_ms = ticks_add(now_ms, 50)
            _mode_hold_ms = 0
            _mode_sub = _AP_CLOSE
            _close_ok_cnt = 0
            return False
        _mode_hold_ms = 0
        base._face_req_err = face_err
        base._face_req_active = 1
        base._face_req_seq += 1
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
        # 规划切向速度越高、绕行半径越小，车辆动态滞后造成的向外扩圈越明显。
        # 只给普通 ORBIT 叠加朝圆心的前馈；重靠近沿用原控制，避免扩大影响面。
        if not rc and _ORBIT_INWARD_FF_GAIN > 0.0:
            v_rad_ff = _ORBIT_INWARD_FF_GAIN * orb_spd * orb_spd / rw
        v_rad_raw = v_rad_fb + v_rad_ff
        v_rad = v_rad_raw
        if v_rad > orb_rmax:
            v_rad = orb_rmax
        if v_rad < -orb_rmax:
            v_rad = -orb_rmax
        cmd_vx = tx_u * orb_spd - rx_u * v_rad
        cmd_vy = ty_u * orb_spd - ry_u * v_rad
        base.request_world(cmd_vx, cmd_vy, base._car.current_angle)
        return False
    if ap_sub == _AP_CLOSE:
        rc = _push_route_phase == 1
        cls_spd = _ROUTE_RECLOSE_CLOSE_SPEED if rc else _CLOSE_SPEED
        cls_kp = _ROUTE_RECLOSE_CLOSE_RADIAL_KP if rc else _CLOSE_RADIAL_KP
        cls_lat_kp = _ROUTE_RECLOSE_CLOSE_LAT_KP if rc else _CLOSE_LAT_KP
        cls_lat_max = _ROUTE_RECLOSE_CLOSE_LAT_MAX if rc else _CLOSE_LAT_MAX
        cls_ok = _ROUTE_RECLOSE_CLOSE_OK_REQUIRE if rc else _CLOSE_OK_REQUIRE
        cls_pos_eps = _ROUTE_RECLOSE_CLOSE_POS_EPS if rc else _APPROACH_POS_EPS
        cls_fce_eps = _ROUTE_RECLOSE_CLOSE_FACE_EPS if rc else _FACE_EPS
        base.clear_face()
        obj_id = _target_obj_id
        if 0 <= obj_id < len(_CLOSE_LAT_OFFSET_BY_OBJ):
            lat_offset = _CLOSE_LAT_OFFSET_BY_OBJ[obj_id]
        else:
            lat_offset = 0.0
        lat_err = rel_x - lat_offset
        if abs(lat_err) < cls_fce_eps:
            vx_b = 0.0
        else:
            vx_b = cls_lat_kp * lat_err
            if vx_b > cls_lat_max:
                vx_b = cls_lat_max
            if vx_b < -cls_lat_max:
                vx_b = -cls_lat_max
        dist_err = rel_y - (_close_dist_for_reclose(obj_id) if rc else _approach_close_dist_for_obj(obj_id))
        if abs(dist_err) < cls_pos_eps:
            vy_b = 0.0
        else:
            vy_b = cls_kp * dist_err
            if vy_b > cls_spd:
                vy_b = cls_spd
            if vy_b < -cls_spd:
                vy_b = -cls_spd
        if abs(dist_err) < cls_pos_eps and abs(lat_err) < cls_fce_eps:
            _close_ok_cnt += 1
            base.request_hold(target_yaw)
            if _close_ok_cnt < cls_ok:
                _mode_hold_ms = 0
                return False
            if _mode_hold_ms == 0:
                _mode_hold_ms = now_ms
            if ticks_diff(now_ms, _mode_hold_ms) > _CLOSE_HOLD_MS:
                return True
            return False
        _close_ok_cnt = 0
        _mode_hold_ms = 0
        base.request_world(c * vx_b + s * vy_b, -s * vx_b + c * vy_b, target_yaw)
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


# 功能：网球固定槽位。网球是黄绿色，作为被推物体就夹在两杆之间、离从车镜头最近，
# 在画面里比 2.5cm 的标记球大一个量级，而 detect_balls 是按像素数取最大色块，
# 于是它会把橙球整个顶掉。紫球（A 正 B 负）和网球色度相距最远，不受干扰。
# 所以搬网球时强制让从车去 push+deg 那一侧（槽位1）——从车在那一侧本来就该看紫球、
# 用槽位1那组球参考，整条链路（_expected_side / _slot_ball_ref / _vdock_bl）自动自洽，
# 不需要在取球环节做任何特判。
# 输入参数：leader_yaw/follower_yaw 为动态分配结果；left_yaw 为 push+deg 一侧的角。
# 返回值：三元组 (leader_yaw, follower_yaw, 是否发生了交换)。
def _apply_tennis_slot(leader_yaw, follower_yaw, left_yaw):
    if _SLOT_FIXED_TENNIS and _target_obj_id == _OBJ_ID_TENNIS and follower_yaw != left_yaw:
        return (follower_yaw, leader_yaw, True)
    return (leader_yaw, follower_yaw, False)

# 功能：选择从当前角到目标角更短的环绕方向。
# 输入参数：now_yaw 为当前角，target_yaw 为目标角，单位度。
# 返回值：1 或 -1，表示沿哪个方向到达目标角代价更小。
def _best_dir(now_yaw, target_yaw):
    pos_cost = _delta_dir(now_yaw, target_yaw, 1)
    neg_cost = _delta_dir(now_yaw, target_yaw, -1)
    if pos_cost <= neg_cost:
        return 1
    return -1

# 功能：为主车生成普通接近阶段的双车夹角规划。
# 主车会选择离自己当前目标方位更近的一侧，从车通过无线命令收到同一目标角和旋转方向，用于形成协同接近。
def _side_entry_sign(leader_now):
    # +90/-90 只是相对关系，主车可以处于任意绝对 yaw。
    rel = _angle_diff(base._Other_Car_Angle, leader_now)
    if abs(rel - 90.0) <= _APPROACH_SIDE_ENTRY_TOL_DEG:
        return 1
    if abs(rel + 90.0) <= _APPROACH_SIDE_ENTRY_TOL_DEG:
        return -1
    return 0


def _dir_avoiding_yaw(start_yaw, target_yaw, avoid_yaw):
    pos_hits = _point_on_arc(start_yaw, target_yaw, 1, avoid_yaw)
    neg_hits = _point_on_arc(start_yaw, target_yaw, -1, avoid_yaw)
    if pos_hits and not neg_hits:
        return -1
    if neg_hits and not pos_hits:
        return 1
    return _best_dir(start_yaw, target_yaw)


# 功能：从车以相对主车 ±90° 的侧向姿态进入 APPROACH 时，按三种槽位布局规划。
# 返回值依次为主车槽位、从车槽位、主车方向、从车方向和是否延迟从车方向。
def _make_side_entry_plan(leader_now, side_sign, left_yaw, right_yaw):
    left_err = _angle_diff(left_yaw, leader_now)
    right_err = _angle_diff(right_yaw, leader_now)
    follower_virtual_yaw = (int((base._Other_Car_Angle + 45.0) // 90.0) * 90.0) % 360.0

    # 情况一：两个槽位分列主车当前朝向的两边（从主车看位于物体对面）。
    # 主车拿离从车虚拟位置更远的槽，走自己的最短方向；从车拿另一槽并反向绕。
    if left_err * right_err <= 0.0:
        left_from_follower = abs(_angle_diff(left_yaw, follower_virtual_yaw))
        right_from_follower = abs(_angle_diff(right_yaw, follower_virtual_yaw))
        if left_from_follower >= right_from_follower:
            leader_yaw, follower_yaw = left_yaw, right_yaw
        else:
            leader_yaw, follower_yaw = right_yaw, left_yaw
        leader_dir = _best_dir(leader_now, leader_yaw)
        if abs(left_err) + abs(right_err) <= 180.0:
            return (leader_yaw, follower_yaw, leader_dir, leader_dir, 0)
        return (leader_yaw, follower_yaw, leader_dir, -leader_dir, 0)

    # 情况二/三：两个槽位在主车同一侧。主车拿离自己更远的槽，并强制选择
    # 不经过从车虚拟 ±90° 位置的圆弧。
    if abs(left_err) >= abs(right_err):
        leader_yaw, follower_yaw = left_yaw, right_yaw
    else:
        leader_yaw, follower_yaw = right_yaw, left_yaw
    leader_dir = _dir_avoiding_yaw(leader_now, leader_yaw, follower_virtual_yaw)

    slots_on_follower_side = (left_err > 0.0 and side_sign > 0) or (left_err < 0.0 and side_sign < 0)
    if slots_on_follower_side:
        # 情况二：从车真正到达 ORBIT 起点后，主车再按其实时环绕角计算最短方向。
        return (leader_yaw, follower_yaw, leader_dir, 0, 1)
    # 情况三：槽位与从车异侧，两车同向绕行。
    return (leader_yaw, follower_yaw, leader_dir, leader_dir, 0)


def _make_leader_face_plan():
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw
    global _approach_follower_yaw, _approach_follower_dir_pending
    global _cmd_seq, _follower_cmd_yaw_dir
    edge = _target_edge
    left_yaw, right_yaw = _candidate_yaws(edge)
    leader_now = base._car.current_angle
    side_sign = _side_entry_sign(leader_now)

    if side_sign:
        leader_yaw, follower_yaw, leader_dir, follower_dir, follower_pending = _make_side_entry_plan(
            leader_now, side_sign, left_yaw, right_yaw
        )
    else:
        # 普通同向进入保持原规划器不变。
        left_err = _angle_diff(left_yaw, leader_now)
        right_err = _angle_diff(right_yaw, leader_now)
        if left_err * right_err < 0.0 and abs(left_err) + abs(right_err) <= 180.0:
            if abs(left_err) >= abs(right_err):
                leader_yaw = left_yaw
                follower_yaw = right_yaw
            else:
                leader_yaw = right_yaw
                follower_yaw = left_yaw
            leader_dir = _best_dir(leader_now, leader_yaw)
            follower_dir = -leader_dir
        elif left_err * right_err > 0.0:
            pos_left = _delta_dir(leader_now, left_yaw, 1)
            pos_right = _delta_dir(leader_now, right_yaw, 1)
            neg_left = _delta_dir(leader_now, left_yaw, -1)
            neg_right = _delta_dir(leader_now, right_yaw, -1)
            if min(pos_left, pos_right) <= min(neg_left, neg_right):
                leader_dir = 1
                if pos_left >= pos_right:
                    leader_yaw = left_yaw
                    follower_yaw = right_yaw
                else:
                    leader_yaw = right_yaw
                    follower_yaw = left_yaw
            else:
                leader_dir = -1
                if neg_left >= neg_right:
                    leader_yaw = left_yaw
                    follower_yaw = right_yaw
                else:
                    leader_yaw = right_yaw
                    follower_yaw = left_yaw
            follower_dir = leader_dir
        else:
            if abs(left_err) >= abs(right_err):
                leader_yaw = left_yaw
                follower_yaw = right_yaw
            else:
                leader_yaw = right_yaw
                follower_yaw = left_yaw
            leader_dir = _best_dir(leader_now, leader_yaw)
            follower_dir = -leader_dir
        opposite_dir = follower_dir == -leader_dir
        leader_yaw, follower_yaw, swapped = _apply_tennis_slot(leader_yaw, follower_yaw, left_yaw)
        if swapped and opposite_dir:
            leader_dir = _best_dir(leader_now, leader_yaw)
            follower_dir = -leader_dir
        follower_pending = 0

    _cmd_seq = (_cmd_seq + 1) & 255
    _approach_plan_valid = 1
    _approach_plan_edge = edge
    _approach_plan_obj_id = _target_obj_id
    _approach_self_yaw = leader_yaw
    _approach_self_dir = leader_dir
    _approach_follower_yaw = follower_yaw
    _approach_follower_dir_pending = follower_pending
    if follower_pending:
        # 两条命令都无效，阻止从车通过旧的兼容回退逻辑提前自行规划。
        _approach_cmd_to_other = 999.9
        _follower_cmd_yaw_dir = 999.9
    else:
        _approach_cmd_to_other = _encode_approach_cmd(leader_yaw, leader_dir)
        _follower_cmd_yaw_dir = _encode_approach_cmd(follower_yaw, follower_dir)


# 功能：为 PUSH 丢目标后的重新就位生成双车规划。
# 主车选择离自身当前物体方位最近的槽位，并沿最短方向前往；另一个槽位
# 分配给从车，从车使用与主车相反的绕行方向。当前不附加路径安全校验。
def _make_push_lost_reentry_plan():
    global _approach_cmd_to_other, _approach_from_push_lost
    global _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid
    global _approach_self_dir, _approach_self_yaw
    global _cmd_seq, _follower_cmd_yaw_dir
    edge = _target_edge
    left_yaw, right_yaw = _candidate_yaws(edge)
    leader_now = _orbit_heading_of_car(base._car.Position_X, base._car.Position_Y)
    left_cost = abs(_angle_diff(left_yaw, leader_now))
    right_cost = abs(_angle_diff(right_yaw, leader_now))
    if left_cost <= right_cost:
        leader_yaw = left_yaw
        follower_yaw = right_yaw
    else:
        leader_yaw = right_yaw
        follower_yaw = left_yaw
    leader_yaw, follower_yaw, unused_swapped = _apply_tennis_slot(leader_yaw, follower_yaw, left_yaw)
    leader_dir = _best_dir(leader_now, leader_yaw)
    follower_dir = -leader_dir
    _cmd_seq = (_cmd_seq + 1) & 255
    _approach_plan_valid = 1
    _approach_plan_edge = edge
    _approach_plan_obj_id = _target_obj_id
    _approach_self_yaw = leader_yaw
    _approach_self_dir = leader_dir
    _approach_cmd_to_other = _encode_approach_cmd(leader_yaw, leader_dir)
    _follower_cmd_yaw_dir = _encode_approach_cmd(follower_yaw, follower_dir)
    _approach_from_push_lost = False


# 功能：生成一个强制推送方向对应的接近规划。
# 当推送过程中需要绕开路径障碍、恢复普通推送方向或重新建立夹推姿态时，主车会把指定 push_yaw 转换成自己的接近角，同时给从车下发另一侧的接近角。
# 输入参数：push_yaw 为希望最终沿哪个方向推送；spin_dir 为指定环绕方向，0 表示自动选择；do_back 表示进入接近前是否先后退拉开距离；from_push_yaw 为上一次推送方向，保留用于路线恢复判断。
# 返回值：True 表示成功生成计划；False 表示目标边、目标坐标或角度无效。
def start_forced_push_yaw_plan(push_yaw, spin_dir=0, do_back=True, from_push_yaw=999.9):
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw, _cmd_seq, _follower_cmd_yaw_dir, _master_cmd_sub
    global _reset_first_sub, _follower_last_plan_cmd
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
    if spin_dir == 0:
        left_cost = min(_delta_dir(now_heading, left_yaw, 1), _delta_dir(now_heading, left_yaw, -1))
        right_cost = min(_delta_dir(now_heading, right_yaw, 1), _delta_dir(now_heading, right_yaw, -1))
        if left_cost <= right_cost:
            target_yaw = left_yaw
            follower_yaw = right_yaw
        else:
            target_yaw = right_yaw
            follower_yaw = left_yaw
        spin_dir = _best_dir(now_heading, target_yaw)
        _cmd_seq = _cmd_seq + 1 & 255
        _follower_cmd_yaw_dir = _encode_approach_cmd(follower_yaw, -spin_dir)
        _master_cmd_sub = _CMD_SUB_ROUTE_RESTORE
    else:
        target_yaw = left_yaw
        if spin_dir == 0:
            spin_dir = _best_dir(now_heading, target_yaw)
    _approach_plan_valid = 1
    _approach_plan_edge = edge
    _approach_plan_obj_id = _target_obj_id
    _approach_self_yaw = target_yaw
    _approach_self_dir = spin_dir
    _approach_cmd_to_other = _encode_approach_cmd(target_yaw, spin_dir)
    _reset_first_sub = _AP_ROUTE_BACK if do_back else _AP_FACE
    return True

# 功能：为推送路径绕障生成一次“重新靠近”计划。
# 当目标前方推送走廊有其他物体阻挡时，主车和从车会先移动到清障方向的两侧，再重新靠近目标，准备把目标侧向带出障碍区域。
# 输入参数：route_move_yaw 为绕障时目标应该移动的方向角。
# 返回值：True 表示成功生成重靠近计划；False 表示当前没有有效目标或路线方向无效。
def start_reclose_plan(route_move_yaw):
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw, _cmd_seq, _follower_cmd_yaw_dir, _master_cmd_sub
    global _reset_first_sub, _follower_last_plan_cmd
    edge = _target_edge
    if edge <= 0 or route_move_yaw >= 900.0:
        return False
    if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        return False
    pos_push = (route_move_yaw + 180.0) % 360.0
    pos_guide = route_move_yaw % 360.0
    now_heading = _orbit_heading_of_car(base._car.Position_X, base._car.Position_Y)
    cost_push = min(_delta_dir(now_heading, pos_push, 1), _delta_dir(now_heading, pos_push, -1))
    cost_guide = min(_delta_dir(now_heading, pos_guide, 1), _delta_dir(now_heading, pos_guide, -1))
    if cost_push <= cost_guide:
        target_yaw = pos_push
        follower_yaw = pos_guide
    else:
        target_yaw = pos_guide
        follower_yaw = pos_push
    spin_dir = _best_dir(now_heading, target_yaw)
    _cmd_seq = _cmd_seq + 1 & 255
    _approach_cmd_to_other = _encode_approach_cmd(target_yaw, spin_dir)
    _follower_cmd_yaw_dir = _encode_approach_cmd(follower_yaw, -spin_dir)
    _master_cmd_sub = _CMD_SUB_ROUTE_RECLOSE_1
    _approach_plan_valid = 1
    _approach_plan_edge = edge
    _approach_plan_obj_id = _target_obj_id
    _approach_self_yaw = target_yaw
    _approach_self_dir = spin_dir
    _reset_first_sub = _AP_PRE_ORBIT
    return True

# 功能：判断主车在绕障接近阶段是否与从车的环绕角过近。
# 如果双方正在同一环形路径上且角度间隔太小，主车会暂停，避免两车在目标附近互相挤压。
# 返回值：True 表示需要主车暂停让从车先通过；False 表示角度间隔安全。
def _route_form_yaw_too_close():
    if _push_route_phase not in (1, 3):
        return False
    if abs(_angle_diff(base._car.current_angle, base._Other_Car_Angle)) >= _READY_ROUTE_MIN_YAW_SEP_DEG:
        return False
    spin_dir = _approach_self_dir
    if spin_dir == 0:
        return False
    self_to_other = _delta_dir(base._car.current_angle, base._Other_Car_Angle, spin_dir)
    other_to_self = _delta_dir(base._Other_Car_Angle, base._car.current_angle, spin_dir)
    if abs(self_to_other - other_to_self) < 1.0:
        return False
    return self_to_other < other_to_self


# 功能：确保普通接近规划只生成一次，并复用当前目标已有规划。
# 返回值：True 表示当前已有或已成功生成接近规划。
def _make_face_plan_once():
    global _approach_plan_valid
    if _approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id):
        return True
    if _approach_from_push_lost:
        _make_push_lost_reentry_plan()
    else:
        _make_leader_face_plan()
    return True

# 功能：更新接近模式。
# 主车在这里根据当前目标和已有计划调用 _run_approach_phase；如果接近完成，就把本车 ready 清零并切到 WAIT_READY，等待从车也到达目标两侧。
# 该函数也处理从 PUSH_SYNC 回到 APPROACH 时从车已经搜索完成的情况，及时清空目标并回到搜索。
# 输入参数：now_ms 为当前时间戳。
# 对方状态新鲜度阈值，单位 ms。approach_update 内也会使用，必须声明在该
# 函数之前，下划线 const 在函数编译后才声明会退化成不存在的全局变量导致 NameError。
_PUSH_PEER_FRESH_MS = const(700)

_APPROACH_PUSH_LOST_SPIN_DPS = 90.0


# 功能：完成侧向入场“同侧槽位”分支里被延迟的从车方向规划。
# 从车先以 AP_ORBIT 上报自己已经面向物体并进入环绕半径，但在收到本函数生成的
# 完整“槽位+方向”命令前保持不动。返回 True 表示无需等待或本周期已成功下发。
def _finalize_pending_follower_plan(now_ms):
    global _approach_cmd_to_other, _approach_follower_dir_pending
    global _cmd_seq, _follower_cmd_yaw_dir
    if not _approach_follower_dir_pending:
        return True
    if not (_approach_plan_valid
            and _approach_plan_edge == _target_edge
            and _approach_plan_obj_id == _target_obj_id):
        return False
    if (base._Other_Car_Mode != _MODE_APPROACH
            or base._Other_Car_Push_Sub != _AP_ORBIT
            or base._Other_Car_Ready_Ts == 0
            or ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _PUSH_PEER_FRESH_MS
            or base._Other_Target_Edge != _target_edge
            or base._Other_Target_ObjId != _target_obj_id):
        return False

    # 从车刚完成 FACE 时，车头角本身就是“从车指向物体”的环绕角。若双方世界
    # 坐标也自洽，则使用坐标反算值；坐标与车头角偏差过大时保守采用实时报头角。
    follower_now = base._Other_Car_Angle
    if (base._Other_Car_X < 900.0 and base._Other_Car_Y < 900.0
            and _target_obj_world_x < 900.0 and _target_obj_world_y < 900.0):
        follower_geo = _orbit_heading_of_car(base._Other_Car_X, base._Other_Car_Y)
        if abs(_angle_diff(follower_geo, follower_now)) <= _APPROACH_FOLLOWER_HEADING_CHECK_DEG:
            follower_now = follower_geo
    follower_dir = _best_dir(follower_now, _approach_follower_yaw)

    _cmd_seq = (_cmd_seq + 1) & 255
    _approach_cmd_to_other = _encode_approach_cmd(_approach_self_yaw, _approach_self_dir)
    _follower_cmd_yaw_dir = _encode_approach_cmd(_approach_follower_yaw, follower_dir)
    _approach_follower_dir_pending = 0
    # 从车正在 AP_ORBIT 起点停车等待，规划一补齐就立即发送，避免再多等一个
    # 普通无线轮询周期。
    base.wireless_send_now()
    return True

def approach_update(now_ms):
    global _approach_cmd_to_other, _approach_plan_edge, _approach_plan_obj_id, _approach_plan_valid, _approach_self_dir, _approach_self_yaw, _follower_cmd_yaw_dir, _master_cmd_sub, _next_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _self_sub, _self_ready, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y, _target_sel_id_for_cam
    global _prelock_sub
    # 重就位避障（reclose/restore 在 APPROACH 阶段执行）：双声蜂鸣持续标记。
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
    if _prev_task_mode != _MODE_SEARCH:
        _self_ready = 0
        if _find_target_in_cam() < 0:
            base.clear_face()
            if _prev_task_mode == _MODE_PUSH_SYNC:
                spin_dir = _approach_self_dir
                if spin_dir != 1 and spin_dir != -1:
                    spin_dir = 1 if _angle_diff(_ap_lost_lock_yaw, _push_yaw(_target_edge)) >= 0.0 else -1
                base.request_yaw_rate(spin_dir * _APPROACH_PUSH_LOST_SPIN_DPS)
            _self_sub = _mode_sub
            return
        if base._Other_Car_Mode != _MODE_APPROACH or base._Other_Car_Ready_Ts == 0 or ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _PUSH_PEER_FRESH_MS or base._Other_Target_Edge != _target_edge or base._Other_Target_ObjId != _target_obj_id:
            base.clear_face()
            base.request_hold(_ap_lost_lock_yaw)
            _self_sub = _mode_sub
            return
        if _mode_sub == _AP_ORBIT and not (_approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id)):
            if base._Other_Car_Mode != _MODE_APPROACH or base._Other_Car_Push_Sub != _AP_ORBIT:
                base.clear_face()
                base.request_hold(_ap_lost_lock_yaw)
                _self_sub = _mode_sub
                return
            left_yaw, right_yaw = _candidate_yaws(_target_edge)
            leader_now = base._car.current_angle
            if _delta_dir(leader_now, base._Other_Car_Angle, 1) >= 180.0:
                spin_dir = 1
            else:
                spin_dir = -1
            if _delta_dir(leader_now, left_yaw, spin_dir) >= _delta_dir(leader_now, right_yaw, spin_dir):
                _approach_self_yaw = left_yaw
                _follower_cmd_yaw_dir = _encode_approach_cmd(right_yaw, spin_dir)
            else:
                _approach_self_yaw = right_yaw
                _follower_cmd_yaw_dir = _encode_approach_cmd(left_yaw, spin_dir)
            _approach_self_dir = spin_dir
            _approach_cmd_to_other = _encode_approach_cmd(_approach_self_yaw, spin_dir)
            _approach_plan_valid = 1
            _approach_plan_edge = _target_edge
            _approach_plan_obj_id = _target_obj_id
    # 主车自身可以继续按已生成的槽位运行；同侧分支只延迟从车的贪心方向。
    # 一旦从车上报 AP_ORBIT，本函数会补齐无线规划并解除从车起点等待。
    _finalize_pending_follower_plan(now_ms)
    edge = _target_edge
    if _approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id):
        target_yaw = _approach_self_yaw
        spin_dir = _approach_self_dir
    else:
        target_yaw = _lock_yaw(edge)
        spin_dir = None
    if _prelock_sub != _APPROACH_READY_SUB:
        _prelock_sub = _APPROACH_READY_SUB
    if _run_approach_phase(now_ms, target_yaw, spin_dir):
        # 不能先离开 APPROACH：从车需要看到主车仍处于 APPROACH/WAIT_READY，
        # 并以 AP_ORBIT 回报到位状态；等完整从车规划已下发后再进入 WAIT_READY。
        if not _approach_follower_dir_pending:
            _self_ready = 0
            _next_task_mode = _MODE_WAIT_READY
    _self_sub = _mode_sub

_PUSH_DONE_MARGIN = const(6)

# PUSH 黄线绝对坐标只有距任一对应场地边界小于该值时才允许修正里程计。
_PUSH_POS_FIX_EDGE_GATE_CM = const(50)

_PUSH_LOST_CONFIRM_MS = const(800)

_PUSH_LOST_SPIN_MS = const(3200)

_PUSH_LOST_SPIN_EPS = const(8)

_PUSH_PEER_LOST_CONFIRM_MS = const(2000)

_PUSH_RECOVER_FOUND_READY = const(5)

_PUSH_RECOVER_WAIT_MS = const(4000)

_PUSH_GO_TIMEOUT_MS = const(1500)

# PUSH 起步时序按从车所在球侧区分：
# 橙球侧主车先走，100ms 后通知从车；紫球侧先通知从车，主车等待100ms 后再走。
_PUSH_START_LEADER_ORANGE_LEAD_MS = const(100)
_PUSH_START_DELAY_LEADER_PURPLE_MS = const(100)

_PUSH_GO_ADVANCE_MS = const(100)

# 网球一旦进入普通 PUSH_RUN，视觉丢失只关闭视觉修正，不允许进入 PUSH_SPIN。
# _PUSH_ROUTE 在 push_update 的前置分支中单独处理，不受本开关影响。
_DISABLE_NORMAL_TENNIS_LOST_SEARCH = const(1)

_PUSH_SIDE_KP_LEADER = const(5)


_PUSH_SIDE_KD_LEADER = const(10)


_PUSH_FWD_KP_LEADER = const(0)


# 前向 D 项。
_PUSH_FWD_KD_LEADER = const(0)


_PUSH_SIDE_MAX_LEADER = const(50)


# 前向修正限幅。两车都用固定基座速度120 + 各自物体环，前向权限要够大，
# 否则一方冲前把物体顶走时另一方追不回来，V 被拉歪、物体从侧边漏出。
_PUSH_FWD_MAX_LEADER = const(30)


_PUSH_SIDE_SLEW_LEADER = const(15)


_PUSH_FWD_SLEW_LEADER = const(15)


_PUSH_D_LPF_LEADER = 0.35


_PUSH_SIDE_DEADBAND = const(0)

_PUSH_FWD_DEADBAND = const(0)

_PUSH_SIDE_MIN = const(0)

_PUSH_FWD_MIN = const(0)

_PUSH_VISUAL_LOOP_LEADER_ENABLE = const(1)


_PUSH_WORLD_SIDE_LOCK_ENABLE = const(1)

# 普通 PUSH 的 y 目标使用物体紧贴推杆标定值；x 目标按物体和槽位标定：
# 网球/两熊在 push_yaw±30° 槽位使用±2cm；两个沙包都使用0cm。
_PUSH_REF_SLOT_X_CM_BY_OBJ = (0.0, 2.0, 0.0, 0.0, 2.0, 2.0)
_PUSH_REF_REL_LEADER_BY_OBJ = (None, (0.0, 16.0), (0.0, 13.0), (0.0, 13.0), (0.0, 13.0), (0.0, 13.0))

_PUSH_DIAG_CONFIRM_FRAMES = const(2)

_PUSH_DIAG_HALF = const(16)

_PUSH_DIAG_BIAS_DEG = const(15)

# _PUSH_DIAG_DIST（普通斜避横移距离，20cm）已上移到文件顶端的"现场常调参数"区。

# 预避障：按 config._OBSTACLE_EDGE_LAYOUT 提前预判方向的斜避，不依赖本轮
# 视觉实际识别到障碍。偏角固定 40°（与 _CONE_DIAG_BIAS_DEG 一致），横向
# 移动距离复用 _PUSH_DIAG_DIST(20cm，已上移到文件顶端"现场常调参数"区)。
_PUSH_PRE_DIAG_BIAS_DEG = 40.0

_PUSH_DIAG_MAX_MS = const(1600)

# 与正常 PUSH 起步相同：命令先发给从车，主车延时 100 ms 再执行。
# 进入和退出斜避共用该补偿时间，减小无线传输造成的两车动作时差。
_PUSH_DIAG_LEADER_DELAY_MS = _PUSH_GO_ADVANCE_MS
_PUSH_DIAG_PHASE_IDLE = const(0)
_PUSH_DIAG_PHASE_START_DELAY = const(1)
_PUSH_DIAG_PHASE_ACTIVE = const(2)
_PUSH_DIAG_PHASE_EXIT_DELAY = const(3)

# cone 和 brick 共用这一组 PUSH 避障参数。
# 其中 _CONE_DIAG_BIAS_DEG（斜推偏角）和 _PUSH_OBS_CLEAR_DIST（横向间距阈值，
# 决定是否斜避以及斜推多远）已上移到文件顶端的"现场常调参数"区。
_CONE_MEMORY_MS = const(3000)
_CONE_DIAG_MAX_MS = const(3000)
_CONE_DIAG_HALF = const(40)

_push_lock_yaw = 0.0

_push_ref_valid = False

_push_ref_rel_x = 0.0

_push_ref_rel_y = 0.0

_push_done = False

_push_started = False

_mode_sub = 0

_push_lost_t0 = 0

_push_sub_t0 = 0

_push_spin_start_yaw = 0.0

_push_peer_lost_t0 = 0

_push_recover_found = False

_push_recover_hold_yaw = 0.0

_push_recover_wait_t0 = 0

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

_push_diag_phase = _PUSH_DIAG_PHASE_IDLE

_push_diag_phase_t0 = 0

_push_diag_t0 = 0

_push_diag_start_side = 999.9

_push_diag_move_yaw = 999.9

_push_diag_dist = 999.9

_push_diag_is_obstacle = False

_push_diag_confirm_side = 0

_push_diag_confirm_count = 0

_push_diag_confirm_token = 0

# 本次 PUSH 首次由 cone/brick 触发斜避时锁存的障碍物世界侧向。
# 该方向与斜避的侧向分量相反，只取 0/90/180/270 度；999.9 表示未触发。
_push_diag_obstacle_yaw = 999.9

# True 表示当前 _push_diag_active 这一轮是预避障（按 config 提前预判方向），
# 而不是视觉实际识别到障碍触发的正常斜避。用于区分：预避障期间如果摄像头
# 真正识别到了障碍，需要放弃预判方向、交回正常斜避检测逻辑重新确认。
_push_pre_diag_active = False

# 网球容易在斜避方向突变时滚出夹持区域。本轮 PUSH 只要确定过一次斜避的
# 侧向（无论是布局预避障还是视觉实际识别触发），就把 avoid_sign（不是
# 最终 yaw）锁存到下一次 push_reset；之后不管障碍出现在哪一侧，都只复用
# 锁存的侧向，不能把网球的斜避方向改到另一侧。但偏转角度（bias_deg）仍按
# 触发来源正常取值——预避障和视觉确认到真实障碍现在都是 40°——只锁"哪一
# 侧"，不锁角度大小。999.9 表示本轮没有可锁侧向。
_push_tennis_diag_sign = 999.9

# 红沙包同上，同一轮只锁侧向，角度仍按正常规则（预避障/视觉触发）取值。
_push_redbag_diag_sign = 999.9

_push_fix_x_snap = 0

_push_fix_y_snap = 0

_PUSH_SPIN = const(2)

_PUSH_ROUTE = const(3)

_PUSH_ROUTE_TIMEOUT_MS = const(4000)

_PUSH_ROUTE_EXTRA_DIST = const(15)

# 横推（对推）阶段两车共用的基座标称速度。两车必须相等：速度差会在纯 P
# 距离环下留下 Δ/kp 的稳态间距偏差，把修正权限提前吃满；相等时视觉丢失
# 退化为等速平移，间距冻结不变，退化行为是安全的。
_PUSH_ROUTE_SPEED = const(80)

# 推进车（车头与推送方向夹角<=90°）的距离参考值，单位 cm。推进车主要靠
# 基座速度顶物体前进，距离环只用于脱开后重新接触，权限很小。
_PUSH_ROUTE_PUSH_REF_Y = 12.0

# 引导/后退车（车头与推送方向相反）的距离参考值，单位 cm。比接近阶段
# 贴近距(14)略小，给 2cm 预压；预压由参考距离提供，不再依赖速度差。
_PUSH_ROUTE_CONTACT_DIST = 12.0

_PUSH_ROUTE_PUSH_FWD_MAX = const(20)

_PUSH_ROUTE_GUIDE_FWD_MAX = const(50)

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

# 功能：查询当前目标的主车推送视觉闭环参数。
# 返回值：四元组 side_kp、side_kd、fwd_kp、fwd_kd；目标没有专用参数时返回默认参数。
def _push_pid_params():
    obj_id = _target_obj_id
    table = _PUSH_PID_LEADER_BY_OBJ
    if table is not None and 0 < obj_id < len(table) and (table[obj_id] is not None):
        return table[obj_id]
    return (_PUSH_SIDE_KP_LEADER, _PUSH_SIDE_KD_LEADER, _PUSH_FWD_KP_LEADER, _PUSH_FWD_KD_LEADER)

# 功能：查询当前目标的主车横向修正限幅。
def _push_side_max():
    obj_id = _target_obj_id
    table = _PUSH_SIDE_MAX_LEADER_BY_OBJ
    if table is not None and 0 < obj_id < len(table):
        return table[obj_id]
    return _PUSH_SIDE_MAX_LEADER

# 功能：查询当前目标的固定视觉相对参考点。
# 固定参考点用于推送时让目标保持在车体坐标的期望位置，而不是每次以刚看到的位置为参考。
# 返回值：三元组，依次为是否有效、参考 rel_x、参考 rel_y。
def _fixed_push_ref():
    obj_id = _target_obj_id
    table = _PUSH_REF_REL_LEADER_BY_OBJ
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
# 对上边推送使用 x 轴做横向轴，对左右边推送使用 y 轴做横向轴。
# 输入参数：x/y 为点坐标，edge 为目标边。
# 返回值：该点在当前边对应轴上的值。
def _push_axis_value(x, y, edge):
    return x if edge == 1 or edge == 4 else y

# 功能：在推送阶段观测目标前方走廊中的其他物体。
# 主车会找出最可能挡住目标前进路线的未完成物体，把它的轴坐标和类别发给从车，供诊断推送或绕障协同使用。
def _update_push_obs():
    global _route_cmd_to_other, _route_obs_axis_to_other
    if _target_obj_world_x >= 900.0 or _target_obj_world_y >= 900.0:
        _route_obs_axis_to_other = 999.9
        return
    edge = _target_edge
    target_axis = _push_axis_value(_target_obj_world_x, _target_obj_world_y, edge)
    push_rad = radians(_push_yaw_for_edge(edge))
    px = sin(push_rad)
    py = cos(push_rad)
    sx = -py
    sy = px
    half = _READY_ROUTE_SAFE_HALF
    want_high = _push_axis_value(base._car.Position_X, base._car.Position_Y, edge) < target_axis
    best_idx = -1
    best_axis = 0.0
    best_fwd = 999.9
    for i in range(base._cam_obj_count):
        obj_type = base._cam_obj_id[i]
        if obj_type == _target_obj_id or obj_type == _OBJ_ID_CAR:
            continue
        if not 1 <= obj_type < len(_obj_remain):
            continue
        if _obj_remain[obj_type] <= 0:
            continue
        ox = base._cam_obj_x[i]
        oy = base._cam_obj_y[i]
        if ox >= 900.0 or oy >= 900.0:
            continue
        dx = ox - _target_obj_world_x
        dy = oy - _target_obj_world_y
        proj_fwd = dx * px + dy * py
        proj_side = dx * sx + dy * sy
        if proj_fwd <= 0.0:
            continue
        if proj_fwd < _READY_ROUTE_FWD_MIN_DIST:
            continue
        if proj_fwd > _READY_ROUTE_FWD_MAX_DIST:
            continue
        if abs(proj_side) > half:
            continue
        axis = _push_axis_value(ox, oy, edge)
        if best_idx < 0:
            best_idx = i
            best_axis = axis
            best_fwd = proj_fwd
        elif want_high and axis > best_axis:
            best_idx = i
            best_axis = axis
            best_fwd = proj_fwd
        elif not want_high and axis < best_axis:
            best_idx = i
            best_axis = axis
            best_fwd = proj_fwd
    if best_idx < 0:
        _route_obs_axis_to_other = 999.9
    else:
        _route_obs_axis_to_other = best_axis

# 功能：计算诊断推送时的斜向移动 yaw。
# 当推送走廊一侧检测到障碍时，主车和从车会用带侧向偏置的移动方向把目标短距离斜推，尝试避开障碍。
# 输入参数：edge 为目标边；side_sign 表示向哪一侧斜推。
# 返回值：斜向推送 yaw，单位度。
# 功能：根据障碍所在侧算出斜避的 side_sign，使偏置方向【远离】障碍。
# _diag_move_yaw 里偏置向量在 s_hat 上的投影随 edge 变号（edge 1/2: ss=+1→投影-1；
# edge 3/4: ss=+1→投影+1）。要偏向障碍的反侧：
#   障碍在 s 正侧(obs_pos=True) → 偏置 s 分量取负 → edge1/2 用 ss=+1，edge3/4 用 ss=-1
#   障碍在 s 负侧(obs_pos=False)→ 反之
# 输入参数：edge 目标边；obs_pos 障碍是否在 s_hat 正侧。
# 返回值：side_sign（±1）。
def _diag_avoid_sign(edge, obs_pos):
    if edge == 3 or edge == 4:
        return -1.0 if obs_pos else 1.0
    return 1.0 if obs_pos else -1.0

def _diag_move_yaw(edge, side_sign, bias_deg=_PUSH_DIAG_BIAS_DEG):
    normal_yaw = _push_yaw_for_edge(edge)
    beta = radians(bias_deg)
    n = radians(normal_yaw)
    vx = sin(n) * cos(beta)
    vy = cos(n) * cos(beta)
    if edge == 1 or edge == 4:
        vx += side_sign * sin(beta)
    else:
        vy += side_sign * sin(beta)
    return degrees(atan2(vx, vy)) % 360.0

# 功能：把"障碍靠近该边坐标轴小端还是大端"换算成 _diag_avoid_sign 需要的
# obs_pos。边1/边2的小端对应推送方向左手边（s_hat 正侧），边3/边4刚好相反
# （小端对应右手边），这是几何上由各边推送朝向决定的固定关系，见工程说明。
# 输入参数：edge 为目标边；near_small_end 为 True 表示障碍靠近坐标小端。
# 返回值：等效的 obs_pos 布尔值，可直接传给 _diag_avoid_sign。
def _push_pre_diag_obs_pos(edge, near_small_end):
    if edge == 3 or edge == 4:
        return not near_small_end
    return near_small_end

# 功能：障碍配置在该边"中"位置时，按被推物体实时世界坐标判断更靠近
# 该边两条垂直边界中的哪一条（哪条近就预避障往哪条靠）。
# 输入参数：edge 为目标边。
# 返回值：True 表示更靠近坐标小端边界；坐标暂时无效时兜底返回 True。
def _push_pre_diag_mid_near_small(edge):
    if edge == 1 or edge == 4:
        lateral = _target_obj_world_x
        span = _FIELD_W
    else:
        lateral = _target_obj_world_y
        span = _FIELD_H
    if lateral >= 900.0:
        return True
    return lateral <= span - lateral

# 功能：根据 config._OBSTACLE_EDGE_LAYOUT 判断目标边是否已知有障碍，
# 有则提前算好预避障方向并直接置位斜避状态机（复用 _push_diag_active 这
# 整套：起步延时、40°偏角斜行、移动 _PUSH_DIAG_DIST 后自动退出）。
# 输入参数：edge 为目标边。
# 返回值：无。该边未配置障碍时直接返回，不改动任何状态，正常 PUSH（含
# 正常视觉触发的斜避）不受影响。
def _push_pre_diag_start(edge):
    global _push_diag_active, _push_diag_phase, _push_diag_phase_t0
    global _push_diag_t0, _push_diag_start_side, _push_diag_move_yaw, _push_diag_dist
    global _push_diag_is_obstacle, _push_diag_obstacle_yaw
    global _push_pre_diag_active
    global _push_tennis_diag_sign
    global _push_redbag_diag_sign
    global _master_cmd_sub, _follower_cmd_yaw_dir
    if edge < 1 or edge > 4:
        return
    layout = _OBSTACLE_EDGE_LAYOUT[edge - 1]
    if layout[0] != 0:
        obs_pos = _push_pre_diag_obs_pos(edge, True)
    elif layout[2] != 0:
        obs_pos = _push_pre_diag_obs_pos(edge, False)
    elif layout[1] != 0:
        # "中"是"靠谁避谁"（往近的那侧边界靠），跟"小/大"已知障碍时"避开它"
        # 是相反的语义，所以这里要把 near_small_end 取反再喂给 _diag_avoid_sign
        # ——避开"远的那一端"，效果上就是往近的那一端走。
        obs_pos = _push_pre_diag_obs_pos(edge, not _push_pre_diag_mid_near_small(edge))
    else:
        return
    avoid_sign = _diag_avoid_sign(edge, obs_pos)
    move_yaw = _diag_move_yaw(edge, avoid_sign, _PUSH_PRE_DIAG_BIAS_DEG)
    if _target_obj_id == _OBJ_ID_TENNIS:
        _push_tennis_diag_sign = avoid_sign
    elif _target_obj_id == _OBJ_ID_RED_SANDBAG:
        _push_redbag_diag_sign = avoid_sign
    _push_diag_move_yaw = move_yaw
    _push_diag_start_side = _push_axis_value(base._car.Position_X, base._car.Position_Y, edge)
    move_rad = radians(move_yaw)
    side_speed_sign = sin(move_rad) if edge == 1 or edge == 4 else cos(move_rad)
    _push_diag_dist = _PUSH_DIAG_DIST if side_speed_sign >= 0.0 else -_PUSH_DIAG_DIST
    _push_diag_t0 = 0
    _push_diag_active = True
    _push_diag_phase = _PUSH_DIAG_PHASE_START_DELAY
    _push_diag_phase_t0 = ticks_ms()
    _push_diag_is_obstacle = True
    if _push_diag_obstacle_yaw >= 900.0:
        push_rad = radians(_push_yaw_for_edge(edge))
        ox = -cos(push_rad)
        oy = sin(push_rad)
        if not obs_pos:
            ox = -ox
            oy = -oy
        raw_obstacle_yaw = degrees(atan2(ox, oy)) % 360.0
        _push_diag_obstacle_yaw = (int((raw_obstacle_yaw + 45.0) // 90.0) * 90.0) % 360.0
    _push_pre_diag_active = True
    _master_cmd_sub = _CMD_SUB_DIAG_PUSH
    _follower_cmd_yaw_dir = _push_diag_move_yaw
    base.wireless_send_now()

# 功能：获取普通推送时的主车锁定角。
# 如果接近阶段已有有效规划，优先使用规划角；否则根据目标边重新计算默认锁定角。
# 输入参数：edge 为目标边。
# 返回值：锁定 yaw，单位度。
def _normal_lock_yaw(edge):
    if _approach_plan_valid and _approach_plan_edge == _target_edge and (_approach_plan_obj_id == _target_obj_id):
        return _approach_self_yaw
    return _lock_yaw(edge)

# 功能：判断当前 PUSH 中从车是否位于橙球侧。
# 主车在 push+30° 时，从车位于橙球侧；主车在 push-30° 时，从车位于紫球侧。
def _push_follower_on_orange_side():
    push_yaw = _push_yaw_for_edge(_target_edge)
    side_err = _angle_diff(_push_lock_yaw, push_yaw)
    return side_err > 1.0

# 功能：清空当前推送目标和与从车共享的推送/绕障协同状态。
# 该函数用于推送丢失后回到搜索，确保旧目标 ID、目标坐标、路线命令和从车命令不会污染下一轮任务。
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

# 功能：判断推送完成判定所需的视觉位置修正是否已经更新。
# 推送完成要求先收到对应轴的视觉修正，避免仅靠编码器误差误判已经推出边界。
# 输入参数：edge 为目标边。
# 返回值：True 表示推送完成轴上的视觉修正计数发生过变化。
def _push_done_axis_fixed(edge):
    if base._position_update_flag == 1:
        return False
    if edge == 1 or edge == 4:
        return base._vis_y_fix_cnt != _push_fix_y_snap
    if edge == 2 or edge == 3:
        return base._vis_x_fix_cnt != _push_fix_x_snap
    return False

# 功能：根据主车当前位置判断目标是否已经被推出指定边界。
# 输入参数：edge 为目标边。
# 返回值：True 表示本车已经越过对应场地边界和安全余量，可认为推送结束。
def _leader_push_done_by_pos(edge):
    if not _push_done_axis_fixed(edge):
        return False
    if edge == 1:
        return base._car.Position_Y >= _FIELD_H + _PUSH_DONE_MARGIN
    if edge == 2:
        return base._car.Position_X <= -_PUSH_DONE_MARGIN
    if edge == 3:
        return base._car.Position_X >= _FIELD_W + _PUSH_DONE_MARGIN
    if edge == 4:
        return base._car.Position_Y <= -_PUSH_DONE_MARGIN
    return False


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
# 会清空推送丢失、路线推送、诊断推送、世界横向锁定等状态，同时按当前目标加载主车推送 PID 参数。
def push_reset():
    global _push_lock_yaw, _push_ref_valid, _push_ref_rel_x, _push_ref_rel_y
    global _push_done, _push_started, _mode_sub, _push_lost_t0
    global _push_sub_t0
    global _push_spin_start_yaw, _push_peer_lost_t0
    global _push_recover_found, _push_recover_hold_yaw, _push_recover_wait_t0
    global _push_route_axis, _push_route_yaw, _push_start_t0
    global _push_world_side_axis, _push_world_side_ref
    global _push_world_side_valid
    global _push_diag_active, _push_diag_phase, _push_diag_phase_t0
    global _push_diag_t0, _push_diag_start_side, _push_diag_move_yaw, _push_diag_dist
    global _push_diag_is_obstacle
    global _push_diag_confirm_side, _push_diag_confirm_count, _push_diag_confirm_token
    global _push_diag_obstacle_yaw
    global _push_pre_diag_active
    global _push_tennis_diag_sign
    global _push_redbag_diag_sign
    global _push_fix_x_snap, _push_fix_y_snap
    _push_lock_yaw = 0.0
    _push_ref_valid = False
    _push_ref_rel_x = 0.0
    _push_ref_rel_y = 0.0
    _push_done = False
    _push_started = False
    _mode_sub = _PUSH_RUN
    _push_lost_t0 = 0
    _push_sub_t0 = 0
    _push_spin_start_yaw = 0.0
    _push_peer_lost_t0 = 0
    _push_recover_found = False
    _push_recover_hold_yaw = 0.0
    _push_recover_wait_t0 = 0
    if _push_route_phase != 2:
        _push_route_axis = 999.9
    _push_route_yaw = 999.9
    _push_start_t0 = 0
    _push_world_side_axis = 0
    _push_world_side_ref = 999.9
    _push_world_side_valid = False
    _push_diag_active = False
    _push_diag_phase = _PUSH_DIAG_PHASE_IDLE
    _push_diag_phase_t0 = 0
    _push_diag_t0 = 0
    _push_diag_start_side = 999.9
    _push_diag_move_yaw = 999.9
    _push_diag_dist = 999.9
    _push_diag_is_obstacle = False
    _push_diag_confirm_side = 0
    _push_diag_confirm_count = 0
    _push_diag_confirm_token = 0
    _push_diag_obstacle_yaw = 999.9
    _push_pre_diag_active = False
    _push_tennis_diag_sign = 999.9
    _push_redbag_diag_sign = 999.9
    _push_fix_x_snap = base._vis_x_fix_cnt
    _push_fix_y_snap = base._vis_y_fix_cnt
    base._push_world_side_cmd = 0.0
    base.reset_push()
    pid = _push_pid_params()
    base.configure_push(speed=_PUSH_SPEED_LEADER, inward_bias=_push_inward_bias(), cam_lost_tol_ms=_CAM_LOST_TOL_MS, side_kp=pid[0], side_kd=pid[1], fwd_kp=pid[2], fwd_kd=pid[3], side_max=_push_side_max(), fwd_max=_PUSH_FWD_MAX_LEADER, side_slew=_PUSH_SIDE_SLEW_LEADER, fwd_slew=_PUSH_FWD_SLEW_LEADER, d_lpf=_PUSH_D_LPF_LEADER, side_deadband=_PUSH_SIDE_DEADBAND, fwd_deadband=_PUSH_FWD_DEADBAND, side_min=_PUSH_SIDE_MIN, fwd_min=_PUSH_FWD_MIN)

# 功能：开始一次推送目标动作。
# 根据目标边设置锁定角、世界横向保持轴和视觉参考点；如果等待阶段规划了绕障路线，则先进入 ROUTE 子状态做侧向带离，否则直接进入普通推送。
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
                _push_lock_yaw = (route_yaw - deg) % 360.0
            else:
                _push_lock_yaw = (route_yaw + deg) % 360.0
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
        base.configure_push(speed=_PUSH_ROUTE_SETTLE_SPEED, inward_bias=_push_inward_bias(), cam_lost_tol_ms=_CAM_LOST_TOL_MS, side_kp=pid[0], side_kd=pid[1], fwd_kp=pid[2], fwd_kd=pid[3], side_max=_push_side_max(), fwd_max=fwd_max, side_slew=_PUSH_SIDE_SLEW_LEADER, fwd_slew=_PUSH_FWD_SLEW_LEADER, d_lpf=_PUSH_D_LPF_LEADER, side_deadband=_PUSH_SIDE_DEADBAND, fwd_deadband=_PUSH_FWD_DEADBAND, side_min=_PUSH_SIDE_MIN, fwd_min=_PUSH_FWD_MIN)
    elif _mode_sub == _PUSH_RUN:
        # 只在正常推送起步时按 config 尝试预避障；换阵列绕障路线（ROUTE）
        # 不受影响，走它自己原有的流程。
        _push_pre_diag_start(edge)
    _push_started = True

# 功能：进入目标视觉丢失后的原地旋转搜索子状态。
# 输入参数：now_ms 为当前时间戳。
def _enter_lost_spin(now_ms):
    global _self_ready
    global _mode_sub, _push_sub_t0, _push_spin_start_yaw
    global _push_recover_found, _push_recover_hold_yaw, _push_recover_wait_t0
    _self_ready = 0
    _push_recover_found = False
    _push_recover_hold_yaw = 0.0
    _push_recover_wait_t0 = 0
    _mode_sub = _PUSH_SPIN
    _push_sub_t0 = now_ms
    _push_spin_start_yaw = base._car.current_angle

# 功能：完成绕障路线推送后，恢复到普通推送方向的接近流程。
# 主车会清掉路线轴和路线 yaw，通知从车进入恢复协同命令，然后请求 APPROACH 重新形成普通夹推角。
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
    _route_cmd_to_other = -2.0
    _master_cmd_sub = _CMD_SUB_ROUTE_RESTORE
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

# 功能：把当前推送控制请求转交给 base.request_push。
# 该函数负责决定是否使用视觉闭环、是否启用世界横向锁定，以及是否附加前馈速度。
# 输入参数：edge 为目标边；lock_yaw 为锁定角；move_yaw 为指定移动方向，999.9 表示普通推送；world_side_enable 表示是否允许横向世界坐标锁定；ff_xxx 为兼容前馈的可选参数。
def _request_push_control(edge, lock_yaw, move_yaw=999.9, world_side_enable=True, ff_valid=False, ff_vx=0.0, ff_vy=0.0):
    cam_obj = _find_target_in_cam()
    visual_enable = _PUSH_VISUAL_LOOP_LEADER_ENABLE
    world_side_valid = _PUSH_WORLD_SIDE_LOCK_ENABLE and world_side_enable and (_mode_sub != _PUSH_ROUTE) and _push_world_side_valid
    if visual_enable and cam_obj >= 0:
        base.request_push(edge, lock_yaw, True, base._cam_obj_rel_x[cam_obj], base._cam_obj_rel_y[cam_obj], _push_ref_valid, _push_ref_rel_x, _push_ref_rel_y, move_yaw, world_side_valid, _push_world_side_axis, _push_world_side_ref, ff_valid, ff_vx, ff_vy)
    else:
        base.request_push(edge, lock_yaw, False, 0.0, 0.0, False, 0.0, 0.0, move_yaw, world_side_valid, _push_world_side_axis, _push_world_side_ref, ff_valid, ff_vx, ff_vy)

# 功能：主车进入推送后按球侧安排主从车的起步先后。
# 橙球侧主车立即起步，100ms 后广播 ready=4；紫球侧立即广播 ready=4，
# 主车保持100ms后再起步。GO 边沿会即时触发一次无线发送。
# 输入参数：now_ms 为当前时间戳；lock_yaw 为保持期间的车头角。
# 返回值：True 表示延时结束、可以真正推送；False 表示仍需保持等待。
def _push_start_delay_elapsed(now_ms, lock_yaw):
    global _self_ready
    global _push_start_t0
    if _push_start_t0 == 0:
        _push_start_t0 = now_ms
    elapsed = ticks_diff(now_ms, _push_start_t0)
    follower_on_orange = _push_follower_on_orange_side()
    leader_delay_ms = 0 if follower_on_orange else _PUSH_START_DELAY_LEADER_PURPLE_MS
    go_delay_ms = _PUSH_START_LEADER_ORANGE_LEAD_MS if follower_on_orange else 0
    if elapsed >= go_delay_ms:
        if _mode_sub == _PUSH_RUN or _mode_sub == _PUSH_ROUTE:
            if _self_ready != 4:
                _self_ready = 4
                # GO 边沿即时广播，不等下一个轮询节拍。
                base.wireless_send_now()
    if elapsed < leader_delay_ms:
        base.request_hold(lock_yaw)
        return False
    return True

# 功能：判断从车推送状态是否在线且足够新鲜。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示从车 ready 有效且无线时间戳未超时。
def _peer_push_ok(now_ms):
    if base._Other_Car_Ready < 1:
        return False
    if base._Other_Car_Ready_Ts == 0:
        return False
    return ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS

# 功能：推送过程中从车丢失或协同异常时，停止推送并回到接近阶段重新组织队形。
def _abort_push_to_approach(lock_yaw, from_push_lost=False):
    global _approach_from_push_lost, _follower_cmd_yaw_dir, _next_task_mode, _self_ready
    base.request_hold(lock_yaw)
    _self_ready = 0
    # PUSH 异常退出时清空上一轮全部 approach 槽位和绕向字段，避免从车
    # 在新规划生成前读取到旧 yaw/dir。新规划会在 FACE 完成后生成。
    _clear_approach_plan()
    _follower_cmd_yaw_dir = 999.9
    _approach_from_push_lost = bool(from_push_lost)
    _next_task_mode = _MODE_APPROACH

# 功能：把世界横向锁定参考值重设为当前车辆位置。
# 诊断推送结束后调用它，避免斜推造成的横向偏移继续被旧参考拉回。
def _reset_world_side_ref_to_current():
    global _push_world_side_ref
    if _push_world_side_axis == 1:
        _push_world_side_ref = base._car.Position_X
    elif _push_world_side_axis == 2:
        _push_world_side_ref = base._car.Position_Y

# 功能：清除诊断斜推命令。
# 输入参数：rebase 表示如果刚结束诊断推送，是否把横向锁定参考重设到当前车辆位置。
def _clear_diag_cmd(rebase=False):
    global _follower_cmd_yaw_dir, _master_cmd_sub
    global _push_diag_active, _push_diag_phase, _push_diag_phase_t0
    global _push_diag_t0, _push_diag_start_side, _push_diag_move_yaw, _push_diag_dist
    global _push_diag_is_obstacle
    global _push_diag_confirm_side, _push_diag_confirm_count, _push_diag_confirm_token
    was_active = _push_diag_active
    _push_diag_active = False
    _push_diag_phase = _PUSH_DIAG_PHASE_IDLE
    _push_diag_phase_t0 = 0
    _push_diag_t0 = 0
    _push_diag_start_side = 999.9
    _push_diag_move_yaw = 999.9
    _push_diag_dist = 999.9
    _push_diag_is_obstacle = False
    _push_diag_confirm_side = 0
    _push_diag_confirm_count = 0
    _push_diag_confirm_token = 0
    base._push_world_side_cmd = 0.0
    if rebase and was_active:
        _reset_world_side_ref_to_current()
    if _master_cmd_sub == _CMD_SUB_DIAG_PUSH:
        _master_cmd_sub = _CMD_SUB_NONE
        _follower_cmd_yaw_dir = 999.9

# 功能：主车诊断推送决策。
# 当目标前方走廊只有一侧被障碍占用时，主车会生成一个短时间斜推方向，同时通知从车采用相同诊断方向，尝试把目标带离障碍。
# 输入参数：now_ms 为当前时间戳；edge 为目标边；cam_obj 为当前目标摄像头槽位。
# 返回值：诊断推送 yaw；999.9 表示当前不需要诊断推送。
def _push_obstacle_valid(x, y, ts, now_ms):
    return x < 900.0 and y < 900.0 and ts != 0 and ticks_diff(now_ms, ts) <= _CONE_MEMORY_MS


def _push_obstacle_sides(now_ms, edge, obj_x, obj_y):
    # cone/brick 不再限制相对本车的前后距离；本车位置只用于筛选横向走廊。
    # 真正的左右方向和斜推距离使用障碍与当前被推物体的横向坐标差。
    push_rad = radians(_push_yaw_for_edge(edge))
    px = sin(push_rad)
    py = cos(push_rad)
    sx = -py
    sy = px
    car_x = base._car.Position_X
    car_y = base._car.Position_Y
    pos_obs = False
    neg_obs = False
    pos_gap = 999.9
    neg_gap = 999.9
    newest_ts = 0
    cone_allowed = _obstacle_layout_allowed(edge, _OBSTACLE_CONE)
    brick_allowed = _obstacle_layout_allowed(edge, _OBSTACLE_BRICK)
    obs_index = 0
    while obs_index < 4:
        if ((obs_index < 2 and not cone_allowed)
                or (obs_index >= 2 and not brick_allowed)):
            obs_index += 1
            continue
        if obs_index == 0:
            obs_x = base._cam_cone_x
            obs_y = base._cam_cone_y
            obs_ts = base._cam_cone_ts
        elif obs_index == 1:
            obs_x = base._Other_Cone_X
            obs_y = base._Other_Cone_Y
            obs_ts = base._Other_Cone_Ts
        elif obs_index == 2:
            obs_x = base._cam_brick_x
            obs_y = base._cam_brick_y
            obs_ts = base._cam_brick_ts
        else:
            obs_x = base._Other_Brick_X
            obs_y = base._Other_Brick_Y
            obs_ts = base._Other_Brick_Ts
        obs_index += 1
        if not _push_obstacle_valid(obs_x, obs_y, obs_ts, now_ms):
            continue
        dx = obs_x - car_x
        dy = obs_y - car_y
        proj_side = dx * sx + dy * sy
        if abs(proj_side) > _CONE_DIAG_HALF:
            continue
        # 对所有被推物体统一按横向世界坐标求差：沿 Y 推时比较 X，沿 X 推时比较 Y。
        # 再投影到 s_hat 的正负侧，供现有避让方向函数使用；绝对值就是用户定义的
        # 清空间距计算输入。已经达到 _PUSH_OBS_CLEAR_DIST 的障碍不触发斜避。
        obj_dx = obs_x - obj_x
        obj_dy = obs_y - obj_y
        obj_side = obj_dx * sx + obj_dy * sy
        gap = abs(obj_side)
        if gap >= _PUSH_OBS_CLEAR_DIST:
            continue
        if obs_ts > newest_ts:
            newest_ts = obs_ts
        if obj_side >= 0.0:
            pos_obs = True
            if gap < pos_gap:
                pos_gap = gap
        else:
            neg_obs = True
            if gap < neg_gap:
                neg_gap = gap
    return pos_obs, neg_obs, pos_gap, neg_gap, newest_ts


def _leader_diag_update(now_ms, edge, cam_obj):
    global _follower_cmd_yaw_dir, _master_cmd_sub
    global _push_diag_active, _push_diag_phase, _push_diag_phase_t0
    global _push_diag_t0, _push_diag_start_side, _push_diag_move_yaw, _push_diag_dist
    global _push_diag_is_obstacle
    global _push_diag_confirm_side, _push_diag_confirm_count, _push_diag_confirm_token
    global _push_diag_obstacle_yaw
    global _push_pre_diag_active
    global _push_tennis_diag_sign
    global _push_redbag_diag_sign
    if not _PUSH_DIAG_ENABLE:
        _clear_diag_cmd()
        return 999.9
    # 优先使用当前视觉物体坐标，视觉暂失时才退回起推缓存。
    # 沿 Y 推送时比较 X；沿 X 推送时比较 Y，因此所有被推物体类型共用该逻辑。
    if cam_obj >= 0:
        target_x = base._cam_obj_x[cam_obj]
        target_y = base._cam_obj_y[cam_obj]
    else:
        target_x = _target_obj_world_x
        target_y = _target_obj_world_y
    target_axis = target_x if edge == 1 or edge == 4 else target_y
    target_valid = target_axis < 900.0
    if _push_diag_active:
        tennis_pre_direction_locked = (
            _target_obj_id == _OBJ_ID_TENNIS
            and _push_tennis_diag_sign < 900.0
        )
        redbag_direction_locked = (
            _target_obj_id == _OBJ_ID_RED_SANDBAG
            and _push_redbag_diag_sign < 900.0
        )
        if (_push_pre_diag_active
                and not tennis_pre_direction_locked
                and not redbag_direction_locked
                and _push_diag_phase != _PUSH_DIAG_PHASE_EXIT_DELAY
                and target_x < 900.0 and target_y < 900.0):
            obs_pos_live, obs_neg_live, unused1, unused2, obstacle_frame_token = _push_obstacle_sides(
                now_ms, edge, target_x, target_y
            )
            if (obs_pos_live or obs_neg_live) and obstacle_frame_token != 0:
                # 预避障期间摄像头已经真正识别到障碍：放弃预判方向，交回下面
                # 的正常视觉触发检测逻辑，让它按真实数据重新确认方向。
                _push_pre_diag_active = False
                # 预避障启动时写入的障碍侧只是按布局推测的临时值。真实障碍
                # 出现后必须一并作废，否则后续正常检测因“已有锁存值”不会刷新，
                # RECOVER 就可能把与实际斜避方向相反的旧方向发给从车。
                _push_diag_obstacle_yaw = 999.9
                _clear_diag_cmd()
                return 999.9
        if _push_diag_phase == _PUSH_DIAG_PHASE_EXIT_DELAY:
            # 退出命令已经先发给从车；主车继续保持斜推100 ms，再恢复普通 PUSH。
            _master_cmd_sub = _CMD_SUB_NONE
            _follower_cmd_yaw_dir = 999.9
            if ticks_diff(now_ms, _push_diag_phase_t0) < _PUSH_DIAG_LEADER_DELAY_MS:
                return _push_diag_move_yaw
            _clear_diag_cmd(True)
            return 999.9
        _master_cmd_sub = _CMD_SUB_DIAG_PUSH
        _follower_cmd_yaw_dir = _push_diag_move_yaw
        if _push_diag_phase == _PUSH_DIAG_PHASE_START_DELAY:
            # 斜避命令先广播，从车收到后先执行；主车沿用 PUSH 起步的100 ms补偿。
            if ticks_diff(now_ms, _push_diag_phase_t0) < _PUSH_DIAG_LEADER_DELAY_MS:
                return 999.9
            _push_diag_phase = _PUSH_DIAG_PHASE_ACTIVE
            _push_diag_phase_t0 = now_ms
            # 斜避距离和超时均从主车真正开始斜推时计算，不包含命令提前量。
            _push_diag_start_side = _push_axis_value(base._car.Position_X, base._car.Position_Y, edge)
            _push_diag_t0 = now_ms
        # 完成条件使用【本车横向避障轴】的带符号里程计位移：
        # 边1/4检查 X，边2/3检查 Y；_push_diag_dist 的符号锁定首次避障方向。
        cur_side = _push_axis_value(base._car.Position_X, base._car.Position_Y, edge)
        side_delta = cur_side - _push_diag_start_side
        diag_dist = _push_diag_dist
        diag_max_ms = _CONE_DIAG_MAX_MS if _push_diag_is_obstacle else _PUSH_DIAG_MAX_MS
        if diag_dist >= 0.0:
            done_dist = _push_diag_start_side < 900.0 and side_delta >= diag_dist
        else:
            done_dist = _push_diag_start_side < 900.0 and side_delta <= diag_dist
        done_time = _push_diag_t0 != 0 and ticks_diff(now_ms, _push_diag_t0) >= diag_max_ms
        if done_dist or done_time:
            # 先广播退出，让从车恢复普通推送；主车延时100 ms后再退出斜避。
            _push_diag_phase = _PUSH_DIAG_PHASE_EXIT_DELAY
            _push_diag_phase_t0 = now_ms
            _master_cmd_sub = _CMD_SUB_NONE
            _follower_cmd_yaw_dir = 999.9
            base.wireless_send_now()
            return _push_diag_move_yaw
        base.avoid_beep(1)
        return _push_diag_move_yaw
    half = _PUSH_DIAG_HALF
    low_obs = high_obs = False
    obs_frame_token = 0
    # 待搬物体挡路（依赖物体坐标，仅在其有效时判）。
    if target_valid:
        self_obs = _route_obs_axis_to_other
        other_obs = 999.9
        other_fresh = base._Other_Car_Ready_Ts != 0 and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS
        if other_fresh and (base._Other_Car_Mode == _MODE_PUSH_SYNC and base._Other_Target_Edge == _target_edge and (base._Other_Target_ObjId == _target_obj_id)):
            other_obs = base._Other_Route_Obs_Axis
        for ax, obs_ts in (
            (self_obs, base._cam_rx_last_ms),
            (other_obs, base._Other_Car_Ready_Ts),
        ):
            if ax < 900.0 and target_axis - half <= ax <= target_axis + half:
                if obs_ts > obs_frame_token:
                    obs_frame_token = obs_ts
                if ax <= target_axis:
                    low_obs = True
                else:
                    high_obs = True
    # cone/brick 挡路：不限制前后距离；本车参考系只筛选横向走廊，
    # 物体参考系计算左右方向和所需斜推距离。
    if target_x < 900.0 and target_y < 900.0:
        obs_pos, obs_neg, obs_pos_gap, obs_neg_gap, obstacle_frame_token = _push_obstacle_sides(
            now_ms, edge, target_x, target_y
        )
        if obstacle_frame_token > obs_frame_token:
            obs_frame_token = obstacle_frame_token
    else:
        obs_pos = False
        obs_neg = False
        obs_pos_gap = 999.9
        obs_neg_gap = 999.9
        obstacle_frame_token = 0
    obstacle_active = obs_pos or obs_neg
    # 待搬物体的 low/high（物体轴参考）换算成 s_hat 侧向：low=s负侧、high=s正侧，
    # 与 _push_obstacle_sides 统一（详见 _diag_avoid_sign 的符号推导）。
    if high_obs:
        obs_pos = True
    if low_obs:
        obs_neg = True
    if obs_pos == obs_neg:
        # 两侧都有障碍 或 都没有：无法避让，退出。
        _clear_diag_cmd()
        return 999.9
    # 同一侧必须在两个不同的新观测时间戳中连续成立。同一个相机帧会被
    # 主循环重复处理多次，时间戳去重可避免把单帧误算成连续两帧。
    confirm_side = 1 if obs_pos else -1
    if obs_frame_token == 0:
        _clear_diag_cmd()
        return 999.9
    if _push_diag_confirm_side != confirm_side:
        _push_diag_confirm_side = confirm_side
        _push_diag_confirm_count = 1
        _push_diag_confirm_token = obs_frame_token
        return 999.9
    if obs_frame_token == _push_diag_confirm_token:
        return 999.9
    _push_diag_confirm_token = obs_frame_token
    _push_diag_confirm_count += 1
    if _push_diag_confirm_count < _PUSH_DIAG_CONFIRM_FRAMES:
        return 999.9
    _push_diag_confirm_side = 0
    _push_diag_confirm_count = 0
    _push_diag_confirm_token = 0
    _push_diag_is_obstacle = obstacle_active
    if obstacle_active and _push_diag_obstacle_yaw >= 900.0:
        # s_hat=(-push_y, push_x)。obs_pos/obs_neg 是障碍物在该侧向轴上的
        # 正负侧；锁存障碍物所在侧，而不是斜推合成后的移动 yaw。
        push_rad = radians(_push_yaw_for_edge(edge))
        obs_x = -cos(push_rad)
        obs_y = sin(push_rad)
        if not obs_pos:
            obs_x = -obs_x
            obs_y = -obs_y
        raw_obstacle_yaw = degrees(atan2(obs_x, obs_y)) % 360.0
        _push_diag_obstacle_yaw = (int((raw_obstacle_yaw + 45.0) // 90.0) * 90.0) % 360.0
    if obstacle_active:
        obstacle_gap = obs_pos_gap if obs_pos else obs_neg_gap
        _push_diag_dist = max(0.0, _PUSH_OBS_CLEAR_DIST - obstacle_gap)
    else:
        _push_diag_dist = _PUSH_DIAG_DIST
    # 障碍在 s_hat 正侧就往负侧偏，反之亦然：side_sign 使偏置的 s 分量与障碍反向。
    avoid_sign = _diag_avoid_sign(edge, obs_pos)
    bias_deg = _CONE_DIAG_BIAS_DEG if obstacle_active else _PUSH_DIAG_BIAS_DEG
    tennis_locked = _target_obj_id == _OBJ_ID_TENNIS and _push_tennis_diag_sign < 900.0
    redbag_locked = _target_obj_id == _OBJ_ID_RED_SANDBAG and _push_redbag_diag_sign < 900.0
    if tennis_locked:
        # 网球本轮已锁存过斜避侧向（avoid_sign，不是最终 yaw）：不管这次
        # 障碍在哪一侧，都严格复用锁存的侧向；偏转角度仍按下面的正常规则
        # （视觉确认到真实障碍用 _CONE_DIAG_BIAS_DEG，否则用 _PUSH_DIAG_BIAS_DEG）
        # 实时取值，不锁角度大小，只锁往哪一侧偏。
        avoid_sign = _push_tennis_diag_sign
    elif redbag_locked:
        # 红沙包同上，只锁侧向，角度仍按正常规则计算。
        avoid_sign = _push_redbag_diag_sign
    _push_diag_move_yaw = _diag_move_yaw(edge, avoid_sign, bias_deg)
    if _target_obj_id == _OBJ_ID_TENNIS and not tennis_locked:
        # 网球本轮第一次真正确定斜避侧向：锁存供后续复用。
        _push_tennis_diag_sign = avoid_sign
    elif _target_obj_id == _OBJ_ID_RED_SANDBAG and not redbag_locked:
        # 红沙包本轮第一次真正确定斜避侧向：锁存供后续复用。
        _push_redbag_diag_sign = avoid_sign
    _push_diag_start_side = _push_axis_value(base._car.Position_X, base._car.Position_Y, edge)
    move_rad = radians(_push_diag_move_yaw)
    side_speed_sign = sin(move_rad) if edge == 1 or edge == 4 else cos(move_rad)
    if side_speed_sign < 0.0:
        _push_diag_dist = -_push_diag_dist
    _push_diag_t0 = 0
    _push_diag_active = True
    _push_diag_phase = _PUSH_DIAG_PHASE_START_DELAY
    _push_diag_phase_t0 = now_ms
    _master_cmd_sub = _CMD_SUB_DIAG_PUSH
    _follower_cmd_yaw_dir = _push_diag_move_yaw
    base._push_world_side_cmd = 0.0
    base.avoid_beep(1)
    base.wireless_send_now()
    return 999.9


# 功能：更新推送阶段的障碍观测并运行诊断推送决策。
# 输入参数：now_ms 为当前时间戳；edge 为目标边；cam_obj 为当前目标摄像头槽位。
# 返回值：诊断推送 yaw；999.9 表示不启用诊断方向。
def _push_diag_update(now_ms, edge, cam_obj):
    try:
        _update_push_obs()
    except Exception:
        pass
    return _leader_diag_update(now_ms, edge, cam_obj)

# 功能：更新推送同步模式。
# 该函数处理推送起步一帧等待、从车掉线保护、目标视觉丢失后退/旋转搜索、绕障路线推送、诊断斜推和推出边界后的完成切换。
# 输入参数：now_ms 为当前时间戳；为 None 时函数内部读取 ticks_ms()。
def push_update(now_ms=None):
    global _recover_edge
    global _follower_cmd_yaw_dir, _master_cmd_sub, _next_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _route_cmd_to_other, _self_sub, _self_ready, _target_obj_world_x, _target_obj_world_y
    global _push_done, _push_started, _push_lost_t0, _push_peer_lost_t0
    global _push_sub_t0, _push_start_t0
    global _push_recover_found, _push_recover_hold_yaw, _push_recover_wait_t0
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
                _push_recover_wait_t0 = now_ms
            _self_ready = _PUSH_RECOVER_FOUND_READY
            base.request_hold(_push_recover_hold_yaw)
            other_found = (
                base._Other_Car_Mode == _MODE_PUSH_SYNC
                and base._Other_Car_Ready == _PUSH_RECOVER_FOUND_READY
                and base._Other_Car_Ready_Ts != 0
                and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _PUSH_PEER_FRESH_MS
            )
            if other_found:
                _abort_push_to_approach(lock_yaw, True)
                return
            if ticks_diff(now_ms, _push_recover_wait_t0) > _PUSH_RECOVER_WAIT_MS:
                _clear_push_target_lock()
                _next_task_mode = _MODE_SEARCH
                return
            return
        elapsed = ticks_diff(now_ms, _push_sub_t0)
        if elapsed >= _PUSH_LOST_SPIN_MS:
            base.request_hold(base._car.current_angle)
            _clear_push_target_lock()
            _next_task_mode = _MODE_SEARCH
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
        if not _push_route_cruise_applied:
            if _push_route_settle_t0 == 0:
                _push_route_settle_t0 = now_ms
            elif ticks_diff(now_ms, _push_route_settle_t0) >= _PUSH_ROUTE_SETTLE_MS:
                _push_route_cruise_applied = True
                pid = _push_pid_params()
                fwd_max = _PUSH_ROUTE_GUIDE_FWD_MAX if _push_route_is_guide else _PUSH_ROUTE_PUSH_FWD_MAX
                base.configure_push(speed=_PUSH_ROUTE_SPEED, inward_bias=_push_inward_bias(), cam_lost_tol_ms=_CAM_LOST_TOL_MS, side_kp=pid[0], side_kd=pid[1], fwd_kp=pid[2], fwd_kd=pid[3], side_max=_push_side_max(), fwd_max=fwd_max, side_slew=_PUSH_SIDE_SLEW_LEADER, fwd_slew=_PUSH_FWD_SLEW_LEADER, d_lpf=_PUSH_D_LPF_LEADER, side_deadband=_PUSH_SIDE_DEADBAND, fwd_deadband=_PUSH_FWD_DEADBAND, side_min=_PUSH_SIDE_MIN, fwd_min=_PUSH_FWD_MIN)
        if cam_obj < 0:
            if _push_lost_t0 == 0:
                _push_lost_t0 = now_ms
            elif ticks_diff(now_ms, _push_lost_t0) > _PUSH_LOST_CONFIRM_MS:
                _enter_lost_spin(now_ms)
                return
        elif cam_obj >= 0:
            _push_lost_t0 = 0
            _target_obj_world_x = base._cam_obj_x[cam_obj]
            _target_obj_world_y = base._cam_obj_y[cam_obj]
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
    if cam_obj >= 0:
        _push_lost_t0 = 0
    elif _DISABLE_NORMAL_TENNIS_LOST_SEARCH and _target_obj_id == _OBJ_ID_TENNIS:
        # 普通 PUSH（包含预避障/视觉斜避覆盖层）中，网球丢失后继续按固定参考
        # 推向边界。即使本轮此前执行过 ROUTE，也不能在回到 PUSH_RUN 后退出。
        _push_lost_t0 = 0
    elif _push_lost_t0 == 0:
        _push_lost_t0 = now_ms
    elif ticks_diff(now_ms, _push_lost_t0) > _PUSH_LOST_CONFIRM_MS:
        _enter_lost_spin(now_ms)
        return
    diag_yaw = _push_diag_update(now_ms, edge, cam_obj)
    if diag_yaw < 900.0:
        _request_push_control(edge, lock_yaw, diag_yaw, True)
    else:
        _request_push_control(edge, lock_yaw)
    if _leader_push_done_by_pos(edge):
        # PUSH 直接切入 RECOVER 时必须先清掉向外的推送速度；否则缓停残速会在
        # BACK 开始后继续把车带向场外，并可能被位移判据误认为已经完成后退。
        base.request_hard_hold(lock_yaw)
        recover_edge = _target_edge
        _mark_target_done()
        _self_ready = 0
        _push_route_axis = 999.9
        _push_route_phase = 0
        _push_route_move_yaw = 999.9
        _push_route_restore_push = 0
        _route_cmd_to_other = 999.9
        _master_cmd_sub = _CMD_SUB_PUSH_DONE
        _follower_cmd_yaw_dir = 999.9
        base._push_world_side_cmd = 0.0
        set_recover_target(recover_edge)
        _next_task_mode = _MODE_RECOVER

_READY_EXIT_POS_EPS = const(12)

_READY_EXIT_LAT_EPS = const(12)

_route_avoid_obs_logged = False

_route_avoid_obs_prev = 999.9

# 功能：确认主车在 WAIT_READY 阶段是否仍保持在适合开始推送的位置。
# 只有目标仍在视觉中，且相对前后距离和横向偏差都接近预期，才允许进入同步推送。
# 返回值：True 表示当前靠近姿态仍可用；False 表示需要重新接近。
def target_still_ok():
    cam_rel = _find_target_in_cam()
    if cam_rel < 0:
        return False
    rel_x = base._cam_obj_rel_x[cam_rel]
    rel_y = base._cam_obj_rel_y[cam_rel]
    obj_id = _target_obj_id
    if 0 <= obj_id < len(_CLOSE_LAT_OFFSET_BY_OBJ):
        lat_offset = _CLOSE_LAT_OFFSET_BY_OBJ[obj_id]
    else:
        lat_offset = 0.0
    close_dist = _CLOSE_DIST_LEADER_BY_OBJ[obj_id] if 0 <= obj_id < len(_CLOSE_DIST_LEADER_BY_OBJ) else _FORM_BACK_DIST
    return abs(rel_y - close_dist) < _READY_EXIT_POS_EPS and abs(rel_x - lat_offset) < _READY_EXIT_LAT_EPS

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

# 功能：根据清障轴坐标计算目标需要横向移动的方向。
# 输入参数：clear_axis 为希望目标移动到的清障轴坐标。
# 返回值：路线移动 yaw；目标坐标或清障轴无效时返回 999.9。
def route_move_yaw(clear_axis):
    edge = _target_edge
    ref_x = _target_obj_world_x
    ref_y = _target_obj_world_y
    obj_axis = _route_axis_value(ref_x, ref_y, _route_axis_is_x(edge))
    if obj_axis >= 900.0 or clear_axis >= 900.0:
        return 999.9
    if edge == 1 or edge == 4:
        return 90.0 if clear_axis >= obj_axis else 270.0
    if edge == 2 or edge == 3:
        return 0.0 if clear_axis >= obj_axis else 180.0
    return 999.9

# 功能：更新等待阶段对目标前方推送走廊的障碍观测。
# 主车会先融合目标地图，再在目标前方一定距离和宽度范围内寻找未完成物体，记录最可能挡路的障碍轴坐标并共享给从车。
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
    _update_obj_world(now_ms)
    _prune_obj_map(now_ms)
    axis_is_x = _route_axis_is_x(_target_edge)
    target_axis = _route_axis_value(_target_obj_world_x, _target_obj_world_y, axis_is_x)
    push_rad = radians(_push_yaw(_target_edge))
    px = sin(push_rad)
    py = cos(push_rad)
    sx = -py
    sy = px
    half = _READY_ROUTE_SAFE_HALF
    want_high = _route_axis_value(base._car.Position_X, base._car.Position_Y, axis_is_x) < target_axis
    best_idx = -1
    best_axis = 0.0
    for i in range(_obj_map_count):
        obj_type = _obj_map_type[i]
        if obj_type == _target_obj_id or obj_type == _OBJ_ID_CAR:
            continue
        if not 1 <= obj_type < len(_obj_remain):
            continue
        if _obj_remain[obj_type] <= 0:
            continue
        ox = _obj_map_x[i]
        oy = _obj_map_y[i]
        if ox >= 900.0 or oy >= 900.0:
            continue
        dx = ox - _target_obj_world_x
        dy = oy - _target_obj_world_y
        proj_fwd = dx * px + dy * py
        proj_side = dx * sx + dy * sy
        if proj_fwd <= 0.0:
            continue
        if proj_fwd < _READY_ROUTE_FWD_MIN_DIST:
            continue
        if proj_fwd > _READY_ROUTE_FWD_MAX_DIST:
            continue
        if abs(proj_side) > half:
            continue
        axis = _route_axis_value(ox, oy, axis_is_x)
        if best_idx < 0:
            best_idx = i
            best_axis = axis
        elif want_high and axis > best_axis:
            best_idx = i
            best_axis = axis
        elif not want_high and axis < best_axis:
            best_idx = i
            best_axis = axis
    global _route_avoid_obs_prev
    if best_idx < 0:
        _route_avoid_obs_prev = 999.9
        _route_obs_axis_to_other = 999.9
        if _push_route_phase not in (1, 3):
            _route_cmd_to_other = 999.9
        return
    if abs(best_axis - _route_avoid_obs_prev) > 2.0:
        _route_avoid_obs_prev = best_axis
    _route_obs_axis_to_other = best_axis

# 功能：判断某个障碍轴坐标是否位于目标推送走廊内。
# 输入参数：obs_axis 为障碍在当前清障轴上的坐标。
# 返回值：True 表示该障碍落在目标推送走廊宽度范围内。
def _route_obs_in_push_corridor(obs_axis):
    target_axis = _route_axis_value(_target_obj_world_x, _target_obj_world_y, _route_axis_is_x(_target_edge))
    half = _READY_ROUTE_SAFE_HALF
    return obs_axis < 900.0 and target_axis - half <= obs_axis <= target_axis + half

# 功能：结合主车和从车观测，选择一个可共享的清障轴坐标。
# 如果任一车辆看到目标推送走廊中有障碍，就按目标配置和推送方向选择从高侧或低侧绕开。
# 返回值：清障轴坐标；999.9 表示当前不需要绕障。
def _route_shared_clear_axis():
    target_axis = _route_axis_value(_target_obj_world_x, _target_obj_world_y, _route_axis_is_x(_target_edge))
    half = _READY_ROUTE_SAFE_HALF
    margin = _READY_ROUTE_MARGIN
    if not (_route_obs_in_push_corridor(_route_obs_axis_to_other) or _route_obs_in_push_corridor(base._Other_Route_Obs_Axis)):
        return 999.9
    low_clear = target_axis - half - margin
    high_clear = target_axis + half + margin
    forced = _READY_ROUTE_OBJ_DIR[_target_obj_id]
    push_rad = radians(_push_yaw(_target_edge))
    if _route_axis_is_x(_target_edge):
        right_is_high = cos(push_rad) > 0
    else:
        right_is_high = -sin(push_rad) > 0
    if forced == 1:
        return high_clear if right_is_high else low_clear
    return low_clear if right_is_high else high_clear

# 功能：根据当前障碍观测更新给从车的绕障命令。
# 主车会把清障轴、路线移动 yaw 和命令子状态写入共享变量；若障碍消失，则清空相关命令。
# 返回值：清障轴坐标；999.9 表示未生成绕障命令。
def _route_update_leader_cmd_for_follower():
    global _follower_cmd_yaw_dir, _master_cmd_sub, _push_route_axis, _push_route_move_yaw, _route_cmd_to_other
    global _route_avoid_obs_logged
    if not _READY_ROUTE_AVOID_ENABLE:
        _route_avoid_obs_logged = False
        _route_cmd_to_other = 999.9
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        _follower_cmd_yaw_dir = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
        return 999.9
    _route_cmd_to_other = -1.0
    clear_axis = _route_shared_clear_axis()
    if clear_axis < 900.0:
        if not _route_avoid_obs_logged:
            _route_avoid_obs_logged = True
        _route_cmd_to_other = clear_axis
        _push_route_axis = clear_axis
        _push_route_move_yaw = route_move_yaw(clear_axis)
    else:
        _route_avoid_obs_logged = False
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        _follower_cmd_yaw_dir = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
    return clear_axis

# 功能：WAIT_READY 中更新绕障路线规划。
# 只有主车当前仍贴近目标时才继续规划；若发现推送走廊被挡，会生成路线命令并等待重新接近。
# 输入参数：now_ms 为当前时间戳。
# 返回值：True 表示路线检查完成或已生成路线信息；False 表示目标姿态不合格。
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
    _route_update_leader_cmd_for_follower()
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

# 功能：重置等待从车就位阶段的状态。
# 会清空靠近姿态异常计时，并锁定等待阶段使用的车头角。
def ready_reset():
    global _ready_close_bad_t0
    global _wait_lock_yaw, _wait_lock_valid
    _ready_close_bad_t0 = 0
    _wait_lock_yaw = 0.0
    _wait_lock_valid = False


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


# 功能：更新 WAIT_READY 模式。
# 主车在这里保持接近完成时的角度，检查目标是否仍在合适位置，观测推送走廊是否需要绕障，并等待从车也进入 WAIT_READY 且 ready 有效。
# 如果目标姿态长时间变差，会请求重新接近；如果两车都就绪且路径满足条件，就进入 PUSH_SYNC。
# 输入参数：now_ms 为当前时间戳。
def ready_update(now_ms):
    global _approach_req
    global _follower_cmd_yaw_dir, _master_cmd_sub, _next_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _route_cmd_to_other, _self_ready, _target_obj_world_x, _target_obj_world_y
    global _wait_lock_yaw, _wait_lock_valid
    global _ready_close_bad_t0
    if not _wait_lock_valid:
        _self_ready = 1
        _wait_lock_yaw = base._car.current_angle
        _wait_lock_valid = True
    if debug_switch.READY_HOLD_ENABLE:
        _self_ready = 1
        base.request_hold(_wait_lock_yaw)
        return
    if _find_target_in_cam() < 0:
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
    route_phase = _push_route_phase
    if route_phase == 2:
        axis = _push_route_axis
        if axis < 900.0:
            _route_cmd_to_other = axis
            _master_cmd_sub = _CMD_SUB_ROUTE_PUSH
    if route_phase == 0:
        if base._Other_Car_Mode != _MODE_WAIT_READY or base._Other_Car_Ready < 1 or ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _READY_OTHER_FRESH_MS:
            _route_cmd_to_other = 999.9
            _push_route_axis = 999.9
            _push_route_move_yaw = 999.9
            _follower_cmd_yaw_dir = 999.9
            _master_cmd_sub = _CMD_SUB_NONE
            return
    if route_phase in (1, 2, 3, 4):
        route_ok = True
    else:
        route_ok = update_ready_route(now_ms)
    if not route_ok:
        return
    route_phase = _push_route_phase
    route_axis = _push_route_axis
    if route_phase == 0 and route_axis < 900.0 and (_push_route_restore_push == 0):
        route_yaw = _push_route_move_yaw
        if route_yaw >= 900.0:
            route_yaw = route_move_yaw(route_axis)
            _push_route_move_yaw = route_yaw
        _push_route_phase = 1
        if _start_route_reclose(route_yaw):
            return
        _push_route_phase = 0
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        _follower_cmd_yaw_dir = 999.9
        _route_cmd_to_other = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
        _push_route_restore_push = 0
    if route_phase == 4:
        _push_route_restore_push = 1
        _push_route_phase = 0
        _push_route_axis = 999.9
        _push_route_move_yaw = 999.9
        _route_cmd_to_other = 999.9
        _follower_cmd_yaw_dir = 999.9
        _master_cmd_sub = _CMD_SUB_NONE
    if base._Other_Car_Mode != _MODE_WAIT_READY:
        return
    if base._Other_Car_Ready < 1:
        return
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _READY_OTHER_FRESH_MS:
        return
    if not target_still_ok():
        _self_ready = 0
        return
    if base._Other_Car_Mode != _MODE_WAIT_READY:
        return
    if base._Other_Car_Ready < 1:
        return
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _READY_OTHER_FRESH_MS:
        return
    _do_push_start()

_SUB_BACK = const(0)

_SUB_VIS_FIX = const(1)

_SUB_YAW_FIX = const(2)

_SUB_WAIT_FOLLOWER_FOLLOW = const(3)

_SUB_SEARCH_YAW = const(4)

_RECOVER_BACK_SPEED = const(80)

# BACK 正常以后退位移达到 8 cm 退出；1500 ms 只作为里程计异常或车体卡住时的超时保护。
_RECOVER_BACK_DIST = 8.0

_RECOVER_BACK_TIMEOUT_MS = const(2000)

_TASK_RECOVER_DEPLOY_TURN_EPS_DEG = const(4)

_RECOVER_FIX_SPEED = const(90)

_RECOVER_FIX_TIMEOUT_MS = const(2000)

_RECOVER_FIX_THRESH = const(50)

_RECOVER_FIX_REQUIRE = const(2)

# 从车 SEARCH 内部进入纯平移跟随时上报的子状态编号，与 task_follow.py
# 的 _FSEARCH_TRANSLATE_FORM 保持一致。RECOVER 仍用它确认从车已开始跟随。
_FOLLOWER_SEARCH_TRANSLATE_SUB = const(5)

# 坐标修正结束后的黄线视觉航向精调。视觉角度仅在目标正交航向附近作为
# 局部误差使用：±2 度内为死区并累计稳定，超过 ±25 度视为无效。
_RECOVER_YAW_SETTLE_MS = const(120)

_RECOVER_YAW_TIMEOUT_MS = const(1000)

_RECOVER_YAW_OK_HOLD_MS = const(100)

_RECOVER_YAW_OK_REQUIRE = const(1)

_RECOVER_YAW_EPS = 2.0

_RECOVER_YAW_VALID_GATE = 25.0

# 视觉误差只判断旋转方向，不参与速度大小计算；自转始终使用固定世界 yaw 角速度。
_RECOVER_YAW_RATE_DPS = 50.0

_RECOVER_SEARCH_YAW_EPS_DEG = 6.0

_RECOVER_SEARCH_YAW_TIMEOUT_MS = const(1500)

_mode_sub = _SUB_BACK

_mode_hold_ms = 0

_recover_back_yaw = 0.0

_recover_sub_t0 = 0

_recover_back_x0 = 0.0

_recover_back_y0 = 0.0

_recover_edge = 0

_recover_fix_x_snap = 0

_recover_fix_y_snap = 0

_recover_fix_ok_cnt = 0

_recover_fix_finished = False

_recover_fix_drive_t0 = 0

_recover_yaw_last_cam_ms = 0

_recover_yaw_ok_cnt = 0

_recover_yaw_ok_t0 = 0

_recover_yaw_rate_cmd = 0.0

_recover_wait_yaw = 0.0

# 功能：关闭恢复阶段使用的视觉位置修正开关。
def _clear_recover_fix_flags():
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False

# 功能：根据刚推出的目标边选择恢复阶段的视觉修正方案。
# 输入参数：edge 为刚完成推送的目标边。
# 返回值：六元组，包含是否启用修正、修正轴、目标 yaw、车体系 vx/vy 和是否靠高侧边界。
def _recover_fix_cfg(edge):
    if edge == 1:
        return (True, 'y', 0.0, 0.0, _RECOVER_FIX_SPEED, True)
    if edge == 2:
        return (True, 'x', 270.0, 0.0, _RECOVER_FIX_SPEED, False)
    if edge == 3:
        return (True, 'x', 90.0, 0.0, _RECOVER_FIX_SPEED, True)
    if edge == 4:
        return (True, 'y', 180.0, 0.0, _RECOVER_FIX_SPEED, False)
    return (False, '', 0.0, 0.0, 0.0, False)

# 功能：判断恢复阶段收到的视觉修正值是否位于预期边界附近。
# 输入参数：axis 为修正轴，取 'x' 或 'y'；high_side 表示期望靠近高坐标边界还是低坐标边界。
# 返回值：True 表示视觉修正值位于距预期边界 50 cm 的修正范围内。
def _recover_fix_value_ok(axis, high_side):
    if axis == 'x':
        val = base._Position_X_fix
        if val >= 900.0:
            return False
        return val > _FIELD_W - _RECOVER_FIX_THRESH if high_side else val < _RECOVER_FIX_THRESH
    val = base._Position_Y_fix
    if val >= 900.0:
        return False
    return val > _FIELD_H - _RECOVER_FIX_THRESH if high_side else val < _RECOVER_FIX_THRESH


# 功能：按车体系速度发出世界速度请求。
# 输入参数：vx/vy 为车体系速度，angle 为目标车头角。
def _set_body(vx, vy, angle):
    yaw = radians(base._car.current_angle)
    c = cos(yaw)
    s = sin(yaw)
    base.request_world(c * vx + s * vy, -s * vx + c * vy, angle)


# 功能：清空当前目标和目标地图。
# 恢复结束或任务重新搜索前调用，避免旧目标继续参与后续规划。
def _clear_target():
    global _obj_map_count, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y
    _clear_recover_fix_flags()
    _target_obj_world_x = 999.0
    _target_obj_world_y = 999.0
    _target_edge = 0
    _target_obj_id = 0
    _obj_map_count = 0

# 功能：结束 RECOVER 阶段。
# 如果全部目标都完成则进入 RELOCALIZE，否则清空当前目标后回到 SEARCH，继续处理剩余目标。
# 输入参数：yaw 为结束时保持的车头角；为 None 时保持当前角度。
def _finish_recover(yaw=None):
    global _next_task_mode
    _clear_target()
    base.request_hold(base._car.current_angle if yaw is None else yaw)
    if _all_objects_done():
        _next_task_mode = _MODE_RELOCALIZE
    else:
        _next_task_mode = _MODE_SEARCH


def _publish_recover_diag_obstacle():
    """在有剩余目标时，把本轮斜避障碍方向持续发给从车。"""
    global _master_cmd_sub, _follower_cmd_yaw_dir
    if _push_diag_obstacle_yaw < 900.0 and not _all_objects_done():
        _master_cmd_sub = _CMD_SUB_DIAG_PUSH
        _follower_cmd_yaw_dir = _push_diag_obstacle_yaw
    else:
        # 未触发斜避时保留原有 PUSH_DONE 命令时序；只有曾经发布过本命令才清理。
        if _master_cmd_sub == _CMD_SUB_DIAG_PUSH:
            _master_cmd_sub = _CMD_SUB_NONE
        _follower_cmd_yaw_dir = 999.9

# 功能：判断从车是否已进入下一阶段并真正开始跟随主车。
# 从车进入 SEARCH/TRANSLATE_FORM 且明确发布 ready=2 后，才确认新的
# “世界坐标到主车后方 + 原地修正 search_yaw”编队握手已经完成。
def _recover_follower_follow_ready(now_ms):
    if base._Other_Car_Ready_Ts == 0:
        return False
    if ticks_diff(now_ms, base._Other_Car_Ready_Ts) > _READY_OTHER_FRESH_MS:
        return False
    # 从车 RECOVER 找车完成后仍留在 RECOVER，以 ready=2 表示主车视觉 x 已进入 ±10 cm。
    # 主车先据此结束本轮 RECOVER；从车收到新的 SEARCH/RELOCALIZE 模式后再跟随切换。
    if base._Other_Car_Mode == _MODE_RECOVER:
        return base._Other_Car_Ready >= 2
    if base._Other_Car_Mode == _MODE_SEARCH:
        return (
            base._Other_Car_Push_Sub == _FOLLOWER_SEARCH_TRANSLATE_SUB
            and base._Other_Car_Ready >= 2
        )
    if base._Other_Car_Mode == _MODE_RELOCALIZE:
        return base._Other_Car_Ready >= 2
    return False

# 功能：主车完成 RECOVER 航向修正后进入静止等待。
# 等待期间保持修正后的航向，直到从车已经进入实际跟随阶段。
def _recover_search_yaw_target():
    if _last_completed_push_yaw >= 900.0:
        return base._car.current_angle
    raw_yaw = (_last_completed_push_yaw + 180.0) % 360.0
    return (int((raw_yaw + 45.0) // 90.0) * 90.0) % 360.0


def _begin_recover_search_yaw(now_ms, fallback_yaw):
    global _mode_sub, _recover_sub_t0, _recover_wait_yaw, _self_ready
    if _all_objects_done() or _last_completed_push_yaw >= 900.0:
        _begin_recover_wait_follower(fallback_yaw)
        return
    _recover_wait_yaw = _recover_search_yaw_target()
    _recover_sub_t0 = now_ms
    _self_ready = 0
    _mode_sub = _SUB_SEARCH_YAW
    base.request_world(0.0, 0.0, _recover_wait_yaw)


def _begin_recover_wait_follower(yaw):
    global _mode_sub, _recover_wait_yaw, _recover_yaw_rate_cmd, _self_ready
    _clear_recover_fix_flags()
    _recover_wait_yaw = yaw
    _recover_yaw_rate_cmd = 0.0
    _self_ready = 0
    _mode_sub = _SUB_WAIT_FOLLOWER_FOLLOW
    base.request_hold(yaw)

# 功能：重置恢复阶段状态机。
# 恢复阶段通常发生在目标推出边界后，主车先后退，再依次用边界视觉修正位置和航向。
def recover_reset():
    global _self_ready, _search_yaw_pre_aligned
    global _mode_sub, _mode_hold_ms, _recover_sub_t0
    global _recover_back_x0, _recover_back_y0
    global _recover_fix_x_snap, _recover_fix_y_snap, _recover_fix_ok_cnt, _recover_fix_finished
    global _recover_fix_drive_t0
    global _recover_yaw_last_cam_ms, _recover_yaw_ok_cnt, _recover_yaw_ok_t0, _recover_yaw_rate_cmd
    global _recover_wait_yaw
    global _recover_edge
    _recover_edge = _recover_edge
    _mode_sub = _SUB_BACK
    _mode_hold_ms = 0
    _recover_sub_t0 = 0
    _recover_back_x0 = 0.0
    _recover_back_y0 = 0.0
    _recover_fix_x_snap = 0
    _recover_fix_y_snap = 0
    _recover_fix_ok_cnt = 0
    _recover_fix_finished = False
    _recover_fix_drive_t0 = 0
    _recover_yaw_last_cam_ms = 0
    _recover_yaw_ok_cnt = 0
    _recover_yaw_ok_t0 = 0
    _recover_yaw_rate_cmd = 0.0
    _recover_wait_yaw = 0.0
    _search_yaw_pre_aligned = False
    _self_ready = 0
    _clear_recover_fix_flags()
    _publish_recover_diag_obstacle()
    base.wireless_send_now()

# 功能：设置恢复阶段要依据哪条边进行边界修正。
# 输入参数：edge 为刚完成推送的目标边。
def set_recover_target(edge):
    global _recover_edge, _last_completed_push_yaw
    global _recover_back_yaw, _recover_sub_t0, _recover_back_x0, _recover_back_y0
    _recover_edge = edge
    # set_recover_target 只在一次推出动作结束、准备恢复时调用；此时记录比在
    # SEARCH 入口读取 _target_edge 更可靠，因为后者会被 _clear_target 清零。
    _last_completed_push_yaw = _push_yaw(edge)
    _recover_back_yaw = 0.0
    _recover_sub_t0 = 0
    _recover_back_x0 = 0.0
    _recover_back_y0 = 0.0

# 功能：更新 RECOVER 模式。
# 主车先沿车体后方退出目标/边界区域，再根据推出边修正 X/Y 坐标，最后用黄线视觉航向精调并校准 IMU yaw。
# 输入参数：now_ms 为当前时间戳。
def recover_update(now_ms):
    global _next_task_mode, _self_ready, _search_yaw_pre_aligned
    global _mode_sub, _mode_hold_ms
    global _recover_back_yaw, _recover_sub_t0, _recover_back_x0, _recover_back_y0
    global _recover_fix_x_snap, _recover_fix_y_snap
    global _recover_fix_ok_cnt, _recover_fix_finished
    global _recover_fix_drive_t0
    global _recover_yaw_last_cam_ms, _recover_yaw_ok_cnt, _recover_yaw_ok_t0, _recover_yaw_rate_cmd
    global _recover_wait_yaw
    _publish_recover_diag_obstacle()
    if _mode_sub == _SUB_SEARCH_YAW:
        target_yaw = _recover_wait_yaw
        yaw_err = abs(_angle_diff(base._car.current_angle, target_yaw))
        if yaw_err <= _RECOVER_SEARCH_YAW_EPS_DEG:
            base.correct_yaw(target_yaw)
            base.request_hold(target_yaw)
            _search_yaw_pre_aligned = True
            _begin_recover_wait_follower(target_yaw)
        elif ticks_diff(now_ms, _recover_sub_t0) >= _RECOVER_SEARCH_YAW_TIMEOUT_MS:
            base.request_yaw_rate(0.0)
            _search_yaw_pre_aligned = False
            _begin_recover_wait_follower(base._car.current_angle)
        else:
            base.request_world(0.0, 0.0, target_yaw)
        return
    if _mode_sub == _SUB_WAIT_FOLLOWER_FOLLOW:
        _clear_recover_fix_flags()
        _self_ready = 0
        base.request_hold(_recover_wait_yaw)
        if _recover_follower_follow_ready(now_ms):
            _finish_recover(_recover_wait_yaw)
        return
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
        # _set_body(0, -speed) 对应的世界车尾单位向量为 (-sin(yaw), -cos(yaw))。
        # 只累计沿车尾方向的有向位移；PUSH 残速造成的向外滑行以及横向漂移均不计入。
        back_progress = back_dx * (-sin(back_rad)) + back_dy * (-cos(back_rad))
        back_done = back_progress >= _RECOVER_BACK_DIST
        back_timeout = ticks_diff(now_ms, _recover_sub_t0) > _RECOVER_BACK_TIMEOUT_MS
        if back_done or back_timeout:
            # BACK结束先急停，清除后退速度及速度环积分；下一周期再开始黄线转向。
            base.request_hard_hold(_recover_back_yaw)
            enabled, unused, unused, unused, unused, unused = _recover_fix_cfg(_recover_edge)
            if enabled:
                _mode_sub = _SUB_VIS_FIX
                _recover_sub_t0 = 0
                _recover_fix_ok_cnt = 0
                _recover_fix_finished = False
                _recover_fix_drive_t0 = 0
                _mode_hold_ms = 0
            else:
                _begin_recover_search_yaw(now_ms, _recover_back_yaw)
        return
    if _mode_sub == _SUB_VIS_FIX:
        enabled, axis, fix_yaw, vx, vy, high_side = _recover_fix_cfg(_recover_edge)
        if not enabled:
            _clear_recover_fix_flags()
            _begin_recover_search_yaw(now_ms, base._car.current_angle)
            return
        if _recover_sub_t0 == 0:
            _recover_sub_t0 = now_ms
            _recover_fix_x_snap = base._vis_x_fix_cnt
            _recover_fix_y_snap = base._vis_y_fix_cnt
            _recover_fix_ok_cnt = 0
            _recover_fix_finished = False
            _recover_fix_drive_t0 = 0
        err = _angle_diff(base._car.current_angle, fix_yaw)
        if abs(err) >= _TASK_RECOVER_DEPLOY_TURN_EPS_DEG:
            _clear_recover_fix_flags()
            base.request_world(0.0, 0.0, fix_yaw)
            _mode_hold_ms = 0
            return
        if _recover_fix_drive_t0 == 0:
            # 800 ms只统计车头已经进入±4°、真正开始后退修坐标的时间；
            # 转正期间不消耗修坐标窗口，也不累计转正前的视觉帧。
            _recover_fix_drive_t0 = now_ms
            _recover_fix_x_snap = base._vis_x_fix_cnt
            _recover_fix_y_snap = base._vis_y_fix_cnt
            _recover_fix_ok_cnt = 0
            _recover_fix_finished = False
        if _recover_fix_finished:
            _clear_recover_fix_flags()
            changed = False
        elif axis == 'x':
            base._vis_x_fix_en = True
            base._vis_y_fix_en = False
            changed = base._vis_x_fix_cnt != _recover_fix_x_snap
            if changed:
                _recover_fix_x_snap = base._vis_x_fix_cnt
        else:
            base._vis_x_fix_en = False
            base._vis_y_fix_en = True
            changed = base._vis_y_fix_cnt != _recover_fix_y_snap
            if changed:
                _recover_fix_y_snap = base._vis_y_fix_cnt
        _set_body(vx, -vy, fix_yaw)
        if not _recover_fix_finished:
            if changed and _recover_fix_value_ok(axis, high_side):
                _recover_fix_ok_cnt += 1
            elif changed:
                _recover_fix_ok_cnt = 0
        if not _recover_fix_finished and (
                _recover_fix_ok_cnt >= _RECOVER_FIX_REQUIRE
                or ticks_diff(now_ms, _recover_fix_drive_t0) > _RECOVER_FIX_TIMEOUT_MS):
            _recover_fix_finished = True
            _clear_recover_fix_flags()
        if _recover_fix_finished:
            _clear_recover_fix_flags()
            # 坐标修正以速度140运行，不能沿用全局缓停斜坡，否则进入YAW_FIX后
            # 仍会多滑十几到几十厘米。这里直接清平移目标和速度环积分后再修角度。
            base.request_hard_hold(fix_yaw)
            _mode_sub = _SUB_YAW_FIX
            _recover_sub_t0 = now_ms
            _recover_yaw_last_cam_ms = base._cam_rx_last_ms
            _recover_yaw_ok_cnt = 0
            _recover_yaw_ok_t0 = 0
            _recover_yaw_rate_cmd = 0.0
            return
    if _mode_sub == _SUB_YAW_FIX:
        enabled, unused, fix_yaw, unused, unused, unused = _recover_fix_cfg(_recover_edge)
        _clear_recover_fix_flags()
        if not enabled:
            _begin_recover_search_yaw(now_ms, base._car.current_angle)
            return
        elapsed = ticks_diff(now_ms, _recover_sub_t0)
        if elapsed >= _RECOVER_YAW_TIMEOUT_MS:
            base.request_yaw_rate(0.0)
            _begin_recover_search_yaw(now_ms, base._car.current_angle)
            return
        if elapsed < _RECOVER_YAW_SETTLE_MS:
            _recover_yaw_rate_cmd = 0.0
            base.request_yaw_rate(0.0)
            return
        cam_ms = base._cam_rx_last_ms
        if cam_ms != 0 and cam_ms != _recover_yaw_last_cam_ms:
            _recover_yaw_last_cam_ms = cam_ms
            vis_yaw = base._YawAngle_fix
            if 0.0 <= vis_yaw < 361.0:
                vis_err = _angle_diff(vis_yaw, fix_yaw)
                abs_err = abs(vis_err)
                if abs_err <= _RECOVER_YAW_EPS:
                    _recover_yaw_rate_cmd = 0.0
                    if _recover_yaw_ok_cnt == 0:
                        _recover_yaw_ok_t0 = now_ms
                    _recover_yaw_ok_cnt += 1
                    if (_recover_yaw_ok_cnt >= _RECOVER_YAW_OK_REQUIRE
                            and ticks_diff(now_ms, _recover_yaw_ok_t0) >= _RECOVER_YAW_OK_HOLD_MS):
                        base.correct_yaw(fix_yaw)
                        base.request_hold(fix_yaw)
                        _begin_recover_search_yaw(now_ms, fix_yaw)
                        return
                elif abs_err <= _RECOVER_YAW_VALID_GATE:
                    _recover_yaw_ok_cnt = 0
                    _recover_yaw_ok_t0 = 0
                    # vis_err > 0 表示视觉 yaw 偏大，应让世界 yaw 减小；反之增大。
                    _recover_yaw_rate_cmd = -_RECOVER_YAW_RATE_DPS if vis_err > 0.0 else _RECOVER_YAW_RATE_DPS
                else:
                    _recover_yaw_ok_cnt = 0
                    _recover_yaw_ok_t0 = 0
                    _recover_yaw_rate_cmd = 0.0
            else:
                _recover_yaw_ok_cnt = 0
                _recover_yaw_ok_t0 = 0
                _recover_yaw_rate_cmd = 0.0
        base.request_yaw_rate(_recover_yaw_rate_cmd)
        return
_RELOC_PREMOVE = const(0)

_RELOC_X_TURN = const(1)

_RELOC_X_DRIVE = const(2)

_RELOC_Y_TURN = const(3)

_RELOC_Y_DRIVE = const(4)

_RELOC_FORMATION_WAIT = const(5)

_mode_sub = _RELOC_FORMATION_WAIT

_mode_hold_ms = 0

_premove_start_x = 0.0

_premove_start_y = 0.0

_premove_start_valid = False

_relocalize_sub_t0 = 0

_relocalize_wait_yaw = 0.0

_reloc_x_snap = 0

_reloc_y_snap = 0

_RELOC_DRIVE_SETTLE_MS = const(90)

_RELOC_DRIVE_TIMEOUT_MS = const(7000)

# 黄线 Y 坐标修正允许更长的搜索距离；X 修正仍使用上面的通用超时。
_RELOC_Y_DRIVE_TIMEOUT_MS = const(20000)

_RELOC_FIX_THRESH = const(60)

_RELOC_FIX_REQUIRE = const(2)

_x_ok_cnt = 0

_y_ok_cnt = 0

_RELOC_SPEED = const(80)

# X 方向边走边修：读数进入 _RELOC_FIX_THRESH（60）视为触发黄线识别，
# 减速到该速度继续行驶，不停车，等下一次读数确认进入更严的 _RELOC_X_FIX_THRESH_TIGHT
# （40）后直接转向进入 Y 修正。
_RELOC_X_SLOW_SPEED = const(60)

_RELOC_X_FIX_THRESH_TIGHT = const(40)

# Y 视觉值必须先接近当前里程坐标，才允许底层直接写入，防止错误首帧造成坐标瞬移。
_RELOC_Y_SLOW_SPEED = const(60)

_RELOC_Y_FIX_THRESH_TIGHT = const(40)

_RELOC_Y_FIX_REQUIRE = const(3)

_x_slow_armed = False

_y_slow_armed = False

# PREMOVE 段速度 _RELOC_PREMOVE_SPEED 已上移到文件顶端的"现场常调参数"区。

_RELOC_PREMOVE_EPS_X = const(15)

_RELOC_PREMOVE_EPS_Y = const(15)

_RELOC_PREMOVE_TIMEOUT_MS = const(8000)

_TASK_RELOCALIZE_DEPLOY_TURN_EPS_DEG = const(6)

# 功能：重置全局重定位状态机。
# 主车进入 RELOCALIZE 后先原地等待从车完成“主车居中+距离合格”的轻量跟随准备，再转到 270 度，
# 锁角移动到预设点并沿 X/Y 两个方向靠视觉线修正坐标。
def relocalize_reset():
    global _mode_sub, _mode_hold_ms, _reloc_x_snap, _reloc_y_snap, _relocalize_sub_t0
    global _premove_start_x, _premove_start_y, _premove_start_valid
    global _relocalize_wait_yaw, _self_ready
    global _x_ok_cnt, _y_ok_cnt, _x_slow_armed, _y_slow_armed
    _mode_sub = _RELOC_FORMATION_WAIT
    _mode_hold_ms = 0
    _premove_start_x = 0.0
    _premove_start_y = 0.0
    _premove_start_valid = False
    _relocalize_sub_t0 = 0
    _relocalize_wait_yaw = base._car.current_angle
    _reloc_x_snap = 0
    _reloc_y_snap = 0
    _x_ok_cnt = 0
    _y_ok_cnt = 0
    _x_slow_armed = False
    _y_slow_armed = False
    _self_ready = 0
    base._vis_x_fix_cnt = 0
    base._vis_y_fix_cnt = 0
    base.avoid_beep(0)
    base.request_hold(_relocalize_wait_yaw)

# 功能：结束全局重定位。
# 会关闭视觉修正开关、清空当前目标和目标地图，并请求进入 DONE 模式。
def _finish_relocalize():
    global _next_task_mode, _obj_map_count, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y
    base.avoid_beep(0)
    base._vis_yaw_fix_en = False
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False
    _target_obj_world_x = 999.0
    _target_obj_world_y = 999.0
    _target_edge = 0
    _target_obj_id = 0
    _obj_map_count = 0
    gc.collect()
    _next_task_mode = _MODE_DONE

# 功能：不停车地开始 RELOCALIZE 的 X 方向视觉修正。
# PREMOVE 已锁定 270 度；到位后当周期直接向低 X 前进。
def _relocalize_begin_x_drive(now):
    global _mode_sub, _relocalize_sub_t0, _reloc_x_snap, _x_ok_cnt, _self_sub
    global _premove_start_valid, _x_slow_armed
    _relocalize_sub_t0 = now
    _premove_start_valid = False
    _reloc_x_snap = base._vis_x_fix_cnt
    _x_ok_cnt = 0
    _x_slow_armed = False
    base._vis_x_fix_en = True
    base._vis_y_fix_en = False
    _mode_sub = _RELOC_X_DRIVE
    _self_sub = _mode_sub
    _relocalize_request_with_repulse(-_RELOC_SPEED, 0.0, 270.0, now)

# 功能：更新 RELOCALIZE 模式。
# 主车先停车等待从车在 RELOCALIZE 中完成轻量跟随准备并上报 ready=2；随后转到 270 度并
# 锁角移动到预设重定位起点，到位后沿低 X 方向开视觉修正，再转向 180 度沿低 Y 修正。
def relocalize_update():
    global _self_sub
    global _mode_sub, _mode_hold_ms, _reloc_x_snap, _reloc_y_snap, _relocalize_sub_t0
    global _premove_start_x, _premove_start_y, _premove_start_valid
    global _relocalize_wait_yaw, _self_ready
    global _x_ok_cnt, _y_ok_cnt, _x_slow_armed, _y_slow_armed
    now = ticks_ms()
    base.avoid_beep(0)
    _self_sub = _mode_sub
    # 只调从车 SEARCH 时，主车进入 RELOCALIZE 就固定停在入口姿态。
    # 不推进 formation ready、转向和坐标修正状态，避免从车结束日志后上报
    # ready=2 又让主车起步。关闭调试开关后完整 RELOCALIZE 流程保持不变。
    if debug_switch.SEARCH_TUNE_LEADER_RELOCALIZE_HOLD_ENABLE:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        _self_ready = 0
        base.request_hold(_relocalize_wait_yaw)
        return
    if _mode_sub == _RELOC_FORMATION_WAIT:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        _self_ready = 0
        base.request_hold(_relocalize_wait_yaw)
        follower_ready = (
            base._Other_Car_Mode == _MODE_RELOCALIZE
            and base._Other_Car_Ready >= 2
            and base._Other_Car_Ready_Ts != 0
            and ticks_diff(now, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
        )
        if follower_ready:
            _mode_sub = _RELOC_X_TURN
            _self_sub = _mode_sub
            base.request_world(0.0, 0.0, 270.0)
        return
    elif _mode_sub == _RELOC_PREMOVE:
        base._vis_yaw_fix_en = False
        base._vis_x_fix_en = False
        base._vis_y_fix_en = False
        tx, ty = _RELOCALIZE_PRESET_POS[0]
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

        # Once the initial target direction has been traversed, the target
        # perpendicular has been crossed even if encoder quantization skipped
        # over the rectangular arrival band.
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
            # 已经先完成 270 度转向；PREMOVE 到位后不停车，当周期直接沿
            # 270 度车头方向向低 X 前进并进入 X 视觉坐标修正。
            _relocalize_begin_x_drive(now)
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
            tx, ty = _RELOCALIZE_PRESET_POS[0]
            dx = tx - base._car.Position_X
            dy = ty - base._car.Position_Y
            distance = sqrt(dx * dx + dy * dy)
            arrived = _near(base._car.Position_X, tx, _RELOC_PREMOVE_EPS_X) and _near(
                base._car.Position_Y, ty, _RELOC_PREMOVE_EPS_Y
            )
            if arrived or distance <= 1.0:
                _relocalize_begin_x_drive(now)
            else:
                _relocalize_sub_t0 = now
                _premove_start_x = base._car.Position_X
                _premove_start_y = base._car.Position_Y
                _premove_start_valid = True
                _mode_sub = _RELOC_PREMOVE
                _self_sub = _mode_sub
                # 转向进入 6 度范围后不停车，当周期直接开始锁定 270 度 PREMOVE。
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
            # 去掉刹停：无缝切 DONE 起步回库。
            base._vis_x_fix_en = False
            _relocalize_sub_t0 = 0
            _x_ok_cnt = 0
            _x_slow_armed = False
            _finish_relocalize()
            return
        elif elapsed >= _RELOC_DRIVE_SETTLE_MS:
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
            if _x_slow_armed and _x_ok_cnt >= _RELOC_FIX_REQUIRE:
                # 减速后确认黄线识别在 40cm 以内，边走边修已经完成一次坐标
                # 修正，不停车、不做静止态复核，直接转向进入 Y 修正。
                _relocalize_sub_t0 = 0
                _x_ok_cnt = 0
                _x_slow_armed = False
                base._vis_x_fix_en = False
                _mode_sub = _RELOC_Y_TURN
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
            base._vis_x_fix_en = False
            base._vis_y_fix_en = False
            _mode_sub = _RELOC_Y_DRIVE
            _self_sub = _mode_sub
            # 转向进入 6 度范围后不停车，当周期直接向低 Y 前进。
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
            # 去掉刹停：无缝切 DONE 起步回库。
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
        and base._cam_car_x < 900.0
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
# DONE 阶段主车回到起始点并保持当前锁定角，作为任务结束后的停靠状态。
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
# 主车移动回起始点附近，到达后保持锁定角不再执行新的任务目标。
# 输入参数：now_ms 为当前时间戳。
def done_update(now_ms):
    global _mode_sub, _mode_hold_ms, _self_ready
    global _done_form_stable_t0
    global _done_orbit_accum, _done_orbit_prev_yaw, _done_hold_yaw
    tx, ty = config._DONE_TARGET_POS[0]
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

_SEARCH_MOVE_TO_POINT = const(0)
# 每一次进入 SWEEP 前的主车本地急停保持；不等待从车 ready，不是编队握手。
_SEARCH_PRE_SWEEP_HOLD = const(1)
_SEARCH_TURN_YAW = const(2)
_SEARCH_SWEEP = const(4)
_SEARCH_FIRST_DIAG = const(5)
_SEARCH_STEP_FORWARD = const(6)
_SEARCH_FIRST_TURN_YAW = const(7)
_SEARCH_LINE_STOP = const(8)
_SEARCH_LINE_TURN = const(9)
_SEARCH_LINE_POS_FIX = const(10)
_SEARCH_LINE_TURN_BACK = const(11)

# 主车移动到蛇形搜索起点时的平移速度，以及横扫和换行前移速度。
# 换行前进直接采用横扫基准速度；从车在连续运动中动态修正阵型。
# _SEARCH_SWEEP_SPEED 已上移到文件顶端的"现场常调参数"区。
_SEARCH_MOVE_SPEED = const(120)
_SEARCH_STEP_SPEED = _SEARCH_SWEEP_SPEED

# 各运动子状态的超时保护。超过对应时长后状态机按原流程切换，单位为 ms。
_SEARCH_MOVE_TIMEOUT_MS = const(3000)
_SEARCH_TURN_TIMEOUT_MS = const(1500)
_SEARCH_STEP_TIMEOUT_MS = const(3000)

# 蛇形搜索边界参数，距离单位均为 cm。
# 车辆在搜索矩形内做无前向偏置的蛇形扫描。到横向边界后立即沿车头方向
# 换行前进最多 30cm，再停车等从车到位后反向横扫；余量不足时只走到搜索区边界。
# 前进步长 _SEARCH_FORWARD_STEP_CM 和四个 _SEARCH_LINE_*_AFTER_PUSH_* 起点坐标
# 已上移到文件顶端的"现场常调参数"区。
_SEARCH_BOUNDARY_STOP_TOL_CM = 5.0
_SEARCH_AREA_X_MIN = 100.0
_SEARCH_AREA_X_MAX = 220.0
_SEARCH_AREA_Y_MIN = 60.0
_SEARCH_AREA_Y_MAX = 180.0

# SEARCH 把场地四周的 cone/brick 当作横扫物理边界：SWEEP 中只用本机视觉
# 相对坐标触发提前换行，不叠加速度；STEP 中锁存触发障碍的世界坐标，即使
# 离开视野仍继续退让。退让方向由 _search_avoid_dir 按本车与障碍的实际世界
# 坐标规划，并受搜索矩形约束。RELOCALIZE 继续使用独立的固定世界 -Y 斥力。
_SEARCH_AVOID_REL_X_CM = 50.0
_SEARCH_AVOID_REL_Y_CM = 50.0
_SEARCH_AVOID_TRIGGER_FRESH_MS = const(250)
_SEARCH_REPULSE_TRIGGER_CM = 60.0
_SEARCH_REPULSE_KP = 3.0
_SEARCH_REPULSE_MAX_SPEED = 1.4*_SEARCH_SWEEP_SPEED

# 退让方向的搜索区内边距，单位 cm。退让最多把车推到距搜索矩形边界该距离处；
# 越过后该方向即判为不可用，避免避障把车挤出物体放置区、贴到场地边界。
# 调大 = 离边界更远就停止退让（更保守）；调小 = 允许退让更贴近搜索区边界。
_SEARCH_AVOID_AREA_MARGIN_CM = 10.0
# RELOCALIZE 保持独立的50cm线性、固定世界 -Y 斥力，避免SEARCH调参连带改变重定位轨迹。
# 总开关 _RELOCALIZE_AVOID_ENABLE 和 _RELOCALIZE_REPULSE_TRIGGER_CM /
# _RELOCALIZE_REPULSE_SPEED 都在文件顶端。
# 下面这组障碍记忆表参数由 SEARCH 和 RELOCALIZE 共用。
_SEARCH_OBS_MEMORY_MS = const(3000)
_SEARCH_OBS_SAME_DIST_CM = 40.0
_SEARCH_OBS_SLOT_MAX = const(4)

# 到达搜索起点、每次 SWEEP 前停车和转向完成的判定参数。
# 位置误差不超过 8 cm 即视为到点；所有 SWEEP 入口先急停并保持 500 ms；
# 航向误差不超过 6 度即视为转向完成。
_SEARCH_POINT_EPS_CM = 8.0
_SEARCH_PRE_SWEEP_HOLD_MS = const(500)
_SEARCH_YAW_EPS_DEG = 6.0

# SEARCH 横扫过程中使用黄色线做一次单轴坐标补偿。
# line_cx 是 0~320 的原始像素中心，减去 160 后得到左右有符号位置。
_SEARCH_LINE_CX_CENTER_PX = 160.0
_SEARCH_LINE_CX_TRIGGER_PX = 130.0
_SEARCH_LINE_CX_RELEASE_PX = 150.0
_SEARCH_LINE_SEEN_REQUIRE = const(2)
_SEARCH_LINE_STOP_MS = const(80)
_SEARCH_LINE_TURN_EPS_DEG = 6.0
_SEARCH_LINE_TURN_TIMEOUT_MS = const(1500)
_SEARCH_LINE_FIX_SETTLE_MS = const(100)
_SEARCH_LINE_FIX_TIMEOUT_MS = const(1000)
_SEARCH_LINE_FIX_REQUIRE = const(1)
_SEARCH_LINE_FIX_EDGE_GATE_CM = 70.0

# slot0 同一类别目标的累计确认次数。
_SEARCH_TARGET_STABLE_COUNT = 1

# 首次固定斜移结束后，主车必须先原地转到本轮搜索航向，再直接开始横扫。
# 航向需连续落在误差带内一小段时间，避免刚穿过目标角就立即开始横扫。
_SEARCH_FIRST_TURN_STABLE_MS = const(100)


_mode_sub = _SEARCH_MOVE_TO_POINT

_search_sub_t0_ms = 0

_search_leader_t0 = 0

_search_point_x = 0.0
_search_point_y = 0.0
_search_yaw = 0.0

_search_yaw_pre_aligned = False
# SEARCH 发现目标切入 APPROACH 时锁存本车搜索航向。后续 APPROACH
# 异常退回 SEARCH 时恢复，避免按当前位置重新朝场内选向。
_search_resume_yaw = 0.0
_search_resume_yaw_valid = False
_search_sweep_sign = 1.0

# SWEEP 横向位置闭环：每次进入 SWEEP 时把车体当前的“不该动”那根轴坐标
# 存成基准，横扫过程中持续用小幅横向速度把该轴拉回基准，防止里程计/
# 麦轮机械误差在纯开环横移里逐渐累积成斜线。
_search_sweep_cross_ref = 0.0

_SEARCH_SWEEP_LAT_KP = 5.0

_SEARCH_SWEEP_LAT_MAX = const(40)

_SEARCH_FIRST_SWEEP_MODE_INVALID = const(0)
# Frame3 flags.bit4~5 保留“本轮 SEARCH 持续保持 RECOVER 侧阵列”信号。
# 变量名保留 first_sweep 仅为兼容现有字段；该信号不再在第一次 Forward 后撤销。
_SEARCH_FIRST_SWEEP_KEEP_RECOVER_SIDE = const(3)
_search_first_sweep_pending = False
_search_step_x0 = 0.0
_search_step_y0 = 0.0
_search_step_target_cm = 0.0
_search_first_diag_vx = 0.0
_search_first_diag_vy = 0.0
_search_seen_id = 0
_search_seen_cnt = 0
_search_boot_peer_pending = False
_search_target_wait = False
_search_hold_yaw = 0.0
_search_line_fix_used = False
_search_line_near_latched = False
_search_line_last_seq = 0
_search_line_seen_cnt = 0
_search_line_seen_side = 0
_search_line_saved_yaw = 0.0
_search_line_look_yaw = 0.0
_search_line_fix_axis = 0
_search_line_fix_snap = 0
_search_line_fix_ok_cnt = 0
_search_line_pause_t0 = 0
_search_obs_x = [999.0] * _SEARCH_OBS_SLOT_MAX
_search_obs_y = [999.0] * _SEARCH_OBS_SLOT_MAX
_search_obs_ts = [0] * _SEARCH_OBS_SLOT_MAX
_search_step_avoid_valid = False
_search_step_avoid_x = 999.0
_search_step_avoid_y = 999.0

def _search_step_avoid_clear():
    global _search_step_avoid_valid, _search_step_avoid_x, _search_step_avoid_y
    _search_step_avoid_valid = False
    _search_step_avoid_x = 999.0
    _search_step_avoid_y = 999.0

# 功能：判断指定的两个物体类别是否都已搬完。
# 仅数量模式 2/3 具有可靠的类别/类别组剩余标志；模式 1 始终返回 False。
def _search_obj_pair_done(first_obj_id, second_obj_id):
    if _obj_count_mode != 2 and _obj_count_mode != 3:
        return False
    if first_obj_id >= len(_obj_remain) or second_obj_id >= len(_obj_remain):
        return False
    return _obj_remain[first_obj_id] <= 0 and _obj_remain[second_obj_id] <= 0


# 功能：重置搜索状态机。
# 首次搜索会先按进入时锁存的方向匀速移向首次斜移目标，之后进入蛇形搜索流程。
def _reset_search():
    global _search_first_boot
    global _search_yaw_pre_aligned
    global _mode_sub, _search_sub_t0_ms, _search_leader_t0
    global _search_point_x, _search_point_y, _search_yaw, _search_sweep_sign
    global _search_resume_yaw_valid
    global _search_first_sweep_pending, _search_first_sweep_mode_to_other
    global _search_step_x0, _search_step_y0, _search_step_target_cm
    global _search_first_diag_vx, _search_first_diag_vy
    global _search_seen_id, _search_seen_cnt
    global _search_boot_peer_pending
    global _search_target_wait, _search_hold_yaw
    global _search_line_fix_used, _search_line_near_latched, _search_line_last_seq
    global _search_line_seen_cnt, _search_line_seen_side
    global _search_line_saved_yaw, _search_line_look_yaw, _search_line_fix_axis
    global _search_line_fix_snap, _search_line_fix_ok_cnt, _search_line_pause_t0
    global _search_obs_x, _search_obs_y, _search_obs_ts
    global _self_ready

    # Search entry target:
    # - first boot: fixed diagonal motion, then sweep toward the positive world axis;
    # - normal RECOVER: keep the entry coordinate on one axis and align the other axis;
    # - abnormal re-entry: stay at the entry position.

    # 首次搜索由配置模式选择 90/0 度；以后采用最近一次推出方向的反方向。
    # 四舍五入到 90 度整数倍，保证 SEARCH 中车头只出现四个正交角。
    # Yaw source: first boot -> configured yaw; APPROACH lost -> resume the yaw saved
    # when this SEARCH entered APPROACH; normal RECOVER -> opposite push yaw;
    # otherwise -> face center.
    first_search = _search_first_boot
    prealigned_search_yaw = (
        _search_yaw_pre_aligned
        and _prev_task_mode == _MODE_RECOVER
        and not first_search
    )
    if first_search:
        raw_yaw = 0.0 if config._SEARCH_INITIAL_YAW_MODE == 1 else 90.0
        _search_point_x = base._car.Position_X
        _search_point_y = base._car.Position_Y
        diag_dx = _SEARCH_FIRST_DIAG_END_X - base._car.Position_X
        diag_dy = _SEARCH_FIRST_DIAG_END_Y - base._car.Position_Y
        diag_dist = sqrt(diag_dx * diag_dx + diag_dy * diag_dy)
        if diag_dist > 0.01:
            _search_first_diag_vx = _SEARCH_FIRST_DIAG_SPEED * diag_dx / diag_dist
            _search_first_diag_vy = _SEARCH_FIRST_DIAG_SPEED * diag_dy / diag_dist
        else:
            _search_first_diag_vx = 0.0
            _search_first_diag_vy = 0.0
        _search_first_boot = False
    elif _prev_task_mode == _MODE_APPROACH and _search_resume_yaw_valid:
        raw_yaw = _search_resume_yaw
        _search_point_x = base._car.Position_X
        _search_point_y = base._car.Position_Y
        _search_resume_yaw_valid = False
    elif _prev_task_mode == _MODE_RECOVER and _last_completed_push_yaw < 900.0:
        raw_yaw = _last_completed_push_yaw + 180.0
        _search_point_x = base._car.Position_X
        _search_point_y = base._car.Position_Y
        if abs(_angle_diff(_last_completed_push_yaw, 270.0)) <= 1.0:
            # 2=蓝沙包、3=红沙包；模式 2 的沙包组归零时两项会同步清零。
            if _search_obj_pair_done(2, 3):
                _search_point_x = _SEARCH_LINE_X_AFTER_PUSH_270_SANDBAGS_DONE
            else:
                _search_point_x = _SEARCH_LINE_X_AFTER_PUSH_270
        elif abs(_angle_diff(_last_completed_push_yaw, 0.0)) <= 1.0:
            _search_point_y = _SEARCH_LINE_Y_AFTER_PUSH_0
        elif abs(_angle_diff(_last_completed_push_yaw, 90.0)) <= 1.0:
            # 4=白熊、5=棕熊；模式 2 的熊组归零时两项会同步清零。
            if _search_obj_pair_done(4, 5):
                _search_point_x = _SEARCH_LINE_X_AFTER_PUSH_90_TEDDIES_DONE
            else:
                _search_point_x = _SEARCH_LINE_X_AFTER_PUSH_90
        elif abs(_angle_diff(_last_completed_push_yaw, 180.0)) <= 1.0:
            _search_point_y = _SEARCH_LINE_Y_AFTER_PUSH_180
    else:
        _search_point_x = base._car.Position_X
        _search_point_y = base._car.Position_Y
        dx_center = base._car.Position_X - _CENTER_X
        dy_center = base._car.Position_Y - _CENTER_Y
        if abs(dx_center) >= abs(dy_center):
            raw_yaw = 270.0 if dx_center >= 0.0 else 90.0
        else:
            raw_yaw = 180.0 if dy_center >= 0.0 else 0.0
    _search_yaw = (int((raw_yaw + 45.0) // 90.0) * 90.0) % 360.0

    # 首次横扫固定朝世界坐标增大方向：0/180航向时X增大，90/270时Y增大。
    # 后续搜索仍根据搜索线所在场地半区选择朝场内的方向。
    if first_search:
        _search_sweep_sign = 1.0
    elif _search_yaw == 0.0 or _search_yaw == 180.0:
        _search_sweep_sign = 1.0 if _search_point_x < _CENTER_X else -1.0
    else:
        _search_sweep_sign = 1.0 if _search_point_y < _CENTER_Y else -1.0

    # 侧向编队属于正常 RECOVER 后的整轮 SEARCH，跨越所有 SWEEP 和 Forward；
    # 开局首次 SEARCH 和 APPROACH 丢失等异常重入仍保持原流程。
    _search_first_sweep_pending = (
        (not first_search)
        and _prev_task_mode == _MODE_RECOVER
        and _last_completed_push_yaw < 900.0
    )
    _search_first_sweep_mode_to_other = _SEARCH_FIRST_SWEEP_MODE_INVALID

    # 开局固定斜移属于第一次 SEARCH，而不是 BOOT。一次性标志在进入这里时
    # 立即消耗，因此即使斜移途中锁定目标并进入 APPROACH，后续 SEARCH 也不会重跑。
    _mode_sub = (
        _SEARCH_FIRST_DIAG if first_search
        else (_SEARCH_MOVE_TO_POINT if prealigned_search_yaw else _SEARCH_TURN_YAW)
    )
    _search_yaw_pre_aligned = False
    _search_sub_t0_ms = 0
    _search_leader_t0 = 0
    _search_step_x0 = 0.0
    _search_step_y0 = 0.0
    _search_step_target_cm = 0.0
    if not first_search:
        _search_first_diag_vx = 0.0
        _search_first_diag_vy = 0.0
    _search_seen_id = 0
    _search_seen_cnt = 0
    _search_boot_peer_pending = first_search
    _search_target_wait = False
    _search_hold_yaw = 0.0
    _search_line_fix_used = False
    _search_line_near_latched = False
    _search_line_last_seq = base._cam_line_seq
    _search_line_seen_cnt = 0
    _search_line_seen_side = 0
    _search_line_saved_yaw = 0.0
    _search_line_look_yaw = 0.0
    _search_line_fix_axis = 0
    _search_line_fix_snap = 0
    _search_line_fix_ok_cnt = 0
    _search_line_pause_t0 = 0
    for i in range(_SEARCH_OBS_SLOT_MAX):
        _search_obs_x[i] = 999.0
        _search_obs_y[i] = 999.0
        _search_obs_ts[i] = 0
    _search_step_avoid_clear()
    _self_ready = 0


# 功能：把主车本机视觉看到的 cone/brick 写入 SEARCH/RELOCALIZE 共用障碍记忆。
# 同一位置 40cm 内视为同一障碍；槽满时覆盖最旧项。时间戳使用视觉原始时间，
# 因此摄像头停止更新时不会被任务循环错误续期。
def _search_obstacle_add(x, y, ts, now_ms):
    if x >= 900.0 or y >= 900.0 or ts == 0:
        return
    if ticks_diff(now_ms, ts) > _SEARCH_OBS_MEMORY_MS:
        return
    same_i = -1
    empty_i = -1
    old_i = 0
    old_ts = 0
    same_d2 = _SEARCH_OBS_SAME_DIST_CM * _SEARCH_OBS_SAME_DIST_CM
    for i in range(_SEARCH_OBS_SLOT_MAX):
        old_slot_ts = _search_obs_ts[i]
        if old_slot_ts != 0 and ticks_diff(now_ms, old_slot_ts) > _SEARCH_OBS_MEMORY_MS:
            _search_obs_x[i] = 999.0
            _search_obs_y[i] = 999.0
            _search_obs_ts[i] = 0
            old_slot_ts = 0
        if old_slot_ts == 0:
            if empty_i < 0:
                empty_i = i
            continue
        dx = x - _search_obs_x[i]
        dy = y - _search_obs_y[i]
        if dx * dx + dy * dy <= same_d2:
            same_i = i
            break
        if old_ts == 0 or ticks_diff(old_slot_ts, old_ts) < 0:
            old_i = i
            old_ts = old_slot_ts
    if same_i >= 0:
        i = same_i
    elif empty_i >= 0:
        i = empty_i
    else:
        i = old_i
    _search_obs_x[i] = x
    _search_obs_y[i] = y
    _search_obs_ts[i] = ts


# 功能：检查当前帧 cone/brick 是否位于本次 SWEEP 前方触发窗，并锁存最近一项。
# rel_x 先按当前世界横扫方向换成有向距离；只有 0<x<=50、0<y<50 且视觉
# 时间戳新鲜才触发。锁存使用世界坐标，STEP 中不再依赖障碍继续可见。
def _search_latch_sweep_obstacle(now_ms):
    global _search_step_avoid_valid, _search_step_avoid_x, _search_step_avoid_y
    edge, along_pos = _search_sweep_edge_and_along()
    # 只有对面边（_recover_edge 的另一侧）上、布局表允许的类型才会触发。
    # 刚推完物体那条边（_recover_edge）已知的旧障碍即使还在视野里也直接忽略。
    # 首次 SEARCH 还没有 _recover_edge（值为0）时不做边归属过滤，保持旧行为。
    recover_edge_known = 1 <= _recover_edge <= 4
    expect_edge = _opposite_edge(_recover_edge)
    cone_allowed = _obstacle_layout_allowed(edge, _OBSTACLE_CONE, along_pos)
    brick_allowed = _obstacle_layout_allowed(edge, _OBSTACLE_BRICK, along_pos)
    if _search_yaw == 0.0:
        side_sign = _search_sweep_sign
    elif _search_yaw == 90.0:
        side_sign = -_search_sweep_sign
    elif _search_yaw == 180.0:
        side_sign = -_search_sweep_sign
    else:
        side_sign = _search_sweep_sign
    found = False
    best_d2 = 999999.0
    if (cone_allowed and base._cam_cone_seen and base._cam_cone_ts != 0
            and ticks_diff(now_ms, base._cam_cone_ts) <= _SEARCH_AVOID_TRIGGER_FRESH_MS):
        rel_x = base._cam_cone_rel_x
        rel_y = base._cam_cone_rel_y
        sweep_x = rel_x * side_sign
        obs_edge = _obstacle_world_edge(base._cam_cone_x, base._cam_cone_y, _recover_edge)
        if (sweep_x > 0.0 and sweep_x <= _SEARCH_AVOID_REL_X_CM
                and rel_y > 0.0 and rel_y < _SEARCH_AVOID_REL_Y_CM
                and (not recover_edge_known or obs_edge != _recover_edge)
                and (not recover_edge_known or obs_edge == 0 or obs_edge == expect_edge)):
            best_d2 = rel_x * rel_x + rel_y * rel_y
            _search_step_avoid_x = base._cam_cone_x
            _search_step_avoid_y = base._cam_cone_y
            found = True
    if (brick_allowed and base._cam_brick_seen and base._cam_brick_ts != 0
            and ticks_diff(now_ms, base._cam_brick_ts) <= _SEARCH_AVOID_TRIGGER_FRESH_MS):
        rel_x = base._cam_brick_rel_x
        rel_y = base._cam_brick_rel_y
        sweep_x = rel_x * side_sign
        d2 = rel_x * rel_x + rel_y * rel_y
        obs_edge = _obstacle_world_edge(base._cam_brick_x, base._cam_brick_y, _recover_edge)
        if (sweep_x > 0.0 and sweep_x <= _SEARCH_AVOID_REL_X_CM
                and rel_y > 0.0 and rel_y < _SEARCH_AVOID_REL_Y_CM
                and (not recover_edge_known or obs_edge != _recover_edge)
                and (not recover_edge_known or obs_edge == 0 or obs_edge == expect_edge)
                and (not found or d2 < best_d2)):
            _search_step_avoid_x = base._cam_brick_x
            _search_step_avoid_y = base._cam_brick_y
            found = True
    _search_step_avoid_valid = found
    if not found:
        _search_step_avoid_x = 999.0
        _search_step_avoid_y = 999.0
    return found


# 功能：按本车与锁存障碍的实际世界坐标规划 STEP 退让方向。
# 退让仍只在横扫轴上进行（前进轴由 STEP 沿 _search_yaw 自己负责），但方向不再
# 盲取"下一次横扫方向"。原来的做法在障碍出现在横扫途中时会把车往来路推，
# 而来路那一侧往往正是刚扫过的搜索区边界，于是越退越靠场地外沿。改为：
#   1) 先按 car_axis - obs_axis 的符号取"远离障碍"的一侧；
#   2) 再用搜索矩形做边界校验，会把车推出物体放置区的方向判为不可用；
#   3) 不可用时退让归零，只保留 STEP 沿 _search_yaw 的前进分量——此时车仍在
#      往搜索区纵深走，本身就在远离边界，不需要也不应该再横向退。
# 每个控制周期重新计算，因此边界校验相当于一堵实时的墙，退让到边距处即停。
# 输入参数：fallback_vx/fallback_vy 为原"下一次横扫方向"，仅在障碍世界坐标
#           无效、或本车与障碍在退让轴上完全重合时作为退路使用。
# 返回值：退让方向单位向量 (vx, vy)；(0.0, 0.0) 表示本周期不做横向退让。
def _search_avoid_dir(fallback_vx, fallback_vy):
    if _search_step_avoid_x >= 900.0 or _search_step_avoid_y >= 900.0:
        return (fallback_vx, fallback_vy)
    # 横扫轴：车头 0/180 度时沿世界 X 横扫，90/270 度时沿世界 Y 横扫。
    axis_is_x = _search_yaw == 0.0 or _search_yaw == 180.0
    if axis_is_x:
        car_axis = base._car.Position_X
        obs_axis = _search_step_avoid_x
        lo = _SEARCH_AREA_X_MIN + _SEARCH_AVOID_AREA_MARGIN_CM
        hi = _SEARCH_AREA_X_MAX - _SEARCH_AVOID_AREA_MARGIN_CM
        fallback_d = fallback_vx
    else:
        car_axis = base._car.Position_Y
        obs_axis = _search_step_avoid_y
        lo = _SEARCH_AREA_Y_MIN + _SEARCH_AVOID_AREA_MARGIN_CM
        hi = _SEARCH_AREA_Y_MAX - _SEARCH_AVOID_AREA_MARGIN_CM
        fallback_d = fallback_vy
    diff = car_axis - obs_axis
    if diff > 0.0:
        avoid_d = 1.0
    elif diff < 0.0:
        avoid_d = -1.0
    else:
        avoid_d = fallback_d
    if (avoid_d > 0.0 and car_axis >= hi) or (avoid_d < 0.0 and car_axis <= lo):
        return (0.0, 0.0)
    if axis_is_x:
        return (avoid_d, 0.0)
    return (0.0, avoid_d)


# 功能：STEP 中按锁存障碍计算退让速度，方向由 _search_avoid_dir 按坐标规划。
# 锁存不受3秒障碍表超时影响；只有STEP完成或下一次SEARCH重置才清除。
def _search_step_request(vx, vy, next_vx, next_vy, yaw):
    repulse = 0.0
    avoid_vx = 0.0
    avoid_vy = 0.0
    if _search_step_avoid_valid:
        dx = _search_step_avoid_x - base._car.Position_X
        dy = _search_step_avoid_y - base._car.Position_Y
        dist = sqrt(dx * dx + dy * dy)
        if dist < _SEARCH_REPULSE_TRIGGER_CM:
            repulse = _SEARCH_REPULSE_KP * (_SEARCH_REPULSE_TRIGGER_CM - dist)
            if repulse > _SEARCH_REPULSE_MAX_SPEED:
                repulse = _SEARCH_REPULSE_MAX_SPEED
            avoid_vx, avoid_vy = _search_avoid_dir(next_vx, next_vy)
            # 方向被搜索区边界否决时不产生任何横向速度，本周期只走 STEP 前进。
            if avoid_vx == 0.0 and avoid_vy == 0.0:
                repulse = 0.0
    # 只有实际产生退让速度时才持续长鸣。障碍虽已锁存、但仍在60cm触发距离外，
    # 或退让方向被边界否决时都不响；每个控制周期刷新一次，斥力消失后由底层
    # 200ms看门狗自动拉低蜂鸣器。
    if repulse > 0.0:
        base.avoid_beep(3)
    base.request_world(vx + avoid_vx * repulse, vy + avoid_vy * repulse, yaw)


# 功能：提交主车 RELOCALIZE 平移速度，并固定沿世界坐标 -Y 方向叠加斥力。
# 文件顶端 _RELOCALIZE_AVOID_ENABLE = 0 时退化为直接提交原速度，不做任何避障。
def _relocalize_request_with_repulse(vx, vy, yaw, now_ms):
    if not _RELOCALIZE_AVOID_ENABLE:
        base.avoid_beep(0)
        base.request_world(vx, vy, yaw)
        return
    _search_obstacle_add(base._cam_cone_x, base._cam_cone_y, base._cam_cone_ts, now_ms)
    _search_obstacle_add(base._cam_brick_x, base._cam_brick_y, base._cam_brick_ts, now_ms)
    if vx * vx + vy * vy <= 0.0001:
        base.avoid_beep(0)
        base.request_world(vx, vy, yaw)
        return
    nearest_d2 = _RELOCALIZE_REPULSE_TRIGGER_CM * _RELOCALIZE_REPULSE_TRIGGER_CM
    found = False
    car_x = base._car.Position_X
    car_y = base._car.Position_Y
    for i in range(_SEARCH_OBS_SLOT_MAX):
        ts = _search_obs_ts[i]
        if ts == 0:
            continue
        if ticks_diff(now_ms, ts) > _SEARCH_OBS_MEMORY_MS:
            _search_obs_x[i] = 999.0
            _search_obs_y[i] = 999.0
            _search_obs_ts[i] = 0
            continue
        dx = _search_obs_x[i] - car_x
        dy = _search_obs_y[i] - car_y
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

# 功能：判断主车当前摄像头是否看到了可处理目标。
# 看到目标后会记录目标 ID、目标边和目标世界坐标，并清空旧接近规划，让下一阶段重新生成双车接近角。
# 返回值：True 表示锁定了新目标；False 表示当前帧没有可用目标。
def _leader_search_target_seen():
    global _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y
    global _search_seen_id, _search_seen_cnt
    if base._cam_obj_count <= 0:
        _search_seen_id = 0
        _search_seen_cnt = 0
        return False
    obj_id = base._cam_obj_id[0]
    if not 1 <= obj_id < len(_obj_remain) or _obj_remain[obj_id] <= 0:
        _search_seen_id = 0
        _search_seen_cnt = 0
        return False
    if (obj_id == _OBJ_ID_RED_SANDBAG
            and _search_red_sandbag_in_brick_zone(base._cam_obj_x[0], base._cam_obj_y[0])):
        # 固定砖区域内的红沙包结果视为红砖误检。本帧不累计目标稳定次数，
        # 保持 SEARCH；等误检丢锁后，OpenART 会重新在 slot0 发布其他候选。
        _search_seen_id = 0
        _search_seen_cnt = 0
        return False
    # MCU 任务层再做一次连续帧确认。OpenART 的轨迹筛选可减少跳变，但不能
    # 代替状态切换防抖；只有同一类目标连续出现才允许锁定目标并等待编队。
    if obj_id == _search_seen_id:
        _search_seen_cnt += 1
    else:
        _search_seen_id = obj_id
        _search_seen_cnt = 1
    if _search_seen_cnt < _SEARCH_TARGET_STABLE_COUNT:
        return False
    _target_obj_id = obj_id
    _target_edge = _obj_to_edge(obj_id)
    _target_obj_world_x = base._cam_obj_x[0]
    _target_obj_world_y = base._cam_obj_y[0]
    _clear_approach_plan()
    return True

# 功能：处理扩展视觉帧中的黄色线矩形中心，并在满足条件时返回黄线所在侧。
# 输入参数：allow_trigger 表示本周期是否允许启动补坐标；物体正在确认或当前不在
# SEARCH_SWEEP 时只维护迟滞锁存，不累计触发。
# 返回值：-1 表示左侧黄线，1 表示右侧黄线，0 表示本周期不触发。
def _search_line_trigger_side(allow_trigger):
    global _search_line_last_seq, _search_line_near_latched
    global _search_line_seen_cnt, _search_line_seen_side
    if base._cam_line_seq == _search_line_last_seq:
        return 0
    _search_line_last_seq = base._cam_line_seq
    line_cx = base._cam_line_cx
    if line_cx >= 900.0:
        _search_line_near_latched = False
        _search_line_seen_cnt = 0
        _search_line_seen_side = 0
        return 0
    line_x = line_cx - _SEARCH_LINE_CX_CENTER_PX
    abs_x = abs(line_x)
    if abs_x > _SEARCH_LINE_CX_RELEASE_PX:
        _search_line_near_latched = False
        _search_line_seen_cnt = 0
        _search_line_seen_side = 0
        return 0
    if (not allow_trigger) or _search_line_fix_used or _search_line_near_latched:
        _search_line_seen_cnt = 0
        _search_line_seen_side = 0
        return 0
    if abs_x >= _SEARCH_LINE_CX_TRIGGER_PX or line_x == 0.0:
        _search_line_seen_cnt = 0
        _search_line_seen_side = 0
        return 0
    side = -1 if line_x < 0.0 else 1
    # 黄线必须与当前横扫方向同侧才允许触发：横扫方向为负时只认左线，
    # 横扫方向为正时只认右线。避免首次 SEARCH 向右出库时被左侧旧边线
    # 抢占，停车转向做一次无用的坐标修正。
    move_side = -1 if _search_sweep_sign < 0.0 else 1
    if side != move_side:
        _search_line_seen_cnt = 0
        _search_line_seen_side = 0
        return 0
    if side == _search_line_seen_side:
        _search_line_seen_cnt += 1
    else:
        _search_line_seen_side = side
        _search_line_seen_cnt = 1
    if _search_line_seen_cnt < _SEARCH_LINE_SEEN_REQUIRE:
        return 0
    _search_line_seen_cnt = 0
    _search_line_seen_side = 0
    _search_line_near_latched = True
    return side

# 功能：启动 SEARCH 内的一次性黄线补坐标流程。
# 输入参数：now_ms 为当前时间；side 为黄线侧，-1 左、1 右。
def _search_line_fix_begin(now_ms, side):
    global _mode_sub, _search_sub_t0_ms, _search_seen_id, _search_seen_cnt
    global _search_line_fix_used, _search_line_saved_yaw
    global _search_line_look_yaw, _search_line_fix_axis
    global _search_line_fix_snap, _search_line_fix_ok_cnt, _search_line_pause_t0
    _search_line_fix_used = True
    _search_line_saved_yaw = _search_yaw
    _search_line_look_yaw = (_search_yaw + side * 90.0) % 360.0
    # 车头为 90/270 度时横扫 Y，侧面黄线用于修 Y；车头为 0/180 度时反之。
    _search_line_fix_axis = 2 if _search_yaw == 90.0 or _search_yaw == 270.0 else 1
    _search_line_fix_snap = 0
    _search_line_fix_ok_cnt = 0
    _search_line_pause_t0 = now_ms
    _search_seen_id = 0
    _search_seen_cnt = 0
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False
    _mode_sub = _SEARCH_LINE_STOP
    _search_sub_t0_ms = now_ms

# 功能：判断黄线视觉坐标是否与刚才看到的边界方向一致。
# 返回值：True 表示坐标轴和高低侧均可信，可计入补坐标稳定帧。
def _search_line_fix_value_ok():
    if _search_line_fix_axis == 1:
        val = base._Position_X_fix
        limit = _FIELD_W
        high_side = _search_line_look_yaw == 90.0
    else:
        val = base._Position_Y_fix
        limit = _FIELD_H
        high_side = _search_line_look_yaw == 0.0
    if val >= 900.0:
        return False
    if high_side:
        return limit - _SEARCH_LINE_FIX_EDGE_GATE_CM < val < limit + 40.0
    return -40.0 < val < _SEARCH_LINE_FIX_EDGE_GATE_CM

# If line-based visual positioning times out after the turn completed, the
# detected yellow boundary still gives one exact field coordinate. Submit a
# single-axis fix through base so the car pose and encoder origin stay aligned.
def _search_line_force_edge_fix():
    if _search_line_fix_axis == 1:
        base._pos_fix_y_valid = False
        base._pos_fix_x = float(_FIELD_W) if _search_line_look_yaw == 90.0 else 0.0
        base._pos_fix_x_valid = True
    else:
        base._pos_fix_x_valid = False
        base._pos_fix_y = float(_FIELD_H) if _search_line_look_yaw == 0.0 else 0.0
        base._pos_fix_y_valid = True
    base._pos_fix_req = True

# 功能：结束坐标取样并把下一次横扫方向强制设置为远离刚看到的边界。
def _search_line_fix_to_turn_back(now_ms):
    global _mode_sub, _search_sub_t0_ms, _search_sweep_sign
    base._vis_x_fix_en = False
    base._vis_y_fix_en = False
    # 0/90 度看的是高坐标侧边界，180/270 度看的是低坐标侧边界。
    _search_sweep_sign = -1.0 if _search_line_look_yaw == 0.0 or _search_line_look_yaw == 90.0 else 1.0
    base.request_hold(_search_line_look_yaw)
    _mode_sub = _SEARCH_LINE_TURN_BACK
    _search_sub_t0_ms = now_ms

# 功能：500 ms 本地急停完成后真正开始横扫；从车在主车运动中持续动态修正阵型。
def _search_start_sweep_now(now_ms):
    global _mode_sub, _search_sub_t0_ms, _self_sub, _self_ready
    global _search_first_sweep_mode_to_other, _search_sweep_cross_ref
    _mode_sub = _SEARCH_SWEEP
    _self_sub = _mode_sub
    _search_sub_t0_ms = now_ms
    _self_ready = 0
    # 车体刚结束硬停车，此刻的位置是这一段横扫的直线基准。
    if _search_yaw == 0.0 or _search_yaw == 180.0:
        _search_sweep_cross_ref = base._car.Position_Y
    else:
        _search_sweep_cross_ref = base._car.Position_X
    if _search_first_sweep_pending and not _search_target_wait:
        _search_first_sweep_mode_to_other = _SEARCH_FIRST_SWEEP_KEEP_RECOVER_SIDE
    else:
        _search_first_sweep_mode_to_other = _SEARCH_FIRST_SWEEP_MODE_INVALID
    base.wireless_send_now()


# 功能：统一进入每次 SWEEP 前的主车本地急停保持。
# 这里只清除主车上一段平移速度并保持搜索航向 500 ms，不检查从车 ready，
# 因此不会恢复已经删除的 SWEEP 编队握手。
def _search_begin_sweep(now_ms):
    global _mode_sub, _search_sub_t0_ms, _self_sub, _self_ready
    _mode_sub = _SEARCH_PRE_SWEEP_HOLD
    _self_sub = _mode_sub
    _search_sub_t0_ms = now_ms
    _self_ready = 0
    base.request_hard_hold(_search_yaw)
    base.wireless_send_now()


# 功能：更新 SEARCH 模式。
# 主车按“纯横移到边界 -> 前进 -> 反向纯横移”连续扫描；
# 锁定目标后直接进入 APPROACH。
def _update_search(now_ms):
    global _OPENART_MODEL_ENABLE, _next_task_mode, _self_sub, _self_ready
    global _master_cmd_sub, _boot_search_signal_t0
    global _mode_sub, _search_sub_t0_ms, _search_leader_t0
    global _search_sweep_sign, _search_step_x0, _search_step_y0, _search_step_target_cm
    global _search_target_wait, _search_hold_yaw
    global _search_seen_id, _search_seen_cnt
    global _search_boot_peer_pending
    global _search_line_fix_snap, _search_line_fix_ok_cnt, _search_line_pause_t0
    global _search_first_sweep_pending, _search_first_sweep_mode_to_other
    _self_sub = _mode_sub
    peer_search_online = (
        base._Other_Car_Mode == _MODE_SEARCH
        and base._Other_Car_Ready_Ts != 0
        and ticks_diff(now_ms, base._Other_Car_Ready_Ts) <= _READY_OTHER_FRESH_MS
    )
    if _search_boot_peer_pending and peer_search_online:
        _search_boot_peer_pending = False
    if _boot_search_signal_t0 != 0:
        # 从车已直接用 SEARCH 跟随开局横移；在确认它进入 SEARCH 且无线状态
        # 新鲜前持续广播首次 SEARCH 信号，之后再补发 300 ms 兼顾偶发丢包。
        if (_search_boot_peer_pending
                or ticks_diff(now_ms, _boot_search_signal_t0) <= _BOOT_SEARCH_SIGNAL_MS):
            _master_cmd_sub = _CMD_SUB_BOOT_SEARCH_SWEEP
        elif _master_cmd_sub == _CMD_SUB_BOOT_SEARCH_SWEEP:
            _master_cmd_sub = _CMD_SUB_NONE
    # 首次 SEARCH 只需确认从车已进入 SEARCH 且无线状态新鲜，即允许锁物切入
    # APPROACH；不再等待从车编队 ready=2，剩余相对位置误差交给后续状态修正。
    # 等待从车模式同步期间仍禁止黄线补坐标，避免启动同步被边线流程打断。
    target_confirming = _search_boot_peer_pending
    # 首次斜移、横扫和换行前进都允许累计并锁定目标；目标确认优先于黄线补坐标。
    target_state = (
        _mode_sub == _SEARCH_FIRST_DIAG
        or _mode_sub == _SEARCH_SWEEP
        or _mode_sub == _SEARCH_STEP_FORWARD
        or _mode_sub == _SEARCH_PRE_SWEEP_HOLD
    )
    if target_state and not _search_target_wait and not _search_boot_peer_pending:
        if _leader_search_target_seen():
            # 目标连续确认完成后直接进入 APPROACH，不再执行第二次 SEARCH
            # 编队等待；从车会通过无线同步目标并跟随切换到 APPROACH。
            _search_hold_yaw = base._car.current_angle
            _search_step_avoid_clear()
            _self_ready = 0
            _search_first_sweep_pending = False
            _search_first_sweep_mode_to_other = _SEARCH_FIRST_SWEEP_MODE_INVALID
            _next_task_mode = _MODE_APPROACH
            return
        # OpenART 的 200 ms 自由选物观察窗通过空 slot0 的保留 track_id 标记。
        # 它不是有效目标，但必须继续压住黄线补坐标，保持“物体优先”。
        target_confirming = base._cam_target_observing or _search_seen_cnt > 0

    if _mode_sub == _SEARCH_FIRST_DIAG:
        _self_ready = 0
        # vx/vy 已在首次 SEARCH 入口按当前位置到终点的方向计算并锁存；运行中
        # 不重新追点。只对有运动分量的轴判定越线，任一轴到达即结束斜移。
        x_reached = (
            (_search_first_diag_vx > 0.01 and base._car.Position_X >= _SEARCH_FIRST_DIAG_END_X)
            or (_search_first_diag_vx < -0.01 and base._car.Position_X <= _SEARCH_FIRST_DIAG_END_X)
        )
        y_reached = (
            (_search_first_diag_vy > 0.01 and base._car.Position_Y >= _SEARCH_FIRST_DIAG_END_Y)
            or (_search_first_diag_vy < -0.01 and base._car.Position_Y <= _SEARCH_FIRST_DIAG_END_Y)
        )
        no_move = abs(_search_first_diag_vx) <= 0.01 and abs(_search_first_diag_vy) <= 0.01
        if x_reached or y_reached or no_move:
            # 首次斜移到位后先停车转到配置的搜索航向。90 度配置会很快通过，
            # 0 度等其他正交航向则在这里完成原地转向，禁止边转边开始横扫。
            _mode_sub = _SEARCH_FIRST_TURN_YAW
            _search_sub_t0_ms = 0
        else:
            base.request_world(
                _search_first_diag_vx,
                _search_first_diag_vy,
                90.0,
            )
            return

    if _mode_sub == _SEARCH_FIRST_TURN_YAW:
        _self_ready = 0
        base.request_world(0.0, 0.0, _search_yaw)
        if abs(_angle_diff(base._car.current_angle, _search_yaw)) <= _SEARCH_YAW_EPS_DEG:
            if _search_sub_t0_ms == 0:
                _search_sub_t0_ms = now_ms
            elif ticks_diff(now_ms, _search_sub_t0_ms) >= _SEARCH_FIRST_TURN_STABLE_MS:
                # 主车自身航向稳定后直接开始第一次 SWEEP；从车动态跟随修正。
                _search_begin_sweep(now_ms)
        else:
            _search_sub_t0_ms = 0
        return

    line_side = _search_line_trigger_side(
        _mode_sub == _SEARCH_SWEEP
        and not _search_target_wait
        and not target_confirming
    )
    if line_side != 0:
        _search_line_fix_begin(now_ms, line_side)
        _self_sub = _mode_sub
        _self_ready = 0
        base.request_hold(base._car.current_angle)
        return

    if _mode_sub == _SEARCH_LINE_STOP:
        _self_ready = 0
        base.request_hold(_search_line_saved_yaw)
        if ticks_diff(now_ms, _search_sub_t0_ms) >= _SEARCH_LINE_STOP_MS:
            _mode_sub = _SEARCH_LINE_TURN
            _search_sub_t0_ms = now_ms
        return

    if _mode_sub == _SEARCH_LINE_TURN:
        _self_ready = 0
        base.request_world(0.0, 0.0, _search_line_look_yaw)
        turn_elapsed = ticks_diff(now_ms, _search_sub_t0_ms)
        if abs(_angle_diff(base._car.current_angle, _search_line_look_yaw)) <= _SEARCH_LINE_TURN_EPS_DEG:
            base.request_hold(_search_line_look_yaw)
            _search_line_fix_snap = base._vis_x_fix_cnt if _search_line_fix_axis == 1 else base._vis_y_fix_cnt
            _search_line_fix_ok_cnt = 0
            _mode_sub = _SEARCH_LINE_POS_FIX
            _search_sub_t0_ms = now_ms
        elif turn_elapsed >= _SEARCH_LINE_TURN_TIMEOUT_MS:
            _search_line_fix_to_turn_back(now_ms)
        return

    if _mode_sub == _SEARCH_LINE_POS_FIX:
        _self_ready = 0
        base.request_hold(_search_line_look_yaw)
        fix_elapsed = ticks_diff(now_ms, _search_sub_t0_ms)
        if fix_elapsed < _SEARCH_LINE_FIX_SETTLE_MS:
            base._vis_x_fix_en = False
            base._vis_y_fix_en = False
            return
        if _search_line_fix_axis == 1:
            base._vis_x_fix_en = True
            base._vis_y_fix_en = False
            changed = base._vis_x_fix_cnt != _search_line_fix_snap
            if changed:
                _search_line_fix_snap = base._vis_x_fix_cnt
        else:
            base._vis_x_fix_en = False
            base._vis_y_fix_en = True
            changed = base._vis_y_fix_cnt != _search_line_fix_snap
            if changed:
                _search_line_fix_snap = base._vis_y_fix_cnt
        if changed:
            if _search_line_fix_value_ok():
                _search_line_fix_ok_cnt += 1
            else:
                _search_line_fix_ok_cnt = 0
        if _search_line_fix_ok_cnt >= _SEARCH_LINE_FIX_REQUIRE:
            _search_line_fix_to_turn_back(now_ms)
        elif fix_elapsed >= _SEARCH_LINE_FIX_TIMEOUT_MS:
            _search_line_force_edge_fix()
            _search_line_fix_to_turn_back(now_ms)
        return

    if _mode_sub == _SEARCH_LINE_TURN_BACK:
        _self_ready = 0
        base.request_world(0.0, 0.0, _search_line_saved_yaw)
        turn_elapsed = ticks_diff(now_ms, _search_sub_t0_ms)
        if (abs(_angle_diff(base._car.current_angle, _search_line_saved_yaw)) <= _SEARCH_LINE_TURN_EPS_DEG
                or turn_elapsed >= _SEARCH_LINE_TURN_TIMEOUT_MS):
            base._vis_x_fix_en = False
            base._vis_y_fix_en = False
            base.request_hold(_search_line_saved_yaw)
            if _search_leader_t0 != 0 and _search_line_pause_t0 != 0:
                _search_leader_t0 = ticks_add(
                    _search_leader_t0,
                    ticks_diff(now_ms, _search_line_pause_t0)
                )
            _search_line_pause_t0 = 0
            _search_begin_sweep(now_ms)
        return

    if _mode_sub == _SEARCH_MOVE_TO_POINT:
        if _search_sub_t0_ms == 0:
            _search_sub_t0_ms = now_ms
        if _prev_task_mode == _MODE_RECOVER and _last_completed_push_yaw < 900.0:
            # 正常 RECOVER 后的搜索线移动均沿 _search_yaw 的车头方向单轴进行。
            # 不使用会随位置误差减速的 request_pos；以恒定速度越过目标线后，
            # 恒速越过目标线后直接开始 SWEEP，从车在运动中继续动态修正。
            if abs(_angle_diff(_last_completed_push_yaw, 270.0)) <= 1.0:
                line_reached = base._car.Position_X >= _search_point_x
            elif abs(_angle_diff(_last_completed_push_yaw, 0.0)) <= 1.0:
                line_reached = base._car.Position_Y <= _search_point_y
            elif abs(_angle_diff(_last_completed_push_yaw, 90.0)) <= 1.0:
                line_reached = base._car.Position_X <= _search_point_x
            elif abs(_angle_diff(_last_completed_push_yaw, 180.0)) <= 1.0:
                line_reached = base._car.Position_Y >= _search_point_y
            else:
                line_reached = False
            timed_out = ticks_diff(now_ms, _search_sub_t0_ms) >= _SEARCH_MOVE_TIMEOUT_MS
            if line_reached or timed_out:
                _search_begin_sweep(now_ms)
                return
            else:
                yaw_rad = radians(_search_yaw)
                base.request_world(
                    sin(yaw_rad) * _SEARCH_MOVE_SPEED,
                    cos(yaw_rad) * _SEARCH_MOVE_SPEED,
                    _search_yaw,
                )
                return
        else:
            dx = _search_point_x - base._car.Position_X
            dy = _search_point_y - base._car.Position_Y
            dist = sqrt(dx * dx + dy * dy)
            # 首次 SEARCH 和异常回退仍允许斜线位置闭环；这些入口需要准确到达
            # 任意预设点，不能套用正常 RECOVER 的单轴恒速越线逻辑。
            if dist <= _SEARCH_POINT_EPS_CM or ticks_diff(now_ms, _search_sub_t0_ms) >= _SEARCH_MOVE_TIMEOUT_MS:
                _search_begin_sweep(now_ms)
                return
            base.request_pos(_search_point_x, _search_point_y, _SEARCH_MOVE_SPEED, _search_yaw)
            return

    if _mode_sub == _SEARCH_PRE_SWEEP_HOLD:
        base.request_hold(_search_yaw)
        hold_elapsed = ticks_diff(now_ms, _search_sub_t0_ms)
        if hold_elapsed >= _SEARCH_PRE_SWEEP_HOLD_MS:
            # 停车时间不是有效扫描时间；已有 SWEEP 计时器时把本次等待补回去。
            if _search_leader_t0 != 0:
                _search_leader_t0 = ticks_add(_search_leader_t0, hold_elapsed)
            _search_start_sweep_now(now_ms)
        return

    if _mode_sub == _SEARCH_TURN_YAW:
        if _search_sub_t0_ms == 0:
            _search_sub_t0_ms = now_ms
        base.request_world(0.0, 0.0, _search_yaw)
        if abs(_angle_diff(base._car.current_angle, _search_yaw)) <= _SEARCH_YAW_EPS_DEG or ticks_diff(now_ms, _search_sub_t0_ms) >= _SEARCH_TURN_TIMEOUT_MS:
            _mode_sub = _SEARCH_MOVE_TO_POINT
            _search_sub_t0_ms = now_ms
            # 正常 RECOVER 后不在转向和到线移动之间停车；本周期直接开始沿
            # 搜索航向恒速前进，下一周期由 MOVE_TO_POINT 继续判断目标线。
            if _prev_task_mode == _MODE_RECOVER and _last_completed_push_yaw < 900.0:
                yaw_rad = radians(_search_yaw)
                base.request_world(
                    sin(yaw_rad) * _SEARCH_MOVE_SPEED,
                    cos(yaw_rad) * _SEARCH_MOVE_SPEED,
                    _search_yaw,
                )
        return

    if _mode_sub == _SEARCH_SWEEP:
        if _search_leader_t0 == 0:
            _search_leader_t0 = now_ms
        if not _search_target_wait and ticks_diff(now_ms, _search_leader_t0) >= config._SEARCH_TIMEOUT_MS:
            _search_first_sweep_pending = False
            _search_first_sweep_mode_to_other = _SEARCH_FIRST_SWEEP_MODE_INVALID
            _next_task_mode = _MODE_RELOCALIZE
            return
        if _search_yaw == 0.0 or _search_yaw == 180.0:
            stop_x = _SEARCH_AREA_X_MAX if _search_sweep_sign > 0.0 else _SEARCH_AREA_X_MIN
            remaining = stop_x - base._car.Position_X if _search_sweep_sign > 0.0 else base._car.Position_X - stop_x
            vx = _search_sweep_sign
            vy = 0.0
            gx = stop_x
            gy = base._car.Position_Y
        else:
            stop_y = _SEARCH_AREA_Y_MAX if _search_sweep_sign > 0.0 else _SEARCH_AREA_Y_MIN
            remaining = stop_y - base._car.Position_Y if _search_sweep_sign > 0.0 else base._car.Position_Y - stop_y
            vx = 0.0
            vy = _search_sweep_sign
            gx = base._car.Position_X
            gy = stop_y
        reached = remaining <= _SEARCH_BOUNDARY_STOP_TOL_CM
        avoid_triggered = _search_latch_sweep_obstacle(now_ms)
        if reached or avoid_triggered:
            if _search_yaw == 0.0:
                forward_remaining = _SEARCH_AREA_Y_MAX - base._car.Position_Y
            elif _search_yaw == 90.0:
                forward_remaining = _SEARCH_AREA_X_MAX - base._car.Position_X
            elif _search_yaw == 180.0:
                forward_remaining = base._car.Position_Y - _SEARCH_AREA_Y_MIN
            else:
                forward_remaining = base._car.Position_X - _SEARCH_AREA_X_MIN
            # 前进轴已经到搜索区尽头：不再向场外前进，只在最后一行反复横扫。
            if forward_remaining <= _SEARCH_BOUNDARY_STOP_TOL_CM:
                # 最后一行没有 STEP_FORWARD；RECOVER 侧阵型仍保持到离开 SEARCH。
                _search_step_avoid_clear()
                _search_sweep_sign = -_search_sweep_sign
                _search_begin_sweep(now_ms)
                return
            # 只有视觉障碍导致提前进入 STEP 才短鸣一次；正常到达坐标边界不提示。
            if avoid_triggered:
                base.avoid_beep(1)
            _search_step_x0 = base._car.Position_X
            _search_step_y0 = base._car.Position_Y
            _search_step_target_cm = min(_SEARCH_FORWARD_STEP_CM, forward_remaining)
            _mode_sub = _SEARCH_STEP_FORWARD
            _search_sub_t0_ms = now_ms
            yaw_rad = radians(_search_yaw)
            next_sweep_sign = -_search_sweep_sign
            if _search_yaw == 0.0 or _search_yaw == 180.0:
                next_vx = next_sweep_sign
                next_vy = 0.0
            else:
                next_vx = 0.0
                next_vy = next_sweep_sign
            _search_step_request(
                sin(yaw_rad) * _SEARCH_STEP_SPEED,
                cos(yaw_rad) * _SEARCH_STEP_SPEED,
                next_vx,
                next_vy,
                _search_yaw,
            )
            return

        # SWEEP 本身不叠加斥力；视觉外围障碍只负责提前切入 STEP_FORWARD。
        # 横向位置闭环：主扫描轴之外的那根轴理论上应该保持在 _search_sweep_cross_ref，
        # 用里程计当前值和基准的偏差生成一个小幅修正速度叠加上去，防止纯开环横移
        # 因为麦轮机械或标定误差越走越斜。
        if _search_yaw == 0.0 or _search_yaw == 180.0:
            lat_corr = _SEARCH_SWEEP_LAT_KP * (_search_sweep_cross_ref - base._car.Position_Y)
        else:
            lat_corr = _SEARCH_SWEEP_LAT_KP * (_search_sweep_cross_ref - base._car.Position_X)
        if lat_corr > _SEARCH_SWEEP_LAT_MAX:
            lat_corr = _SEARCH_SWEEP_LAT_MAX
        elif lat_corr < -_SEARCH_SWEEP_LAT_MAX:
            lat_corr = -_SEARCH_SWEEP_LAT_MAX
        if _search_yaw == 0.0 or _search_yaw == 180.0:
            base.request_world(vx * _SEARCH_SWEEP_SPEED, lat_corr, _search_yaw)
        else:
            base.request_world(lat_corr, vy * _SEARCH_SWEEP_SPEED, _search_yaw)
        return

    if _mode_sub == _SEARCH_STEP_FORWARD:
        # 沿固定车头方向前进，并沿下一次反向横扫方向叠加 cone/brick 斥力。
        # 完成后直接反向横扫。
        if _search_yaw == 0.0:
            forward_remaining = _SEARCH_AREA_Y_MAX - base._car.Position_Y
        elif _search_yaw == 90.0:
            forward_remaining = _SEARCH_AREA_X_MAX - base._car.Position_X
        elif _search_yaw == 180.0:
            forward_remaining = base._car.Position_Y - _SEARCH_AREA_Y_MIN
        else:
            forward_remaining = base._car.Position_X - _SEARCH_AREA_X_MIN
        dx = base._car.Position_X - _search_step_x0
        dy = base._car.Position_Y - _search_step_y0
        yaw_rad = radians(_search_yaw)
        moved = dx * sin(yaw_rad) + dy * cos(yaw_rad)
        if (moved >= _search_step_target_cm
                or forward_remaining <= _SEARCH_BOUNDARY_STOP_TOL_CM
                or ticks_diff(now_ms, _search_sub_t0_ms) >= _SEARCH_STEP_TIMEOUT_MS):
            # Forward 结束后先由统一入口急停保持 500 ms，再反向 SWEEP；
            # 从车在动态环境中继续修正侧阵型，不增加 ready 握手。
            _search_step_avoid_clear()
            _search_sweep_sign = -_search_sweep_sign
            _search_begin_sweep(now_ms)
            return
        next_sweep_sign = -_search_sweep_sign
        if _search_yaw == 0.0 or _search_yaw == 180.0:
            next_vx = next_sweep_sign
            next_vy = 0.0
        else:
            next_vx = 0.0
            next_vy = next_sweep_sign
        _search_step_request(
            sin(yaw_rad) * _SEARCH_STEP_SPEED,
            cos(yaw_rad) * _SEARCH_STEP_SPEED,
            next_vx,
            next_vy,
            _search_yaw,
        )
        return
    # 兜底：如果状态无效，立即停车
    base.request_hold(base._car.current_angle)
# 功能：根据当前任务模式设置 OpenART 识别开关。
# 输入参数：mode 为即将进入或正在运行的任务模式。
def _set_openart_switches_for_mode(mode):
    global _OPENART_BALL_ENABLE, _OPENART_LINE_ENABLE, _OPENART_MODEL_ENABLE
    _OPENART_LINE_ENABLE = mode == _MODE_RECOVER or mode == _MODE_RELOCALIZE or mode == _MODE_SEARCH or (mode == _MODE_PUSH_SYNC)
    _OPENART_MODEL_ENABLE = mode == _MODE_SEARCH or mode == _MODE_APPROACH or mode == _MODE_WAIT_READY or mode == _MODE_RELOCALIZE or mode == _MODE_DONE or (mode == _MODE_PUSH_SYNC)
    _OPENART_BALL_ENABLE = debug_switch.OPENART_BALL_DEBUG_ENABLE

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


# 功能：主车任务层每周期更新入口。
# base.loop 会周期性调用它；本函数根据当前模式分发到对应 update，处理模式切换请求，并返回当前任务模式供调试或通信使用。
# 返回值：当前任务模式编号。
def update():
    global _next_task_mode, _self_mode, _target_rel_x_for_cam, _target_rel_y_for_cam
    now = ticks_ms()
    mode = _task_mode
    _self_mode = mode
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
    return _task_mode

# 功能：进入新的主任务模式。
# 该函数统一处理模式切换时的清理和初始化：更新 OpenART 开关、停止面向目标控制、调用各模式 reset、设置视觉修正开关、清理旧目标和协同命令。
# 输入参数：new_mode 为目标任务模式编号。
def enter_mode(new_mode):
    global _approach_cmd_to_other, _approach_plan_valid, _follower_cmd_yaw_dir, _follower_reenter_creep, _master_cmd_sub, _next_task_mode, _prev_task_mode, _push_route_axis, _push_route_move_yaw, _push_route_phase, _push_route_restore_push, _route_cmd_to_other, _route_obs_axis_to_other, _target_edge, _target_obj_id, _target_obj_world_x, _target_obj_world_y, _target_sel_id_for_cam, _task_mode
    global _approach_from_push_lost
    global _search_first_sweep_pending, _search_first_sweep_mode_to_other
    global _search_resume_yaw, _search_resume_yaw_valid
    gc.collect()
    if new_mode == _MODE_APPROACH:
        if _task_mode == _MODE_SEARCH:
            _search_resume_yaw = _search_yaw
            _search_resume_yaw_valid = True
    if _task_mode == _MODE_SEARCH and new_mode != _MODE_SEARCH:
        _search_step_avoid_clear()
        _search_first_sweep_pending = False
        _search_first_sweep_mode_to_other = _SEARCH_FIRST_SWEEP_MODE_INVALID
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
        _approach_from_push_lost = False
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
