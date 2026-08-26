# Split base/runtime layer generated from app.py.
import config
import gc
import micropython
from machine import *
from math import atan2, cos, degrees, pi, radians, sin, sqrt
from micropython import const
from seekfree import *
from smartcar import *
from time import sleep_ms, ticks_add, ticks_diff, ticks_ms, ticks_us

# First-boot open-loop bootstrap motion settings used by both roles.
# 这些参数被 task_follow / task_leader 以 base.xxx 跨模块引用，不能用 const()：
# 下划线开头的 const 是编译期常量，mpy-cross 会内联成字面量、不生成模块属性，
# 固件上 base._BOOT_LATERAL_SPEED 会 AttributeError。用普通模块变量。
#
# 开局横移：两车在 BOOT_SYNC 里各自锁 90° 车头，以世界 +Y 为主方向离开出发点，
# 同时沿车体前后轴叠加小速度（主车前进、从车后退）以拉开前后间距。
# 主从横移距离分开配置；判据仍是 Position_Y 相对起点的增量，
# 达到该距离（task_leader 的 _boot_lateral_y0 分支、task_follow 同名分支），
# 纯里程计开环，没有视觉参与，所以里程计标定不准会直接体现为横移距离偏差。
_BOOT_LATERAL_DISTANCE_LEADER_CM = 60
_BOOT_LATERAL_DISTANCE_FOLLOWER_CM = 60

# 上面这段开局横移的世界 Y 速度，主从共用。
_BOOT_LATERAL_SPEED = 160

# 开局横移期间叠加的车体前后速度幅值，主从共用。
# 车头锁定 90° 时，车体前后轴对应世界 X 轴：主车取正值前进，从车取负值后退。
_BOOT_LONGITUDINAL_SPEED = 20

# 主从车共同的固定上电世界航向。PoseCar 会在 task 模块导入完成前创建，
# 因此该值必须属于底层，不能反向读取 task_leader/task_follow。
_INIT_YAW = const(90)

# 调试显示开关。1 表示初始化并刷新 LCD，0 表示不启用 LCD 调试界面。
_DEBUG_LCD = const(1)

# 视觉识别目标类别上限。1 到 _OBJ_ID_MAX 表示普通任务目标编号。
_OBJ_ID_MAX = const(5)

# 左球、右球在 OpenART/摄像头识别结果中的类别编号。
_BALL_L_CLASS_ID = const(6)

_BALL_R_CLASS_ID = const(7)

_CONE_CLASS_ID = const(8)

_BRICK_CLASS_ID = const(9)

# 红沙包/红砖同色误识别过滤已经改到上位机（openart.py 的
# RED_SANDBAG_X_FILTER_ENABLE），那边能在检测结果送出前就用同一份
# 位姿数据滤掉，不再需要下位机这边重复判断，避免占用解析逻辑和 CPU 时间。

# 摄像头协议中的无效目标 ID。收到 255 表示该槽位没有有效目标。
_CAM_OBJ_INVALID_ID = const(255)
_CAM_TARGET_OBSERVING_TRACK_MARKER = const(254)

# 任务状态机初始模式和搜索模式编号，与 task.py 中的模式编号配合使用。
_MODE_BOOT_SYNC = const(1)

_MODE_SEARCH = const(12)

# 周期中断计数器，用于按不同频率调度通信、姿态、运动和电机控制。
_ticker_count = 0

# 临时索引/计时变量，串口环形缓冲解析和定时循环中会复用。
_t = 0

# 主循环单次执行耗时的最大记录，单位 ms，用于调试卡顿。
_loop_max_ms = 0

# IMU 原始 Z 轴陀螺仪值和姿态积分角度相关变量。
_gyro_z = 0

_YawAngle_Trans = 0

_Yaw_Angle_Old = 0

_YawAngle_imu = 0

# 视觉修正角度。999 表示当前没有可用的视觉角度修正值。
_YawAngle_fix = 999

# 视觉/定位数据更新标志。置 1 表示中断或串口刚收到新数据，等待融合到车辆位姿。
_angle_update_flag = 0

_position_update_flag = 0

# 是否允许视觉对 yaw/x/y 进行修正。关闭时只记录视觉数据，不覆盖里程计。
_vis_yaw_fix_en = False

_vis_x_fix_en = False

_vis_y_fix_en = False

# 视觉坐标修正的边界距离门槛，单位 cm。0 表示不限制；大于 0 时，
# 只有落在对应轴任一场地边界附近的坐标才允许计数并写入里程计。
# PUSH 由任务层设置具体门槛，其他状态保持 0，避免改变原有修正行为。
_vis_pos_fix_edge_gate_cm = 0.0

_VIS_FIX_FIELD_W = const(310)

_VIS_FIX_FIELD_H = const(230)

_vis_x_fix_cnt = 0

_vis_y_fix_cnt = 0

# 视觉给出的 X/Y 修正值。
_Position_X_fix = 0

_Position_Y_fix = 0

# 修正成功后的蜂鸣器提示状态和结束时间。
_fix_beep_active = 0

# 避障专用蜂鸣：独立于 _fix_beep（视觉修正会每 50ms 抢占 _fix_beep，斜避的响声混在里面
# 分辨不出）。任务层调 avoid_beep(kind) 触发，_update_avoid_beep 每周期驱动蜂鸣器。
# kind: 1=斜避(单声) 2=路线重就位(双声) 3=持续高电平。
_avoid_beep_kind = 0

_avoid_beep_pattern_t0 = 0

_avoid_beep_last = 0

_fix_beep_until_ms = 0

# 四个编码器的本次增量读数。
_encoder1 = 0

_encoder2 = 0

_encoder3 = 0

_encoder4 = 0

# 仅由编码器里程计积分得到的车体位置，视觉或外部修正会同步覆盖它。
_Position_X_encoder = 0

_Position_Y_encoder = 0

# 请求清空速度 PID 积分/历史项，常用于停车保持前避免残余输出。
_speed_reset_req = False

# 外部位置修正请求。pos_fix_x/y_valid 控制是否只修正某一个轴。
_pos_fix_req = False

_pos_fix_x = 999.0

_pos_fix_y = 999.0

_pos_fix_x_valid = False

_pos_fix_y_valid = False

# 底盘控制模式：
# 速度模式直接跟踪世界速度
_CTRL_VEL = const(0)
#位置模式先用位置 PID 生成速度
_CTRL_POS = const(1)

_ctrl_mode = _CTRL_VEL

# 位置模式目标点和最大平移速度。
_target_pos_x = 0.0

_target_pos_y = 0.0

_pos_speed_max = 200.0

# 世界坐标系下的目标速度，之后会根据当前 yaw 转成车体系速度。
_target_world_vx = 0.0

_target_world_vy = 0.0

# 串口发送调度标志。定时器只置位，主循环/通信更新函数负责真正发送。
_send_flag = 0

_all_send_flag = 0

_wireless_send_flag = 0

# 双车无线通信应答状态和时间戳，用于主车等待从车回包、从车延时应答。
_wireless_reply_pending = 0

_wireless_reply_due_ms = 0

_wireless_wait_reply = 0

_wireless_last_tx_ms = 0

# 摄像头和无线接收时间统计，用于判断链路是否在线以及调试最大间隔/RTT。
_cam_rx_last_ms = 0

_wireless_rx_last_ms = 0

_cam_intv_ms = 0

_cam_intv_max_ms = 0

_wireless_intv_max_ms = 0

_wireless_rtt_max_ms = 0

# 接收字节计数，主要用于 LCD/串口调试显示链路是否有数据流动。
_cam_rx_bytes = 0

_wire_rx_bytes = 0

# 摄像头帧 CRC 校验失败计数：高负载下 OpenART 发送被截断/错位时，
# uart_get_frame_v2 会静默丢弃该帧；这个计数器把丢帧变得可观测，
# 用来区分"感知/发送慢"（_cam_intv_max_ms 大但本计数不涨）
# 和"链路本身在丢坏帧"（本计数持续增长）。
_cam_crc_fail_cnt = 0

# Frame3 发送帧序号。
_frame3_seq = 0

# 对方车辆的最近状态。999/999.9 表示还没有收到有效值。
_Other_Car_X = 999.0

_Other_Car_Y = 999.0

_Other_Car_Angle = 90.0

_Other_Car_Ready = 0

_Other_Car_Ready_Ts = 0

_Other_Car_Mode = 0

_Other_Car_Push_Sub = 0

# 主车在 RECOVER 后第一次 SEARCH/SWEEP 前发送的横扫方向：
# 0=无效，1=相对 search_yaw 向左，2=相对 search_yaw 向右，3=保留。
_Other_Search_First_Sweep_Mode = 0

# 对方是否在当前 SEARCH/RELOCALIZE 跟随阶段已经通过视觉看到本车。
_Other_Car_Seen_Me = False

# 对方选择的目标物体、目标边和协同指令。
_Other_Target_X = 999.0

_Other_Target_Y = 999.0

_Other_Cone_X = 999.0

_Other_Cone_Y = 999.0

_Other_Cone_Ts = 0

_Other_Brick_X = 999.0

_Other_Brick_Y = 999.0

_Other_Brick_Ts = 0

_Other_Target_Edge = 0

_Other_Target_ObjId = 0

_Other_Approach_Cmd = 999.9

_Other_Route_Obs_Axis = 999.9

_Other_Route_Cmd = 999.9

_Other_Master_Cmd_Sub = 0

_Other_Follower_Cmd_Yaw_Dir = 999.9

_Other_World_Vx = 999.0

_Other_World_Vy = 999.0

# 对方最近下发/回报的命令序号、命令字和横向保持速度（主车侧收到的是从车 ACK）。
_Other_Cmd_Seq = 0

_Other_Push_Side_Cmd = 0.0

# 推送目标时给对方共享的横向修正量，以及面向目标时直接使用的角速度命令。
_push_world_side_cmd = 0.0

_face_obj_active = False

_face_obj_gyro_cmd = 0.0

# 固定世界 yaw 角速度请求。启用时绕过外层角度 PID，直接给角速度环目标值。
_yaw_rate_active = False

_yaw_rate_target_raw = 0.0

# 摄像头一帧中本程序会保存的普通目标数量，以及协议允许的最大目标槽位数量。
_CAM_OBJ_MAX = const(2)
_CAM_FRAME_OBJ_MAX = const(7)
# 正式摄像头 Frame2 协议固定为类型 0x07、7 个槽位和 40 字节。
_CAM_FRAME_TYPE = const(7)
_CAM_FRAME_LEN = const(40)
_CAM_OBJ_DATA_OFF = const(9)
_CAM_LINE_DATA_OFF = const(37)

_cam_obj_count = 0
_cam_target_observing = False

_cam_obj_x = [0.0] * _CAM_OBJ_MAX

_cam_obj_y = [0.0] * _CAM_OBJ_MAX

_cam_obj_rel_x = [999.0] * _CAM_OBJ_MAX

_cam_obj_rel_y = [999.0] * _CAM_OBJ_MAX

_cam_obj_id = [0] * _CAM_OBJ_MAX

_cam_car_x = 999.0
_cam_car_y = 999.0
_cam_car_rel_x = 999.0
_cam_car_rel_y = 999.0
_cam_cone_x = 999.0
_cam_cone_y = 999.0
_cam_cone_rel_x = 999.0
_cam_cone_rel_y = 999.0
_cam_cone_ts = 0
_cam_cone_seen = 0
_cam_cone_seen_seq = 0

_cam_brick_x = 999.0
_cam_brick_y = 999.0
_cam_brick_rel_x = 999.0
_cam_brick_rel_y = 999.0
_cam_brick_ts = 0
_cam_brick_seen = 0
_cam_brick_seen_seq = 0

# 0x07 摄像头帧中黄色线矩形框中心的原始像素 x。999 表示本帧无有效黄线框。
_cam_line_cx = 999.0
_cam_line_seq = 0
_cam_line_last_ms = 0

# cone/brick 视觉坐标的新鲜度时间，单位 ms。
_CONE_MEMORY_MS = const(3000)

_cam_ball_rel_x = [999.0, 999.0]
_cam_ball_rel_y = [999.0, 999.0]

class PoseCar:

    # 初始化车辆位姿与控制目标数据。
    # 保存当前位置、速度、目标速度、目标角度等底盘运行状态。
    # 根据 config._IS_LEADER 区分主车和从车的初始坐标。
    def __init__(self):
        self.Position_Pointer = 0
        self.Speed_X = 0.0
        self.Speed_Y = 0.0
        self.Speed_Z = 0.0
        self.target_Speed_X = 0.0
        self.target_Speed_Y = 0.0
        self.target_Speed_Z = 0.0
        self.GYRO_Z = 0.0
        self.current_angle = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_GYRO_Z = 0.0
        self.target_angle = 0.0
        self.target_carry_x = 0.0
        self.target_carry_y = 0.0
        self.target_distanceX = 0.0
        self.target_distanceY = 0.0
        self.target_Position_X = 0.0
        self.target_Position_Y = 0.0
        if config._IS_LEADER:
            self.Position_X, self.Position_Y = config._LEADER_INIT_POS
        else:
            self.Position_X, self.Position_Y = config._FOLLOWER_INIT_POS
        self.current_angle = _INIT_YAW

_car = PoseCar()

# 功能：初始化任务层和通信层的运行状态，清空目标、协同指令、视觉缓存和收发标志。
def state_init():
    global _send_flag, _all_send_flag, _wireless_send_flag
    global _push_world_side_cmd
    global _face_obj_active, _face_obj_gyro_cmd
    global _yaw_rate_active, _yaw_rate_target_raw
    global _Other_Target_X, _Other_Target_Y, _Other_Target_Edge, _Other_Target_ObjId
    global _Other_Approach_Cmd, _Other_Route_Obs_Axis
    global _Other_Route_Cmd, _Other_Master_Cmd_Sub
    global _Other_Follower_Cmd_Yaw_Dir
    global _Other_Car_Ready, _Other_Car_Ready_Ts, _Other_Car_Mode, _Other_Car_Push_Sub, _Other_Car_Seen_Me
    global _Other_Search_First_Sweep_Mode
    global _cam_obj_count, _cam_target_observing
    global _cam_car_x, _cam_car_y, _cam_car_rel_x, _cam_car_rel_y
    global _cam_cone_x, _cam_cone_y, _cam_cone_rel_x, _cam_cone_rel_y, _cam_cone_ts, _cam_cone_seen, _cam_cone_seen_seq
    global _cam_brick_x, _cam_brick_y, _cam_brick_rel_x, _cam_brick_rel_y, _cam_brick_ts, _cam_brick_seen, _cam_brick_seen_seq
    global _cam_line_cx, _cam_line_seq, _cam_line_last_ms
    global _Other_Cone_X, _Other_Cone_Y, _Other_Cone_Ts
    global _Other_Brick_X, _Other_Brick_Y, _Other_Brick_Ts
    global _cam_rx_last_ms, _wireless_rx_last_ms
    global _cam_intv_ms, _cam_intv_max_ms
    global _wireless_intv_max_ms, _wireless_rtt_max_ms
    global _Other_Cmd_Seq, _Other_Push_Side_Cmd
    global _Other_World_Vx, _Other_World_Vy
    count_mode = int(getattr(config, "_OBJ_COUNT_MODE", 2))
    if count_mode != 1 and count_mode != 2 and count_mode != 3:
        count_mode = 2
    task._obj_count_mode = count_mode
    task._obj_total_remaining = 0
    task._obj_group_remain = []
    if count_mode == 1:
        task._obj_total_remaining = max(0, int(getattr(config, "_OBJ_UNKNOWN_TOTAL", 0)))
        enabled = 1 if task._obj_total_remaining > 0 else 0
        for i in range(1, len(task._obj_remain)):
            task._obj_remain[i] = enabled
            task._obj_done[i] = 0
    elif count_mode == 2:
        groups = getattr(config, "_OBJ_GROUPS", ())
        group_total = getattr(config, "_OBJ_GROUP_TOTAL", ())
        for i in range(1, len(task._obj_remain)):
            task._obj_remain[i] = 0
            task._obj_done[i] = 0
        for group_idx in range(len(groups)):
            remain = 0
            if group_idx < len(group_total):
                remain = max(0, int(group_total[group_idx]))
            task._obj_group_remain.append(remain)
            task._obj_total_remaining += remain
            if remain > 0:
                for obj_id in groups[group_idx]:
                    if 1 <= obj_id < len(task._obj_remain):
                        task._obj_remain[obj_id] = 1
    else:
        # 模式 3：按类别 ID 分别保存剩余数量。_obj_remain 不再只是开关，
        # 而是每一类尚需成功推出的真实个数；目标筛选仍可直接用 > 0 判断。
        class_total = getattr(config, "_OBJ_CLASS_TOTAL", ())
        for obj_id in range(1, len(task._obj_remain)):
            remain = 0
            if obj_id < len(class_total):
                remain = max(0, int(class_total[obj_id]))
            task._obj_remain[obj_id] = remain
            task._obj_done[obj_id] = 0
            task._obj_total_remaining += remain
    task._task_mode = _MODE_BOOT_SYNC
    task._next_task_mode = -1
    task._prev_task_mode = _MODE_SEARCH
    task._self_ready = 0
    task._self_mode = 0
    task._self_sub = 0
    task._wireless_car_seen = False
    _push_world_side_cmd = 0.0
    task._cmd_seq = 0
    task._master_cmd_sub = 0
    task._follower_cmd_yaw_dir = 999.9
    task._search_first_sweep_mode_to_other = 0
    _send_flag = 0
    _all_send_flag = 0
    _wireless_send_flag = 0
    _face_obj_active = False
    _face_obj_gyro_cmd = 0.0
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    task._target_obj_id = 0
    task._target_edge = 0
    task._target_obj_world_x = 999.0
    task._target_obj_world_y = 999.0
    task._target_sel_id_for_cam = 0
    task._target_rel_x_for_cam = 999.0
    task._target_rel_y_for_cam = 999.0
    task._approach_plan_valid = 0
    task._approach_plan_edge = 0
    task._approach_plan_obj_id = 0
    task._approach_self_yaw = 0.0
    task._approach_self_dir = 0
    task._approach_cmd_to_other = 999.9
    task._route_obs_axis_to_other = 999.9
    task._route_cmd_to_other = 999.9
    task._push_route_axis = 999.9
    task._push_route_phase = 0
    task._push_route_move_yaw = 999.9
    task._push_route_restore_push = 0
    _Other_Target_X = 999.0
    _Other_Target_Y = 999.0
    _Other_Target_Edge = 0
    _Other_Target_ObjId = 0
    _Other_Approach_Cmd = 999.9
    _Other_Route_Obs_Axis = 999.9
    _Other_Route_Cmd = 999.9
    _Other_Master_Cmd_Sub = 0
    _Other_Follower_Cmd_Yaw_Dir = 999.9
    _Other_Cmd_Seq = 0
    _Other_Push_Side_Cmd = 0.0
    _Other_Car_Ready = 0
    _Other_Car_Ready_Ts = 0
    _Other_Car_Mode = 0
    _Other_Car_Push_Sub = 0
    _Other_Search_First_Sweep_Mode = 0
    _Other_Car_Seen_Me = False
    _Other_World_Vx = 999.0
    _Other_World_Vy = 999.0
    _cam_rx_last_ms = 0
    _wireless_rx_last_ms = 0
    _cam_intv_ms = 0
    _cam_intv_max_ms = 0
    _wireless_intv_max_ms = 0
    _wireless_rtt_max_ms = 0
    _cam_obj_count = 0
    _cam_target_observing = False
    _cam_car_x = 999.0
    _cam_car_y = 999.0
    _cam_car_rel_x = 999.0
    _cam_car_rel_y = 999.0
    _cam_cone_x = 999.0
    _cam_cone_y = 999.0
    _cam_cone_ts = 0
    _cam_cone_seen = 0
    _cam_cone_seen_seq = 0
    _cam_brick_x = 999.0
    _cam_brick_y = 999.0
    _cam_brick_rel_x = 999.0
    _cam_brick_rel_y = 999.0
    _cam_brick_ts = 0
    _cam_brick_seen = 0
    _cam_brick_seen_seq = 0
    _cam_line_cx = 999.0
    _cam_line_seq = 0
    _cam_line_last_ms = 0
    _cam_ball_rel_x[0] = 999.0
    _cam_ball_rel_y[0] = 999.0
    _cam_ball_rel_x[1] = 999.0
    _cam_ball_rel_y[1] = 999.0
    _cam_cone_rel_x = 999.0
    _cam_cone_rel_y = 999.0
    _Other_Cone_X = 999.0
    _Other_Cone_Y = 999.0
    _Other_Cone_Ts = 0
    _Other_Brick_X = 999.0
    _Other_Brick_Y = 999.0
    _Other_Brick_Ts = 0
    task._obj_map_count = 0

# 功能：应用基础控制配置，把运动控制、位置修正请求和世界速度恢复到默认初始状态。
# 输入参数：无。
# 返回值：无。该函数通过全局变量重置控制模式和目标速度/位置。
def apply_config():
    global _speed_reset_req
    _speed_reset_req = False
    global _ctrl_mode, _target_pos_x, _target_pos_y, _pos_speed_max
    global _target_world_vx, _target_world_vy
    global _yaw_rate_active, _yaw_rate_target_raw
    _ctrl_mode = _CTRL_VEL
    _target_pos_x = 0.0
    _target_pos_y = 0.0
    _pos_speed_max = 200.0
    _target_world_vx = 0.0
    _target_world_vy = 0.0
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    global _pos_fix_req, _pos_fix_x, _pos_fix_y, _pos_fix_x_valid, _pos_fix_y_valid
    _pos_fix_req = False
    _pos_fix_x = 999.0
    _pos_fix_y = 999.0
    _pos_fix_x_valid = False
    _pos_fix_y_valid = False

# 运动请求模式编号。上层任务通过 request_xxx 写入请求，motion_step 再执行。
_MODE_HOLD = const(1)

_MODE_WORLD = const(2)

_MODE_POS = const(3)

_MODE_PUSH = const(4)

# 当前运动请求缓存。_req_seq 每次请求递增，用来判断是否有新命令。
_req_mode = 0

_req_vx = 0.0

_req_vy = 0.0

_req_angle = 0.0

_req_tx = 0.0

_req_ty = 0.0

_req_speed = 0.0

_req_seq = 0

# 推送目标请求缓存。999.9 常表示不指定角度/轴/命令，由内部逻辑自动选择。
_push_req_edge = 0

_push_req_lock_yaw = 0.0

_push_req_cam_seen = 0

_push_req_rel_x = 0.0

_push_req_rel_y = 0.0

_push_req_ref_valid = 0

_push_req_ref_x = 0.0

_push_req_ref_y = 0.0

_push_req_move_yaw = 999.9

_push_req_world_side_valid = 0

_push_req_world_side_axis = 0

_push_req_world_side_ref = 0.0

_push_req_ff_valid = 0

_push_req_ff_vx = 0.0

_push_req_ff_vy = 0.0

_push_req_seq = 0

_push_reset_seq = 0

_face_req_active = 0

_face_req_err = 0.0

_face_req_seq = 0

# 功能：请求底盘按世界坐标系速度运行。
# 输入参数：vx 为世界 X 方向目标速度；vy 为世界 Y 方向目标速度；angle 为期望车头角度，单位度。
# 返回值：无。请求会写入全局缓存，并通过 _req_seq 通知 motion_step 处理。
def request_world(vx, vy, angle):
    global _req_mode, _req_vx, _req_vy, _req_angle, _req_seq
    global _yaw_rate_active, _yaw_rate_target_raw
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    _req_vx = vx
    _req_vy = vy
    _req_angle = angle
    _req_mode = _MODE_WORLD
    _req_seq += 1

# 功能：请求车辆以固定世界 yaw 角速度原地自转。
# 该请求绕过外层车头角度 PID，但保留底层陀螺角速度环；正值使世界 yaw 增大，
# 负值使世界 yaw 减小。输入使用度每秒，内部换算为陀螺原始角速度单位。
# 输入参数：yaw_rate_dps 为固定世界 yaw 角速度，单位度每秒。
def request_yaw_rate(yaw_rate_dps):
    global _req_mode, _req_vx, _req_vy, _req_angle, _req_seq
    global _yaw_rate_active, _yaw_rate_target_raw
    global _face_req_active, _face_req_seq
    _req_vx = 0.0
    _req_vy = 0.0
    _req_angle = _car.current_angle
    _req_mode = _MODE_WORLD
    _req_seq += 1
    # 当前角度积分的世界 yaw 与 IMU 原始 Z 轴符号相反；70/1000 是原始值到 deg/s 的比例。
    _yaw_rate_target_raw = -yaw_rate_dps * (1000.0 / 70.0)
    _yaw_rate_active = True
    _face_req_active = 0
    _face_req_seq += 1

# 功能：请求底盘移动到指定世界坐标位置。
# 输入参数：tx 为目标 X 坐标；ty 为目标 Y 坐标；speed 为位置模式最大速度；angle 为到点过程中的目标车头角度，单位度。
# 返回值：无。请求会写入全局缓存，并通过 _req_seq 通知 motion_step 处理。
def request_pos(tx, ty, speed, angle):
    global _req_mode, _req_tx, _req_ty, _req_speed, _req_angle, _req_seq
    global _yaw_rate_active, _yaw_rate_target_raw
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    _req_tx = tx
    _req_ty = ty
    _req_speed = speed
    _req_angle = angle
    _req_mode = _MODE_POS
    _req_seq += 1

# 功能：请求车辆原地保持，不再主动平移，只保持指定车头角度。
# 输入参数：angle 为目标车头角度，单位度。
# 返回值：无。请求会写入全局缓存，并通过 _req_seq 通知 motion_step 处理。
def request_hold(angle):
    global _req_mode, _req_vx, _req_vy, _req_angle, _req_seq
    global _yaw_rate_active, _yaw_rate_target_raw
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    _req_vx = 0.0
    _req_vy = 0.0
    _req_angle = angle
    _req_mode = _MODE_HOLD
    _req_seq += 1

# 功能：请求车辆立即清零平移速度并保持指定航向。
# 与 request_hold 不同，本函数绕过世界速度缓停斜坡，同时清零当前车体系速度目标，
# 并请求速度环在下一个电机控制周期清空积分，用于必须限制额外滑行距离的状态交接。
def request_hard_hold(angle):
    global _req_mode, _req_vx, _req_vy, _req_angle, _req_seq
    global _yaw_rate_active, _yaw_rate_target_raw
    global _target_world_vx, _target_world_vy, _speed_reset_req
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    _req_vx = 0.0
    _req_vy = 0.0
    _req_angle = angle
    _req_mode = _MODE_HOLD
    _req_seq += 1
    _target_world_vx = 0.0
    _target_world_vy = 0.0
    _car.target_Speed_X = 0.0
    _car.target_Speed_Y = 0.0
    _speed_reset_req = True

# 功能：请求进入推送目标控制，根据目标边、视觉相对位置、参考位置和可选前馈速度生成推送运动。
# 输入参数：edge 为推送边编号；lock_yaw 为需要锁定的车头角度；cam_seen 表示本周期是否看到目标；rel_x/rel_y 为目标相对坐标；ref_valid/ref_x/ref_y 为视觉误差参考点；move_yaw 为指定推送移动方向，999.9 表示自动按 edge 选择；world_side_valid/world_side_axis/world_side_ref 为世界坐标横向约束；ff_valid/ff_vx/ff_vy 为外部前馈世界速度。
# 返回值：无。函数只更新推送目标请求缓存，并递增请求序号。
def request_push(edge, lock_yaw, cam_seen, rel_x, rel_y, ref_valid, ref_x, ref_y, move_yaw=999.9, world_side_valid=False, world_side_axis=0, world_side_ref=0.0, ff_valid=False, ff_vx=0.0, ff_vy=0.0):
    global _req_mode, _req_seq
    global _yaw_rate_active, _yaw_rate_target_raw
    global _push_req_edge, _push_req_lock_yaw, _push_req_cam_seen
    global _push_req_rel_x, _push_req_rel_y
    global _push_req_ref_valid, _push_req_ref_x, _push_req_ref_y
    global _push_req_move_yaw, _push_req_world_side_valid
    global _push_req_world_side_axis, _push_req_world_side_ref, _push_req_seq
    global _push_req_ff_valid, _push_req_ff_vx, _push_req_ff_vy
    _push_req_edge = edge
    _push_req_lock_yaw = lock_yaw
    _push_req_cam_seen = 1 if cam_seen else 0
    _push_req_rel_x = rel_x
    _push_req_rel_y = rel_y
    _push_req_ref_valid = 1 if ref_valid else 0
    _push_req_ref_x = ref_x
    _push_req_ref_y = ref_y
    _push_req_move_yaw = move_yaw
    _push_req_world_side_valid = 1 if world_side_valid else 0
    _push_req_world_side_axis = world_side_axis
    _push_req_world_side_ref = world_side_ref
    _push_req_ff_valid = 1 if ff_valid else 0
    _push_req_ff_vx = ff_vx
    _push_req_ff_vy = ff_vy
    _push_req_seq += 1
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    _req_mode = _MODE_PUSH
    _req_seq += 1

# 功能：清空推送目标请求和推送控制器的内部参考量。
# 返回值：无。函数递增 _push_reset_seq，使 _push_step 在下一次运行时同步重置内部状态。
def reset_push():
    global _push_req_edge, _push_req_lock_yaw, _push_req_cam_seen
    global _push_req_rel_x, _push_req_rel_y
    global _push_req_ref_valid, _push_req_ref_x, _push_req_ref_y
    global _push_req_move_yaw, _push_req_world_side_valid
    global _push_req_world_side_axis, _push_req_world_side_ref, _push_req_seq
    global _push_req_ff_valid, _push_req_ff_vx, _push_req_ff_vy
    global _push_reset_seq
    _push_req_edge = 0
    _push_req_lock_yaw = 0.0
    _push_req_cam_seen = 0
    _push_req_rel_x = 0.0
    _push_req_rel_y = 0.0
    _push_req_ref_valid = 0
    _push_req_ref_x = 0.0
    _push_req_ref_y = 0.0
    _push_req_move_yaw = 999.9
    _push_req_world_side_valid = 0
    _push_req_world_side_axis = 0
    _push_req_world_side_ref = 0.0
    _push_req_ff_valid = 0
    _push_req_ff_vx = 0.0
    _push_req_ff_vy = 0.0
    _push_req_seq = 0
    _push_reset_seq += 1

# 功能：取消面向目标的角速度控制。
# 函数通过 _face_req_seq 通知 _face_step 停止覆盖角速度。
def clear_face():
    global _face_req_active, _face_req_seq
    _face_req_active = 0
    _face_req_seq += 1

_beep = None

_encoder_1 = None

_encoder_2 = None

_encoder_3 = None

_encoder_4 = None

_motor_1 = None

_motor_2 = None

_motor_3 = None

_motor_4 = None

_imu = None

_imu_data = None

_lcd = None

_key = None

_uart1 = None

_uart_wireless = None

# 功能：直接设置车辆当前位姿，并同步编码器里程计的内部位置。
# 输入参数：x 为当前 X 坐标；y 为当前 Y 坐标；angle 为当前车头角度，单位度。
# 结果写入 _car、_YawAngle_Trans 和编码器位置缓存。
def set_position(x, y, angle):
    global _Position_X_encoder, _Position_Y_encoder, _YawAngle_Trans
    _car.Position_X = x
    _car.Position_Y = y
    _YawAngle_Trans = angle
    _car.current_angle = angle
    _Position_X_encoder = x
    _Position_Y_encoder = y

# 功能：初始化底层硬件外设，包括 ADC、蜂鸣器、按键、编码器、电机、IMU、LCD 和串口。
# 函数会创建并保存各个硬件对象，最后按主车/从车身份设置初始位置。
def hw_init():
    global _imu
    global _encoder_1, _encoder_2, _encoder_3, _encoder_4
    global _motor_1, _motor_2, _motor_3, _motor_4
    global _imu_data
    global _beep, _lcd, _key
    global _uart1, _uart_wireless
    global _wireless
    adc_group = ADC_Group(1)
    adc_group.addch('B12')
    adc_group.addch('B14')
    adc_group.addch('B15')
    adc_group.addch('B17')
    power_adc = ADC('B27')
    led = Pin('C4', Pin.OUT, value=True)
    _beep = Pin('B26', Pin.OUT, value=False)
    Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K).value()
    _key = KEY_HANDLER(10)
    _key.get()
    _encoder_1 = encoder('C0', 'C1', False, capture_div=6)
    _encoder_2 = encoder('D15', 'D16', True, capture_div=6)
    _encoder_3 = encoder('C2', 'C3', True, capture_div=6)
    _encoder_4 = encoder('D13', 'D14', True, capture_div=6)
    _motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty=0, invert=True)
    _motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=False)
    _motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty=0, invert=True)
    _motor_4 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=False)
    _imu = IMU660RX()
    _imu_data = [0, 0, 0, 0, 0, 0]
    if _DEBUG_LCD:
        from display import LCD_Drv, LCD
        cs = Pin('B29', Pin.OUT, value=True)
        cs.high()
        cs.low()
        rst = Pin('B31', Pin.OUT, value=True)
        dc = Pin('B5', Pin.OUT, value=True)
        blk = Pin('C21', Pin.OUT, value=True)
        _lcd = LCD(LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD114_TYPE))
        _lcd.color(65535, 0)
        _lcd.mode(0)
        _lcd.clear(0)
    Pin('C5', Pin.OUT, value=0)
    _uart_wireless = UART(2)
    _uart_wireless.init(230400, 8, None)
    _uart1 = UART(3)
    _uart1.init(230400, 8, None)
    sleep_ms(100)
    _beep.high()
    sleep_ms(100)
    _beep.low()
    sleep_ms(100)
    _beep.high()
    sleep_ms(100)
    _beep.low()
    if config._IS_LEADER:
        set_position(config._LEADER_INIT_POS[0], config._LEADER_INIT_POS[1], _INIT_YAW)
    else:
        set_position(config._FOLLOWER_INIT_POS[0], config._FOLLOWER_INIT_POS[1], _INIT_YAW)

# IMU 姿态解算参数。
# _IMU_KP/_IMU_KI 为加速度修正陀螺积分的 PI 系数，_IMU_INTEGRAL_LIMIT 限制积分量。
_IMU_KP = 0.08

_IMU_KI = 0.0008

_IMU_INTEGRAL_LIMIT = 0.5

# 判断静止时的角速度阈值，单位度每秒，用于慢速更新陀螺零偏。
_IMU_STATIC_THR_DEG = 0.5

# 加速度低通滤波系数，越大越相信当前采样，越小越平滑。
_IMU_ACC_LPF_ALPHA = 0.5

# IMU 安装位置相对车辆旋转中心的偏移，用于扣除旋转带来的附加加速度。
_IMU_OFFSET_X = const(0)

_IMU_OFFSET_Y = const(0)

# 重力加速度和陀螺原始值缩放系数。
_IMU_GRAVITY = 9.80665

_IMU_BIAS_LPF_ALPHA = 0.9

_IMU_GYRO_SCALE = 57.2957795

# 姿态解算周期，单位秒，需要与定时器实际调用周期匹配。
_DELTA_T = 0.01

# 加速度可信带（g）。|acc| 偏离 1g 超过它就说明含明显运动加速度，本帧不用加速度
# 修正姿态。无门限地信它会把 roll/pitch 拽偏，再经四元数运动学泄漏进 yaw；
# 推送时加速度是单向的，这个误差不会自己抵消，会单向累积。
# 取 0.03 而不是常见的 0.1：模长对【水平】加速度很不敏感（勾股，0.2g 才让模长
# 偏离 0.02），而推送时的加速度恰恰是水平的。0.03 对应约 0.25g 水平加速度，
# 既能挡住起步/急停，又不会因为静态安装误差和噪声频繁误关。
_IMU_ACC_TRUST_BAND = 0.03

# 姿态解算实际周期的合法范围（秒）。超出视为异常，退回标称 _DELTA_T。
_IMU_DT_MIN = 0.002

_IMU_DT_MAX = 0.05

# 上一次姿态解算的时间戳（us）。0 表示尚未开始。
_imu_last_us = 0

# 本次姿态解算的实际周期（秒），由 ticks_us 实测。
_imu_dt = 0.01

# rad/s 的零偏换回原始陀螺计数。
_IMU_GYRO_RAD_TO_LSB = _IMU_GYRO_SCALE / 0.07

# Roll/Pitch 小角度零点更新：每 4 秒检查一次，只有两轴绝对值都小于 1 度时，
# 才把当前姿态记为新的水平零点。只修正 Roll/Pitch 输出，不改变连续偏航角。
_IMU_TILT_ZERO_INTERVAL_MS = const(4000)

_IMU_TILT_ZERO_THRESH_DEG = 1.0

_imu_tilt_zero_last_ms = 0

_imu_roll_zero_deg = 0.0

_imu_pitch_zero_deg = 0.0

# 姿态互补滤波的误差积分项和 Z 轴零偏估计。
_I_ex = 0.0

_I_ey = 0.0

_I_ez = 0.0

_gyro_bias_z = 0.0

_prev_gyro_z = 0.0

# IMU 安装补偿角度及其三角函数缓存，校准后用于把传感器坐标旋到车体坐标。
_mount_roll_deg = -1.043

_mount_pitch_deg = 0.32

_mount_cr = 1.0

_mount_sr = 0.0

_mount_cp = 1.0

_mount_sp = 0.0

class GyroData:

    # 功能：保存三轴陀螺仪原始或偏置数据。
    # 输入参数：self 为对象自身；x/y/z 分别为三轴初值。
    # 返回值：无。初始化结果保存在 GyroData 实例属性中。
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class IMUData:

    # 功能：保存一次 IMU 解算所需的加速度和陀螺仪数据。
    # 输入参数：self 为对象自身；该初始化函数没有外部输入参数。
    # 返回值：无。初始化结果保存在 IMUData 实例属性中。
    def __init__(self):
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self._gyro_z = 0.0

class Quaternion:

    # 功能：保存姿态解算使用的四元数。
    # 输入参数：self 为对象自身；q0/q1/q2/q3 为四元数初值。
    # 返回值：无。初始化结果保存在 Quaternion 实例属性中。
    def __init__(self, q0=1.0, q1=0.0, q2=0.0, q3=0.0):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

class AngleData:

    # 功能：保存由四元数换算出的欧拉角。
    # 输入参数：self 为对象自身；该初始化函数没有外部输入参数。
    # 返回值：无。初始化结果保存在 AngleData 实例属性中。
    def __init__(self):
        self.RollAngle = 0.0
        self.PitchAngle = 0.0
        self.YawAngle = 0.0

_GyroOffset = GyroData()

_ZeroPoint = IMUData()

_IMU_Data = IMUData()

_IMU1_Data = IMUData()

_Q_info = Quaternion()

_GyroAngle = AngleData()

# 功能：设置 IMU 安装姿态补偿角，并预先计算补偿矩阵需要的三角函数。
# 输入参数：roll_deg 为安装横滚补偿角，单位度；pitch_deg 为安装俯仰补偿角，单位度。
# 返回值：无。结果写入安装角和三角函数缓存。
def _set_mount_compensation(roll_deg, pitch_deg):
    global _mount_roll_deg, _mount_pitch_deg
    global _mount_cr, _mount_sr, _mount_cp, _mount_sp
    _mount_roll_deg = roll_deg
    _mount_pitch_deg = pitch_deg
    roll_rad = roll_deg * pi / 180.0
    pitch_rad = pitch_deg * pi / 180.0
    _mount_cr = cos(roll_rad)
    _mount_sr = sin(roll_rad)
    _mount_cp = cos(-pitch_rad)
    _mount_sp = sin(-pitch_rad)

_set_mount_compensation(_mount_roll_deg, _mount_pitch_deg)

# 功能：车辆静止时采样 IMU，计算陀螺仪零偏，并根据平均重力方向修正安装补偿角。
# 输入参数：n_samples 为采样次数，默认 300 次。
# 返回值：无。结果写入 _GyroOffset、安装补偿和姿态积分状态。
def calibrate_gyro(n_samples=300):
    global _gyro_bias_z, _I_ex, _I_ey, _I_ez, _prev_gyro_z
    global _imu_last_us, _imu_dt
    global _imu_tilt_zero_last_ms, _imu_roll_zero_deg, _imu_pitch_zero_deg
    sum_x, sum_y, sum_z = (0, 0, 0)
    sum_acc_x, sum_acc_y, sum_acc_z = (0, 0, 0)
    for unused in range(n_samples):
        _imu.capture()
        data = _imu.get()
        sum_acc_x += data[0]
        sum_acc_y += data[1]
        sum_acc_z += data[2]
        sum_x += data[3]
        sum_y += data[4]
        sum_z += data[5]
        sleep_ms(5)
    _GyroOffset.x = sum_x / n_samples
    _GyroOffset.y = sum_y / n_samples
    _GyroOffset.z = sum_z / n_samples
    acc_x = sum_acc_x / n_samples * 0.244 / 1000
    acc_y = sum_acc_y / n_samples * 0.244 / 1000
    acc_z = sum_acc_z / n_samples * 0.244 / 1000
    acc_h = sqrt(acc_y * acc_y + acc_z * acc_z)
    if acc_h > 1e-06:
        _set_mount_compensation(atan2(acc_y, acc_z) * 180.0 / pi, atan2(acc_x, acc_h) * 180.0 / pi)
    _gyro_bias_z = 0.0
    _prev_gyro_z = 0.0
    _imu_last_us = 0
    _imu_dt = _DELTA_T
    _I_ex = 0.0
    _I_ey = 0.0
    _I_ez = 0.0
    _imu_tilt_zero_last_ms = 0
    _imu_roll_zero_deg = 0.0
    _imu_pitch_zero_deg = 0.0
    _Q_info.q0 = 1.0
    _Q_info.q1 = 0.0
    _Q_info.q2 = 0.0
    _Q_info.q3 = 0.0

# 功能：判断 IMU 当前角速度是否足够接近静止。
# 输入参数：gx/gy/gz 为三轴角速度，单位为弧度每秒。
# 返回值：布尔值。True 表示角速度平方和低于静止阈值，可用于零偏缓慢更新。
def imu_is_nearly_static(gx, gy, gz):
    thr = _IMU_STATIC_THR_DEG * pi / 180.0
    return gx * gx + gy * gy + gz * gz < 3.0 * thr * thr

# 功能：把 IMU 坐标系中的三轴量转换到补偿后的车体坐标系。
# 输入参数：x/y/z 为需要补偿的三轴数据，可以是加速度或角速度。
# 返回值：三元组，依次为补偿后的 x、y、z。
def _apply_mount_compensation(x, y, z):
    z1 = _mount_sr * y + _mount_cr * z
    return (_mount_cp * x + _mount_sp * z1, _mount_cr * y - _mount_sr * z, -_mount_sp * x + _mount_cp * z1)

# 功能：扣除 IMU 安装偏移在旋转时造成的平面加速度误差。
# 输入参数：acc_x/acc_y 为补偿前平面加速度，单位 g；gyro_z 为 Z 轴角速度，单位弧度每秒。
# 返回值：二元组，依次为修正后的 acc_x、acc_y。
def _remove_offset_accel(acc_x, acc_y, gyro_z):
    global _prev_gyro_z
    gyro_accel_z = (gyro_z - _prev_gyro_z) / _DELTA_T
    _prev_gyro_z = gyro_z
    return (acc_x - (-gyro_z * gyro_z * _IMU_OFFSET_X - gyro_accel_z * _IMU_OFFSET_Y) / _IMU_GRAVITY, acc_y - (gyro_accel_z * _IMU_OFFSET_X - gyro_z * gyro_z * _IMU_OFFSET_Y) / _IMU_GRAVITY)

# 功能：读取并换算 IMU 原始数据，完成安装补偿、低通滤波、零偏更新和角速度输出。
# 输入参数：无。函数使用全局 _imu_data 中最近一次 IMU 原始采样。
def IMU_Get_Values():
    global _gyro_z
    alpha = _IMU_ACC_LPF_ALPHA
    global _gyro_bias_z, _prev_gyro_z
    acc_x = float(_imu_data[0]) * 0.244 / 1000
    acc_y = float(_imu_data[1]) * 0.244 / 1000
    acc_z = float(_imu_data[2]) * 0.244 / 1000
    acc_x, acc_y, acc_z = _apply_mount_compensation(acc_x, acc_y, acc_z)
    raw_gyro_x = (float(_imu_data[3]) - _GyroOffset.x) / _IMU_GYRO_SCALE * 70 / 1000
    raw_gyro_y = (float(_imu_data[4]) - _GyroOffset.y) / _IMU_GYRO_SCALE * 70 / 1000
    raw_gyro_z = (float(_imu_data[5]) - _GyroOffset.z) / _IMU_GYRO_SCALE * 70 / 1000
    raw_gyro_x, raw_gyro_y, raw_gyro_z = _apply_mount_compensation(raw_gyro_x, raw_gyro_y, raw_gyro_z)
    acc_x, acc_y = _remove_offset_accel(acc_x, acc_y, raw_gyro_z)
    _IMU_Data.acc_x = acc_x * alpha + _IMU1_Data.acc_x * (1 - alpha)
    _IMU_Data.acc_y = acc_y * alpha + _IMU1_Data.acc_y * (1 - alpha)
    _IMU_Data.acc_z = acc_z * alpha + _IMU1_Data.acc_z * (1 - alpha)
    _IMU1_Data.acc_x = _IMU_Data.acc_x
    _IMU1_Data.acc_y = _IMU_Data.acc_y
    _IMU1_Data.acc_z = _IMU_Data.acc_z
    _gyro_z = _imu_data[5]
    # 减掉零偏估计，与 _IMU_Data._gyro_z（姿态解算用）保持同一物理量。
    # _gyro_bias_z 是 rad/s，这里是原始计数，需按刻度换算回去。
    _IMU1_Data._gyro_z = float(_imu_data[5]) - _GyroOffset.z - _gyro_bias_z * _IMU_GYRO_RAD_TO_LSB
    # 传入已扣除当前零偏估计的 gz：静止时它应接近 0，与零偏大小无关，判据不会自锁。
    if imu_is_nearly_static(raw_gyro_x, raw_gyro_y, raw_gyro_z - _gyro_bias_z):
        _gyro_bias_z = _IMU_BIAS_LPF_ALPHA * _gyro_bias_z + (1.0 - _IMU_BIAS_LPF_ALPHA) * (raw_gyro_z - _ZeroPoint._gyro_z)
    _IMU_Data.gyro_x = raw_gyro_x - _ZeroPoint.gyro_x
    _IMU_Data.gyro_y = raw_gyro_y - _ZeroPoint.gyro_y
    _IMU_Data._gyro_z = raw_gyro_z - _ZeroPoint._gyro_z - _gyro_bias_z

# 功能：用加速度修正陀螺积分，并更新四元数姿态。
# 输入参数：IMU 为 IMUData 对象，包含当前加速度和角速度。
# 返回值：无。结果写入全局四元数 _Q_info；加速度模长异常时直接返回。
def IMU_AHRS_Update(IMU):
    global _I_ex, _I_ey, _I_ez
    global _imu_last_us, _imu_dt
    # 实测积分周期。_DELTA_T 是写死的 0.01，而 ticker 实际周期有偏差、ISR 偶尔超时，
    # dt 的比例误差与陀螺刻度误差完全等效（1% -> 每转 360° 差 3.6°），且单向累积。
    # 必须用 ticks_us：ticks_ms 在 10ms 周期上是 10% 量化，比要修的误差还大。
    now_us = ticks_us()
    if _imu_last_us == 0:
        dt = _DELTA_T
    else:
        dt = ticks_diff(now_us, _imu_last_us) * 1e-06
        if dt < _IMU_DT_MIN or dt > _IMU_DT_MAX:
            dt = _DELTA_T
    _imu_last_us = now_us
    _imu_dt = dt
    half_t = dt * 0.5
    q0 = _Q_info.q0
    q1 = _Q_info.q1
    q2 = _Q_info.q2
    q3 = _Q_info.q3
    q0q0 = q0 * q0
    q0q1 = q0 * q1
    q0q2 = q0 * q2
    q1q1 = q1 * q1
    q1q3 = q1 * q3
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q3q3 = q3 * q3
    acc_sq = IMU.acc_x * IMU.acc_x + IMU.acc_y * IMU.acc_y + IMU.acc_z * IMU.acc_z
    if acc_sq <= 1e-06:
        return
    # 1g 门限：加速度模长明显偏离重力时，本帧含运动加速度，不能拿它当重力参考。
    # KP、KI 一起清零——只关 KP 而留 KI，误差照样会攒进积分项。
    a_norm = sqrt(acc_sq)
    if a_norm > 1.0 + _IMU_ACC_TRUST_BAND or a_norm < 1.0 - _IMU_ACC_TRUST_BAND:
        acc_kp = 0.0
        acc_gate = 0.0
    else:
        acc_kp = _IMU_KP
        acc_gate = 1.0
    norm = 1.0 / a_norm
    IMU.acc_x *= norm
    IMU.acc_y *= norm
    IMU.acc_z *= norm
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3
    ex = IMU.acc_y * vz - IMU.acc_z * vy
    ey = IMU.acc_z * vx - IMU.acc_x * vz
    # acc_gate=0 时本帧不累加（冻结积分），=1 时正常累加。
    # 注意 _I_* 保持"未乘 Ki"的原语义，否则 _IMU_INTEGRAL_LIMIT=0.5 会变成实质无限幅。
    _I_ex += dt * ex * acc_gate
    _I_ey += dt * ey * acc_gate
    _I_ez += dt * (IMU.acc_x * vy - IMU.acc_y * vx) * acc_gate
    if _I_ex > _IMU_INTEGRAL_LIMIT:
        _I_ex = _IMU_INTEGRAL_LIMIT
    elif _I_ex < -_IMU_INTEGRAL_LIMIT:
        _I_ex = -_IMU_INTEGRAL_LIMIT
    if _I_ey > _IMU_INTEGRAL_LIMIT:
        _I_ey = _IMU_INTEGRAL_LIMIT
    elif _I_ey < -_IMU_INTEGRAL_LIMIT:
        _I_ey = -_IMU_INTEGRAL_LIMIT
    if _I_ez > _IMU_INTEGRAL_LIMIT:
        _I_ez = _IMU_INTEGRAL_LIMIT
    elif _I_ez < -_IMU_INTEGRAL_LIMIT:
        _I_ez = -_IMU_INTEGRAL_LIMIT
    IMU.gyro_x = IMU.gyro_x + acc_kp * ex + _IMU_KI * _I_ex
    IMU.gyro_y = IMU.gyro_y + acc_kp * ey + _IMU_KI * _I_ey
    old_q0 = q0
    old_q1 = q1
    old_q2 = q2
    old_q3 = q3
    q0 = old_q0 + (-old_q1 * IMU.gyro_x - old_q2 * IMU.gyro_y - old_q3 * IMU._gyro_z) * half_t
    q1 = old_q1 + (old_q0 * IMU.gyro_x + old_q2 * IMU._gyro_z - old_q3 * IMU.gyro_y) * half_t
    q2 = old_q2 + (old_q0 * IMU.gyro_y - old_q1 * IMU._gyro_z + old_q3 * IMU.gyro_x) * half_t
    q3 = old_q3 + (old_q0 * IMU._gyro_z + old_q1 * IMU.gyro_y - old_q2 * IMU.gyro_x) * half_t
    norm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    _Q_info.q0 = q0 * norm
    _Q_info.q1 = q1 * norm
    _Q_info.q2 = q2 * norm
    _Q_info.q3 = q3 * norm

# 功能：采样 IMU 并更新车辆的 Roll、Pitch、Yaw 姿态角。
# 结果写入 _GyroAngle 和连续化后的 _YawAngle_Trans。
def Gyro_Get_All_Angles():
    global _YawAngle_Trans, _Yaw_Angle_Old, _imu_data
    global _imu_tilt_zero_last_ms, _imu_roll_zero_deg, _imu_pitch_zero_deg
    _imu.capture()
    _imu_data = _imu.get()
    IMU_Get_Values()
    IMU_AHRS_Update(_IMU_Data)
    q2 = _Q_info.q2
    q3 = _Q_info.q3
    q0 = _Q_info.q0
    q1 = _Q_info.q1
    raw_roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180 / pi
    p = 2 * (q0 * q2 - q3 * q1)
    if p > 1.0:
        p = 1.0
    elif p < -1.0:
        p = -1.0
    raw_pitch = atan2(p, sqrt(1 - p * p)) * 180 / pi
    roll = raw_roll - _imu_roll_zero_deg
    pitch = raw_pitch - _imu_pitch_zero_deg
    now = ticks_ms()
    if _imu_tilt_zero_last_ms == 0:
        _imu_tilt_zero_last_ms = now
    elif ticks_diff(now, _imu_tilt_zero_last_ms) >= _IMU_TILT_ZERO_INTERVAL_MS:
        _imu_tilt_zero_last_ms = now
        if abs(roll) < _IMU_TILT_ZERO_THRESH_DEG and abs(pitch) < _IMU_TILT_ZERO_THRESH_DEG:
            _imu_roll_zero_deg = raw_roll
            _imu_pitch_zero_deg = raw_pitch
            roll = 0.0
            pitch = 0.0
    _GyroAngle.RollAngle = roll
    _GyroAngle.PitchAngle = pitch
    _GyroAngle.YawAngle = -atan2(2 * (_Q_info.q1 * q2 + _Q_info.q0 * q3), -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / pi
    delta_yaw = _GyroAngle.YawAngle - _Yaw_Angle_Old
    if delta_yaw > 180:
        delta_yaw -= 360
    elif delta_yaw < -180:
        delta_yaw += 360
    _YawAngle_Trans = (_YawAngle_Trans + delta_yaw) % 360.0
    _Yaw_Angle_Old = _GyroAngle.YawAngle

# 主车和从车的 PID 参数。
if config._IS_LEADER:
    _MOTOR_SPEED_KP = (2, 2, 2.2, 2.2)
    _MOTOR_SPEED_KI = (0.3, 0.3, 0.3, 0.3)
    _MOTOR_SPEED_KD = (0, 0, 0, 0)
    _GYRO_Z_KP = 0.05
    _GYRO_Z_KI = 0.0
    _GYRO_Z_KD = 0.01
    _ANGLE_KP = 110.0
    _ANGLE_KI = 0.0
    _ANGLE_KD = 20.0
    _POS_X_KD = 0.4
    _POS_Y_KD = 0.4
    _MOTOR_DEADBAND = ((0, 0), (0, 0), (0, 0), (0, 0))
else:
    _MOTOR_SPEED_KP = (2, 2, 2.2, 2.1)
    _MOTOR_SPEED_KI = (0.3, 0.32, 0.3, 0.3)
    _MOTOR_SPEED_KD = (0, 0, 0, 0)
    _GYRO_Z_KP = 0.05
    _GYRO_Z_KI = 0.0
    _GYRO_Z_KD = 0.01
    _ANGLE_KP = 110.0
    _ANGLE_KI = 0.0
    _ANGLE_KD = 20.0
    _POS_X_KD = 0.4
    _POS_Y_KD = 0.4
    _MOTOR_DEADBAND = ((0, 0), (0, 0), (0, 0), (0, 0))

_POS_X_KP = const(2)

_POS_Y_KP = const(2)

_MOVE_Z_AB = const(1)

# 速度环输出总限幅（主从车共用）。同时管两处：占空比累加值 _motor_dutyN_sp 的钳位，
# 和 duty_to_pwm 最终写给电机的 PWM 上限。10000 是 MOTOR_CONTROLLER.duty 的满量程。
_MOTOR_PWM_DUTY_SPEED_PID_MAX = const(10000)

# 速度环单项限幅（主从车共用）：增量式 PID 单次计算出的 Δduty 的钳位。
# 限制的是每 2ms 一拍能往 _motor_dutyN_sp 上叠多少，越大阶跃越猛、越容易冲过头。
_MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX = const(3000)

_MOTOR_PWM_DUTY_MAX_SPEED = const(3000)

_MOTOR_PWM_DUTY_GYRO_Z_PID_MAX = const(250)

_MOTOR_PWM_DUTY_ANGLE_PID_MAX = const(30000)

_MOTOR_PWM_DUTY_POSITION_PID_MAX = const(200)

_MOTOR_PWM_DUTY_MAX_GYRO_Z = const(150)

_MOTOR_PWM_DUTY_MAX_ANGLE = const(10000)

_motor_duty1 = 0

_motor_duty2 = 0

_motor_duty3 = 0

_motor_duty4 = 0

_motor_duty1_sp = 0.0

_motor_duty2_sp = 0.0

_motor_duty3_sp = 0.0

_motor_duty4_sp = 0.0

class PidParam:

    # 功能：创建 PID 参数和运行时状态，保存比例/积分/微分系数、输出限制和历史误差。
    # 输入参数：self 为对象自身；kp/ki/kd 为 PID 系数；kp2/gkd/low_pass 为兼容旧接口的预留参数；p_max/i_max/d_max 为各项输出限制。
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, kp2=0.0, gkd=0.0, p_max=0.0, i_max=0.0, d_max=0.0, low_pass=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.p_max = p_max
        self.d_max = d_max
        self.out_p = 0.0
        self.out_i = 0.0
        self.out_d = 0.0
        self.pre_error = 0.0
        self.pre_pre_error = 0.0

class MotorParam:

    # 功能：创建单个电机或控制环的速度状态和 PID 控制器。
    # 输入参数：self 为对象自身；kp/ki/kd 为 PID 系数；kp2/gkd/low_pass 为预留参数；p_max/i_max/d_max 为 PID 输出限制。
    # 返回值：无。初始化结果保存在 MotorParam 实例属性中。
    def __init__(self, kp=0, ki=0, kd=0, kp2=0, gkd=0, low_pass=0, p_max=0, i_max=0, d_max=0):
        self.encoder_speed = 0.0
        self.target_speed = 0.0
        self.pid = PidParam(kp, ki, kd, kp2, gkd, p_max, i_max, d_max, low_pass)

_motor_1_speed = MotorParam(_MOTOR_SPEED_KP[0], _MOTOR_SPEED_KI[0], _MOTOR_SPEED_KD[0], 0, 0, 1, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED)

_motor_2_speed = MotorParam(_MOTOR_SPEED_KP[1], _MOTOR_SPEED_KI[1], _MOTOR_SPEED_KD[1], 0, 0, 1, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED)

_motor_3_speed = MotorParam(_MOTOR_SPEED_KP[2], _MOTOR_SPEED_KI[2], _MOTOR_SPEED_KD[2], 0, 0, 1, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED)

_motor_4_speed = MotorParam(_MOTOR_SPEED_KP[3], _MOTOR_SPEED_KI[3], _MOTOR_SPEED_KD[3], 0, 0, 1, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED, _MOTOR_PWM_DUTY_MAX_SPEED)

_motor_gyro_z = MotorParam(_GYRO_Z_KP, _GYRO_Z_KI, _GYRO_Z_KD, 0, 0, 1, _MOTOR_PWM_DUTY_MAX_GYRO_Z, _MOTOR_PWM_DUTY_MAX_GYRO_Z, _MOTOR_PWM_DUTY_MAX_GYRO_Z)

_motor_angle_z = MotorParam(_ANGLE_KP, _ANGLE_KI, _ANGLE_KD, 0, 0, 1, _MOTOR_PWM_DUTY_MAX_ANGLE, _MOTOR_PWM_DUTY_MAX_ANGLE, _MOTOR_PWM_DUTY_MAX_ANGLE)

_position_x = MotorParam(_POS_X_KP, 0, _POS_X_KD, 0, 0, 1, _MOTOR_PWM_DUTY_POSITION_PID_MAX, _MOTOR_PWM_DUTY_POSITION_PID_MAX, _MOTOR_PWM_DUTY_POSITION_PID_MAX)

_position_y = MotorParam(_POS_Y_KP, 0, _POS_Y_KD, 0, 0, 1, _MOTOR_PWM_DUTY_POSITION_PID_MAX, _MOTOR_PWM_DUTY_POSITION_PID_MAX, _MOTOR_PWM_DUTY_POSITION_PID_MAX)

# 功能：把当前全局 PID 参数重新写入各个 PID 对象。
# 返回值：无。用于修改参数后让电机速度环、角速度环、角度环和位置环立即生效。
def apply_pid_params():
    for i, motor in enumerate((_motor_1_speed, _motor_2_speed, _motor_3_speed, _motor_4_speed)):
        motor.pid.kp = _MOTOR_SPEED_KP[i]
        motor.pid.ki = _MOTOR_SPEED_KI[i]
        motor.pid.kd = _MOTOR_SPEED_KD[i]
    _motor_gyro_z.pid.kp = _GYRO_Z_KP
    _motor_gyro_z.pid.ki = _GYRO_Z_KI
    _motor_gyro_z.pid.kd = _GYRO_Z_KD
    _motor_angle_z.pid.kp = _ANGLE_KP
    _motor_angle_z.pid.ki = _ANGLE_KI
    _motor_angle_z.pid.kd = _ANGLE_KD
    _position_x.pid.kp = _POS_X_KP
    _position_x.pid.kd = _POS_X_KD
    _position_y.pid.kp = _POS_Y_KP
    _position_y.pid.kd = _POS_Y_KD

# 功能：把当前连续偏航角一次性校准到给定的世界航向。
# 该接口只重置 yaw 的世界坐标基准，不修改陀螺零偏、刻度因子或四元数；
# 后续 IMU 周期会继续在新基准上累加真实转角。
# 输入参数：angle 为校准后的世界航向，单位度。
def correct_yaw(angle):
    global _YawAngle_Trans, _YawAngle_imu, _angle_update_flag
    global _yaw_rate_active, _yaw_rate_target_raw
    angle = angle % 360.0
    _yaw_rate_active = False
    _yaw_rate_target_raw = 0.0
    _YawAngle_Trans = angle
    _YawAngle_imu = angle
    _car.current_angle = angle
    _car.target_angle = angle
    _car.target_GYRO_Z = 0.0
    _angle_update_flag = 0
    pid = _motor_angle_z.pid
    pid.out_p = 0.0
    pid.out_i = 0.0
    pid.out_d = 0.0
    pid.pre_error = 0.0
    pid.pre_pre_error = 0.0

# 功能：把数值限制在指定范围内。
# 输入参数：val 为待限制的值；min_val 为下限；max_val 为上限。
# 返回值：限制后的数值
@micropython.native
def minmax(val, min_val, max_val):
    if val >= max_val:
        return max_val
    if val <= min_val:
        return min_val
    return val

# 功能：把世界坐标系目标速度转换为车体坐标系目标速度。
@micropython.native
def world_vel_step():
    yaw = radians(_car.current_angle)
    c = cos(yaw)
    s = sin(yaw)
    vx_w = _target_world_vx
    vy_w = _target_world_vy
    _car.target_Speed_X = c * vx_w - s * vy_w
    _car.target_Speed_Y = s * vx_w + c * vy_w

# 功能：位置式 PID 计算，适合角度环、位置环这类直接根据当前误差输出控制量的场景。
# 输入参数：pid 为 PidParam 对象；error 为当前误差。
# 返回值：PID 输出值
@micropython.native
def place_pid_solve(pid, error):
    pid.out_d = minmax(pid.kd * (error - pid.pre_error), -pid.d_max, pid.d_max)
    pid.out_p = minmax(pid.kp * error, -pid.p_max, pid.p_max)
    pid.out_i += pid.ki * error
    pid.pre_error = error
    pid.out_i = minmax(pid.out_i, -1000, 1000)
    return pid.out_p + pid.out_i + pid.out_d

# 功能：增量式 PID 计算，适合电机速度闭环，通过每次输出增量累加到 PWM 目标上。
# 输入参数：pid 为 PidParam 对象；error 为当前速度误差。
# 返回值：本周期需要叠加的控制增量。
@micropython.native
def increment_pid_solve(pid, error):
    pid.out_d = pid.kd * (error - 2 * pid.pre_error + pid.pre_pre_error)
    pid.out_p = pid.kp * (error - pid.pre_error)
    pid.out_i = pid.ki * error
    pid.pre_pre_error = pid.pre_error
    pid.pre_error = error
    return pid.out_p + pid.out_i + pid.out_d

# 功能：根据目标位置和当前位置计算世界坐标系下的 X/Y 速度命令。
# 输入参数：target_x/target_y 为目标坐标；current_x/current_y 为当前坐标。
# 返回值：无。结果写入 _car.target_Speed_X 和 _car.target_Speed_Y，随后会在 position_pid_step 中再转成车体系。
@micropython.native
def position_pid(target_x, target_y, current_x, current_y):
    error_y = target_y - current_y
    out_x = place_pid_solve(_position_x.pid, target_x - current_x)
    out_y = place_pid_solve(_position_y.pid, error_y)
    _car.target_Speed_X = minmax(out_x, -_MOTOR_PWM_DUTY_POSITION_PID_MAX, _MOTOR_PWM_DUTY_POSITION_PID_MAX)
    _car.target_Speed_Y = minmax(out_y, -_MOTOR_PWM_DUTY_POSITION_PID_MAX, _MOTOR_PWM_DUTY_POSITION_PID_MAX)

# 功能：执行一次位置控制，把目标点误差转换为车体系速度，并限制最大平移速度。
@micropython.native
def position_pid_step():
    position_pid(_target_pos_x, _target_pos_y, _car.Position_X, _car.Position_Y)
    vx_w = _car.target_Speed_X
    vy_w = _car.target_Speed_Y
    spd = sqrt(vx_w * vx_w + vy_w * vy_w)
    if spd > _pos_speed_max and spd > 0.0:
        scale = _pos_speed_max / spd
        vx_w *= scale
        vy_w *= scale
    yaw = radians(_car.current_angle)
    c = cos(yaw)
    s = sin(yaw)
    _car.target_Speed_X = c * vx_w - s * vy_w
    _car.target_Speed_Y = s * vx_w + c * vy_w

# 功能：角度闭环 PID，把目标车头角与当前车头角的误差转换为目标角速度。
# 输入参数：target_angle 为目标角度；current_angle 为当前角度，单位均为度。
# 返回值：角度环输出，已按 _MOTOR_PWM_DUTY_ANGLE_PID_MAX 限幅。
@micropython.native
def angle_pid(target_angle, current_angle):
    return minmax(place_pid_solve(_motor_angle_z.pid, (current_angle - target_angle + 180) % 360 - 180), -_MOTOR_PWM_DUTY_ANGLE_PID_MAX, _MOTOR_PWM_DUTY_ANGLE_PID_MAX)

# 功能：角速度闭环 PID，把目标 Z 轴角速度和当前角速度误差转换为旋转控制量。
# 输入参数：target_gyro_z 为目标角速度；current_gyro_z 为当前角速度。
# 返回值：角速度环输出，已按 _MOTOR_PWM_DUTY_GYRO_Z_PID_MAX 限幅。
@micropython.native
def gyro_z_pid(target_gyro_z, current_gyro_z):
    return minmax(place_pid_solve(_motor_gyro_z.pid, target_gyro_z - current_gyro_z), -_MOTOR_PWM_DUTY_GYRO_Z_PID_MAX, _MOTOR_PWM_DUTY_GYRO_Z_PID_MAX)

# 功能：四轮全向底盘运动学分解，把车体系 X/Y 平移和 Z 旋转分配到四个轮子目标速度。
# 输入参数：x 为车体系横向速度；y 为车体系纵向速度；z 为旋转速度命令。
@micropython.native
def car_omni(x, y, z):
    _motor_1_speed.target_speed = y - x + _MOVE_Z_AB * z
    _motor_2_speed.target_speed = y + x - _MOVE_Z_AB * z
    _motor_3_speed.target_speed = y + x + _MOVE_Z_AB * z
    _motor_4_speed.target_speed = y - x - _MOVE_Z_AB * z

# 功能：把带符号的占空比目标转换为电机 PWM，并补偿正反转死区。
# 输入参数：duty_sp 为期望占空比；db_pos/db_neg 为正反转死区补偿；pwm_max 为最大 PWM 幅值。
# 返回值：带方向符号的 PWM 输出值，0 表示停止。
@micropython.native
def duty_to_pwm(duty_sp, db_pos, db_neg, pwm_max):
    if duty_sp == 0:
        return 0
    if duty_sp > 0:
        out = int(duty_sp) + db_pos
        return pwm_max if out > pwm_max else out
    out = int(-duty_sp) + db_neg
    return -pwm_max if out > pwm_max else -out

# 功能：执行四个电机的速度 PID，计算最终 PWM 占空比。
@micropython.native
def speed_pid():
    global _motor_duty1, _motor_duty2, _motor_duty3, _motor_duty4
    global _motor_duty1_sp, _motor_duty2_sp, _motor_duty3_sp, _motor_duty4_sp
    err2 = _motor_2_speed.target_speed - _motor_2_speed.encoder_speed
    err3 = _motor_3_speed.target_speed - _motor_3_speed.encoder_speed
    err4 = _motor_4_speed.target_speed - _motor_4_speed.encoder_speed
    _motor_duty1_sp += minmax(increment_pid_solve(_motor_1_speed.pid, _motor_1_speed.target_speed - _motor_1_speed.encoder_speed), -_MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX, _MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX)
    _motor_duty1_sp = minmax(_motor_duty1_sp, -_MOTOR_PWM_DUTY_SPEED_PID_MAX, _MOTOR_PWM_DUTY_SPEED_PID_MAX)
    _motor_duty1 = int(duty_to_pwm(_motor_duty1_sp, _MOTOR_DEADBAND[0][0], _MOTOR_DEADBAND[0][1], _MOTOR_PWM_DUTY_SPEED_PID_MAX))
    _motor_duty2_sp += minmax(increment_pid_solve(_motor_2_speed.pid, err2), -_MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX, _MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX)
    _motor_duty2_sp = minmax(_motor_duty2_sp, -_MOTOR_PWM_DUTY_SPEED_PID_MAX, _MOTOR_PWM_DUTY_SPEED_PID_MAX)
    _motor_duty2 = int(duty_to_pwm(_motor_duty2_sp, _MOTOR_DEADBAND[1][0], _MOTOR_DEADBAND[1][1], _MOTOR_PWM_DUTY_SPEED_PID_MAX))
    _motor_duty3_sp += minmax(increment_pid_solve(_motor_3_speed.pid, err3), -_MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX, _MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX)
    _motor_duty3_sp = minmax(_motor_duty3_sp, -_MOTOR_PWM_DUTY_SPEED_PID_MAX, _MOTOR_PWM_DUTY_SPEED_PID_MAX)
    _motor_duty3 = int(duty_to_pwm(_motor_duty3_sp, _MOTOR_DEADBAND[2][0], _MOTOR_DEADBAND[2][1], _MOTOR_PWM_DUTY_SPEED_PID_MAX))
    _motor_duty4_sp += minmax(increment_pid_solve(_motor_4_speed.pid, err4), -_MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX, _MOTOR_PWM_DUTY_SPEED_PID_INTEGRATE_MAX)
    _motor_duty4_sp = minmax(_motor_duty4_sp, -_MOTOR_PWM_DUTY_SPEED_PID_MAX, _MOTOR_PWM_DUTY_SPEED_PID_MAX)
    _motor_duty4 = int(duty_to_pwm(_motor_duty4_sp, _MOTOR_DEADBAND[3][0], _MOTOR_DEADBAND[3][1], _MOTOR_PWM_DUTY_SPEED_PID_MAX))

# 功能：清空电机速度环的积分累计和历史误差，避免停车或模式切换后残余输出。
def _apply_speed_reset():
    global _speed_reset_req
    global _motor_duty1_sp, _motor_duty2_sp, _motor_duty3_sp, _motor_duty4_sp
    _motor_duty1_sp = 0.0
    _motor_duty2_sp = 0.0
    _motor_duty3_sp = 0.0
    _motor_duty4_sp = 0.0
    for motor in (_motor_1_speed, _motor_2_speed, _motor_3_speed, _motor_4_speed):
        motor.pid.pre_error = 0.0
        motor.pid.pre_pre_error = 0.0
        motor.pid.out_i = 0.0
    _speed_reset_req = False

# 功能：根据 run 参数决定是否执行电机速度闭环，并把 PWM 输出写到四个电机控制器。
# 输入参数：run 为运行开关，非零表示闭环运行，0 表示四个电机输出清零。
def motor_control(run):
    global _motor_duty1, _motor_duty2, _motor_duty3, _motor_duty4
    if _speed_reset_req:
        _apply_speed_reset()
    if run:
        speed_pid()
    else:
        _motor_duty1 = 0
        _motor_duty2 = 0
        _motor_duty3 = 0
        _motor_duty4 = 0
    _motor_1.duty(_motor_duty1)
    _motor_2.duty(_motor_duty2)
    _motor_3.duty(_motor_duty3)
    _motor_4.duty(_motor_duty4)

apply_pid_params()

# 面向目标控制参数：KP/KD 根据视觉角度误差生成目标角速度，gyro_max 限制最大旋转命令。
_p_face_kp = 0.0

_p_face_kd = 0.0

_p_face_gyro_max = 0.0

# 推送目标基础参数：speed 为基础前进速度，inward_bias 为向场内侧偏转角，cam_lost_tol_ms 为视觉丢失容忍时间。
_p_push_speed = 0.0

_p_push_inward_bias = 0.0

_p_cam_lost_tol_ms = 0

# 推送目标视觉闭环参数：side/fwd 分别控制横向和前进方向，kp/kd 为 PD 系数。
_p_push_side_kp = 0.0

_p_push_side_kd = 0.0

_p_push_fwd_kp = 0.0

_p_push_fwd_kd = 0.0

# 推送目标输出限制与平滑参数：max 限制最大速度，slew 限制每周期变化量，deadband/min 抑制小误差抖动并克服静摩擦。
_p_push_side_max = 0.0

_p_push_fwd_max = 0.0

_p_push_side_slew = 0.0

_p_push_fwd_slew = 0.0

_p_push_d_lpf = 0.0

_p_push_side_deadband = 0.0

_p_push_fwd_deadband = 0.0

_p_push_side_min = 0.0

_p_push_fwd_min = 0.0

# 推送目标世界坐标横向约束参数，用来沿指定 X 或 Y 轴贴边/保持位置。

# 功能：配置推送控制器的所有速度、PD、限幅、斜率和丢失容忍参数。
def configure_push(speed, inward_bias, cam_lost_tol_ms, side_kp, side_kd, fwd_kp, fwd_kd, side_max, fwd_max, side_slew, fwd_slew, d_lpf, side_deadband, fwd_deadband, side_min, fwd_min):
    global _p_push_speed, _p_push_inward_bias, _p_cam_lost_tol_ms
    global _p_push_side_kp, _p_push_side_kd, _p_push_fwd_kp, _p_push_fwd_kd
    global _p_push_side_max, _p_push_fwd_max, _p_push_side_slew, _p_push_fwd_slew
    global _p_push_d_lpf, _p_push_side_deadband, _p_push_fwd_deadband
    global _p_push_side_min, _p_push_fwd_min
    _p_push_speed = speed
    _p_push_inward_bias = inward_bias
    _p_cam_lost_tol_ms = cam_lost_tol_ms
    _p_push_side_kp = side_kp
    _p_push_side_kd = side_kd
    _p_push_fwd_kp = fwd_kp
    _p_push_fwd_kd = fwd_kd
    _p_push_side_max = side_max
    _p_push_fwd_max = fwd_max
    _p_push_side_slew = side_slew
    _p_push_fwd_slew = fwd_slew
    _p_push_d_lpf = d_lpf
    _p_push_side_deadband = side_deadband
    _p_push_fwd_deadband = fwd_deadband
    _p_push_side_min = side_min
    _p_push_fwd_min = fwd_min

# 功能：配置面向目标的角速度控制参数。
def configure_face(kp, kd, gyro_max):
    global _p_face_kp, _p_face_kd, _p_face_gyro_max
    _p_face_kp = kp
    _p_face_kd = kd
    _p_face_gyro_max = gyro_max

_act_mode = 0

_act_seq = -1

_hold_reset_done = 0

_face_act_seq = -1

_push_act_reset_seq = -1

_push_edge = 0

_pid__push_lock_yaw = 0.0

_push_push_yaw = 0.0

_push_move_yaw_req = 999.9

_pid__push_ref_valid = 0

_pid__push_ref_rel_x = 0.0

_pid__push_ref_rel_y = 0.0

_push_err_x_prev = 0.0

_push_err_y_prev = 0.0

_push_dx_f = 0.0

_push_dy_f = 0.0

_push_side_cmd = 0.0

_push_fwd_cmd = 0.0

_push_world_side_cmd = 0.0

_pid__push_lost_t0 = 0

_push_seen_seq = -1

# 功能：根据推送目标边选择基础推送方向角。
def _mot_push_yaw(edge):
    if edge == 1:
        return 0.0
    if edge == 2:
        return 270.0
    if edge == 3:
        return 90.0
    if edge == 4:
        return 180.0
    return 0.0

# 功能：对运动控制量做对称限幅。
def _mot_limit(v, lim):
    if v > lim:
        return lim
    if v < -lim:
        return -lim
    return v

# 功能：限制控制量每个周期的变化速度，避免速度突变。
# 输入参数：target 为目标值；current 为当前值；step 为单周期最大变化量。
# 返回值：向 target 逼近后的新值。
def _mot_slew(target, current, step):
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target

# 功能：对误差加入死区，小误差范围内不输出，超过死区后扣除死区量。
# 输入参数：err 为原始误差；db 为死区宽度。
# 返回值：处理后的误差。
def _mot_deadband(err, db):
    if err > db:
        return err - db
    if err < -db:
        return err + db
    return 0.0

# 功能：给非零方向的控制量补最小速度
# 输入参数：v 为原始速度命令；vmin 为最小有效速度幅值。
# 返回值：带最小速度补偿的速度命令。
def _mot_min_speed(v, vmin):
    if v > 0.0 and v < vmin:
        return vmin
    if v < 0.0 and v > -vmin:
        return -vmin
    return v

# 功能：根据基础推送方向和锁定车头角，加入向场内侧偏转的角度。
# 输入参数：base_yaw 为基础推送方向角，单位度。
# 返回值：加入内偏后的推送移动方向角，单位度。
def _push_move_yaw(base_yaw):
    bias = _p_push_inward_bias
    side_err = (_pid__push_lock_yaw - base_yaw + 180) % 360 - 180
    if abs(side_err) > 1.0:
        if side_err > 0.0:
            return (base_yaw + bias) % 360.0
        return (base_yaw - bias) % 360.0
    if config._IS_LEADER:
        return (base_yaw + bias) % 360.0
    return (base_yaw - bias) % 360.0

# 功能：执行一次推送目标控制，融合基础推送速度、目标视觉闭环和世界坐标横向约束。
# 输入参数：无。函数读取 request_push 写入的全局请求缓存。
# 返回值：无。结果写入 _target_world_vx/_target_world_vy、_car.target_angle 和 _ctrl_mode。
def _push_step():
    global _ctrl_mode, _push_world_side_cmd, _target_world_vx, _target_world_vy
    global _push_edge, _pid__push_lock_yaw, _push_push_yaw, _push_move_yaw_req, _pid__push_ref_valid
    global _pid__push_ref_rel_x, _pid__push_ref_rel_y, _push_err_x_prev, _push_err_y_prev
    global _push_dx_f, _push_dy_f, _push_side_cmd, _push_fwd_cmd, _pid__push_lost_t0
    global _push_seen_seq, _push_act_reset_seq
    if _push_act_reset_seq != _push_reset_seq:
        _push_act_reset_seq = _push_reset_seq
        _push_edge = 0
        _pid__push_lock_yaw = 0.0
        _push_push_yaw = 0.0
        _push_move_yaw_req = 999.9
        _pid__push_ref_valid = 0
        _pid__push_ref_rel_x = 0.0
        _pid__push_ref_rel_y = 0.0
        _push_err_x_prev = 0.0
        _push_err_y_prev = 0.0
        _push_dx_f = 0.0
        _push_dy_f = 0.0
        _push_side_cmd = 0.0
        _push_fwd_cmd = 0.0
        _push_world_side_cmd = 0.0
        _pid__push_lost_t0 = 0
        _push_seen_seq = -1
    if _push_edge != _push_req_edge or _pid__push_lock_yaw != _push_req_lock_yaw or _push_move_yaw_req != _push_req_move_yaw:
        _push_edge = _push_req_edge
        _pid__push_lock_yaw = _push_req_lock_yaw
        _push_move_yaw_req = _push_req_move_yaw
        if _push_move_yaw_req < 900.0:
            _push_push_yaw = _push_move_yaw_req % 360.0
        else:
            _push_push_yaw = _mot_push_yaw(_push_edge)
    if _push_req_ref_valid and (not _pid__push_ref_valid):
        _pid__push_ref_rel_x = _push_req_ref_x
        _pid__push_ref_rel_y = _push_req_ref_y
        _pid__push_ref_valid = 1
    if _push_req_cam_seen and _push_seen_seq != _push_req_seq:
        _push_seen_seq = _push_req_seq
        _pid__push_lost_t0 = ticks_ms()
        rel_x = _push_req_rel_x
        rel_y = _push_req_rel_y
        if not _pid__push_ref_valid:
            _pid__push_ref_rel_x = rel_x
            _pid__push_ref_rel_y = rel_y
            _pid__push_ref_valid = 1
        err_x = rel_x - _pid__push_ref_rel_x
        err_y = rel_y - _pid__push_ref_rel_y
        # 直接使用视觉车体系的两个误差通道，不再先旋转到推送坐标系：
        # rel_x 只进入车体系横移 PD，rel_y 只进入推送轴前后 PD。
        err_side = err_x
        err_fwd = err_y
        _push_dx_f = _push_dx_f * (1.0 - _p_push_d_lpf) + (err_side - _push_err_x_prev) * _p_push_d_lpf
        _push_dy_f = _push_dy_f * (1.0 - _p_push_d_lpf) + (err_fwd - _push_err_y_prev) * _p_push_d_lpf
        _push_err_x_prev = err_side
        _push_err_y_prev = err_fwd
        side_raw = _p_push_side_kp * _mot_deadband(err_side, _p_push_side_deadband) + _p_push_side_kd * _push_dx_f
        fwd_raw = _p_push_fwd_kp * _mot_deadband(err_fwd, _p_push_fwd_deadband) + _p_push_fwd_kd * _push_dy_f
        side_raw = _mot_min_speed(_mot_limit(side_raw, _p_push_side_max), _p_push_side_min)
        fwd_raw = _mot_min_speed(_mot_limit(fwd_raw, _p_push_fwd_max), _p_push_fwd_min)
        _push_side_cmd = _mot_slew(side_raw, _push_side_cmd, _p_push_side_slew)
        _push_fwd_cmd = _mot_slew(fwd_raw, _push_fwd_cmd, _p_push_fwd_slew)
    if _pid__push_lost_t0 and ticks_diff(ticks_ms(), _pid__push_lost_t0) > _p_cam_lost_tol_ms:
        _push_side_cmd = 0.0
        _push_fwd_cmd = 0.0
    if _push_move_yaw_req < 900.0:
        pyaw = _push_push_yaw
    else:
        pyaw = _push_move_yaw(_push_push_yaw)
    push_rad = radians(pyaw)
    # 基座：两车都用各自的标称速度，不再跟随 ff。
    # 跟 ff 意味着从车的开环速度是主车速度延迟一个无线周期的拷贝，主车任何抖动都会
    # 被复制过来；两车同为固定标称、各自用物体环贴住物体，反而更稳。
    vx = sin(push_rad) * _p_push_speed
    vy = cos(push_rad) * _p_push_speed
    yaw_rad = radians(_pid__push_lock_yaw)
    c = cos(yaw_rad)
    s = sin(yaw_rad)
    # 前后修正仍只沿推送轴；横向修正直接沿各车当前车头的车体系横移轴，
    # 使相机看到的 rel_x 偏差直接对应本车横移速度。
    # fwd_sign：本车车头与推送方向夹角 <=90° 时（正推、横推的推进车）视觉前后
    # 误差与世界位移同号，环路是负反馈；车头与推送方向接近相反时（横推的引导/
    # 后退车，车头朝向推送方向的反向）符号必须翻转，否则该车的距离环会变成
    # 正反馈（越远越退、越近越顶）。
    fwd_push_rad = radians(_push_push_yaw)
    fwd_sign = 1.0 if abs((_pid__push_lock_yaw - _push_push_yaw + 180.0) % 360.0 - 180.0) <= 90.0 else -1.0
    vx += c * _push_side_cmd + sin(fwd_push_rad) * _push_fwd_cmd * fwd_sign
    vy += -s * _push_side_cmd + cos(fwd_push_rad) * _push_fwd_cmd * fwd_sign
    # 世界横向锁定已删除：它锁的正好也是垂直于推送方向的世界轴，参考取自起推瞬间
    # 各自的世界坐标，两车里程计一有偏差就会把两车往不同位置拉，和球的垂直约束直接冲突。
    _target_world_vx = vx
    _target_world_vy = vy
    _car.target_angle = _pid__push_lock_yaw
    _ctrl_mode = _CTRL_VEL

# 功能：执行一次面向目标角速度控制，把视觉角度误差转换为直接覆盖的 Z 轴角速度命令。
# 输入参数：无。函数读取 _face_req_active、_face_req_err 和面向目标控制参数。
# 返回值：无。结果写入 _face_obj_active 和 _face_obj_gyro_cmd。
def _face_step():
    global _face_obj_active, _face_obj_gyro_cmd
    global _face_act_seq
    if not _face_req_active:
        if _face_act_seq == _face_req_seq:
            return
        _face_act_seq = _face_req_seq
        _face_obj_active = False
        _face_obj_gyro_cmd = 0.0
        return
    _face_act_seq = _face_req_seq
    gyro_cmd = -_p_face_kp * _face_req_err - _p_face_kd * _IMU1_Data._gyro_z
    if gyro_cmd > _p_face_gyro_max:
        gyro_cmd = _p_face_gyro_max
    if gyro_cmd < -_p_face_gyro_max:
        gyro_cmd = -_p_face_gyro_max
    _face_obj_gyro_cmd = gyro_cmd
    _face_obj_active = True

# 功能：运动控制状态机主步骤，处理面向目标、世界速度、位置、保持和推送目标模式。
# 输入参数：无。函数读取 request_xxx 写入的请求缓存。
# 返回值：无。结果写入当前控制模式、目标速度、目标角度和相关状态。
# ── 世界速度斜坡（缓停）────────────────────────────────────────────────────
# 原来 request_hold 会把 _target_world_vx/vy 一步置 0，车速起来以后这种急停会让
# 两车因惯性翘头。这里对世界速度加变化率限制，把停车摊开成一段可控的减速。
#
# 关键约定：斜坡状态就用 _target_world_vx/vy 本身，不另设变量。好处有两个：
#   ① 经 Frame3 发给对方车做前馈的就是斜坡后的真实命令，不会前馈一个没执行的速度；
#   ② state_init / _show_error 里已有的"直接清零"天然就是斜坡硬复位，无需改动。
#
# 只作用于 WORLD / HOLD 两种模式：
#   - PUSH 由 _push_step 自己的 side_slew / fwd_slew 限速，再叠一层会拖慢推送响应；
#   - POS 由位置环 P 项在接近目标时自然减速，本来就不是阶跃停车。
#
# 单位与 request_world 的 vx/vy 一致。motion_step 每 5 ms 执行一次，
# 因此 单周期步长 = 速率 × 0.005。
_VEL_RAMP_DT = 0.005
# 减速率：700 → 单周期 3.5。从合速度 240 减到 _VEL_RAMP_MIN_SPEED 约需 220 ms，
# 从 100 减到 40 约需 67 ms。翘头仍明显就调小，停车太拖沓就调大。
_VEL_DECEL_RATE = 700.0
# 加速率：0 = 不限制，保持原有起步响应。要一并压住起步前倾时再设成正值。
_VEL_ACCEL_RATE = 0.0
# 当前合速度低于该值时直接归零，不再走斜坡。
# 目的是把影响面限制在真正的高速段：APPROACH CLOSE、VDOCK 这类低速精定位，
# 以及各状态里"停车后等 N ms 判稳"的时序，行为与改前完全一致。
_VEL_RAMP_MIN_SPEED = 40.0

_VEL_DECEL_STEP = _VEL_DECEL_RATE * _VEL_RAMP_DT

_VEL_ACCEL_STEP = _VEL_ACCEL_RATE * _VEL_RAMP_DT

# 功能：从 POS 模式切入 WORLD/HOLD 时，用当前车体系命令反算世界速度作为斜坡起点。
# POS 模式下 _target_world_vx/vy 不被维护，直接拿它当起点会从一个陈旧值开始减速。
def _vel_ramp_seed():
    global _target_world_vx, _target_world_vy
    yaw = radians(_car.current_angle)
    c = cos(yaw)
    s = sin(yaw)
    bx = _car.target_Speed_X
    by = _car.target_Speed_Y
    _target_world_vx = c * bx + s * by
    _target_world_vy = -s * bx + c * by

# 功能：把 _target_world_vx/vy 按速率限制逼近 _req_vx/_req_vy。
# 对速度矢量整体限幅而不是分轴限幅，这样掉头这类大幅换向也会被正确限制。
def _vel_ramp_step():
    global _target_world_vx, _target_world_vy, _speed_reset_req, _hold_reset_done
    tvx = _req_vx
    tvy = _req_vy
    dvx = tvx - _target_world_vx
    dvy = tvy - _target_world_vy
    d2 = dvx * dvx + dvy * dvy
    if d2 > 0.0:
        cur2 = _target_world_vx * _target_world_vx + _target_world_vy * _target_world_vy
        # 用"目标速度在当前速度方向上的投影"判断是不是在减速，而不是比较合速度大小。
        # 比大小会漏掉等速掉头（+100 -> -100 合速度相同，却是 200 单位的冲击）和
        # 大角度换向；用投影则停车、掉头、转向被同一个判据覆盖。
        # 小幅修正因为 d2 <= step*step 会直接到位，不受影响。
        if tvx * _target_world_vx + tvy * _target_world_vy < cur2:
            step = _VEL_DECEL_STEP
        else:
            step = _VEL_ACCEL_STEP
        # 低速段和未启用限制时直接到位，保持原有时序。
        if step <= 0.0 or cur2 <= _VEL_RAMP_MIN_SPEED * _VEL_RAMP_MIN_SPEED or d2 <= step * step:
            _target_world_vx = tvx
            _target_world_vy = tvy
        else:
            k = step / sqrt(d2)
            _target_world_vx += dvx * k
            _target_world_vy += dvy * k
    # 轮速环积分清零推迟到真正停稳之后：减速过程中仍需要闭环维持减速轨迹，
    # 一进 HOLD 就清积分等于直接刹死，缓停会失效。
    if _act_mode == _MODE_HOLD and not _hold_reset_done:
        if _target_world_vx == 0.0 and _target_world_vy == 0.0:
            _speed_reset_req = True
            _hold_reset_done = 1

def motion_step():
    global _ctrl_mode, _pos_speed_max, _speed_reset_req, _target_pos_x, _target_pos_y, _target_world_vx, _target_world_vy
    global _act_mode, _act_seq, _hold_reset_done
    _face_step()
    if _act_mode == _MODE_PUSH and _req_mode == _MODE_PUSH:
        _act_seq = _req_seq
        _push_step()
        return
    if _act_seq != _req_seq:
        mode = _req_mode
        _act_seq = _req_seq
        if mode == _MODE_POS:
            _target_pos_x = _req_tx
            _target_pos_y = _req_ty
            _pos_speed_max = _req_speed
            _car.target_angle = _req_angle
            _ctrl_mode = _CTRL_POS
            _act_mode = _MODE_POS
            _hold_reset_done = 0
            return
        if mode == _MODE_PUSH:
            _act_mode = _MODE_PUSH
            _hold_reset_done = 0
            _push_step()
            return
        if mode == _MODE_WORLD or mode == _MODE_HOLD:
            _car.target_angle = _req_angle
            _ctrl_mode = _CTRL_VEL
            if _act_mode == _MODE_POS:
                _vel_ramp_seed()
            if mode == _MODE_WORLD:
                _hold_reset_done = 0
            _act_mode = mode
    # 没有新请求时也要继续推进斜坡，所以这一步放在早退之外。
    if _act_mode == _MODE_WORLD or _act_mode == _MODE_HOLD:
        _vel_ramp_step()

# 编码器里程计比例系数。KY 管前后轴、KX 管左右轴（见 omni_mileage）。
# 7.22 实车标定结果（卷尺实测 / 里程计读数）。当时用的板载标定模式已因主车 RAM
# 不足移除，需要重新标定时从《已移除代码备份.md》的 B1/B5 两块恢复：
#   主车 KY x78.5/72 = +9.0%   KX x73.5/64 = +14.8%
#   从车 KY x78.5/76 = +3.3%   KX x65/68   = -4.4%
if config._IS_LEADER:
    _MILEAGE_KX = 0.0050085777167
    _MILEAGE_KY = 0.0052485901645
else:
    _MILEAGE_KX = 0.0043663012800
    _MILEAGE_KY = 0.0052209660057

# 编码器低通滤波系数
_ENCODER_FILTER_RC = 0.25

_enc1_f = 0.0

_enc2_f = 0.0

_enc3_f = 0.0

_enc4_f = 0.0

# 功能：对编码器速度或增量做一阶 RC 低通滤波。
# 输入参数：value 为当前原始值；old_value 为上一次滤波后的值。
# 返回值：本次滤波后的值。
@micropython.native
def rc_filter(value, old_value):
    return (1.0 - _ENCODER_FILTER_RC) * old_value + _ENCODER_FILTER_RC * float(value)

# 功能：根据四轮编码器增量和当前车头角，积分得到世界坐标系下的 X/Y 位移。
# 输入参数：无。函数读取 _encoder1 到 _encoder4 和 _car.current_angle。
# 返回值：无。结果累加到 _Position_X_encoder 和 _Position_Y_encoder。
@micropython.native
def omni_mileage():
    global _Position_X_encoder, _Position_Y_encoder
    delta_y = (_encoder1 + _encoder2 + _encoder3 + _encoder4) / 4.0
    delta_x = (-_encoder1 + _encoder2 + _encoder3 - _encoder4) / 4.0
    yaw = radians(_car.current_angle)
    cos_yaw = cos(yaw)
    sin_yaw = sin(yaw)
    _Position_X_encoder += sin_yaw * delta_y * _MILEAGE_KY + cos_yaw * delta_x * _MILEAGE_KX
    _Position_Y_encoder += -sin_yaw * delta_x * _MILEAGE_KX + cos_yaw * delta_y * _MILEAGE_KY

# 功能：读取四个编码器，更新滤波后的电机反馈速度，并刷新编码器里程计位置。
# 结果写入编码器变量、电机 encoder_speed 和位置积分缓存。
def encoder_get():
    global _encoder1, _encoder2, _encoder3, _encoder4
    global _enc1_f, _enc2_f, _enc3_f, _enc4_f
    _encoder1 = _encoder_1.get()
    _encoder2 = _encoder_2.get()
    _encoder3 = _encoder_3.get()
    _encoder4 = _encoder_4.get()
    _enc1_f = rc_filter(_encoder1, _enc1_f)
    _enc2_f = rc_filter(_encoder2, _enc2_f)
    _enc3_f = rc_filter(_encoder3, _enc3_f)
    _enc4_f = rc_filter(_encoder4, _enc4_f)
    _motor_1_speed.encoder_speed = _enc1_f
    _motor_2_speed.encoder_speed = _enc2_f
    _motor_3_speed.encoder_speed = _enc3_f
    _motor_4_speed.encoder_speed = _enc4_f
    omni_mileage()

# 串口通信参数。
# _UART_READ_MAX 为单次读取暂存区大小，_CAM_BUFFER_MAX/_WIRELESS_BUFFER_MAX 为环形缓冲区容量。
_UART_READ_MAX = const(120)

_CAM_BUFFER_MAX = const(400)

_WIRELESS_BUFFER_MAX = const(200)

# 无线应答延时和超时，单位 ms。用于主从车一问一答，减少同时发送造成的冲突。
_WIRELESS_REPLY_DELAY_MS = const(5)

_WIRELESS_REPLY_TIMEOUT_MS = const(60)

# Frame3 v4 协议：保留 brick 世界坐标，并把世界速度升级为 0.1 精度的有符号 int16。
# 帧类型字节同时作为协议版本号；旧协议帧不解析。
_FRAME3_TYPE = const(6)

_FRAME3_LEN = const(37)

# 无线命令 TTL：超过该时间没收到对方帧，清除对方下发的命令，防止旧命令在链路短丢后继续生效。
_CMD_TTL_MS = const(1000)

# 摄像头普通目标最大有效相对距离，超过该距离认为目标不可靠。
_CAM_OBJ_MAX_REL_DIST = const(75)

# Frame1 v3：帧类型 8；把三个识别开关、数量模式和 0~7 总剩余数
# 合并到同一个控制字节，删除原独立的 count_mode/total_remaining 字节。
# 更换帧类型可防止新旧两端字段错位时仍把 CRC 正确的帧当作有效数据。
_FRAME1_TYPE = const(8)

_FRAME1_LEN = const(17)

_tx_frame1 = bytearray(_FRAME1_LEN)

_tx_frame3 = bytearray(_FRAME3_LEN)

_CAM_BUF = bytearray(_CAM_BUFFER_MAX)

_WIRE_BUF = bytearray(_WIRELESS_BUFFER_MAX)

_cam_mv = memoryview(_CAM_BUF)

_wire_mv = memoryview(_WIRE_BUF)

_cam_head = 0

_cam_tail = 0

_cam_count = 0

_wire_head = 0

_wire_tail = 0

_wire_count = 0

_rx_scratch = bytearray(_UART_READ_MAX)

_rx_scratch_mv = memoryview(_rx_scratch)

# 功能：把浮点数按通信协议打包成 0.1 精度的有符号 16 位整数。
# 输入参数：buf 为目标字节数组；i 为写入起始下标；v 为待打包数值。
# 返回值：无。若 v 超过有效范围或接近 999 哨兵，会打包为 9990 表示无效。
def _pack_i16(buf, i, v):
    if v >= 900.0 or v <= -900.0:
        raw = 9990
    else:
        raw = int(round(v * 10.0))
        if raw > 32767:
            raw = 32767
        if raw < -32768:
            raw = -32768
    if raw < 0:
        raw += 65536
    buf[i] = raw >> 8 & 255
    buf[i + 1] = raw & 255

# 功能：按通信协议从两个字节解码 0.1 精度的有符号数。
# 输入参数：buf 为源字节数组；i 为读取起始下标。
# 返回值：解码后的浮点数。
def _dec(buf, i):
    raw = buf[i] << 8 | buf[i + 1]
    if raw > 32767:
        raw -= 65536
    return raw / 10.0

# 功能：计算协议帧 CRC8 校验值。
# 输入参数：buf 为数据缓冲区；s 为起始下标；e 为结束下标，计算范围为 [s, e)。
# 返回值：CRC8 校验字节。
def _dec_i8(buf, i):
    raw = buf[i]
    if raw > 127:
        raw -= 256
    return raw

# 功能：把一对坐标按 0.5cm 精度打包成两个 12 位有符号数，共占 3 字节。
# 输入参数：buf 为目标字节数组；i 为写入起始下标；x、y 为坐标值，单位 cm。
# 返回值：无。范围钳位到 ±1023.5cm，无效值应由调用方改发 0 并用 fresh_flags 标记。
def _pack_s12pair(buf, i, x, y):
    rx = int(round(x * 2.0))
    if rx > 2047:
        rx = 2047
    if rx < -2048:
        rx = -2048
    if rx < 0:
        rx += 4096
    ry = int(round(y * 2.0))
    if ry > 2047:
        ry = 2047
    if ry < -2048:
        ry = -2048
    if ry < 0:
        ry += 4096
    buf[i] = rx >> 4 & 255
    buf[i + 1] = (rx & 15) << 4 | ry >> 8
    buf[i + 2] = ry & 255

# 功能：解码 _pack_s12pair 打包的第一个 12 位有符号数。
# 输入参数：buf 为源字节数组；i 为该 3 字节组的起始下标。
# 返回值：解码后的坐标，单位 cm，精度 0.5。
def _dec_s12a(buf, i):
    raw = buf[i] << 4 | buf[i + 1] >> 4
    if raw > 2047:
        raw -= 4096
    return raw / 2.0

# 功能：解码 _pack_s12pair 打包的第二个 12 位有符号数。
# 输入参数：buf 为源字节数组；i 为该 3 字节组的起始下标。
# 返回值：解码后的坐标，单位 cm，精度 0.5。
def _dec_s12b(buf, i):
    raw = (buf[i + 1] & 15) << 8 | buf[i + 2]
    if raw > 2047:
        raw -= 4096
    return raw / 2.0


# 功能：检查 OpenART 给出的绝对坐标是否允许参与视觉位置修正。
# PUSH 启用边界门槛后，只接受距低侧或高侧边界小于门槛的结果；
# 无效值 999 始终拒绝。其他状态门槛为 0，沿用原来的有效值规则。
def _vis_pos_fix_value_ok(value, axis_limit):
    if value >= 900.0:
        return False
    gate = _vis_pos_fix_edge_gate_cm
    if gate <= 0.0:
        return True
    return value < gate or value > axis_limit - gate

# 功能：把数值按四舍五入打包成 1 字节有符号数，超范围钳位到 ±127/-128。
# 输入参数：buf 为目标字节数组；i 为写入下标；v 为待打包数值（调用方先乘好比例）。
# 返回值：无。解码使用已有 _dec_i8。
def _pack_s8(buf, i, v):
    raw = int(round(v))
    if raw > 127:
        raw = 127
    if raw < -128:
        raw = -128
    if raw < 0:
        raw += 256
    buf[i] = raw


@micropython.native
def _crc8(buf, s, e):
    crc = 0
    for i in range(s, e):
        crc ^= buf[i]
        for unused in range(8):
            if crc & 128:
                crc = (crc << 1 ^ 49) & 255
            else:
                crc = crc << 1 & 255
    return crc

# 功能：向摄像头串口发送本车位姿、目标选择和识别功能开关。
# 返回值：无。函数按照 Frame1 v3（帧类型 8）打包并通过 _uart1 发送。
def uart_send_pose():
    remain_mask = 0
    for i in range(1, min(_OBJ_ID_MAX, len(task._obj_remain) - 1) + 1):
        if task._obj_remain[i] > 0:
            remain_mask |= 1 << i - 1
    line_en = 1 if task._OPENART_LINE_ENABLE else 0
    model_en = 1 if task._OPENART_MODEL_ENABLE else 0
    ball_en = 1 if task._OPENART_BALL_ENABLE else 0
    f = _tx_frame1
    f[0] = 170
    f[1] = 85
    f[2] = _FRAME1_TYPE
    _pack_i16(f, 3, _car.current_angle)
    _pack_i16(f, 5, _car.Position_X)
    _pack_i16(f, 7, _car.Position_Y)
    # OpenART 协议只有一位：1=只知道全局总数，0=已知哪些类别仍需搬运。
    # 模式 2 和模式 3 都通过 remain_mask 告知相机剩余类别，因此都发送 0。
    count_mode = 1 if task._obj_count_mode == 1 else 0
    total_remaining = max(0, min(7, int(task._obj_total_remaining)))
    f[9] = (line_en & 1
            | (model_en & 1) << 1
            | (ball_en & 1) << 2
            | count_mode << 3
            | total_remaining << 4)
    f[10] = task._target_sel_id_for_cam & 255
    f[11] = remain_mask & 255
    _pack_i16(f, 12, task._target_rel_x_for_cam)
    _pack_i16(f, 14, task._target_rel_y_for_cam)
    f[16] = _crc8(f, 2, 16)
    _uart1.write(f)

# 功能：解析类型 0x07 的摄像头完整数据帧，更新视觉定位、目标物、对方车辆和球体相对坐标。
# 输入参数：buf 为包含完整帧的缓冲区；off 为帧起始偏移。
# 返回值：无。解析结果写入视觉修正变量和 _cam_obj_x/y/id 等缓存。
def _parse_cam_frame(buf, off):
    global _Position_X_fix, _Position_Y_fix, _YawAngle_fix, _angle_update_flag, _cam_intv_max_ms, _cam_intv_ms, _cam_obj_count, _cam_target_observing, _cam_rx_last_ms, _position_update_flag, _vis_x_fix_cnt, _vis_y_fix_cnt
    global _cam_car_x, _cam_car_y, _cam_car_rel_x, _cam_car_rel_y
    global _cam_cone_x, _cam_cone_y, _cam_cone_rel_x, _cam_cone_rel_y, _cam_cone_ts, _cam_cone_seen, _cam_cone_seen_seq
    global _cam_brick_x, _cam_brick_y, _cam_brick_rel_x, _cam_brick_rel_y, _cam_brick_ts, _cam_brick_seen, _cam_brick_seen_seq
    global _cam_line_cx, _cam_line_seq, _cam_line_last_ms
    vis_yaw = _dec(buf, off + 3)
    rod_x = _dec(buf, off + 5)
    rod_y = _dec(buf, off + 7)
    _YawAngle_fix = vis_yaw
    _Position_X_fix = rod_x
    _Position_Y_fix = rod_y
    _angle_update_flag = 1
    _position_update_flag = 1
    if _vis_x_fix_en and _vis_pos_fix_value_ok(rod_x, _VIS_FIX_FIELD_W):
        _vis_x_fix_cnt += 1
    if _vis_y_fix_en and _vis_pos_fix_value_ok(rod_y, _VIS_FIX_FIELD_H):
        _vis_y_fix_cnt += 1
    yaw = radians(_car.current_angle)
    c_yaw = cos(yaw)
    s_yaw = sin(yaw)
    car_x = _car.Position_X
    car_y = _car.Position_Y
    _cam_obj_count = 0
    _cam_target_observing = False
    _cam_obj_id[0] = _CAM_OBJ_INVALID_ID
    _cam_obj_id[1] = _CAM_OBJ_INVALID_ID
    _cam_obj_x[0] = 999.0
    _cam_obj_y[0] = 999.0
    _cam_obj_rel_x[0] = 999.0
    _cam_obj_rel_y[0] = 999.0
    _cam_obj_x[1] = 999.0
    _cam_obj_y[1] = 999.0
    _cam_obj_rel_x[1] = 999.0
    _cam_obj_rel_y[1] = 999.0
    _cam_car_x = 999.0
    _cam_car_y = 999.0
    _cam_car_rel_x = 999.0
    _cam_car_rel_y = 999.0
    _cam_ball_rel_x[0] = 999.0
    _cam_ball_rel_y[0] = 999.0
    _cam_ball_rel_x[1] = 999.0
    _cam_ball_rel_y[1] = 999.0
    now = ticks_ms()
    cone_seen = False
    brick_seen = False
    line_cx = _dec(buf, off + _CAM_LINE_DATA_OFF)
    _cam_line_cx = line_cx
    _cam_line_seq += 1
    _cam_line_last_ms = now if line_cx < 900.0 else 0
    obj_off = off + _CAM_OBJ_DATA_OFF
    for slot in range(_CAM_FRAME_OBJ_MAX):
        rel_x = _dec_i8(buf, obj_off)
        rel_y = _dec_i8(buf, obj_off + 1)
        obj_id = buf[obj_off + 2]
        track_id = buf[obj_off + 3]
        obj_off += 4
        if obj_id == _CAM_OBJ_INVALID_ID:
            if slot == 0 and track_id == _CAM_TARGET_OBSERVING_TRACK_MARKER:
                _cam_target_observing = True
            continue
        if slot < _CAM_OBJ_MAX:
            if 1 <= obj_id <= _OBJ_ID_MAX and sqrt(rel_x * rel_x + rel_y * rel_y) <= _CAM_OBJ_MAX_REL_DIST:
                world_x = car_x + c_yaw * rel_x + s_yaw * rel_y
                world_y = car_y - s_yaw * rel_x + c_yaw * rel_y
                _cam_obj_x[slot] = world_x
                _cam_obj_y[slot] = world_y
                _cam_obj_rel_x[slot] = rel_x
                _cam_obj_rel_y[slot] = rel_y
                _cam_obj_id[slot] = obj_id
                _cam_obj_count = slot + 1
            continue
        if slot == 2:
            if obj_id == 0:
                _cam_car_rel_x = rel_x
                _cam_car_rel_y = rel_y
                _cam_car_x = car_x + c_yaw * rel_x + s_yaw * rel_y
                _cam_car_y = car_y - s_yaw * rel_x + c_yaw * rel_y
            continue
        if (slot == 3 and obj_id == _BALL_L_CLASS_ID or slot == 4 and obj_id == _BALL_R_CLASS_ID) and sqrt(rel_x * rel_x + rel_y * rel_y) <= task._BALL_REL_MAX_DIST:
            slot -= 3
            _cam_ball_rel_x[slot] = rel_x
            _cam_ball_rel_y[slot] = rel_y
            continue
        if slot == 5 and obj_id == _CONE_CLASS_ID:
            cone_seen = True
            _cam_cone_rel_x = rel_x
            _cam_cone_rel_y = rel_y
            _cam_cone_x = car_x + c_yaw * rel_x + s_yaw * rel_y
            _cam_cone_y = car_y - s_yaw * rel_x + c_yaw * rel_y
            _cam_cone_ts = now
            _cam_cone_seen = 1
            _cam_cone_seen_seq = _cam_cone_seen_seq + 1 if _cam_cone_seen_seq < 65535 else 1
            continue
        if slot == 6 and obj_id == _BRICK_CLASS_ID:
            brick_seen = True
            _cam_brick_rel_x = rel_x
            _cam_brick_rel_y = rel_y
            _cam_brick_x = car_x + c_yaw * rel_x + s_yaw * rel_y
            _cam_brick_y = car_y - s_yaw * rel_x + c_yaw * rel_y
            _cam_brick_ts = now
            _cam_brick_seen = 1
            _cam_brick_seen_seq = _cam_brick_seen_seq + 1 if _cam_brick_seen_seq < 65535 else 1
    if not cone_seen:
        _cam_cone_x = 999.0
        _cam_cone_y = 999.0
        _cam_cone_rel_x = 999.0
        _cam_cone_rel_y = 999.0
        _cam_cone_ts = 0
        _cam_cone_seen = 0
    if not brick_seen:
        _cam_brick_x = 999.0
        _cam_brick_y = 999.0
        _cam_brick_rel_x = 999.0
        _cam_brick_rel_y = 999.0
        _cam_brick_ts = 0
        _cam_brick_seen = 0
    if _cam_rx_last_ms > 0:
        dt = ticks_diff(now, _cam_rx_last_ms)
        _cam_intv_ms = dt
        if dt > _cam_intv_max_ms:
            _cam_intv_max_ms = dt
    _cam_rx_last_ms = now

# 功能：从摄像头串口读取数据，维护环形缓冲区，查找并校验协议帧。
# 成功解析到帧时会调用 _parse_cam_frame 更新视觉数据。
def uart_get_frame_v2():
    global _t
    global _cam_rx_bytes
    global _cam_head, _cam_tail, _cam_count
    global _cam_crc_fail_cnt
    N = _CAM_BUFFER_MAX
    n = _uart1.any()
    if n:
        cl = _uart1.readinto(_rx_scratch, min(n, _UART_READ_MAX))
        if cl:
            _cam_rx_bytes += cl
            if _cam_count + cl > N:
                _cam_head = 0
                _cam_tail = 0
                _cam_count = 0
            h = _cam_head
            first = N - h
            if first >= cl:
                _CAM_BUF[h:h + cl] = _rx_scratch_mv[:cl]
            else:
                _CAM_BUF[h:N] = _rx_scratch_mv[:first]
                _CAM_BUF[0:cl - first] = _rx_scratch_mv[first:cl]
            _cam_head = (h + cl) % N
            _cam_count += cl
    cnt = _cam_count
    _t = _cam_tail
    i = 0
    while i + 2 < cnt:
        if _CAM_BUF[(_t + i) % N] != 170 or _CAM_BUF[(_t + i + 1) % N] != 85:
            i += 1
            continue
        if _CAM_BUF[(_t + i + 2) % N] != _CAM_FRAME_TYPE:
            i += 1
            continue
        if i + _CAM_FRAME_LEN > cnt:
            break
        base = (_t + i) % N
        first = N - base
        if first >= _CAM_FRAME_LEN:
            _rx_scratch[0:_CAM_FRAME_LEN] = _cam_mv[base:base + _CAM_FRAME_LEN]
        else:
            _rx_scratch[0:first] = _cam_mv[base:N]
            _rx_scratch[first:_CAM_FRAME_LEN] = _cam_mv[0:_CAM_FRAME_LEN - first]
        if _crc8(_rx_scratch, 2, _CAM_FRAME_LEN - 1) != _rx_scratch[_CAM_FRAME_LEN - 1]:
            _cam_crc_fail_cnt += 1
            i += 1
            continue
        _parse_cam_frame(_rx_scratch, 0)
        i += _CAM_FRAME_LEN
    _cam_tail = (_t + i) % N
    _cam_count = cnt - i

# 功能：解析双车无线通信帧（Frame3 v4），更新对方车辆、目标、cone/brick、协同指令和链路统计信息。
# 解析只写 _Other_ 镜像和链路统计，不再覆盖本方发送变量；
# 仅从车会把主车下发的横向保持速度写入本地控制变量。
def _parse_wireless_frame(buf, off):
    global _push_world_side_cmd
    global _Other_Approach_Cmd, _Other_Brick_X, _Other_Brick_Y, _Other_Brick_Ts, _Other_Car_Angle, _Other_Car_Mode, _Other_Car_Push_Sub, _Other_Car_Ready, _Other_Car_Ready_Ts, _Other_Car_Seen_Me, _Other_Car_X, _Other_Car_Y, _Other_Cmd_Seq, _Other_Cone_X, _Other_Cone_Y, _Other_Cone_Ts, _Other_Follower_Cmd_Yaw_Dir, _Other_Master_Cmd_Sub, _Other_Push_Side_Cmd, _Other_Route_Cmd, _Other_Route_Obs_Axis, _Other_Search_First_Sweep_Mode, _Other_Target_Edge, _Other_Target_ObjId, _Other_Target_X, _Other_Target_Y, _Other_World_Vx, _Other_World_Vy, _wireless_intv_max_ms, _wireless_reply_due_ms, _wireless_reply_pending, _wireless_rtt_max_ms, _wireless_rx_last_ms, _wireless_wait_reply
    other_angle = (buf[off + 4] << 8 | buf[off + 5]) / 10.0
    other_x = _dec_s12a(buf, off + 6)
    other_y = _dec_s12b(buf, off + 6)
    flags = buf[off + 15]
    if flags & 1:
        target_x = _dec_s12a(buf, off + 9)
        target_y = _dec_s12b(buf, off + 9)
    else:
        target_x = 999.0
        target_y = 999.0
    b = buf[off + 16]
    other_mode = b >> 3
    other_ready = b & 7
    other_push_sub = buf[off + 17]
    b = buf[off + 18]
    other_edge = b >> 4
    other_obj = b & 15
    b = buf[off + 19]
    other_cmd_seq = b >> 3
    other_cmd_sub = b & 7
    approach_cmd = _dec(buf, off + 20)
    route_cmd = _dec(buf, off + 22)
    yaw_dir_cmd = _dec(buf, off + 24)
    route_obs_axis = _dec(buf, off + 26)
    other_world_vx = _dec(buf, off + 28)
    other_world_vy = _dec(buf, off + 30)
    side_cmd = _dec_i8(buf, off + 32) / 2.0
    now_ms = ticks_ms()
    _Other_Car_Angle = other_angle
    _Other_Car_X = other_x
    _Other_Car_Y = other_y
    _Other_Target_X = target_x
    _Other_Target_Y = target_y
    _Other_Car_Ready = other_ready
    _Other_Car_Ready_Ts = now_ms
    _Other_Target_Edge = other_edge
    _Other_Target_ObjId = other_obj
    _Other_Approach_Cmd = approach_cmd
    _Other_Route_Obs_Axis = route_obs_axis
    _Other_Route_Cmd = route_cmd
    _Other_Car_Mode = other_mode
    _Other_Car_Push_Sub = other_push_sub
    _Other_Car_Seen_Me = bool(flags & 8)
    _Other_Search_First_Sweep_Mode = (flags >> 4) & 3
    _Other_Cmd_Seq = other_cmd_seq
    _Other_Master_Cmd_Sub = other_cmd_sub
    _Other_Follower_Cmd_Yaw_Dir = yaw_dir_cmd
    _Other_Push_Side_Cmd = side_cmd
    _Other_World_Vx = other_world_vx
    _Other_World_Vy = other_world_vy
    if not config._IS_LEADER:
        _push_world_side_cmd = side_cmd
    if flags & 2:
        other_cone_x = _dec_s12a(buf, off + 12)
        other_cone_y = _dec_s12b(buf, off + 12)
        _Other_Cone_X = other_cone_x
        _Other_Cone_Y = other_cone_y
        _Other_Cone_Ts = now_ms
    else:
        _Other_Cone_X = 999.0
        _Other_Cone_Y = 999.0
        _Other_Cone_Ts = 0
    if flags & 4:
        _Other_Brick_X = _dec_s12a(buf, off + 33)
        _Other_Brick_Y = _dec_s12b(buf, off + 33)
        _Other_Brick_Ts = now_ms
    else:
        _Other_Brick_X = 999.0
        _Other_Brick_Y = 999.0
        _Other_Brick_Ts = 0
    if _wireless_rx_last_ms > 0:
        dt = ticks_diff(now_ms, _wireless_rx_last_ms)
        if dt > _wireless_intv_max_ms:
            _wireless_intv_max_ms = dt
    if config._IS_LEADER and _wireless_last_tx_ms > 0:
        rtt = ticks_diff(now_ms, _wireless_last_tx_ms)
        if rtt > _wireless_rtt_max_ms:
            _wireless_rtt_max_ms = rtt
    _wireless_rx_last_ms = now_ms
    if config._IS_LEADER:
        _wireless_wait_reply = 0
    else:
        _wireless_reply_pending = 1
        _wireless_reply_due_ms = ticks_add(now_ms, _WIRELESS_REPLY_DELAY_MS)

# 功能：从无线串口读取数据，维护环形缓冲区，查找并校验双车通信帧。
# 成功解析到帧时会调用 _parse_wireless_frame 更新对方车辆状态。
def uart_wireless_read():
    global _t
    global _wire_rx_bytes
    global _wire_head, _wire_tail, _wire_count
    if _uart_wireless is None:
        return
    N = _WIRELESS_BUFFER_MAX
    n = _uart_wireless.any()
    if n:
        cl = _uart_wireless.readinto(_rx_scratch, min(n, _UART_READ_MAX))
        if cl:
            _wire_rx_bytes += cl
            if _wire_count + cl > N:
                _wire_head = 0
                _wire_tail = 0
                _wire_count = 0
            h = _wire_head
            first = N - h
            if first >= cl:
                _WIRE_BUF[h:h + cl] = _rx_scratch_mv[:cl]
            else:
                _WIRE_BUF[h:N] = _rx_scratch_mv[:first]
                _WIRE_BUF[0:cl - first] = _rx_scratch_mv[first:cl]
            _wire_head = (h + cl) % N
            _wire_count += cl
    cnt = _wire_count
    _t = _wire_tail
    i = 0
    while i + 2 < cnt:
        if _WIRE_BUF[(_t + i) % N] != 170 or _WIRE_BUF[(_t + i + 1) % N] != 85:
            i += 1
            continue
        if _WIRE_BUF[(_t + i + 2) % N] != _FRAME3_TYPE:
            i += 1
            continue
        if i + _FRAME3_LEN - 1 >= cnt:
            break
        base = (_t + i) % N
        first = N - base
        if first >= _FRAME3_LEN:
            _rx_scratch[0:_FRAME3_LEN] = _wire_mv[base:base + _FRAME3_LEN]
        else:
            _rx_scratch[0:first] = _wire_mv[base:N]
            _rx_scratch[first:_FRAME3_LEN] = _wire_mv[0:_FRAME3_LEN - first]
        if _crc8(_rx_scratch, 2, _FRAME3_LEN - 1) != _rx_scratch[_FRAME3_LEN - 1]:
            i += 1
            continue
        _parse_wireless_frame(_rx_scratch, 0)
        i += _FRAME3_LEN
    _wire_tail = (_t + i) % N
    _wire_count = cnt - i

# 功能：更新无线发送调度状态，处理主车等待超时和从车延时回包。
# 根据时序置位或清除 _wireless_send_flag、_wireless_wait_reply。
def uart_wireless_update_tx():
    global _wireless_reply_pending, _wireless_send_flag, _wireless_wait_reply
    if _uart_wireless is None:
        _wireless_wait_reply = 0
        _wireless_reply_pending = 0
        _wireless_send_flag = 0
        return
    if config._IS_LEADER and _wireless_wait_reply:
        if ticks_diff(ticks_ms(), _wireless_last_tx_ms) >= _WIRELESS_REPLY_TIMEOUT_MS:
            _wireless_wait_reply = 0
    if _wireless_reply_pending:
        if ticks_diff(ticks_ms(), _wireless_reply_due_ms) >= 0:
            _wireless_reply_pending = 0
            _wireless_send_flag = 1

# 功能：向无线串口发送本车位姿、当前目标、cone/brick、任务状态、推送协同信息和世界速度（Frame3 v3）。
# 主车在命令字段发送自己的命令；从车在同一字段回报最近收到的命令序号和命令字作为 ACK。
def uart_send_pose_wireless():
    global _wireless_last_tx_ms, _wireless_send_flag, _wireless_wait_reply
    global _cam_cone_x, _cam_cone_y, _cam_cone_rel_x, _cam_cone_rel_y, _cam_cone_ts, _cam_cone_seen
    global _cam_brick_x, _cam_brick_y, _cam_brick_rel_x, _cam_brick_rel_y, _cam_brick_ts, _cam_brick_seen
    global _frame3_seq
    if _uart_wireless is None:
        _wireless_send_flag = 0
        _wireless_wait_reply = 0
        return
    now_ms = ticks_ms()
    if _cam_cone_seen and _cam_cone_ts != 0 and ticks_diff(now_ms, _cam_cone_ts) > _CONE_MEMORY_MS:
        _cam_cone_x = 999.0
        _cam_cone_y = 999.0
        _cam_cone_rel_x = 999.0
        _cam_cone_rel_y = 999.0
        _cam_cone_ts = 0
        _cam_cone_seen = 0
    if _cam_brick_seen and _cam_brick_ts != 0 and ticks_diff(now_ms, _cam_brick_ts) > _CONE_MEMORY_MS:
        _cam_brick_x = 999.0
        _cam_brick_y = 999.0
        _cam_brick_rel_x = 999.0
        _cam_brick_rel_y = 999.0
        _cam_brick_ts = 0
        _cam_brick_seen = 0
    if config._IS_LEADER:
        approach_cmd = task._approach_cmd_to_other
        cmd_seq = task._cmd_seq
        cmd_sub = task._master_cmd_sub
        yaw_dir_cmd = task._follower_cmd_yaw_dir
        side_cmd = _push_world_side_cmd
    else:
        approach_cmd = 999.9
        cmd_seq = _Other_Cmd_Seq
        cmd_sub = _Other_Master_Cmd_Sub
        yaw_dir_cmd = 999.9
        side_cmd = 0.0
    f = _tx_frame3
    f[0] = 170
    f[1] = 85
    f[2] = _FRAME3_TYPE
    _frame3_seq = _frame3_seq + 1 & 255
    f[3] = _frame3_seq
    raw = int(round(_car.current_angle * 10.0)) % 3600
    f[4] = raw >> 8 & 255
    f[5] = raw & 255
    _pack_s12pair(f, 6, _car.Position_X, _car.Position_Y)
    flags = 0
    if task._target_obj_world_x < 900.0 and task._target_obj_world_y < 900.0:
        flags |= 1
        _pack_s12pair(f, 9, task._target_obj_world_x, task._target_obj_world_y)
    else:
        _pack_s12pair(f, 9, 0.0, 0.0)
    if _cam_cone_seen:
        flags |= 2
        _pack_s12pair(f, 12, _cam_cone_x, _cam_cone_y)
    else:
        _pack_s12pair(f, 12, 0.0, 0.0)
    if _cam_brick_seen:
        flags |= 4
        _pack_s12pair(f, 33, _cam_brick_x, _cam_brick_y)
    else:
        _pack_s12pair(f, 33, 0.0, 0.0)
    if task._wireless_car_seen:
        flags |= 8
    # flags.bit4~5：3 表示本轮 SEARCH 持续保持 RECOVER 锁存的侧阵列；
    # 离开 SEARCH 后恢复为 0。旧的“第一次 Forward 后回后方阵列”流程已取消。
    # 旧的左右方向编码不再使用。从车回包和非 SEARCH 模式固定发 0。
    if config._IS_LEADER and task._task_mode == _MODE_SEARCH:
        flags |= (task._search_first_sweep_mode_to_other & 3) << 4
    f[15] = flags
    f[16] = (task._self_mode & 31) << 3 | task._self_ready & 7
    # SEARCH（尤其从车）使用独立搜索子状态并映射到 _self_sub；其他模式直接
    # 发送 _mode_sub，避免 RECOVER 等阶段向对方持续广播上一个模式的旧子状态。
    wireless_sub = task._self_sub if task._task_mode == _MODE_SEARCH else task._mode_sub
    f[17] = wireless_sub & 255
    f[18] = (task._target_edge & 15) << 4 | task._target_obj_id & 15
    f[19] = (cmd_seq & 31) << 3 | cmd_sub & 7
    _pack_i16(f, 20, approach_cmd)
    _pack_i16(f, 22, task._route_cmd_to_other)
    _pack_i16(f, 24, yaw_dir_cmd)
    _pack_i16(f, 26, task._route_obs_axis_to_other)
    _pack_i16(f, 28, _target_world_vx)
    _pack_i16(f, 30, _target_world_vy)
    _pack_s8(f, 32, side_cmd * 2.0)
    f[36] = _crc8(f, 2, _FRAME3_LEN - 1)
    _uart_wireless.write(f)
    _wireless_last_tx_ms = now_ms
    if config._IS_LEADER:
        _wireless_wait_reply = 1

# 功能：通信定时器回调，周期性读取摄像头和无线数据，并更新无线发送时序。
# 通信 ISR 周期（ms）与相机解析分频。必须定义在 comm_pit_handler / pit_start 之前：
# MicroPython 里 _NAME = const(...) 是编译期常量，不生成运行期全局，定义之前引用会 NameError。
_COMM_PIT_MS = const(3)

_COMM_CAM_DIV = const(3)

# 无线在 ISR 里的分频计数。pit3 提到 3ms 是为了压缩半双工的转向时间，
# 相机解析仍按每 _COMM_CAM_DIV 次(≈9ms)执行一次，负载和原来 10ms 基本持平。
_comm_tick = 0

def comm_pit_handler(time):
    global _comm_tick, _wireless_send_flag, _all_send_flag, _send_flag
    # 无线：收 -> 调度 -> 发，全部在本 ISR 内闭环。
    # 原来"收在 10ms ISR、发在主循环"，半双工下这两段延迟直接串进转向时间：
    #   对方发完 -> 等本车 10ms ISR 解析 -> 等保护间隔 -> 等主循环发送(0~T_loop)
    # 现在压成 0~3ms + 保护间隔，转向时间不再受主循环耗时影响。
    uart_wireless_read()
    uart_wireless_update_tx()
    # 半双工守卫下沉到发送点：正等回包(_wireless_wait_reply=1)时不发，标志暂存待安全后补发。
    # 拦住 wireless_send_now() 在主循环里"读守卫→置标志"非原子引入的插队竞态。
    # 从车 _wireless_wait_reply 恒为 0，不影响其回包。
    if _wireless_send_flag == 1 and _wireless_wait_reply == 0:
        uart_send_pose_wireless()
        _wireless_send_flag = 0
    _comm_tick += 1
    if _comm_tick >= _COMM_CAM_DIV:
        _comm_tick = 0
        uart_get_frame_v2()
    elif _all_send_flag == 1:
        # Frame1 也在本 ISR 内发送。原来 write 在主循环里，实际周期被
        # task.update()+LCD+gc.collect() 拉长到 15~40ms，OpenART 拿到的位姿就旧一拍，
        # 而它要用这个位姿做 rel->world 换算和黄线的 wall 判定，位姿旧则结果直接偏。
        # 下沉后周期回到 10ms + 0~6ms。
        # 只在非相机解析拍发送：避开"解析 + 两次串口写"叠加在同一个 3ms ISR 里。
        uart_send_pose()
        _send_flag = 0
        _all_send_flag = 0

# 功能：无线命令 TTL 检查。链路超过 _CMD_TTL_MS 没收到对方帧时，
# 清除对方下发的命令类字段，防止旧命令在无线短丢后继续触发动作。
def _cmd_ttl_check():
    global _push_world_side_cmd
    global _Other_Approach_Cmd, _Other_Follower_Cmd_Yaw_Dir, _Other_Master_Cmd_Sub, _Other_Push_Side_Cmd, _Other_Route_Cmd
    global _Other_Search_First_Sweep_Mode
    global _Other_World_Vx, _Other_World_Vy
    if _wireless_rx_last_ms == 0:
        return
    if ticks_diff(ticks_ms(), _wireless_rx_last_ms) <= _CMD_TTL_MS:
        return
    _Other_Master_Cmd_Sub = 0
    _Other_Approach_Cmd = 999.9
    _Other_Route_Cmd = 999.9
    _Other_Follower_Cmd_Yaw_Dir = 999.9
    _Other_Search_First_Sweep_Mode = 0
    _Other_Push_Side_Cmd = 0.0
    _Other_World_Vx = 999.0
    _Other_World_Vy = 999.0
    if not config._IS_LEADER:
        _push_world_side_cmd = 0.0

# 功能：主循环侧的通信维护。Frame1 的实际发送已下沉到 comm_pit_handler（3ms ISR），
# 这里只保留不适合放进 ISR 的无线命令 TTL 检查。
def comm_update():
    _cmd_ttl_check()

# 功能：关键事件即时发起一次无线发送（仅主车），不必等 _WIRELESS_FLAG_PERIOD_TICK 节拍。
# 用于 push GO 等对时序敏感的瞬间，把跨车传播延迟从"最多一个轮询周期"压到"最多一个 3ms ISR"。
# 半双工保护：正在等回包(_wireless_wait_reply)时不插入，交给下一拍常规轮询。
def wireless_send_now():
    global _wireless_send_flag
    if _uart_wireless is None:
        return
    if config._IS_LEADER and _wireless_wait_reply == 0:
        _wireless_send_flag = 1

_pit1 = None

_pit2 = None

_pit3 = None

# 定时调度分频参数，单位为 ticker1 的计数周期。
# 通信、角度环、角速度环、电机环和位置环分别独立分频。
_COMM_FLAG_PERIOD_TICK = const(10)

# Frame1 置位相位。发送已下沉到 3ms 通信 ISR，若与无线轮询同相，
# 同一拍里就会叠加 37 字节(1.6ms) + 17 字节(0.74ms) 两次阻塞写，逼近 3ms ISR 预算。
# 错开半个周期后两者稳定相隔 5ms。
_COMM_FLAG_PHASE_TICK = const(5)

# 无线轮询节拍（ms，ticker1=1ms）。与相机发送 _COMM_FLAG_PERIOD_TICK 拆开，可独立调。
_WIRELESS_FLAG_PERIOD_TICK = const(10)


_ANGLE_PID_PERIOD_TICK = const(10)

_GYRO_PID_PERIOD_TICK = const(5)

_MOTOR_CONTROL_PERIOD_TICK = const(2)

_POSITION_PID_PERIOD_TICK = const(20)

# 功能：高速控制定时器回调，调度通信发送标志、角度环、运动状态机、速度坐标转换和电机控制。
# 该函数会周期性更新目标速度、目标角速度和电机 PWM。
def time_pit_handler1(time):
    global _all_send_flag, _send_flag, _t, _ticker_count, _wireless_send_flag
    _ticker_count = _ticker_count + 1 if _ticker_count < 1000 else 1
    _t += 1
    if _ticker_count % _COMM_FLAG_PERIOD_TICK == _COMM_FLAG_PHASE_TICK:
        if _send_flag == 1:
            _all_send_flag = 1
            _send_flag = 0
    # 无线轮询与相机发送拆开，主车按 _WIRELESS_FLAG_PERIOD_TICK 独立定期发起。
    if config._IS_LEADER and _ticker_count % _WIRELESS_FLAG_PERIOD_TICK == 0:
        if _wireless_wait_reply == 0:
            _wireless_send_flag = 1
    if _ticker_count % _ANGLE_PID_PERIOD_TICK == 0:
        if _yaw_rate_active:
            _car.target_GYRO_Z = _yaw_rate_target_raw
        else:
            _car.target_GYRO_Z = angle_pid(_car.target_angle, _car.current_angle)
            #_car.target_GYRO_Z = angle_pid(90, _car.current_angle)
    if _ticker_count % _GYRO_PID_PERIOD_TICK == 0:
        motion_step()
    if _ticker_count % _POSITION_PID_PERIOD_TICK == 0:
        if _ctrl_mode == _CTRL_POS:
            position_pid_step()
    if _ticker_count % _GYRO_PID_PERIOD_TICK == 0:
        if _face_obj_active:
            _car.target_z = _face_obj_gyro_cmd
        else:
            _car.target_z = gyro_z_pid(_car.target_GYRO_Z, _IMU1_Data._gyro_z)
            #_car.target_z = gyro_z_pid(20000, _IMU1_Data._gyro_z)
        if _ctrl_mode == _CTRL_VEL:
            world_vel_step()
        car_omni(_car.target_Speed_X, _car.target_Speed_Y, -_car.target_z)
        #car_omni(-100, 100, -_car.target_z)
    if _ticker_count % _MOTOR_CONTROL_PERIOD_TICK == 0:
        motor_control(1)

# 功能：传感器定时器回调，更新 IMU 姿态、融合视觉位置修正、读取编码器并触发通信发送。
# 结果写入当前角度、位置、编码器状态和 _send_flag。
def time_pit_handler2(time):
    global _Position_X_encoder, _Position_Y_encoder, _YawAngle_Trans, _YawAngle_imu, _angle_update_flag, _fix_beep_active, _fix_beep_until_ms, _pos_fix_req, _pos_fix_x_valid, _pos_fix_y_valid, _position_update_flag, _send_flag
    Gyro_Get_All_Angles()
    _YawAngle_imu = _YawAngle_Trans
    if _vis_yaw_fix_en and _YawAngle_fix < 361 and _angle_update_flag and ((_YawAngle_imu - _YawAngle_fix + 180) % 360 - 180 < 60):
        _car.current_angle = _YawAngle_fix
        _YawAngle_Trans = _car.current_angle
        _angle_update_flag = 0
    else:
        _car.current_angle = _YawAngle_imu
    if _position_update_flag == 1:
        fixed_pos = False
        if _vis_x_fix_en and _vis_pos_fix_value_ok(_Position_X_fix, _VIS_FIX_FIELD_W):
            _car.Position_X = _Position_X_fix
            _Position_X_encoder = _Position_X_fix
            fixed_pos = True
        if _vis_y_fix_en and _vis_pos_fix_value_ok(_Position_Y_fix, _VIS_FIX_FIELD_H):
            _car.Position_Y = _Position_Y_fix
            _Position_Y_encoder = _Position_Y_fix
            fixed_pos = True
        if fixed_pos:
            _fix_beep_active = 1
            _fix_beep_until_ms = ticks_add(ticks_ms(), 50)
        _position_update_flag = 0
    else:
        _car.Position_X = _Position_X_encoder
        _car.Position_Y = _Position_Y_encoder
    if _car.current_angle > 360:
        _car.current_angle -= 360
    elif _car.current_angle < 0:
        _car.current_angle += 360
    _pit2.capture_list(_encoder_1, _encoder_2, _encoder_3, _encoder_4)
    encoder_get()
    if _pos_fix_req:
        if _pos_fix_x_valid:
            _car.Position_X = _pos_fix_x
            _Position_X_encoder = _pos_fix_x
            _pos_fix_x_valid = False
        if _pos_fix_y_valid:
            _car.Position_Y = _pos_fix_y
            _Position_Y_encoder = _pos_fix_y
            _pos_fix_y_valid = False
        _pos_fix_req = False
    _send_flag = 1

# 功能：启动所有周期中断，先校准陀螺仪，再分别启动控制、传感器和通信 ticker。
def pit_start():
    global _pit1, _pit2, _pit3
    calibrate_gyro()
    _pit1 = ticker(1)
    _pit2 = ticker(2)
    _pit3 = ticker(3)
    _pit1.callback(time_pit_handler1)
    _pit2.callback(time_pit_handler2)
    _pit3.callback(comm_pit_handler)
    _pit1.start(1)
    _pit2.start(10)
    _pit3.start(_COMM_PIT_MS)

# LCD 调试刷新参数。刷新过快会占用主循环时间，所以默认 500 ms 更新一次。
_debug_view__last_ms = 0

_LCD_REFRESH_MS = const(500)

_LCD_MAX_CHARS = const(22)

_LCD_BLANK = const('                      ')
# 功能：刷新 LCD 默认调试界面，显示主从状态、位姿、通信新鲜度和关键视觉结果。
# 第96/112像素行显示 cone/brick，第144/160像素行显示紫球/橙球的车体系相对坐标。
# 输入参数：now_ms 为当前时间戳，单位 ms；为 None 时函数内部读取 ticks_ms()。
# 若 LCD 未初始化或距离上次刷新不足间隔，则直接返回。
def debug_view_update(now_ms=None):
    global _lcd
    global _debug_view__last_ms
    if _lcd is None:
        return
    if now_ms is None:
        now_ms = ticks_ms()
    if ticks_diff(now_ms, _debug_view__last_ms) < _LCD_REFRESH_MS:
        return
    _debug_view__last_ms = now_ms
    # SEARCH 在从车侧使用独立的 _fsearch_state，并通过 _self_sub 对外广播；
    # 其他模式直接显示 _mode_sub，确保 RECOVER 等阶段切换子状态后 LCD 不显示旧值。
    local_sub = task._self_sub if task._task_mode == _MODE_SEARCH else task._mode_sub
    s = 'M:%d S:%d oM:%d oS:%d' % (task._task_mode, local_sub, _Other_Car_Mode, _Other_Car_Push_Sub)
    _lcd.str12(0, 0, _LCD_BLANK)
    _lcd.str12(0, 0, s[:_LCD_MAX_CHARS])
    # 默认页只保留当前 IMU/融合航向；黄线视觉航向留给专项调试。
    s = 'I:%.1f' % _car.current_angle
    _lcd.str12(0, 16, _LCD_BLANK)
    _lcd.str12(0, 16, s)
    # 累计字节数不直观，改为显示距离最近一帧的实时年龄；9999+ 表示超过 10 秒。
    if _cam_rx_last_ms == 0:
        cam_age_text = '--'
    else:
        cam_age = ticks_diff(now_ms, _cam_rx_last_ms)
        cam_age_text = '9999+' if cam_age > 9999 else str(max(0, cam_age))
    # a=距最近一帧的年龄，m=开机以来最大帧间隔，f=CRC校验失败丢帧累计，L=主循环最大耗时。
    # 逐帧抖动的即时间隔（原 i 字段）参考价值不大且屏幕字符数已顶格，改成显示
    # CRC 丢帧计数：m 大但 f 不涨，说明是感知/发送慢；f 持续涨，说明链路在丢坏帧。
    s = 'C%s m%d f%d L%d' % (cam_age_text, _cam_intv_max_ms, _cam_crc_fail_cnt, _loop_max_ms)
    _lcd.str12(0, 32, _LCD_BLANK)
    _lcd.str12(0, 32, s[:_LCD_MAX_CHARS])
    if _wireless_rx_last_ms == 0:
        wire_age_text = '--'
    else:
        wire_age = ticks_diff(now_ms, _wireless_rx_last_ms)
        wire_age_text = '9999+' if wire_age > 9999 else str(max(0, wire_age))
    s = 'W a:%s r:%d s:%d' % (wire_age_text, _Other_Car_Ready, 1 if _Other_Car_Seen_Me else 0)
    _lcd.str12(0, 48, _LCD_BLANK)
    _lcd.str12(0, 48, s[:_LCD_MAX_CHARS])
    # 默认页仅保留目标类别 ID。
    s = 'T:%d' % task._target_obj_id
    _lcd.str12(0, 64, _LCD_BLANK)
    _lcd.str12(0, 64, s[:_LCD_MAX_CHARS])
    # 普通物体检测只保留槽位 O0，O1 不在默认页显示。
    s = 'O0:%d rx:%.0f ry:%.0f' % (_cam_obj_id[0], _cam_obj_rel_x[0], _cam_obj_rel_y[0])
    _lcd.str12(0, 80, _LCD_BLANK)
    _lcd.str12(0, 80, s[:_LCD_MAX_CHARS])
    _lcd.str12(0, 96, _LCD_BLANK)
    _lcd.str12(0, 112, _LCD_BLANK)
    if _cam_cone_seen:
        s = 'C rx:%.0f ry:%.0f' % (_cam_cone_rel_x, _cam_cone_rel_y)
    else:
        s = 'C rx:-- ry:--'
    _lcd.str12(0, 96, s[:_LCD_MAX_CHARS])
    if _cam_brick_seen:
        s = 'B rx:%.0f ry:%.0f' % (_cam_brick_rel_x, _cam_brick_rel_y)
    else:
        s = 'B rx:-- ry:--'
    _lcd.str12(0, 112, s[:_LCD_MAX_CHARS])
# 第一行优先显示主/子状态；本车世界坐标移到末行，保留原有诊断信息。
    _lcd.str12(0, 128, _LCD_BLANK)
    s = 'x:%.0f y:%.0f' % (_car.Position_X, _car.Position_Y)
    _lcd.str12(0, 128, s[:_LCD_MAX_CHARS])
    _lcd.str12(0, 144, _LCD_BLANK)
    if _cam_ball_rel_x[0] < 900.0 and _cam_ball_rel_y[0] < 900.0:
        s = 'BL rx:%.0f ry:%.0f' % (_cam_ball_rel_x[0], _cam_ball_rel_y[0])
    else:
        s = 'BL rx:-- ry:--'
    _lcd.str12(0, 144, s[:_LCD_MAX_CHARS])
    _lcd.str12(0, 160, _LCD_BLANK)
    if _cam_ball_rel_x[1] < 900.0 and _cam_ball_rel_y[1] < 900.0:
        s = 'BR rx:%.0f ry:%.0f' % (_cam_ball_rel_x[1], _cam_ball_rel_y[1])
    else:
        s = 'BR rx:-- ry:--'
    _lcd.str12(0, 160, s[:_LCD_MAX_CHARS])

import task

micropython.alloc_emergency_exception_buf(100)

# 功能：系统主初始化入口，完成垃圾回收、硬件初始化、状态初始化、参数应用和定时器启动。
def main_init():
    gc.collect()
    gc.collect()
    hw_init()
    gc.collect()
    gc.collect()
    gc.collect()
    gc.collect()
    gc.collect()
    gc.collect()
    gc.collect()
    gc.collect()
    gc.collect()
    gc.collect()
    state_init()
    apply_config()
    gc.collect()
    gc.collect()
    pit_start()
    gc.collect()

_ERR_PRINT_MS = const(1000)

_last_err_print_ms = 0

# 功能：根据视觉位置修正状态控制蜂鸣器短响提示。
def _update_fix_beep(now_ms):
    global _fix_beep_active
    if _beep is None:
        return
    # 避障蜂鸣优先：避障期间 _fix_beep 让位，避免视觉修正的滴滴声盖过避障提示。
    if _avoid_beep_kind != 0:
        _update_avoid_beep(now_ms)
        return
    if _fix_beep_active:
        if ticks_diff(now_ms, _fix_beep_until_ms) < 0:
            _beep.high()
        else:
            _beep.low()
            _fix_beep_active = 0
    else:
        _beep.low()

_AVOID_BEEP_ON_MS = const(120)
_AVOID_BEEP_OFF_MS = const(120)
_AVOID_BEEP_GAP_MS = const(500)

# 功能：请求避障蜂鸣。kind 1=单声，2=双声，3=持续高电平。
# kind 1/2 可只调用一次形成短提示；kind 3 必须持续刷新，停止刷新后由 200ms 看门狗关断。
def avoid_beep(kind):
    global _avoid_beep_kind, _avoid_beep_pattern_t0, _avoid_beep_last
    if kind != _avoid_beep_kind:
        _avoid_beep_pattern_t0 = ticks_ms()
    _avoid_beep_kind = kind
    _avoid_beep_last = ticks_ms()

# 功能：按 kind 驱动避障蜂鸣。kind 1/2 按响数输出节奏，kind 3 持续高电平；
# 触发超 200ms 未刷新即判避障结束停响，避免异常状态把蜂鸣器永久卡高。
def _update_avoid_beep(now_ms):
    global _avoid_beep_kind
    if ticks_diff(now_ms, _avoid_beep_last) > 200:
        _avoid_beep_kind = 0
        _beep.low()
        return
    if _avoid_beep_kind == 3:
        _beep.high()
        return
    beeps = _avoid_beep_kind
    period = beeps * (_AVOID_BEEP_ON_MS + _AVOID_BEEP_OFF_MS) + _AVOID_BEEP_GAP_MS
    phase = ticks_diff(now_ms, _avoid_beep_pattern_t0) % period
    on = False
    for k in range(beeps):
        start = k * (_AVOID_BEEP_ON_MS + _AVOID_BEEP_OFF_MS)
        if start <= phase < start + _AVOID_BEEP_ON_MS:
            on = True
            break
    if on:
        _beep.high()
    else:
        _beep.low()

class _ExcCap:

    # 功能：创建一个简易异常文本捕获对象，用于接收 sys.print_exception 输出。
    # 输入参数：self 为对象自身；该初始化函数没有外部输入参数。
    # 返回值：无。异常文本会保存在 s 属性中。
    def __init__(self):
        self.s = ''

    # 功能：接收异常打印片段，并只保留最后 256 个字符，避免占用过多内存。
    # 输入参数：self 为对象自身；t 为本次写入的文本片段。
    # 返回值：无。结果追加到 s 属性中。
    def write(self, t):
        self.s = (self.s + t)[-256:]

# 功能：统一处理主循环异常，停车、蜂鸣报警、LCD 显示错误类型并限频打印异常详情。
# 输入参数：e 为捕获到的异常对象。
# 函数尽量捕获自身内部异常，避免错误处理再次导致死循环崩溃。
def _show_error(e):
    global _ctrl_mode, _target_world_vx, _target_world_vy
    global _last_err_print_ms
    name = type(e).__name__
    clear_face()
    _ctrl_mode = _CTRL_VEL
    _target_world_vx = 0.0
    _target_world_vy = 0.0
    _car.target_Speed_X = 0.0
    _car.target_Speed_Y = 0.0
    _car.target_z = 0.0
    try:
        if True and _beep is not None:
            for unused in range(3):
                _beep.high()
                sleep_ms(60)
                _beep.low()
                sleep_ms(60)
    except Exception:
        pass
    detail = ''
    try:
        import sys
        cap = _ExcCap()
        sys.print_exception(e, cap)
        detail = cap.s
    except Exception:
        try:
            detail = str(e)
        except Exception:
            detail = ''
    try:
        if _lcd is not None:
            _lcd.str12(0, 192, ('ERR:' + name)[:22])
    except Exception:
        pass
    try:
        print('ERR:', name)
        now = ticks_ms()
        if _last_err_print_ms == 0 or ticks_diff(now, _last_err_print_ms) >= _ERR_PRINT_MS:
            _last_err_print_ms = now
            if detail:
                print(detail)
    except Exception:
        pass

# 功能：执行一次主循环任务，包括蜂鸣器更新、通信发送、任务状态机更新、LCD 刷新和耗时统计。
# 若 task.update() 抛出异常，会由 loop() 捕获。
def _loop_step():
    global _loop_max_ms
    t0 = ticks_ms()
    _update_fix_beep(t0)
    comm_update()
    t1 = ticks_ms()
    task.update()
    t2 = ticks_ms()
    if True:
        debug_view_update(t0)
    t3 = ticks_ms()
    dt = ticks_diff(ticks_ms(), t0)
    if dt > _loop_max_ms:
        _loop_max_ms = dt
    gc.collect()

# 功能：程序主循环，持续调用 _loop_step，并在发生异常时进入统一错误处理。
def loop():
    while True:
        try:
            _loop_step()
        except Exception as e:
            _show_error(e)
