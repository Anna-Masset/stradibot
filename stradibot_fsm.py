"""
Stradibot FSM — Python state machine for bowing control.
Runs alongside stradibot_simviz.xml (OpenSai built-in controller).

Run:
    python3 stradibot_fsm.py

States:
    CALIBRATING  → robot floats freely (gravity comp only), user registers string positions
    HOMING       → move to safe home position
    CONTACTING   → orient bow to string, approach along string normal until force detected
    BOWING       → back-and-forth bowing with force control
"""

import numpy as np
import time
import json
import redis
import threading
import queue
from enum import Enum, auto
from dataclasses import dataclass

# ============================================================
# CONFIG
# ============================================================

CONTROLLER     = "cartesian_controller"
LOOP_DT        = 0.01   # 100 Hz    

TARGET_STRING  = 1       # which string to bow (0=G, 1=D, 2=A, 3=E)

MOVING_SPEED            = 0.05    # m/s — how fast to move when not contacting
CONTACT_APPROACH_SPEED  = 0.03   # m/s — how fast to creep toward the string
ANGULAR_SPEED           = np.pi/8    # rad/s — for orientation corrections during bowing
SAFETY_SPEED            = 0.04
CONTACT_FORCE_THRESHOLD = 1.2    # N   — force magnitude to detect contact
BOW_SPEED               = 0.1   # m/s
# BOW_SPEED               = 0.01
BOW_AMPLITUDE           = 0.07   # m   — half-stroke
# DESIRED_BOW_FORCE       = 0.7    # N   — normal force on string
DESIRED_BOW_FORCE       = 1.3
DESIRED_BOW_MOMENT       = 0.2
# DESIRED_BOW_MOMENT       = 0.14
BOW_OFFSET              = 0.35   # m   — offset along bow direction from string position
LIFT_HEIGHT             = 0.03   # m   — how far to lift along string normal when switching
MOVE_THRESHOLD          = 0.15   # m   — lateral distance along bow dir to stop above new string

CHEAT_FORCE_MIN               = 0.8
CHEAT_FORCE_MAX               = 1.2
CHEAT_SETTLE_TIME              = 0.1
CHEAT_POS_STEP                = 0.0003  # m per loop — bow goal nudge along str_normal (~5 cm/s @ 100 Hz)

IS_REAL = True
CALIBRATION = True

if IS_REAL:
    # CONFIG_FILE = "stradibot.xml"
    CONFIG_FILE = "enzo_test.xml"
else:
    CONFIG_FILE = "stradibot_simviz.xml"


# ============================================================
# REDIS KEYS
# ============================================================

if IS_REAL:
    robot_name = "Titania"
else:
    robot_name = "flexiv"   

@dataclass
class RedisKeys:
    goal_position:       str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::goal_position"
    goal_orientation:    str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::goal_orientation"
    current_position:    str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::current_position"
    current_orientation: str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::current_orientation"
    active_controller:   str = f"opensai::controllers::{robot_name}::active_controller_name"
    config_file:         str = f"::sai-interfaces-webui::config_file_name"
    # force sensor (local frame, 3D vector)
    joint_task_kp:       str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::kp"
    if IS_REAL:
        # ft_force:            str = "opensai::sensors::Titania::ft_sensor::tcp_force"
        ft_force:            str = "opensai::sensors::Titania::ft_sensor::flange::force"

    else:
        ft_force:            str = f"opensai::sensors::{robot_name}::ft_sensor::bow::force"

    # force/moment control keys
    force_space_dim:         str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::force_space_dimension"
    force_space_axis:        str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::force_space_axis"
    desired_force:           str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::desired_force"
    desired_moment:           str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::desired_moment"
    moment_space_dim:        str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::moment_space_dimension"
    moment_space_axis:        str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::moment_space_axis"
    linear_vel_sat_limit:    str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::otg_max_linear_velocity"
    angular_vel_sat_limit:   str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::otg_max_angular_velocity"
    vel_sat_enabled:         str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::velocity_saturation_enabled"
    closed_loop:             str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::closed_loop_force_control"

KEYS = RedisKeys()

def set_linear_vel_limit(r, limit):
    r.set(KEYS.linear_vel_sat_limit, limit)


# ============================================================
# STRING GEOMETRY  (from controller.cpp calibration)
# Positions and orientations of each string in world frame.
# ============================================================

if not CALIBRATION:
    STRING_POSITIONS = [
        np.array([0.867011, -0.140365, 0.266309]),   # string 0 (G)
        np.array([0.907412, -0.119718, 0.279737]),   # string 1 (D)
        np.array([0.924652, -0.117822, 0.274284]),   # string 2 (A)
        np.array([0.945377, -0.116711, 0.255744]),   # string 3 (E)
    ]

    STRING_ORIENTATIONS = [
        np.array([[ 0.0934987, -0.511516, -0.854171],
                [-0.994479,  -0.00692422, -0.10471],
                [ 0.0476466,  0.859246,  -0.50934 ]]),  # string 0
        np.array([[ 0.0374749, -0.207924, -0.977427],
                [-0.998624,   0.0281047, -0.0442662],
                [ 0.0366743,  0.977741,  -0.206585]]),  # string 1
        np.array([[-0.0108152,  0.0953699, -0.995383],
                [-0.999024,   0.0416067,  0.0148412],
                [ 0.0428301,  0.994572,   0.0948269]]),  # string 2
        np.array([[-0.063477,   0.480517,  -0.874685],
                [-0.99537,    0.0329028,  0.0903107],
                [ 0.0721755,  0.876368,   0.476204 ]]),  # string 3
    ]

else:
    STRING_POSITIONS = [0.0, 0.0, 0.0, 0.0]
    STRING_ORIENTATIONS = [np.eye(3), np.eye(3), np.eye(3), np.eye(3)]

# ============================================================
# STATES
# ============================================================

class State(Enum):
    CALIBRATING = auto()
    HOMING      = auto()
    PLACING     = auto()
    CONTACTING  = auto()
    CHEATING    = auto()
    BOWING      = auto()
    LIFTING     = auto()
    MOVING      = auto()

# ============================================================
# HELPERS
# ============================================================

def get_pos(r):
    return np.array(json.loads(r.get(KEYS.current_position)))

def get_ori(r):
    return np.array(json.loads(r.get(KEYS.current_orientation)))

def get_force(r):
    """Returns 3D force vector in sensor local frame."""
    return np.array(json.loads(r.get(KEYS.ft_force)))

def set_goal(r, pos, ori):
    r.set(KEYS.goal_position,    json.dumps(pos.tolist()))
    r.set(KEYS.goal_orientation, json.dumps(ori.tolist()))

def set_floating(r):
    r.set(KEYS.active_controller, "joint_controller")
    r.set(KEYS.joint_task_kp, json.dumps([0.0] * 7))

def set_position_control(r):
    """Back to full position control (0 force/moment DOFs)."""
    r.set(KEYS.force_space_dim,  "0")
    r.set(KEYS.moment_space_dim, "0")

def pos_err(target, current):
    return np.linalg.norm(target - current)

def ori_err(target, current):
    return np.linalg.norm(target - current)

def apply_string_geometry(raw_pos, ori):
    """Given a string position and orientation (already in correct frame),
    return (str_pos with bow offset, str_normal, str_bow_dir)."""
    normal  = ori[:, 2]
    bow_dir = ori[:, 0]
    pos = raw_pos + BOW_OFFSET * bow_dir
    return pos, normal, bow_dir

def homing_safety(r, home_pos, home_ori):
    r.set(KEYS.active_controller, "cartesian_controller")
    r.set(KEYS.closed_loop,      "0")
    set_position_control(r)
    time.sleep(0.1)  # wait for controller to switch and update readings
    set_linear_vel_limit(r, SAFETY_SPEED)
    set_goal(r, home_pos, home_ori)
    print(f"\nState: HOMING SAFETY")

# Single keyboard thread — all input goes into key_queue
key_queue = queue.Queue()

def _key_input_thread():
    while True:
        key_queue.put(input())

# ============================================================
# MAIN
# ============================================================

def main():
    global TARGET_STRING
    r = redis.Redis()

    # Verify correct config file
    cfg = r.get(KEYS.config_file)
    if cfg is None or cfg.decode() != CONFIG_FILE:
        print(f"Expected config '{CONFIG_FILE}', got '{cfg}'. Is the sim running?")
        return

    # Activate cartesian controller
    while r.get(KEYS.active_controller).decode() != CONTROLLER:
        r.set(KEYS.active_controller, CONTROLLER)
    print(f"Controller: {CONTROLLER}")

    # Read startup pose — used as home
    home_pos = get_pos(r)
    home_ori = get_ori(r)
    print(f"Home position: {home_pos}")

    # Build cal_positions / cal_orientations with BOW_OFFSET already baked in.
    # For sim: hardcoded values are in bow frame → transform to flange frame first.
    if not CALIBRATION:
        R_bow_in_flange = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        cal_orientations = [o @ R_bow_in_flange.T for o in STRING_ORIENTATIONS] 
    else:
        cal_orientations = list(STRING_ORIENTATIONS)
    
    cal_positions = [
        p + BOW_OFFSET * cal_orientations[i][:, 0]
        for i, p in enumerate(STRING_POSITIONS)
    ]  

    # Active string geometry (updated on calibration or string switch)
    str_ori     = cal_orientations[TARGET_STRING]
    str_pos     = cal_positions[TARGET_STRING]
    str_normal  = str_ori[:, 2]
    str_bow_dir = str_ori[:, 0]

    # ── State machine variables ──────────────────────────────
    state            = State.CALIBRATING if CALIBRATION else State.HOMING
    contact_goal_pos = home_pos.copy()
    bow_displacement = 0.0
    bow_dir          = 1.0

    # Start single keyboard thread
    threading.Thread(target=_key_input_thread, daemon=True).start()

    print(f"\nState: CALIBRATING  (floating — press 1/2/3/4 to register strings, Enter to home)")
    print(f"Target string: {TARGET_STRING}")

    loop_time  = 0.0
    init_time  = time.perf_counter_ns() * 1e-9
    last_print = 0.0
    time.sleep(0.01)

    # enable velocity saturation
    set_linear_vel_limit(r, MOVING_SPEED)
    r.set(KEYS.angular_vel_sat_limit, str(ANGULAR_SPEED))

    if CALIBRATION:
        set_floating(r)
    else:
        set_position_control(r)
    # set_goal(r, np.array([ 0.940193, -0.078024,  0.310439]), home_ori)

    try:
        while True:
            loop_time += LOOP_DT
            time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))

            cur_pos = get_pos(r)
            cur_ori = get_ori(r)

            # during first state, get initial force sensor readings to find bias and remove later on (TODO)

            # ── CALIBRATING ──────────────────────────────────────────────
            if state == State.CALIBRATING:
                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key in ('1', '2', '3', '4'):
                        r.set(KEYS.active_controller, "cartesian_controller")
                        time.sleep(0.1)
                        idx = int(key) - 1
                        raw_ori = get_ori(r).copy()
                        raw_pos = get_pos(r).copy()
                        cal_orientations[idx] = raw_ori
                        cal_positions[idx]    = raw_pos 
                        print(f"  String {key} registered: pos={cal_positions[idx].round(4)}")
                        time.sleep(0.1)
                        set_floating(r)
                    elif key == '':  # bare Enter → done
                        print("\nCalibrated positions:")
                        for i, p in enumerate(cal_positions):
                            print(f"  String {i+1}: {p.round(4)}")
                        str_ori     = cal_orientations[TARGET_STRING]
                        str_pos     = cal_positions[TARGET_STRING]
                        str_normal  = str_ori[:, 2]
                        str_moment  = str_ori[:, 1]
                        str_bow_dir = str_ori[:, 0]
                        r.set(KEYS.active_controller, "cartesian_controller")                    
                        time.sleep(0.1)  # wait for controller to switch and update readings
                        # r.set(KEYS.moment_space_dim, "1")
                        # r.set(KEYS.moment_space_axis, json.dumps(np.array([1, 0, 0]).tolist()))
                        # r.set(KEYS.desired_moment,    json.dumps((DESIRED_BOW_MOMENT * np.array([1, 0, 0])).tolist()))
                        # time.sleep(0.1)
                        set_goal(r, home_pos, home_ori)
                        print(f"\nState: HOMING")
                        state = State.HOMING
                        continue


            # ── HOMING ───────────────────────────────────────────────────
            elif state == State.HOMING:
                p_err = pos_err(home_pos, cur_pos)
                o_err = ori_err(home_ori, cur_ori)
                if loop_time - last_print > 1.0:
                    print(f"  HOMING  pos_err={p_err:.4f}  ori_err={o_err:.4f}")
                    last_print = loop_time

                if not key_queue.empty() and key_queue.get().strip() == '':
                    contact_goal_pos = str_pos - 0.05 * str_normal
                    print(f"\nState: PLACING  (approaching string {TARGET_STRING})")
                    set_goal(r, contact_goal_pos, str_ori)
                    state = State.PLACING

            # ── PLACING ───────────────────────────────────────────────
            elif state == State.PLACING:

                p_err = pos_err(contact_goal_pos, cur_pos)
                o_err = ori_err(str_ori, cur_ori)

                if loop_time - last_print > 1.0:
                    print(f"  PLACING  pos_err={p_err:.4f}  ori_err={o_err:.4f}")
                    last_print = loop_time

                if p_err < 0.02 and o_err < 0.1:
                    # creep toward string along normal
                    contact_goal_pos = cur_pos + 0.1 * str_normal
                    set_linear_vel_limit(r, CONTACT_APPROACH_SPEED)
                    set_goal(r, contact_goal_pos, str_ori)
                    state = State.CONTACTING


            # ── CONTACTING ─────────────────────────────────────────────
            elif state == State.CONTACTING:
                # move goal toward string along normal at constant speed

                force_local = get_force(r)
                force_world = cur_ori @ force_local          # rotate to world frame
                force_normal = np.dot(force_world, str_normal)  # scalar projection onto normal
                force_normal_mag = np.linalg.norm(force_normal)

                if loop_time - last_print > 0.5:
                    print(f"  CONTACTING  goal={contact_goal_pos.round(4)}  |F|={force_normal_mag:.3f} N")
                    last_print = loop_time

                if force_normal_mag > CONTACT_FORCE_THRESHOLD:
                    print(f"\nState: CHEATING  (contact detected |F|={force_normal_mag:.3f} N)")
                    bow_dir = 1.0 ## starting to the left 
                    DESIRED_BOW_MOMENT = 0.1

                    # Pure position control — we'll nudge cheat_goal_pos along str_normal based on force feedback
                    # set_position_control(r)
                    # cheat_settle_start = None
                    # cheat_goal_pos = cur_pos.copy()
                    # set_goal(r, cheat_goal_pos, str_ori)
                    # state = State.CHEATING

                    # r.set(KEYS.force_space_dim,  "1")
                    # r.set(KEYS.force_space_axis, json.dumps(str_normal.tolist()))
                    # r.set(KEYS.desired_force,    json.dumps((DESIRED_BOW_FORCE * str_normal).tolist()))
                    # r.set(KEYS.closed_loop,      "1")

                    r.set(KEYS.moment_space_dim, "1")
                    r.set(KEYS.moment_space_axis, json.dumps(str_moment.tolist()))
                    r.set(KEYS.desired_moment,    json.dumps((DESIRED_BOW_MOMENT * str_moment).tolist()))
                    set_linear_vel_limit(r, BOW_SPEED)
                    state = State.BOWING


            # ── CHEATING ───────────────────────────────────────────
            # Pure position control. Reads force sensor; if magnitude is outside
            # [CHEAT_FORCE_MIN, CHEAT_FORCE_MAX] band, nudge cheat_goal_pos along
            # str_normal by a fixed step. +str_normal points INTO the string, so
            # reaction force_normal is negative when pressing: too-pressed → back
            # off (-str_normal); too-light → push in (+str_normal).
            # elif state == State.CHEATING:
            #     force_local = get_force(r)
            #     force_world = cur_ori @ force_local
            #     force_normal = np.dot(force_world, str_normal)  # signed
            #     force_normal_mag = abs(force_normal)

            #     if force_normal_mag > CHEAT_FORCE_MAX:
            #         cheat_goal_pos -= CHEAT_POS_STEP * str_normal
            #         set_goal(r, cheat_goal_pos, str_ori)
            #         cheat_settle_start = None
            #     elif force_normal_mag < CHEAT_FORCE_MIN:
            #         cheat_goal_pos += CHEAT_POS_STEP * str_normal
            #         set_goal(r, cheat_goal_pos, str_ori)
            #         cheat_settle_start = None
            #     else:
            #         if cheat_settle_start is None:
            #             cheat_settle_start = loop_time
            #         elif loop_time - cheat_settle_start >= CHEAT_SETTLE_TIME:
            #             print(f"  Switching to BOWING str={TARGET_STRING}  Force stabilized at |F|={force_normal_mag:.3f} N")
            #             str_pos = cur_pos.copy()  # anchor bow sweep around the settled contact point
            #             set_goal(r, cur_pos.copy(), str_ori)
            #             set_linear_vel_limit(r, BOW_SPEED)
            #             bow_dir = 1.0
            #             bow_normal_offset = 0.0  # impedance drift along str_normal during BOWING
            #             state = State.BOWING

            #     if loop_time - last_print > 0.5:
            #         settled = (loop_time - cheat_settle_start) if cheat_settle_start else 0.0
            #         print(f"  CHEATING F_n={force_normal:+.3f} N |F|={force_normal_mag:.3f} N settled={settled:.2f}s")
            #         last_print = loop_time


            # ── BOWING ─────────────────────────────────────────────
            elif state == State.BOWING:
                bow_displacement = np.dot(cur_pos - str_pos, str_bow_dir)

                force_local = get_force(r)
                force_world = cur_ori @ force_local
                force_normal = np.dot(force_world, str_normal)
                force_normal_mag = abs(force_normal)

                # ── PURE POSITION BOWING (no force feedback) — kept for fallback ──
                if loop_time - last_print > 0.5:
                    print(f"  BOWING str={TARGET_STRING}  |F|={force_normal_mag:.3f} N")
                    last_print = loop_time
                
                if bow_displacement >= BOW_AMPLITUDE * 0.9:
                    bow_dir = -1.0
                    DESIRED_BOW_MOMENT = 0.4
                elif bow_displacement <= -BOW_AMPLITUDE * 0.9:
                    bow_dir = 1.0
                    DESIRED_BOW_MOMENT = 0.1

                r.set(KEYS.desired_moment,    json.dumps((DESIRED_BOW_MOMENT * str_moment).tolist()))
                
                bowing_goal_pos = str_pos + bow_dir * BOW_AMPLITUDE * str_bow_dir
                set_goal(r, bowing_goal_pos, str_ori)

                # ── IMPEDANCE-STYLE BOWING: sweep along str_bow_dir, nudge along str_normal ──
                # Same band logic as CHEATING: drift the goal toward/away from string
                # to keep |F| in [CHEAT_FORCE_MIN, CHEAT_FORCE_MAX] while sweeping.
                # if bow_displacement >= BOW_AMPLITUDE * 0.9:
                #     bow_dir = -1.0
                # elif bow_displacement <= -BOW_AMPLITUDE * 0.9:
                #     bow_dir = 1.0

                # if force_normal_mag > CHEAT_FORCE_MAX:
                #     bow_normal_offset -= CHEAT_POS_STEP
                # elif force_normal_mag < CHEAT_FORCE_MIN:
                #     bow_normal_offset += CHEAT_POS_STEP

                # bowing_goal_pos = (str_pos
                #                    + bow_dir * BOW_AMPLITUDE * str_bow_dir
                #                    + bow_normal_offset * str_normal)
                # set_goal(r, bowing_goal_pos, str_ori)

                if loop_time - last_print > 0.5:
                    print(f"  BOWING str={TARGET_STRING}  F_n={force_normal:+.3f} N "
                          f"|F|={force_normal_mag:.3f} N offset={bow_normal_offset:+.4f} m")
                    last_print = loop_time

                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key in ('0'):
                        homing_safety(r, home_pos, home_ori)
                        state = state.HOMING

                    new_string = {'5': 0, '6': 1, '7': 2, '8': 3}.get(key)
                    if new_string is not None and new_string != TARGET_STRING:
                        TARGET_STRING = new_string
                        lift_goal_pos = cur_pos - LIFT_HEIGHT * str_normal
                        set_linear_vel_limit(r, MOVING_SPEED)
                        set_goal(r, lift_goal_pos, cur_ori)
                        set_position_control(r)
                        time.sleep(0.1)
                        set_goal(r, lift_goal_pos, cur_ori)
                        print(f"\nSwitching to string {TARGET_STRING} → LIFTING")
                        state = State.LIFTING

            # ── LIFTING ─────────────────────────────────────────────
            elif state == State.LIFTING:
                p_err = pos_err(lift_goal_pos, cur_pos)
                if loop_time - last_print > 0.5:
                    print(f"  LIFTING  pos_err={p_err:.4f}")
                    last_print = loop_time

                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key in ('0'):
                        homing_safety(r, home_pos, home_ori)
                        state = state.HOMING

                if p_err < 0.02:
                    str_ori     = cal_orientations[TARGET_STRING]
                    str_pos     = cal_positions[TARGET_STRING]
                    str_normal  = str_ori[:, 2]
                    str_bow_dir = str_ori[:, 0]
                    move_goal_pos = str_pos - 0.05 * str_normal
                    set_goal(r, move_goal_pos, str_ori)
                    print(f"\nState: MOVING  (toward string {TARGET_STRING})")
                    state = State.MOVING

            # ── MOVING ──────────────────────────────────────────────
            elif state == State.MOVING:
                lateral_dist = abs(np.dot(cur_pos - str_pos, str_bow_dir))
                if loop_time - last_print > 0.5:
                    print(f"  MOVING  lateral_dist={lateral_dist:.4f}")
                    last_print = loop_time

                if lateral_dist <= MOVE_THRESHOLD:
                    contact_goal_pos = str_pos - 0.05 * str_normal
                    set_goal(r, contact_goal_pos, str_ori)
                    print(f"\nState: PLACING  (descending to string {TARGET_STRING})")
                    state = State.PLACING

    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
