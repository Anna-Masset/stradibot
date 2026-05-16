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

MOVING_SPEED            = 0.08    # m/s — how fast to move when not contacting
CONTACT_APPROACH_SPEED  = 0.03   # m/s — how fast to creep toward the string
CONTACT_FORCE_THRESHOLD = 0.1    # N   — force magnitude to detect contact
BOW_SPEED               = 0.06   # m/s
BOW_AMPLITUDE           = 0.15   # m   — half-stroke
DESIRED_BOW_FORCE       = 1.0    # N   — normal force on string
ANGULAR_SPEED           = np.pi/6    # rad/s — for orientation corrections during bowing

IS_REAL = False
CALIBRATION = True

if IS_REAL:
    CONFIG_FILE = "stradibot.xml"
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
    ft_force:            str = f"opensai::sensors::{robot_name}::ft_sensor::bow::force"
    # force/moment control keys
    force_space_dim:         str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::force_space_dimension"
    force_space_axis:        str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::force_space_axis"
    desired_force:           str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::desired_force"
    moment_space_dim:        str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::moment_space_dimension"
    desired_moment:          str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::desired_moment"
    closed_loop_force_ctrl:  str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::closed_loop_force_control"
    linear_vel_sat_limit:    str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::linear_velocity_saturation_limit"
    angular_vel_sat_limit:   str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::angular_velocity_saturation_limit"
    vel_sat_enabled:         str = f"opensai::controllers::{robot_name}::cartesian_controller::cartesian_task::velocity_saturation_enabled"

KEYS = RedisKeys()

def set_linear_vel_limit(r, limit):
    # r.set(KEYS.vel_sat_enabled,    "1")
    r.set(KEYS.linear_vel_sat_limit, str(limit))


# ============================================================
# STRING GEOMETRY  (from controller.cpp calibration)
# Positions and orientations of each string in world frame.
# ============================================================

if not IS_REAL:
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
    STRING_ORIENTATIONS = [0.0, 0.0, 0.0, 0.0]

# ============================================================
# STATES
# ============================================================

class State(Enum):
    CALIBRATING = auto()
    HOMING      = auto()
    PLACING     = auto()
    CONTACTING  = auto()
    BOWING      = auto()

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
    # """Gravity-comp floating: open-loop force control with zero desired force/moment.
    # Closed-loop force control must be OFF — otherwise force sensor noise drives
    # the controller into an unstable feedback loop."""
    # r.set(KEYS.closed_loop_force_ctrl, "0")   # open-loop: no force sensor feedback
    # r.set(KEYS.force_space_dim,        "3")
    # r.set(KEYS.desired_force,          json.dumps([0.0, 0.0, 0.0]))
    # r.set(KEYS.moment_space_dim,       "3")
    # r.set(KEYS.desired_moment,         json.dumps([0.0, 0.0, 0.0]))
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

# Single keyboard thread — all input goes into key_queue
key_queue = queue.Queue()

def _key_input_thread():
    while True:
        key_queue.put(input())

# ============================================================
# MAIN
# ============================================================

def main():
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

    if not CALIBRATION:
        print("Using hardcoded string geometry (from controller.cpp calibration):")
        # String geometry for the target string
        # STRING_ORIENTATIONS are recorded in bow frame; controller tracks flange frame.
        # Transform: R_world_flange = R_world_bow @ R_bow_in_flange.T

        R_bow_in_flange = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        str_pos = STRING_POSITIONS[TARGET_STRING]
        str_ori = STRING_ORIENTATIONS[TARGET_STRING] @ R_bow_in_flange.T
        str_normal   = str_ori[:, 2]   # Z column — normal to string
        str_bow_dir  = str_ori[:, 0]   # X column — bowing direction

    # ── State machine variables ──────────────────────────────
    state            = State.CALIBRATING
    contact_goal_pos = home_pos.copy()
    bow_displacement = 0.0
    bow_dir          = 1.0

    # Calibration storage — start from hardcoded, updated by user
    cal_positions    = list(STRING_POSITIONS)
    cal_orientations = list(STRING_ORIENTATIONS)

    # Start single keyboard thread
    threading.Thread(target=_key_input_thread, daemon=True).start()

    print(f"\nState: CALIBRATING  (floating — press 1/2/3/4 to register strings, Enter to home)")
    print(f"Target string: {TARGET_STRING}")

    loop_time  = 0.0
    init_time  = time.perf_counter_ns() * 1e-9
    last_print = 0.0
    time.sleep(0.01)

    # enable velocity saturation
    r.set(KEYS.vel_sat_enabled,    "1")
    set_linear_vel_limit(r, MOVING_SPEED)
    r.set(KEYS.angular_vel_sat_limit, str(ANGULAR_SPEED))
    set_floating(r)

    try:
        while True:
            loop_time += LOOP_DT
            time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))

            cur_pos = get_pos(r)
            cur_ori = get_ori(r)

            # during first state, get initial force sensor readings to find bias and remove later on (TODO)
        

            # ── CALIBRATING ──────────────────────────────────────────────
            if state == State.CALIBRATING:
                # set_floating(r)

                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key in ('1', '2', '3', '4'):
                        r.set(KEYS.active_controller, "cartesian_controller")
                        set_position_control(r)
                        time.sleep(0.1)  # wait for controller to switch and update readings
                        idx = int(key) - 1
                        cal_positions[idx]    = get_pos(r).copy()
                        cal_orientations[idx] = get_ori(r).copy()
                        print(f"  String {key} registered: pos={cal_positions[idx].round(4)}")
                        time.sleep(0.1)  # wait for controller to switch and update readings
                        set_floating(r)  # back to floating after position control to register
                    elif key == '':  # bare Enter → done
                        print("\nCalibrated positions:")
                        for i, p in enumerate(cal_positions):
                            print(f"  String {i+1}: {p.round(4)}")
                        # apply calibrated values for target string
                        str_pos = cal_positions[TARGET_STRING]
                        str_ori = cal_orientations[TARGET_STRING]
                        str_normal  = str_ori[:, 2]
                        str_bow_dir = str_ori[:, 0]
                        r.set(KEYS.active_controller, "cartesian_controller")
                        set_position_control(r)
                        time.sleep(0.1)  # wait for controller to switch and update readings
                        set_goal(r, home_pos, home_ori)
                        print(f"\nState: HOMING")
                        state = State.HOMING

            # ── HOMING ───────────────────────────────────────────────────
            elif state == State.HOMING:

                # set_goal(r, home_pos, home_ori)

                p_err = pos_err(home_pos, cur_pos)
                o_err = ori_err(home_ori, cur_ori)
                if loop_time - last_print > 1.0:
                    print(f"  HOMING  pos_err={p_err:.4f}  ori_err={o_err:.4f}")
                    last_print = loop_time

                if not key_queue.empty() and key_queue.get().strip() == '':
                    contact_goal_pos = str_pos - 0.05 * str_normal
                    set_linear_vel_limit(r, MOVING_SPEED)
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
                    print(f"\nState: BOWING  (contact detected |F|={force_normal_mag:.3f} N)")
                    bow_dir = 1.0
                    # Enable force control along string normal
                    r.set(KEYS.force_space_dim,  "1")
                    r.set(KEYS.force_space_axis, json.dumps(str_normal.tolist()))
                    r.set(KEYS.desired_force,    json.dumps((DESIRED_BOW_FORCE * str_normal).tolist()))
                    set_linear_vel_limit(r, BOW_SPEED)
                    state = State.BOWING

            elif state == State.BOWING:
                bow_displacement = np.dot(cur_pos - str_pos, str_bow_dir)

                force_local = get_force(r)
                force_world = cur_ori @ force_local          # rotate to world frame
                force_normal = np.dot(force_world, str_normal)  # scalar projection onto normal
                force_normal_mag = np.linalg.norm(force_normal)

                if loop_time - last_print > 0.5:
                    print(f"  BOWING  goal={bowing_goal_pos.round(4)}  |F|={force_normal_mag:.3f} N")
                    last_print = loop_time

                # Bowing motion along str_bow_dir
                # bow_displacement += bow_dir * BOW_SPEED * LOOP_DT
                if bow_displacement >= BOW_AMPLITUDE * 0.9:
                    bow_dir = -1.0
                elif bow_displacement <= -BOW_AMPLITUDE * 0.9:
                    bow_dir = 1.0

                bowing_goal_pos = str_pos + bow_dir * BOW_AMPLITUDE * str_bow_dir

                set_goal(r, bowing_goal_pos, str_ori)


    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
