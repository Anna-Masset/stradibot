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
# MIDI CONFIG  (set MIDI_MODE to enable)
# ============================================================

MIDI_MODE        = False          # True = MIDI drives string switches
MIDI_FILE        = "bach_ave-maria.mid" 
# MIDI_FILE        = "Canon in C - stradibot-Violon.midi"           # e.g. "piece.mid" — None = live keyboard
MIDI_SPEED       = 2.0            # >1.0 slows down file playback
# MIDI_PORT      = None           # None = first available port (live mode)

SOLENOID_PORT    = "/dev/ttyACM0"           # e.g. "/dev/cu.usbmodemXXXX" — None = no solenoids

if MIDI_MODE:
    from midi_input import parse_midi_file, MidiKeyboard, MidiEvent
if SOLENOID_PORT is not None:
    from solenoid import Solenoid

# ============================================================
# CONFIG
# ============================================================

CONTROLLER     = "cartesian_controller"
LOOP_DT        = 0.01   # 100 Hz    

TARGET_STRING  = 1       # which string to bow (0=G, 1=D, 2=A, 3=E)

MOVING_SPEED            = 0.15    # m/s — how fast to move when not contacting
CONTACT_APPROACH_SPEED  = 0.05   # m/s — how fast to creep toward the string
ANGULAR_SPEED           = np.pi/8    # rad/s — for orientation corrections during bowing
SAFETY_SPEED            = 0.04
CONTACT_FORCE_THRESHOLD = 1.2    # N   — force magnitude to detect contact
BOW_SPEED               = 0.08   # m/s
# BOW_SPEED               = 0.01
BOW_AMPLITUDE           = 0.20   # m   — half-stroke
# DESIRED_BOW_FORCE       = 0.7    # N   — normal force on string
DESIRED_BOW_FORCE       = 1.3
STRING_BOW_FORCE        = [0.5, 0.5, 0.4, 0.4]
DESIRED_BOW_MOMENT       = 0.2
# DESIRED_BOW_MOMENT       = 0.14
BOW_OFFSET              = 0.35   # m   — offset along bow direction from string position

LIFT_HEIGHT             = 0.02   # m   — how far to lift along string normal when switching
MOVING_ANGULAR_SPEED    = np.pi/2

MOVE_THRESHOLD          = 0.15   # m   — lateral distance along bow dir to stop above new string

CHEAT_FORCE_MIN               = 0.8
CHEAT_FORCE_MAX               = 1.2
CHEAT_SETTLE_TIME              = 0.1
CHEAT_POS_STEP                = 0.0003  # m per loop — bow goal nudge along str_normal (~5 cm/s @ 100 Hz)

STRING_MOMENT_PLUS          = [0.22, 0.32, 0.40, 0.35] # String 1,2,3,4
STRING_MOMENT_MINUS         = [0.17, 0.25, 0.25, 0.25]

IS_REAL = True
CALIBRATION = True
GOING_TO_ZERO = False
CONTROL = "force"

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
    posture_task:        str = f"opensai::controllers::{robot_name}::cartesian_controller::joint_task::goal_position"
    config_file:         str = f"::sai-interfaces-webui::config_file_name"
    # force sensor (local frame, 3D vector)
    joint_task_kp:       str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::kp"
    joint_controller_task_goal: str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::goal_position"
    cartesian_controller_task_goal: str = f"opensai::controllers::{robot_name}::cartesian_controller::joint_task::goal_position"
    joint_pos:             str = f"opensai::controllers::{robot_name}::joint_controller::joint_task::current_position"

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

# INITIALIZATION CONSTANTS
INIT_JOINT_POS = [1.249260,-1.236980,-1.828240,1.717140,1.172930,-0.126265,1.002390]
#  [1.178460,-1.101560,-1.861230,1.604370,1.033080,-0.245748,0.983508]

ZERO_JOINT_POS = [0.0] * 7

# ============================================================
# STATES
# ============================================================

class State(Enum):
    ZEROING     = auto()
    INITIALIZING = auto()
    CALIBRATING = auto()
    HOMING      = auto()
    PLACING     = auto()
    CONTACTING  = auto()
    CHEATING    = auto()
    BOWING      = auto()
    HOVERING    = auto()   # lifted off string, waiting for next note
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

def homing_safety(r, home_pos, home_ori, sol=None):
    if sol: sol.release()
    r.set(KEYS.active_controller, "cartesian_controller")
    r.set(KEYS.closed_loop,      "0")
    set_position_control(r)
    time.sleep(0.1)  # wait for controller to switch and update readings
    set_linear_vel_limit(r, SAFETY_SPEED)
    r.set(KEYS.angular_vel_sat_limit, str(ANGULAR_SPEED))
    set_goal(r, home_pos, home_ori)
    print(f"\nState: HOMING SAFETY")

# Single keyboard thread — all input goes into key_queue
key_queue = queue.Queue()

def _key_input_thread():
    while True:
        key_queue.put(input())

def get_midi_event(midi_kb, midi_events, midi_event_idx, midi_elapsed, midi_started):
    """
    Returns (MidiEvent, new_idx) if an event is ready, else (None, midi_event_idx).
    - Live mode (midi_kb set): polls the keyboard queue.
    - File mode: checks if midi_elapsed has reached the next event's start_time.
      midi_elapsed only advances when BOWING or HOVERING, so transitions are paused.
    """
    if midi_kb is not None:
        return midi_kb.get_event(), midi_event_idx
    if midi_events and midi_started:
        if midi_event_idx < len(midi_events):
            ev = midi_events[midi_event_idx]
            if midi_elapsed >= ev.start_time:
                return ev, midi_event_idx + 1
    return None, midi_event_idx

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
    time.sleep(0.1)

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
    str_moment_axis  = str_ori[:, 1]
    str_bow_dir = str_ori[:, 0]

    # ── State machine variables ──────────────────────────────
    if GOING_TO_ZERO and CALIBRATION:
        state = State.ZEROING
    elif not GOING_TO_ZERO and CALIBRATION:
        state = State.CALIBRATING
    # elif not GOING_TO_ZERO and not CALIBRATION and not IS_REAL:
    #     r.set(KEYS.active_controller, "cartesian_controller")
    #     time.sleep(0.1)
    #     state = State.HOMING
    #     home_pos = get_pos(r)
    #     home_ori = get_ori(r)

    # state            = State.ZEROING if CALIBRATION and GOING_TO_ZERO else State.HOMING
    contact_goal_pos = home_pos.copy()
    bow_displacement = 0.0
    bow_dir          = 1.0

    # Start single keyboard thread
    threading.Thread(target=_key_input_thread, daemon=True).start()

    # MIDI setup
    midi_events    = []    # file mode: pre-parsed event list
    midi_kb        = None  # live mode: MidiKeyboard instance
    midi_event_idx = 0     # file mode: index into midi_events
    midi_start_time = None # set when 'p' is pressed — enables playback
    midi_elapsed   = 0.0   # only advances when BOWING or HOVERING

    if MIDI_MODE:
        if MIDI_FILE:
            midi_events = parse_midi_file(MIDI_FILE, speed_multiplier=MIDI_SPEED)
            print(f"MIDI file mode: {len(midi_events)//2} notes loaded from {MIDI_FILE}")
        else:
            midi_kb = MidiKeyboard()
            midi_kb.start()
            print(f"MIDI keyboard mode: live input active")

    # Solenoid setup
    sol = None
    current_fret = 0  # track active fret to avoid redundant serial commands
    pending_fret = None       # (fret, time) — delayed fret activation on string switch
    FRET_SWITCH_DELAY = 0.2   # seconds to wait before pressing new fret on string switch
    if SOLENOID_PORT is not None:
        sol = Solenoid(SOLENOID_PORT)

    print(f"Target string: {TARGET_STRING}")

    loop_time  = 0.0
    init_time  = time.perf_counter_ns() * 1e-9
    last_print = 0.0
    time.sleep(0.01)

    # enable velocity saturation
    set_linear_vel_limit(r, MOVING_SPEED)
    r.set(KEYS.angular_vel_sat_limit, str(ANGULAR_SPEED))

    if CALIBRATION and not GOING_TO_ZERO:
        set_floating(r)
    else:
        set_position_control(r)
    set_goal(r, np.array([ 0.940193, -0.078024,  0.310439]), home_ori)

    # Initialization joint controller setup
    if GOING_TO_ZERO:
        print(f"\nState: ZEROING")
        r.set(KEYS.active_controller, "joint_controller")
        # r.set(KEYS.joint_task_kp, json.dumps([100.0] * 7))
        time.sleep(0.1)
        r.set(KEYS.joint_controller_task_goal, json.dumps(ZERO_JOINT_POS))

    try:
        while True:
            loop_time += LOOP_DT
            time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))

            cur_pos = get_pos(r)
            cur_ori = get_ori(r)

            # Advance MIDI clock only when actively bowing or hovering
            if MIDI_MODE and midi_start_time is not None and state in (State.BOWING, State.HOVERING):
                midi_elapsed += LOOP_DT

            # Activate pending fret after delay (non-blocking)
            if pending_fret is not None and sol:
                fret_val, fret_time = pending_fret
                if loop_time - fret_time >= FRET_SWITCH_DELAY:
                    if fret_val > 0:
                        sol.set_fret(fret_val)
                    else:
                        sol.release()
                    current_fret = fret_val
                    pending_fret = None
            
            # send solenoid keep alive 
            if sol and current_fret > 0 and state == State.BOWING:
                sol.set_fret(current_fret)

            # ── INITIALIZING ──────────────────────────────────────────────
            if state == State.ZEROING:
                joint_curr_pos = np.array(json.loads(r.get(KEYS.joint_pos)))
                error = np.linalg.norm(np.array(joint_curr_pos) - np.array(ZERO_JOINT_POS))

                if loop_time - last_print > 1.0:
                    print(f"  ZEROING  pos_err={error:.4f}")
                    last_print = loop_time
    
                if error <= 1.0:
                    print(f"\nState: INITIALIZING (PRESS ENTER TO CALIBRATE)   ")
                    r.set(KEYS.joint_controller_task_goal, json.dumps(INIT_JOINT_POS))
                    state = state.INITIALIZING
            
            if state == State.INITIALIZING: 

                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key == '':
                        # save homing position
                        r.set(KEYS.active_controller, "cartesian_controller")
                        time.sleep(0.1)
                        home_pos = get_pos(r)
                        home_ori = get_ori(r)
                        print(f"Home position: {home_pos}")

                        # go back to floating for calibration
                        set_floating(r)
                        time.sleep(0.1)
                        print(f"\nState: CALIBRATING  (floating — press 1/2/3/4 to register strings, Enter to home)")
                        state = State.CALIBRATING

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
                        str_moment_axis  = str_ori[:, 1]
                        str_bow_dir = str_ori[:, 0]
                        r.set(KEYS.active_controller, "cartesian_controller")                    
                        time.sleep(0.1)  # wait for controller to switch and update readings
                        # r.set(KEYS.moment_space_dim, "1")
                        # r.set(KEYS.moment_space_axis, json.dumps(np.array([1, 0, 0]).tolist()))
                        # r.set(KEYS.desired_moment,    json.dumps((DESIRED_BOW_MOMENT * np.array([1, 0, 0])).tolist()))
                        # time.sleep(0.1)

                        r.set(KEYS.posture_task, json.dumps(INIT_JOINT_POS))

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
                    contact_goal_pos = str_pos - LIFT_HEIGHT * str_normal
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
                    bow_dir = 1.0 ## starting to the minus

                    if CONTROL == "force":
                        DESIRED_BOW_FORCE = STRING_BOW_FORCE[TARGET_STRING]
                        r.set(KEYS.force_space_dim, "1")
                        r.set(KEYS.force_space_axis, json.dumps(str_normal.tolist()))
                        r.set(KEYS.desired_force,    json.dumps((DESIRED_BOW_FORCE * str_normal).tolist()))
                        r.set(KEYS.closed_loop, "1")

                    if CONTROL == "moment":
                        DESIRED_BOW_MOMENT = STRING_MOMENT_MINUS[TARGET_STRING]
                        r.set(KEYS.moment_space_dim, "1")
                        r.set(KEYS.moment_space_axis, json.dumps(str_moment_axis.tolist()))
                        r.set(KEYS.desired_moment,    json.dumps((DESIRED_BOW_MOMENT * str_moment_axis).tolist()))

                    set_linear_vel_limit(r, BOW_SPEED)
                    state = State.BOWING


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
                
                if bow_displacement >= BOW_AMPLITUDE * 0.9 and bow_dir > 0:
                    bow_dir = -1.0 # Going to the plus
                    DESIRED_BOW_MOMENT = STRING_MOMENT_PLUS[TARGET_STRING]
                elif bow_displacement <= -BOW_AMPLITUDE * 0.9 and bow_dir < 0:
                    bow_dir = 1.0 # Going to the minus
                    DESIRED_BOW_MOMENT = STRING_MOMENT_MINUS[TARGET_STRING]

                if CONTROL == "moment":
                    r.set(KEYS.desired_moment,    json.dumps((DESIRED_BOW_MOMENT * str_moment_axis).tolist()))
                
                bowing_goal_pos = str_pos + bow_dir * BOW_AMPLITUDE * str_bow_dir
                set_goal(r, bowing_goal_pos, str_ori)

                # if loop_time - last_print > 0.5:
                #     print(f"  BOWING str={TARGET_STRING}  F_n={force_normal:+.3f} N "
                #           f"|F|={force_normal_mag:.3f} N offset={bow_normal_offset:+.4f} m")
                #     last_print = loop_time

                # ── keyboard input ──
                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key == '0':
                        homing_safety(r, home_pos, home_ori, sol)
                        current_fret = 0
                        pending_fret = None
                        state = State.HOMING
                    if key == 'p' and MIDI_MODE and midi_start_time is None:
                        r.set(KEYS.angular_vel_sat_limit, str(MOVING_ANGULAR_SPEED))
                        set_linear_vel_limit(r, MOVING_SPEED)
                        midi_start_time = loop_time
                        print(f"MIDI playback started")
                    new_string = {'5': 0, '6': 1, '7': 2, '8': 3}.get(key)
                    if new_string is not None and new_string != TARGET_STRING:
                        TARGET_STRING = new_string
                        lift_goal_pos = cur_pos - LIFT_HEIGHT * str_normal
                        set_linear_vel_limit(r, MOVING_SPEED)
                        r.set(KEYS.angular_vel_sat_limit, str(MOVING_ANGULAR_SPEED))
                        set_goal(r, lift_goal_pos, cur_ori)
                        set_position_control(r)
                        time.sleep(0.1)
                        set_goal(r, lift_goal_pos, cur_ori)
                        print(f"\nSwitching to string {TARGET_STRING} → LIFTING")
                        state = State.LIFTING

                # ── MIDI input ──
                if MIDI_MODE:
                    ev, midi_event_idx = get_midi_event(midi_kb, midi_events, midi_event_idx, midi_elapsed, midi_start_time is not None)
                    if ev is not None:
                        if ev.note_off:
                            # rest — stop bow in place, wait for next note
                            pending_fret = None
                            if sol and current_fret != 0:
                                sol.release()
                                current_fret = 0
                            set_goal(r, cur_pos, cur_ori)
                            print(f"\nNote off → HOVERING (bow stopped)")
                            state = State.HOVERING
                        elif ev.string_idx != TARGET_STRING:
                            # string switch — delay fret change
                            if sol and ev.fret != current_fret:
                                sol.release()
                                current_fret = 0
                                pending_fret = (ev.fret, loop_time)
                            TARGET_STRING = ev.string_idx
                            lift_goal_pos = cur_pos - LIFT_HEIGHT * str_normal
                            set_goal(r, lift_goal_pos, cur_ori)
                            set_position_control(r)
                            time.sleep(0.1)
                            set_goal(r, lift_goal_pos, cur_ori)
                            r.set(KEYS.angular_vel_sat_limit, str(MOVING_ANGULAR_SPEED))
                            set_linear_vel_limit(r, MOVING_SPEED)
                            print(f"\nMIDI string switch → string {TARGET_STRING}, fret {ev.fret} → LIFTING")
                            state = State.LIFTING
                        else:
                            # same string — update fret only if changed, reverse bow
                            if sol and ev.fret != current_fret:
                                if ev.fret > 0:
                                    sol.set_fret(ev.fret)
                                else:
                                    sol.release()
                                current_fret = ev.fret
                            bow_dir *= -1.0
                            print(f"\nMIDI same string {TARGET_STRING}, fret {ev.fret} → reversing bow direction")

            # ── LIFTING ─────────────────────────────────────────────
            elif state == State.LIFTING:
                p_err = pos_err(lift_goal_pos, cur_pos)
                if loop_time - last_print > 0.5:
                    print(f"  LIFTING  pos_err={p_err:.4f}")
                    last_print = loop_time

                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key in ('0'):
                        homing_safety(r, home_pos, home_ori, sol)
                        current_fret = 0
                        pending_fret = None
                        state = State.HOMING

                if p_err < 0.02:
                    str_ori     = cal_orientations[TARGET_STRING]
                    str_pos     = cal_positions[TARGET_STRING]
                    str_normal  = str_ori[:, 2]
                    str_moment_axis  = str_ori[:, 1]
                    str_bow_dir = str_ori[:, 0]
                    move_goal_pos = str_pos - LIFT_HEIGHT * str_normal
                    set_goal(r, move_goal_pos, str_ori)
                    print(f"\nState: MOVING  (toward string {TARGET_STRING})")
                    state = State.MOVING

            # ── HOVERING ────────────────────────────────────────────
            # Lifted off string, holding position, waiting for next note-on.
            # Only reachable in MIDI_MODE.
            elif state == State.HOVERING:
                if loop_time - last_print > 1.0:
                    print(f"  HOVERING  (waiting for next note)")
                    last_print = loop_time

                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key == '0':
                        homing_safety(r, home_pos, home_ori, sol)
                        current_fret = 0
                        pending_fret = None
                        state = State.HOMING

                if MIDI_MODE:
                    ev, midi_event_idx = get_midi_event(midi_kb, midi_events, midi_event_idx, midi_elapsed, midi_start_time is not None)
                    if ev is not None and not ev.note_off:
                        if ev.string_idx != TARGET_STRING:
                            # string switch — delay fret change
                            if sol and ev.fret != current_fret:
                                sol.release()
                                current_fret = 0
                                pending_fret = (ev.fret, loop_time)
                            TARGET_STRING = ev.string_idx
                            str_ori     = cal_orientations[TARGET_STRING]
                            str_pos     = cal_positions[TARGET_STRING]
                            str_normal  = str_ori[:, 2]
                            str_moment_axis  = str_ori[:, 1]
                            str_bow_dir = str_ori[:, 0]
                            move_goal_pos = str_pos - LIFT_HEIGHT * str_normal
                            set_goal(r, move_goal_pos, str_ori)
                            r.set(KEYS.angular_vel_sat_limit, str(MOVING_ANGULAR_SPEED))
                            set_linear_vel_limit(r, MOVING_SPEED)
                            print(f"\nNote on string {TARGET_STRING}, fret {ev.fret} → MOVING")
                            state = State.MOVING
                        else:
                            # same string — activate fret immediately
                            if sol and ev.fret != current_fret:
                                if ev.fret > 0:
                                    sol.set_fret(ev.fret)
                                else:
                                    sol.release()
                                current_fret = ev.fret
                            contact_goal_pos = str_pos - LIFT_HEIGHT * str_normal
                            set_goal(r, contact_goal_pos, str_ori)
                            r.set(KEYS.angular_vel_sat_limit, str(MOVING_ANGULAR_SPEED))
                            set_linear_vel_limit(r, MOVING_SPEED)
                            print(f"\nNote on same string {TARGET_STRING}, fret {ev.fret} → PLACING")
                            state = State.PLACING

            # ── MOVING ──────────────────────────────────────────────
            elif state == State.MOVING:
                lateral_dist = abs(np.dot(cur_pos - str_pos, str_bow_dir))
                if loop_time - last_print > 0.5:
                    print(f"  MOVING  lateral_dist={lateral_dist:.4f}")
                    last_print = loop_time

                if lateral_dist <= MOVE_THRESHOLD:
                    contact_goal_pos = str_pos - LIFT_HEIGHT * str_normal
                    set_goal(r, contact_goal_pos, str_ori)
                    r.set(KEYS.angular_vel_sat_limit, str(MOVING_ANGULAR_SPEED))
                    set_linear_vel_limit(r, MOVING_SPEED)
                    print(f"\nState: PLACING  (descending to string {TARGET_STRING})")
                    state = State.PLACING

    except KeyboardInterrupt:
        if sol: sol.close()
        print("\nStopped.")

if __name__ == "__main__":
    main()
