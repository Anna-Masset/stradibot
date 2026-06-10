"""
Stradibot FSM — Flexiv RDK version.
Replaces OpenSai/Redis with direct RDK API calls.

Run:
    python3.10 stradibot_fsm_rdk.py <robot_sn>

States:
    CALIBRATING  → free drive, user registers string positions (press 1/2/3/4, Enter to home)
    HOMING       → move to home pose (press Enter to start placing)
    PLACING      → move to approach point above string
    CONTACTING   → creep along normal until force detected
    BOWING       → back-and-forth with open-loop force control
    LIFTING      → lift along current string normal (on string switch)
    MOVING       → move laterally to new string
"""

import time
import argparse
import threading
import queue
import numpy as np
from enum import Enum, auto
from scipy.spatial.transform import Rotation
import flexivrdk

# ============================================================
# CONFIG
# ============================================================

LOOP_DT                 = 0.01    # s   — 100 Hz
TARGET_STRING           = 1       # 0=G, 1=D, 2=A, 3=E

MOVING_SPEED            = 0.1     # m/s
CONTACT_APPROACH_SPEED  = 0.03    # m/s
CONTACT_FORCE_THRESHOLD = 0.5     # N
BOW_SPEED               = 0.06    # m/s
BOW_AMPLITUDE           = 0.15    # m   — half-stroke
DESIRED_BOW_FORCE       = 1.0     # N   — normal force on string (open-loop)
BOW_OFFSET              = 0.35    # m   — offset along bow dir from calibrated string pos
LIFT_HEIGHT             = 0.05    # m   — lift distance when switching strings
MOVE_THRESHOLD          = 0.15    # m   — lateral dist to stop above new string

CALIBRATION = True   # set False to use hardcoded string positions

# Hardcoded string positions (bow frame, from controller.cpp calibration)
# Only used when CALIBRATION = False
STRING_POSITIONS = [
    np.array([0.867011, -0.140365, 0.266309]),
    np.array([0.907412, -0.119718, 0.279737]),
    np.array([0.924652, -0.117822, 0.274284]),
    np.array([0.945377, -0.116711, 0.255744]),
]
STRING_ORIENTATIONS = [
    np.array([[ 0.0934987, -0.511516, -0.854171],
              [-0.994479,  -0.00692422, -0.10471],
              [ 0.0476466,  0.859246,  -0.50934 ]]),
    np.array([[ 0.0374749, -0.207924, -0.977427],
              [-0.998624,   0.0281047, -0.0442662],
              [ 0.0366743,  0.977741,  -0.206585]]),
    np.array([[-0.0108152,  0.0953699, -0.995383],
              [-0.999024,   0.0416067,  0.0148412],
              [ 0.0428301,  0.994572,   0.0948269]]),
    np.array([[-0.063477,   0.480517,  -0.874685],
              [-0.99537,    0.0329028,  0.0903107],
              [ 0.0721755,  0.876368,   0.476204 ]]),
]

# ============================================================
# STATES
# ============================================================

class State(Enum):
    CALIBRATING = auto()
    HOMING      = auto()
    PLACING     = auto()
    CONTACTING  = auto()
    BOWING      = auto()
    LIFTING     = auto()
    MOVING      = auto()

# ============================================================
# HELPERS
# ============================================================

def mat_to_pose(pos, R):
    """Convert [3] position + [3x3] rotation matrix to RDK pose [x,y,z,qw,qx,qy,qz]."""
    q = Rotation.from_matrix(R).as_quat()  # [qx, qy, qz, qw]
    return [pos[0], pos[1], pos[2], q[3], q[0], q[1], q[2]]

def pose_to_pos(pose):
    return np.array(pose[:3])

def pose_to_mat(pose):
    q = [pose[4], pose[5], pose[6], pose[3]]  # [qx, qy, qz, qw]
    return Rotation.from_quat(q).as_matrix()

def get_pos(robot, group):
    return pose_to_pos(robot.states()[group].tcp_pose)

def get_ori(robot, group):
    return pose_to_mat(robot.states()[group].tcp_pose)

def get_force(robot, group):
    """Returns TCP wrench force [3] in world frame."""
    return np.array(robot.states()[group].tcp_wrench[:3])

def send_goal(robot, group, pos, R, max_vel=MOVING_SPEED, wrench=None):
    if wrench is None:
        wrench = [0.0] * 6
    pose = mat_to_pose(pos, R)
    cmd = {group: flexivrdk.NrtCartesianCmd(pose, wrench, [0.0] * 7, max_vel)}
    robot.SendCartesianMotionForce(cmd)

def pos_err(target, current):
    return np.linalg.norm(target - current)

def ori_err(target, current):
    return np.linalg.norm(target - current)

# Single keyboard thread
key_queue = queue.Queue()

def _key_input_thread():
    while True:
        key_queue.put(input())

# ============================================================
# MAIN
# ============================================================

def main():
    global TARGET_STRING

    parser = argparse.ArgumentParser()
    parser.add_argument("robot_sn", help="Robot serial number, e.g. Rizon4s-123456")
    args = parser.parse_args()

    mode  = flexivrdk.Mode
    robot = flexivrdk.Robot(args.robot_sn)

    # Clear fault if any
    if robot.fault():
        print("Fault detected, clearing...")
        if not robot.ClearFault():
            print("Cannot clear fault, exiting.")
            return

    robot.Enable()
    while not robot.operational():
        time.sleep(1)
    print("Robot operational.")

    group = list(robot.groups())[0]  # single group for Rizon4s

    # Zero force/torque sensor
    print("Zeroing F/T sensor — make sure robot is not in contact...")
    robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
    robot.ExecutePrimitive({group: flexivrdk.PrimitiveArgs("ZeroFTSensor", {})})
    while not bool(robot.primitive_states()[group].names_and_values["terminated"]):
        time.sleep(0.5)
    print("F/T sensor zeroed.")

    # Switch to cartesian motion force mode
    robot.SwitchMode(mode.NRT_CARTESIAN_MOTION_FORCE)

    # Read home pose
    home_pos = get_pos(robot, group)
    home_ori = get_ori(robot, group)
    print(f"Home position: {home_pos.round(4)}")

    # Build calibration storage (with BOW_OFFSET baked in)
    if not CALIBRATION:
        cal_orientations = list(STRING_ORIENTATIONS)
        cal_positions = [
            p + BOW_OFFSET * cal_orientations[i][:, 0]
            for i, p in enumerate(STRING_POSITIONS)
        ]
    else:
        cal_positions    = [np.zeros(3)] * 4
        cal_orientations = [np.eye(3)] * 4

    str_ori     = cal_orientations[TARGET_STRING]
    str_pos     = cal_positions[TARGET_STRING]
    str_normal  = str_ori[:, 2]
    str_bow_dir = str_ori[:, 0]

    # State machine variables
    state            = State.CALIBRATING if CALIBRATION else State.HOMING
    contact_goal_pos = home_pos.copy()
    lift_goal_pos    = home_pos.copy()
    bow_dir          = 1.0

    threading.Thread(target=_key_input_thread, daemon=True).start()

    if CALIBRATION:
        # Use free drive via primitive for floating
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
        robot.ExecutePrimitive({group: flexivrdk.PrimitiveArgs("FreeDrive", {})})
        print("\nState: CALIBRATING  (free drive — press 1/2/3/4 to register strings, Enter to home)")
    else:
        print(f"\nState: HOMING  (press Enter to start placing on string {TARGET_STRING})")

    loop_time  = 0.0
    init_time  = time.perf_counter_ns() * 1e-9
    last_print = 0.0
    time.sleep(0.01)

    try:
        while True:
            loop_time += LOOP_DT
            time.sleep(max(0, loop_time - (time.perf_counter_ns() * 1e-9 - init_time)))

            if robot.fault():
                raise Exception("Fault on robot!")

            if state != State.CALIBRATING:
                cur_pos = get_pos(robot, group)
                cur_ori = get_ori(robot, group)

            # ── CALIBRATING ──────────────────────────────────────────────
            if state == State.CALIBRATING:
                if not key_queue.empty():
                    key = key_queue.get().strip()
                    if key in ('1', '2', '3', '4'):
                        idx = int(key) - 1
                        raw_pos = get_pos(robot, group).copy()
                        raw_ori = get_ori(robot, group).copy()
                        cal_orientations[idx] = raw_ori
                        cal_positions[idx]    = raw_pos + BOW_OFFSET * raw_ori[:, 0]
                        print(f"  String {key} registered: pos={cal_positions[idx].round(4)}")
                    elif key == '':  # Enter → done
                        print("\nCalibrated positions:")
                        for i, p in enumerate(cal_positions):
                            print(f"  String {i+1}: {p.round(4)}")
                        str_ori     = cal_orientations[TARGET_STRING]
                        str_pos     = cal_positions[TARGET_STRING]
                        str_normal  = str_ori[:, 2]
                        str_bow_dir = str_ori[:, 0]
                        robot.SwitchMode(mode.NRT_CARTESIAN_MOTION_FORCE)
                        time.sleep(0.5)
                        send_goal(robot, group, home_pos, home_ori)
                        print(f"\nState: HOMING  (press Enter to start placing)")
                        state = State.HOMING

            # ── HOMING ───────────────────────────────────────────────────
            elif state == State.HOMING:
                send_goal(robot, group, home_pos, home_ori)
                p_err = pos_err(home_pos, cur_pos)
                o_err = ori_err(home_ori, cur_ori)
                if loop_time - last_print > 1.0:
                    print(f"  HOMING  pos_err={p_err:.4f}  ori_err={o_err:.4f}")
                    last_print = loop_time

                if not key_queue.empty() and key_queue.get().strip() == '':
                    contact_goal_pos = str_pos - 0.05 * str_normal
                    send_goal(robot, group, contact_goal_pos, str_ori)
                    print(f"\nState: PLACING  (approaching string {TARGET_STRING})")
                    state = State.PLACING

            # ── PLACING ───────────────────────────────────────────────
            elif state == State.PLACING:
                send_goal(robot, group, contact_goal_pos, str_ori, max_vel=MOVING_SPEED)
                p_err = pos_err(contact_goal_pos, cur_pos)
                o_err = ori_err(str_ori, cur_ori)
                if loop_time - last_print > 1.0:
                    print(f"  PLACING  pos_err={p_err:.4f}  ori_err={o_err:.4f}")
                    last_print = loop_time

                if p_err < 0.02 and o_err < 0.1:
                    contact_goal_pos = cur_pos + 0.1 * str_normal
                    send_goal(robot, group, contact_goal_pos, str_ori, max_vel=CONTACT_APPROACH_SPEED)
                    state = State.CONTACTING

            # ── CONTACTING ─────────────────────────────────────────────
            elif state == State.CONTACTING:
                force_world = get_force(robot, group)
                force_normal_mag = abs(np.dot(force_world, str_normal))

                if loop_time - last_print > 0.5:
                    print(f"  CONTACTING  |F|={force_normal_mag:.3f} N")
                    last_print = loop_time

                if force_normal_mag > CONTACT_FORCE_THRESHOLD:
                    print(f"\nState: BOWING  (contact at |F|={force_normal_mag:.3f} N)")
                    # Open-loop force control along TCP Z (= str_normal when at str_ori)
                    robot.SetForceControlFrame(group, flexivrdk.CoordType.TCP)
                    robot.SetForceControlAxis(group, [False, False, True, False, False, False])
                    robot.SetPassiveForceControl(group, True)
                    bow_dir = 1.0
                    state = State.BOWING

            # ── BOWING ─────────────────────────────────────────────
            elif state == State.BOWING:
                bow_displacement = np.dot(cur_pos - str_pos, str_bow_dir)
                force_world = get_force(robot, group)
                force_normal_mag = abs(np.dot(force_world, str_normal))

                if loop_time - last_print > 0.5:
                    print(f"  BOWING str={TARGET_STRING}  |F|={force_normal_mag:.3f} N")
                    last_print = loop_time

                if bow_displacement >= BOW_AMPLITUDE * 0.9:
                    bow_dir = -1.0
                elif bow_displacement <= -BOW_AMPLITUDE * 0.9:
                    bow_dir = 1.0

                bowing_goal_pos = str_pos + bow_dir * BOW_AMPLITUDE * str_bow_dir
                # Wrench: press along TCP Z with desired force (open-loop)
                wrench = [0.0, 0.0, -DESIRED_BOW_FORCE, 0.0, 0.0, 0.0]
                send_goal(robot, group, bowing_goal_pos, str_ori, max_vel=BOW_SPEED, wrench=wrench)

                # String switch: 5→str0, 6→str1, 7→str2, 8→str3
                if not key_queue.empty():
                    key = key_queue.get().strip()
                    new_string = {'5': 0, '6': 1, '7': 2, '8': 3}.get(key)
                    if new_string is not None and new_string != TARGET_STRING:
                        TARGET_STRING = new_string
                        # Disable force control before lifting
                        robot.SetForceControlAxis(group, [False]*6)
                        time.sleep(0.1)
                        lift_goal_pos = cur_pos - LIFT_HEIGHT * str_normal
                        send_goal(robot, group, lift_goal_pos, str_ori, max_vel=MOVING_SPEED)
                        print(f"\nSwitching to string {TARGET_STRING} → LIFTING")
                        state = State.LIFTING

            # ── LIFTING ─────────────────────────────────────────────
            elif state == State.LIFTING:
                p_err = pos_err(lift_goal_pos, cur_pos)
                if loop_time - last_print > 0.5:
                    print(f"  LIFTING  pos_err={p_err:.4f}")
                    last_print = loop_time

                if p_err < 0.02:
                    str_ori     = cal_orientations[TARGET_STRING]
                    str_pos     = cal_positions[TARGET_STRING]
                    str_normal  = str_ori[:, 2]
                    str_bow_dir = str_ori[:, 0]
                    move_goal_pos = str_pos - 0.05 * str_normal
                    send_goal(robot, group, move_goal_pos, str_ori, max_vel=MOVING_SPEED)
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
                    send_goal(robot, group, contact_goal_pos, str_ori, max_vel=CONTACT_APPROACH_SPEED)
                    print(f"\nState: PLACING  (descending to string {TARGET_STRING})")
                    state = State.PLACING

    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"\nError: {e}")

if __name__ == "__main__":
    main()
