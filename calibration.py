"""
calibration.py — Save and load string calibration data.

Usage:
    from calibration import save_calibration, load_calibration

    # After calibrating, save:
    save_calibration(cal_positions, cal_orientations)

    # Next time, load instead of recalibrating:
    cal_positions, cal_orientations = load_calibration()
"""

import numpy as np
import os

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "calibration_data.csv")

def save_calibration(cal_positions, cal_orientations):
    """
    Save 4 string positions (3,) and orientations (3,3) to a CSV file.
    Format: one row per string, 12 values (3 pos + 9 ori flattened row-major).
    """
    rows = []
    for i in range(4):
        pos = cal_positions[i]
        ori = cal_orientations[i].flatten()  # 3x3 → 9 values, row-major
        rows.append(np.concatenate([pos, ori]))

    np.savetxt(CALIBRATION_FILE, np.array(rows), delimiter=",",
               header="px,py,pz,o00,o01,o02,o10,o11,o12,o20,o21,o22",
               comments="")
    print(f"Calibration saved to {CALIBRATION_FILE}")


def load_calibration():
    """
    Load calibration from CSV. Returns (cal_positions, cal_orientations).
    cal_positions:    list of 4 np.array(3,)
    cal_orientations: list of 4 np.array(3,3)
    """
    if not os.path.exists(CALIBRATION_FILE):
        return None, None

    data = np.loadtxt(CALIBRATION_FILE, delimiter=",", skiprows=1)
    cal_positions = []
    cal_orientations = []
    for row in data:
        cal_positions.append(row[:3])
        cal_orientations.append(row[3:].reshape(3, 3))

    print(f"Calibration loaded from {CALIBRATION_FILE}")
    for i, p in enumerate(cal_positions):
        print(f"  String {i+1}: pos={p.round(4)}")
    return cal_positions, cal_orientations


def calibration_exists():
    """Check if a saved calibration file exists."""
    return os.path.exists(CALIBRATION_FILE)
