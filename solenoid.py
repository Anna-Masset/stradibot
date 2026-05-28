"""
solenoid.py — Serial interface to the Arduino solenoid controller.

Usage:
    from solenoid import Solenoid

    sol = Solenoid("/dev/cu.usbmodemXXXX")  # or "/dev/ttyACM0" on Linux
    sol.set_fret(3)   # press fret 3
    sol.release()     # open string (all off)
    sol.close()       # clean up

Install:
    pip install pyserial
"""

import serial
import time

class Solenoid:
    def __init__(self, port: str, baud: int = 9600):
        self._ser = serial.Serial(port, baud)
        time.sleep(2)  # wait for Arduino reset after serial connect
        self.release()
        print(f"Solenoid controller connected on {port}")

    def set_fret(self, fret: int):
        """Activate a fret (1-8). Solenoid stays down until release() or a new fret."""
        if fret < 1 or fret > 8:
            print(f"  WARNING: invalid fret {fret}, ignoring")
            return
        self._ser.write(bytes([fret]))

    def release(self):
        """Release all solenoids (open string)."""
        self._ser.write(bytes([0]))

    def close(self):
        """Release and close serial connection."""
        self.release()
        self._ser.close()
