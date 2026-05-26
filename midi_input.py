"""
midi_input.py — MIDI interface for Stradibot FSM.

Two modes:
  1. MIDI file playback:  parse_midi_file(filename) → list of MidiEvent
  2. Live MIDI keyboard:  MidiKeyboard() → start() / stop() / get_event()

Install dependencies:
  pip install mido python-rtmidi

MidiEvent fields:
  string_idx  : int   — which string to bow (0=G, 1=D, 2=A, 3=E)
  fret        : int   — which solenoid/fret to press (0 = open string)
  start_time  : float — seconds from piece start (file mode only, 0.0 for live)
  duration    : float — note duration in seconds (0.0 if unknown / live)
  note_off    : bool  — True if this is a note-off event (lift off string)
"""

import threading
import queue
from dataclasses import dataclass, field
from typing import Optional

try:
    import mido
except ImportError:
    mido = None

# ============================================================
# NOTE → VIOLIN MAPPING
# ============================================================

OPEN_NOTES = [55, 62, 69, 76]   # G3, D4, A4, E5
NUM_FRETS  = 5                   # adjust once solenoid range is confirmed

def midi_to_violin(note: int) -> tuple[int, int]:
    """
    Map a MIDI note number to (string_idx, fret).
    Prefers the lowest fret (natural left-hand position).
    Returns (string_idx, fret) where fret=0 means open string.
    """
    best_string, best_fret, best_dist = 0, 0, float('inf')
    for s, open_note in enumerate(OPEN_NOTES):
        for fret in range(NUM_FRETS + 1):  # 0 = open string
            dist = abs(note - (open_note + fret))
            if dist < best_dist or (dist == best_dist and fret < best_fret):
                best_dist   = dist
                best_string = s
                best_fret   = fret
    return best_string, best_fret


# ============================================================
# EVENT DATACLASS
# ============================================================

@dataclass
class MidiEvent:
    string_idx : int
    fret       : int
    start_time : float = 0.0   # seconds from piece start (file mode)
    duration   : float = 0.0   # note duration in seconds
    note_off   : bool  = False  # True = lift off string


# ============================================================
# MODE 1 — MIDI FILE PLAYBACK
# ============================================================

def parse_midi_file(filename: str, speed_multiplier: float = 1.0) -> list:
    """
    Parse a MIDI file and return a time-sorted list of MidiEvent.
    speed_multiplier > 1.0 slows down playback (e.g. 2.0 = half speed).

    Each note produces two events:
      - note_off=False at start_time       (bow the string)
      - note_off=True  at start_time+dur   (lift off)
    """
    if mido is None:
        raise ImportError("mido is not installed. Run: pip install mido python-rtmidi")

    mid = mido.MidiFile(filename)
    events = []

    for track in mid.tracks:
        abs_time = 0.0   # seconds
        active   = {}    # note → start_time

        for msg in mido.merge_tracks([track]):
            abs_time += mido.tick2second(msg.time, mid.ticks_per_beat, tempo=500000)

            if msg.type == 'note_on' and msg.velocity > 0:
                active[msg.note] = abs_time

            elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
                if msg.note in active:
                    on_time  = active.pop(msg.note)
                    off_time = abs_time
                    dur      = (off_time - on_time) * speed_multiplier
                    start    = on_time * speed_multiplier
                    s, fret  = midi_to_violin(msg.note)
                    events.append(MidiEvent(s, fret, start,        dur,  note_off=False))
                    events.append(MidiEvent(s, fret, start + dur,  0.0,  note_off=True))

    events.sort(key=lambda e: e.start_time)
    print(f"Parsed {sum(1 for e in events if not e.note_off)} notes from {filename}")
    return events


# ============================================================
# MODE 2 — LIVE MIDI KEYBOARD
# ============================================================

class MidiKeyboard:
    """
    Listens to a live MIDI input device in a background thread.
    Note-on  → puts MidiEvent(note_off=False) into the queue.
    Note-off → puts MidiEvent(note_off=True)  into the queue.

    Usage:
        kb = MidiKeyboard()          # auto-selects first available port
        kb.start()
        ...
        event = kb.get_event()       # returns MidiEvent or None
        ...
        kb.stop()
    """

    def __init__(self, port_name: str = None):
        if mido is None:
            raise ImportError("mido is not installed. Run: pip install mido python-rtmidi")
        self._port_name = port_name
        self._queue     = queue.Queue()
        self._thread    = None
        self._stop_evt  = threading.Event()

    def start(self):
        ports = mido.get_input_names()
        if not ports:
            raise RuntimeError("No MIDI input ports found.")
        port = self._port_name or ports[0]
        print(f"MIDI keyboard: opening port '{port}'")
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, args=(port,), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()

    def get_event(self) -> Optional[MidiEvent]:
        """Non-blocking. Returns only the most recent event, discards older ones."""
        ev = None
        while True:
            try:
                ev = self._queue.get_nowait()
            except queue.Empty:
                break
        return ev

    def available_ports(self) -> list[str]:
        return mido.get_input_names()

    def _run(self, port_name: str):
        with mido.open_input(port_name) as port:
            while not self._stop_evt.is_set():
                for msg in port.iter_pending():
                    if msg.type == 'note_on' and msg.velocity > 0:
                        s, fret = midi_to_violin(msg.note)
                        self._queue.put(MidiEvent(s, fret, note_off=False))
                    elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
                        s, fret = midi_to_violin(msg.note)
                        self._queue.put(MidiEvent(s, fret, note_off=True))
                self._stop_evt.wait(timeout=0.005)  # poll at ~200 Hz
