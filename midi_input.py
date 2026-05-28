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

# Actual note (MIDI number) at each fret for each string.
# Based on physical violin tuning (not strict semitones).
# Frets 5 and 8 are excluded — too close to neighbors for solenoids.
# Available frets: 0, 1, 2, 3, 4, 6, 7
#
#        fret: 0    1    2    3    4    6    7
# G string:   G3   A3   B3   C4   D4   F4   G4
# D string:   D4   E4   F#4  G4   A4   C5   D5
# A string:   A4   B4   C5   D5   E5   G5   A5
# E string:   E5   F#5  G#5  A5   B5   D6   E6

OPEN_NOTES = [55, 62, 69, 76]  # G3, D4, A4, E5 in MIDI numbers

# Full fret mapping — major scale pattern (W W H W W H W W)
# Fret:         0   1   2   3   4   5   6   7   8
# Semitones:    0   2   4   5   7   9  10  12  14
ALL_FRET_OFFSETS = [0, 2, 4, 5, 7, 9, 10, 12, 14]

# Which frets have solenoids installed — change this to match your hardware
FRET_INDICES = [0, 1, 2, 3, 5, 6]   # frets with solenoids (adjust as needed)
FRET_OFFSETS = [ALL_FRET_OFFSETS[i] for i in FRET_INDICES]

# Build lookup: (string, fret_idx) → midi_note, and reverse: midi_note → (string, fret_idx)
_STRING_FRET_TO_MIDI = {}
_MIDI_TO_STRING_FRET = {}

for _s, _open in enumerate(OPEN_NOTES):
    for _fi, _offset in zip(FRET_INDICES, FRET_OFFSETS):
        _note = _open + _offset
        _STRING_FRET_TO_MIDI[(_s, _fi)] = _note
        if _note not in _MIDI_TO_STRING_FRET:
            _MIDI_TO_STRING_FRET[_note] = []
        _MIDI_TO_STRING_FRET[_note].append((_s, _fi))

def midi_to_violin(note: int) -> tuple:
    """
    Map a MIDI note number to (string_idx, fret).
    Uses the actual violin scale mapping with available solenoid frets only.
    Prefers lowest fret (natural left-hand position).
    Falls back to nearest available note if exact match not found.
    Returns (string_idx, fret) where fret=0 means open string.
    """
    # Exact match — prefer lowest fret
    if note in _MIDI_TO_STRING_FRET:
        options = _MIDI_TO_STRING_FRET[note]
        return min(options, key=lambda x: x[1])  # lowest fret

    # No exact match — find nearest available note
    best_s, best_fret, best_dist = 0, 0, float('inf')
    for (s, fret), midi_note in _STRING_FRET_TO_MIDI.items():
        dist = abs(note - midi_note)
        if dist < best_dist or (dist == best_dist and fret < best_fret):
            best_dist  = dist
            best_s     = s
            best_fret  = fret
    return best_s, best_fret


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
                    if msg.note not in _MIDI_TO_STRING_FRET:
                        s, fret = midi_to_violin(msg.note)
                        NOTE_NAMES = ['C','C#','D','D#','E','F','F#','G','G#','A','A#','B']
                        name = f"{NOTE_NAMES[msg.note % 12]}{msg.note // 12 - 1}"
                        STRING_NAMES = ['G','D','A','E']
                        approx_note = _STRING_FRET_TO_MIDI[(s, fret)]
                        approx_name = f"{NOTE_NAMES[approx_note % 12]}{approx_note // 12 - 1}"
                        print(f"  WARNING: {name} (MIDI {msg.note}) not available on installed frets → approximated as {approx_name} ({STRING_NAMES[s]} fret {fret})")
                    else:
                        s, fret = midi_to_violin(msg.note)
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
