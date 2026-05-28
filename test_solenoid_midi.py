"""
Dry-run test: simulate MIDI playback with solenoid commands.
Shows exactly what serial bytes would be sent to the Arduino and when.
No robot or Arduino required.

Usage:
    python3 test_solenoid_midi.py <midi_file> [speed_multiplier]

Examples:
    python3 test_solenoid_midi.py test_scale.mid
    python3 test_solenoid_midi.py "Twinkle Twinkle Little Star (MIDI Version).mid" 2.0
"""

import sys
import time
from midi_input import parse_midi_file, OPEN_NOTES, ALL_FRET_OFFSETS, FRET_INDICES

STRING_NAMES = ['G', 'D', 'A', 'E']
NOTE_NAMES   = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']

def midi_note_name(note):
    return f"{NOTE_NAMES[note % 12]}{note // 12 - 1}"

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    filename = sys.argv[1]
    speed    = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    events = parse_midi_file(filename, speed_multiplier=speed)

    print(f"\n{'='*60}")
    print(f"  Solenoid dry-run:  {filename}  (speed={speed}x)")
    print(f"{'='*60}\n")
    print(f"  Installed frets: {FRET_INDICES}")
    print(f"  Fret 0 = open string (no solenoid activated)\n")

    print(f"{'#':<4} {'Time':<9} {'Note':<6} {'String':<8} {'Fret':<6} {'Duration':<9} {'Serial':<8} {'Action'}")
    print("-" * 70)

    # Only show note-on events (solenoid commands), skip redundant note-offs
    note_ons = [ev for ev in events if not ev.note_off]
    active_fret = 0
    for i, ev in enumerate(note_ons):
        midi_note = OPEN_NOTES[ev.string_idx] + ALL_FRET_OFFSETS[ev.fret]
        name = midi_note_name(midi_note)
        string = STRING_NAMES[ev.string_idx]

        if ev.fret == 0:
            serial_byte = 0
            action = "open string"
        else:
            serial_byte = ev.fret
            action = f"fret {ev.fret}"

        if ev.fret != active_fret:
            action += f"  ← change ({active_fret}→{ev.fret})"
        active_fret = ev.fret

        # Check if string changes from previous note
        if i > 0 and ev.string_idx != note_ons[i-1].string_idx:
            action += f"  [string switch → {string}]"

        print(f"{i+1:<4} {ev.start_time:<9.2f} {name:<6} {string:<8} {ev.fret:<6} {ev.duration:<9.2f} "
              f"0x{serial_byte:02X}      {action}")

    total_notes = len(note_ons)
    last_ev = note_ons[-1]
    print(f"\n  Total notes:    {total_notes}")
    print(f"  Duration:       {last_ev.start_time + last_ev.duration:.2f}s")
    print(f"  String changes: {sum(1 for i in range(1, len(note_ons)) if note_ons[i].string_idx != note_ons[i-1].string_idx)}")
    print(f"  Fret changes:   {sum(1 for i in range(1, len(note_ons)) if note_ons[i].fret != note_ons[i-1].fret)}")

    # Realtime playback simulation
    print(f"\n{'='*60}")
    ans = input("  Run realtime simulation? (y/n): ").strip().lower()
    if ans != 'y':
        return

    print(f"\n  Playing in realtime (speed={speed}x)...")
    print(f"  Watch the fret commands as they would be sent to Arduino.\n")

    start_wall = time.time()
    idx = 0

    while idx < len(note_ons):
        ev = note_ons[idx]
        elapsed = time.time() - start_wall

        if elapsed >= ev.start_time:
            midi_note = OPEN_NOTES[ev.string_idx] + ALL_FRET_OFFSETS[ev.fret]
            name = midi_note_name(midi_note)
            string = STRING_NAMES[ev.string_idx]

            if ev.fret == 0:
                print(f"  [{ev.start_time:6.2f}s]  {name} on {string} — OPEN (send 0x00)")
            else:
                print(f"  [{ev.start_time:6.2f}s]  {name} on {string} — FRET {ev.fret} (send 0x{ev.fret:02X})")

            idx += 1
        else:
            time.sleep(0.01)

    print(f"\n  Done! Total time: {time.time() - start_wall:.2f}s")

if __name__ == "__main__":
    main()
