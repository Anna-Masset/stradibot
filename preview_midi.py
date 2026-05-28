"""
Preview how a MIDI file gets parsed for the Stradibot FSM.
Shows note-on events with string, fret, timing, and duration.

Usage:
    python3 preview_midi.py <midi_file> [speed_multiplier] [track_index]
    python3 preview_midi.py --mapping       # show full note→string/fret table
    python3 preview_midi.py --info <file>   # list tracks in a MIDI file

Examples:
    python3 preview_midi.py twinkle-twinkle-little-star.mid
    python3 preview_midi.py twinkle-twinkle-little-star.mid 1.0 0
    python3 preview_midi.py --mapping
"""

import sys
import mido
from midi_input import OPEN_NOTES

STRING_NAMES = ['G', 'D', 'A', 'E']
NOTE_NAMES   = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']

def midi_note_name(note):
    return f"{NOTE_NAMES[note % 12]}{note // 12 - 1}"

def midi_to_violin(note):
    from midi_input import midi_to_violin as _m
    return _m(note)

def print_track_info(filename):
    mid = mido.MidiFile(filename)
    print(f"\nFile: {filename}  ({len(mid.tracks)} tracks, type={mid.type})\n")
    for i, track in enumerate(mid.tracks):
        notes = [m for m in track if m.type == 'note_on' and m.velocity > 0]
        if notes:
            pitches = [m.note for m in notes]
            print(f"  Track {i}: '{track.name}'  —  {len(notes)} notes, "
                  f"pitch range {midi_note_name(min(pitches))}–{midi_note_name(max(pitches))}")
        else:
            print(f"  Track {i}: '{track.name}'  —  (no notes, likely tempo/meta)")

def parse_single_track(filename, track_idx, speed=1.0):
    mid = mido.MidiFile(filename)
    track = mid.tracks[track_idx]
    tempo = 500000
    events = []
    abs_time = 0.0
    active = {}

    for msg in mido.merge_tracks([track]):
        delta = mido.tick2second(msg.time, mid.ticks_per_beat, tempo)
        abs_time += delta

        if msg.type == 'set_tempo':
            tempo = msg.tempo
        elif msg.type == 'note_on' and msg.velocity > 0:
            active[msg.note] = abs_time
        elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
            if msg.note in active:
                on_time  = active.pop(msg.note)
                off_time = abs_time
                dur      = (off_time - on_time) * speed
                start    = on_time * speed
                s, fret  = midi_to_violin(msg.note)
                events.append((msg.note, s, fret, start, dur))

    events.sort(key=lambda e: e[3])
    return events

def print_mapping():
    from midi_input import OPEN_NOTES, FRET_INDICES, ALL_FRET_OFFSETS

    # Build full mapping (all 9 frets)
    all_midi_to_sf = {}
    for s, open_note in enumerate(OPEN_NOTES):
        for fret, offset in enumerate(ALL_FRET_OFFSETS):
            note = open_note + offset
            if note not in all_midi_to_sf:
                all_midi_to_sf[note] = []
            all_midi_to_sf[note].append((s, fret))

    # Build selected mapping (installed frets only)
    selected_offsets = [ALL_FRET_OFFSETS[i] for i in FRET_INDICES]
    selected_midi = {}
    for s, open_note in enumerate(OPEN_NOTES):
        for fret, offset in zip(FRET_INDICES, selected_offsets):
            note = open_note + offset
            if note not in selected_midi:
                selected_midi[note] = []
            selected_midi[note].append((s, fret))

    print(f"All frets (0-8): {list(range(9))}")
    print(f"Installed frets: {FRET_INDICES}\n")
    print(f"{'Note':<6} {'MIDI':<6} {'All frets':<10} {'Installed':<10} {'Options (all frets)'}")
    print("-" * 70)
    for note in range(OPEN_NOTES[0], OPEN_NOTES[3] + ALL_FRET_OFFSETS[-1] + 1):
        all_opts = all_midi_to_sf.get(note, [])
        sel_opts = selected_midi.get(note, [])
        all_str = "  ".join(f"{STRING_NAMES[s]}f{f}" for s, f in sorted(all_opts, key=lambda x: x[1]))
        avail    = "YES" if sel_opts else "NO"
        avail_all = "YES" if all_opts else "NO"
        print(f"{midi_note_name(note):<6} {note:<6} {avail_all:<10} {avail:<10} {all_str}")

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    if '--mapping' in sys.argv:
        print_mapping()
        return

    filename = sys.argv[1]

    if '--info' in sys.argv:
        print_track_info(filename)
        return

    speed       = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    track_idx   = int(sys.argv[3])   if len(sys.argv) > 3 else None

    if track_idx is not None:
        events = parse_single_track(filename, track_idx, speed)
        print(f"\nTrack {track_idx} only  (speed={speed}x)\n")
    else:
        from midi_input import parse_midi_file, ALL_FRET_OFFSETS
        raw = parse_midi_file(filename, speed_multiplier=speed)
        events = [(OPEN_NOTES[e.string_idx] + ALL_FRET_OFFSETS[e.fret], e.string_idx, e.fret, e.start_time, e.duration)
                  for e in raw if not e.note_off]
        print(f"\nAll tracks merged  (speed={speed}x)\n")

    print(f"{'#':<4} {'Time(s)':<9} {'Note':<6} {'String':<8} {'Fret':<6} {'Dur(s)':<8}")
    print("-" * 45)
    for i, (midi_note, s, fret, start, dur) in enumerate(events):
        print(f"{i+1:<4} {start:<9.2f} {midi_note_name(midi_note):<6} {STRING_NAMES[s]:<8} {fret:<6} {dur:<8.2f}")

    print(f"\nTotal notes: {len(events)}")
    if events:
        last = events[-1]
        print(f"Duration:    {last[3] + last[4]:.2f}s")
    print(f"Speed mult:  {speed}x")

if __name__ == "__main__":
    main()
