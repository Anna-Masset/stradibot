"""
Generate a simple test MIDI file — scale up and down across all 4 violin strings.
Output: test_scale.mid

Notes (open strings): G3=55, D4=62, A4=69, E5=76
Scale: one note per string going up, then back down.
"""

import mido

# One note per string, open string notes
notes = [55, 62, 69, 76, 69, 62, 55]  # G D A E A D G
note_duration = 5.0   # seconds per note
gap = 2.0             # silence between notes

tempo = 500000        # 120 BPM
ticks_per_beat = 480

def secs_to_ticks(secs):
    return int(secs * ticks_per_beat * 1e6 / tempo)

mid = mido.MidiFile(ticks_per_beat=ticks_per_beat)
track = mido.MidiTrack()
mid.tracks.append(track)

track.append(mido.MetaMessage('set_tempo', tempo=tempo, time=0))

current_tick = 0

for note in notes:
    on_tick  = secs_to_ticks(0)           # relative: note on immediately
    off_tick = secs_to_ticks(note_duration)  # relative: note off after duration
    gap_tick = secs_to_ticks(gap)

    track.append(mido.Message('note_on',  note=note, velocity=80, time=on_tick))
    track.append(mido.Message('note_off', note=note, velocity=0,  time=off_tick))
    # silence gap before next note (time=0 on next note_on would work too)
    track.append(mido.Message('note_on',  note=0,    velocity=0,  time=gap_tick))  # dummy gap

track.append(mido.MetaMessage('end_of_track', time=0))

out = "test_scale.mid"
mid.save(out)
print(f"Saved {out}  ({len(notes)} notes, {note_duration}s each, {gap}s gap)")
