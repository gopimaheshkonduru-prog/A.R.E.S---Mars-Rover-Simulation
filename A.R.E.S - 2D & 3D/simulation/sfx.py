"""Tiny sound-effect generator (standard library only).

Writes a few short beeps as .wav files so main_3d.py can play them with
Ursina's built-in Audio - no external sound files to download.
"""
import math
import os
import struct
import wave

RATE = 22050


def _beep(path, notes, volume=0.45):
    """notes = [(frequency_hz, seconds), ...]; (0, s) is silence."""
    frames = bytearray()
    for freq, secs in notes:
        n = int(RATE * secs)
        for i in range(n):
            if freq <= 0:
                v = 0.0
            else:
                # short fade in/out so there is no click
                env = min(1.0, i / 120.0, (n - i) / 400.0)
                v = volume * env * math.sin(2 * math.pi * freq * i / RATE)
            frames += struct.pack("<h", int(v * 32767))
    with wave.open(path, "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(RATE)
        f.writeframes(bytes(frames))


SOUNDS = {
    "pickup": [(660, 0.07), (990, 0.14)],                       # rising ping
    "low": [(440, 0.16), (0, 0.05), (330, 0.22)],               # falling 2-tone
    "critical": [(880, 0.09), (0, 0.06)] * 3,                   # fast triple
}


def make_sounds(folder):
    """Create any missing .wav in `folder`; return {name: absolute path}."""
    os.makedirs(folder, exist_ok=True)
    paths = {}
    for name, notes in SOUNDS.items():
        path = os.path.join(folder, name + ".wav")
        if not os.path.exists(path):
            _beep(path, notes)
        paths[name] = path
    return paths
