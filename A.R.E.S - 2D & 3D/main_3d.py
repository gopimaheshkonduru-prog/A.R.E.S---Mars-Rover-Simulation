"""
ARES 3D - Autonomous Robotic Exploration System (final 3D implementation)

Run from anywhere:
    python main_3d.py            # random Mars map
    python main_3d.py 42         # reproducible map (seed 42)

This is a 3D view of the SAME simulation as main.py: your A*, PID,
ray-sensor avoidance, battery, mission and telemetry code is untouched.
Everything extra (samples, charger, auto-explore, camera modes...) lives in
simulation/world.py and this file.

Controls
    Left click ..... send the rover to that spot (planner + PID)
    P .............. switch route planner: A* -> Theta* -> RL (Q-learning)
    E .............. toggle AUTO-EXPLORE (collects samples, recharges itself)
    C .............. drive to the charging station
    1 2 3 4 ........ camera: Top / Orbit / Chase / Cockpit
    Arrows ......... orbit camera (left/right) and tilt (up/down)
    Mouse wheel .... zoom
    Space .......... pause        Z / X ... slower / faster simulation
    B .............. show/hide sensor beams
    G .............. ghost replay of reports/telemetry.csv (toggle)
    M .............. mute / unmute sound effects
    H .............. collapse / expand the dashboard (or click its header)
    T .............. dashboard: show / hide the full telemetry table
    N .............. collapse / expand the minimap   (or click its header)
    R .............. new random map
    Esc ............ quit (telemetry saved + reports/analyze.py is run)
"""
import atexit
import csv
import math
import os
import random
import sys
import time as pytime
from bisect import bisect_right
from collections import deque
from pathlib import Path

BASE = os.path.dirname(os.path.abspath(__file__))
os.chdir(BASE)                       # keep relative paths (reports/...) valid
sys.path.insert(0, BASE)

from PIL import Image, ImageDraw
from ursina import (Ursina, Entity, Text, Texture, Vec2, Vec3, camera,
                    window, held_keys, mouse, destroy, application, Audio)
from ursina.color import Color

from dashboard import dashboard
from simulation import sfx
from simulation.world import World, LOW_BATTERY, SOLAR_RATE

STEP = 1.0 / 60.0                    # fixed simulation step (seconds)
SPEEDS = [0.5, 1, 2, 4, 8]
CAMERA_NAMES = ["TOP", "ORBIT", "CHASE", "COCKPIT"]
CAMERA_FOV = [40, 40, 55, 75]
WHEEL_RADIUS = 0.12


# ----------------------------------------------------------------- helpers
def rgb(r, g, b, a=255):
    return Color(r / 255, g / 255, b / 255, a / 255)


def yaw(theta):
    """Sim heading (rad) -> Ursina rotation_y (deg).

    Mapping used everywhere: sim x -> world x, sim y -> world -z, so a
    top-down camera looks exactly like the original 2D window.
    """
    return 90 + math.degrees(theta)


def make_ground_texture(cols, rows, seed, surface=None):
    """Procedural Martian sand: per-tile tint + speckles + faint grid."""
    rng = random.Random(seed)
    px = 32
    img = Image.new("RGB", (cols * px, rows * px))
    d = ImageDraw.Draw(img)
    for r in range(rows):
        for c in range(cols):
            k = rng.randint(-9, 9)
            kind = surface[r][c] if surface else None
            col = (178 + k, 106 + k, 64 + k)
            if kind == "soft":                       # pale, powdery sand
                col = (219 + k, 170 + k, 104 + k)
            elif kind == "slope":                    # darker, hatched slope
                col = (150 + k, 88 + k, 54 + k)
            x0, y0 = c * px, r * px
            d.rectangle([x0, y0, x0 + px - 1, y0 + px - 1], fill=col)
            if kind == "slope":
                for i in range(8, px, 8):
                    d.line([(x0 + i, y0), (x0, y0 + i)], fill=(118, 66, 38))
                    d.line([(x0 + px - 1, y0 + i), (x0 + i, y0 + px - 1)],
                           fill=(118, 66, 38))
    for _ in range(cols * rows * 40):
        x, y = rng.randrange(cols * px), rng.randrange(rows * px)
        k = rng.randint(-30, 24)
        d.point((x, y), fill=(178 + k, 106 + k, 64 + k))
    for c in range(cols + 1):
        d.line([(c * px, 0), (c * px, rows * px)], fill=(152, 88, 52))
    for r in range(rows + 1):
        d.line([(0, r * px), (cols * px, r * px)], fill=(152, 88, 52))
    return Texture(img)


COL_BEAM_OK = rgb(100, 255, 100)            # green  : nothing close
COL_BEAM_NEAR = rgb(255, 160, 30)           # orange : object inside SAFE_DISTANCE
COL_BEAM_WARN = rgb(255, 100, 25)           # deep orange : warning distance
COL_BEAM_STOP = rgb(255, 60, 60)            # red    : emergency distance
SAFE_DISTANCE = 0.60                        # metres of clearance before a beam turns orange

FANCY_PROPS = True      # False = lighter scene (no pebbles / crater rim stones,
                        # fewer dust puffs) if the frame rate is low

# dashboard + minimap look
HUD_W = 0.58                                # dashboard panel width (screen units)
HUD_HEAD_H = 0.036                          # header strip height
HUD_SCALE = 0.80                            # text size
HUD_LH = 0.025 * HUD_SCALE                  # height of one text line
HUD_LABELS = ["Position", "Heading", "Wheels L/R", "Target error", "Mission",
              "Distance", "Mode", "Clock", "Storm", "Ghost replay", "Telemetry"]
HUD_TS = 0.72                               # compact dashboard text size
HUD_TL = 0.025 * HUD_TS                     # height of one compact text line
COL_PANEL = rgb(14, 18, 26, 175)
COL_HEAD = rgb(32, 44, 64, 240)
COL_HEAD_HOT = rgb(54, 76, 110, 250)
COL_ACCENT = rgb(255, 170, 40)
COL_LABEL = rgb(150, 165, 185)
COL_VALUE = rgb(240, 244, 250)
COL_BAR_BG = rgb(40, 48, 62)
# The robot's "brain" is a pipeline. The dashboard lights up the stage that
# is in control right now:  planner (A* / Theta* / RL) -> PID (steers along it)
# -> RAYS (sensor safety layer that brakes / dodges what the map missed).
CHIP_TEXT = ("A*  ROUTE", "PID  STEER", "RAYS  SAFETY")
PLANNER_EXPLAIN = {
    "A*": "Plans the cheapest safe route on the grid",
    "Theta*": "Any-angle A*: straighter routes, fewer turns",
    "RL": "Q-learning agent learns the route by trial",
}
EXPLAIN = (("A*", "Plans the cheapest safe route on the grid"),
           ("PID", "Turns heading error into wheel speeds"),
           ("Rays", "Brake or dodge what the map missed"))


def wrap(text, width=50, lines=2):
    """Greedy word-wrap, cut to `lines` lines so the dashboard never overflows."""
    out, cur = [], ""
    for word in text.split():
        if cur and len(cur) + 1 + len(word) > width:
            out.append(cur)
            cur = word
        else:
            cur = (cur + " " + word).strip()
    if cur:
        out.append(cur)
    return "\n".join(out[:lines])


COL_PATH = rgb(255, 220, 0)
COL_PATH_DONE = rgb(150, 120, 40)
COL_TRAIL = rgb(110, 54, 26)


# ----------------------------------------------------------- small layers
class PathLayer:
    """Yellow route waypoints + connecting segments (rebuilt when path changes)."""

    def __init__(self):
        self.key = None
        self.items = []
        self.dots = []
        self.last_index = -1

    def clear(self):
        for e in self.items:
            destroy(e)
        self.items, self.dots, self.key, self.last_index = [], [], None, -1

    def sync(self, path, index):
        key = tuple(path)
        if key != self.key:
            self.clear()
            self.key = key
            pts = [(c + 0.5, 0.04, -(r + 0.5)) for r, c in path]
            for i, (px, py, pz) in enumerate(pts):
                dot = Entity(model="sphere", position=(px, py, pz),
                             scale=(0.16, 0.05, 0.16), color=COL_PATH)
                self.items.append(dot)
                self.dots.append(dot)
                if i:
                    qx, _, qz = pts[i - 1]
                    dx, dz = px - qx, pz - qz
                    seg = Entity(model="cube",
                                 position=((px + qx) / 2, py, (pz + qz) / 2),
                                 scale=(0.05, 0.015, math.hypot(dx, dz)),
                                 rotation_y=math.degrees(math.atan2(dx, dz)),
                                 color=rgb(200, 170, 0))
                    self.items.append(seg)
        if index != self.last_index:
            self.last_index = index
            for i, dot in enumerate(self.dots):
                dot.color = COL_PATH_DONE if i < index else COL_PATH


class Trail:
    """Wheel tracks left in the sand (oldest marks are recycled)."""

    def __init__(self, limit=150):
        self.marks = deque()
        self.limit = limit
        self.last = None

    def clear(self):
        for e in self.marks:
            destroy(e)
        self.marks.clear()
        self.last = None

    def update(self, x, y, theta, moving):
        if not moving:
            return
        if self.last and math.hypot(x - self.last[0], y - self.last[1]) < 0.28:
            return
        self.last = (x, y)
        a = yaw(theta)
        rx, rz = math.cos(math.radians(a)), -math.sin(math.radians(a))
        for side in (-0.33, 0.33):
            self.marks.append(Entity(
                model="quad", rotation=(90, a, 0), scale=(0.07, 0.30),
                position=(x + rx * side, 0.011, -y + rz * side),
                color=COL_TRAIL, double_sided=True))
        while len(self.marks) > self.limit * 2:
            destroy(self.marks.popleft())



class DustPuffs:
    """Short-lived dust clouds kicked up behind the rear wheels."""

    def __init__(self, n=48):
        self.free = [Entity(model="sphere", enabled=False) for _ in range(n)]
        self.live = []                    # [entity, age, life, y0, size]
        self.last = None

    def clear(self):
        for p in self.live:
            p[0].enabled = False
            self.free.append(p[0])
        self.live = []
        self.last = None

    def update(self, dt, x, y, theta, speed, storm):
        if abs(speed) > 0.08 and (
                self.last is None
                or math.hypot(x - self.last[0], y - self.last[1]) > 0.22):
            self.last = (x, y)
            a = yaw(theta)
            fx, fz = math.sin(math.radians(a)), math.cos(math.radians(a))
            rx, rz = math.cos(math.radians(a)), -math.sin(math.radians(a))
            for side in (-0.33, 0.33):
                if not self.free:
                    break
                e = self.free.pop()
                e.position = Vec3(x - fx * 0.34 + rx * side, 0.08,
                                  -y - fz * 0.34 + rz * side)
                e.enabled = True
                size = 0.30 if storm else 0.18
                self.live.append([e, 0.0, random.uniform(0.7, 1.1),
                                  0.08, size])
        keep = []
        for p in self.live:
            e, age, life, y0, size = p
            p[1] = age = age + dt
            f = age / life
            if f >= 1.0:
                e.enabled = False
                self.free.append(e)
                continue
            e.scale = size * (0.5 + 1.6 * f)
            e.y = y0 + 0.30 * age
            e.color = rgb(205, 150, 105, int(120 * (1 - f)))
            keep.append(p)
        self.live = keep


class Ghost:
    """Plays reports/telemetry.csv back as a translucent ghost rover."""

    def __init__(self):
        self.t, self.pose = [], []        # times, (x, y, theta)
        self.active = False
        self.clock = 0.0
        self.note = ""
        self.root = Entity(enabled=False)
        tint = Color(0.65, 0.9, 1.0, 0.45)
        for model, scale, pos in (
                ("cube", (0.50, 0.13, 0.80), (0, 0.30, 0)),
                ("cube", (0.68, 0.025, 0.50), (0, 0.42, -0.05)),
                ("cube", (0.04, 0.36, 0.04), (0.14, 0.58, 0.22))):
            Entity(parent=self.root, model=model, color=tint, scale=scale,
                   position=pos)
        for sx in (-0.33, 0.33):
            for sz in (-0.30, 0.0, 0.30):
                Entity(parent=self.root, model="cube", color=tint,
                       scale=(0.10, 0.24, 0.24), position=(sx, 0.12, sz))

    def stop(self):
        self.active = False
        self.root.enabled = False
        self.note = ""

    def toggle(self, current_seed):
        if self.active:
            self.stop()
            return
        self.t, self.pose = [], []
        try:
            with open("reports/telemetry.csv", newline="") as f:
                for row in csv.DictReader(f):
                    self.t.append(float(row["time"]))
                    self.pose.append((float(row["x"]), float(row["y"]),
                                      float(row["theta"])))
        except (OSError, KeyError, ValueError) as err:
            print("Ghost: cannot read reports/telemetry.csv:", err)
        if len(self.t) < 2:
            self.note = "no telemetry.csv to replay"
            return
        self.active, self.clock = True, 0.0
        self.root.enabled = True
        rec = dashboard.recorded_seed()
        self.note = ""
        if rec is not None and rec != current_seed:
            self.note = f"recorded on map seed {rec}"

    def current(self):
        """Interpolated (x, y, theta) at the playback clock."""
        i = bisect_right(self.t, self.clock)
        if i >= len(self.t):
            return self.pose[-1]
        if i == 0:
            return self.pose[0]
        t0, t1 = self.t[i - 1], self.t[i]
        k = (self.clock - t0) / (t1 - t0) if t1 > t0 else 0.0
        (x0, y0, a0), (x1, y1, a1) = self.pose[i - 1], self.pose[i]
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi   # shortest turn
        return x0 + (x1 - x0) * k, y0 + (y1 - y0) * k, a0 + da * k

    def update(self, sim_dt):
        if not self.active:
            return None
        self.clock += sim_dt
        x, y, th = self.current()
        self.root.position = Vec3(x, 0, -y)
        self.root.rotation_y = yaw(th)
        return x, y

    def hud(self):
        if not self.active:
            return self.note or "off (G = replay telemetry.csv)"
        done = self.clock >= self.t[-1]
        s = f"{min(self.clock, self.t[-1]):5.1f}/{self.t[-1]:.1f}s"
        return ("finished  " if done else "replaying ") + s + (
            "  " + self.note if self.note else "")


class SoundFx:
    """Pickup / low-battery beeps via Ursina's Audio (never crashes the sim)."""

    def __init__(self):
        self.clips, self.muted = {}, False
        try:
            paths = sfx.make_sounds(os.path.join(BASE, "assets", "sfx"))
            for name, path in paths.items():
                self.clips[name] = Audio(Path(path), autoplay=False,
                                         loop=False, volume=1)
        except Exception as err:
            print("Sound unavailable:", err)

    def play(self, name):
        clip = self.clips.get(name)
        if clip and not self.muted:
            try:
                clip.play()
            except Exception as err:
                print("Sound error:", err)


# -------------------------------------------------------------------- game
class Game(Entity):
    def __init__(self, seed=None):
        super().__init__()
        self.sim = World(seed)
        atexit.register(self.sim.save_telemetry)
        dashboard.install(self.sim)        # run reports/analyze.py on quit

        self.static = []              # map entities (rebuilt on new map)
        self.sample_vis = []
        self.ground = None
        self.charger_light = None
        self.speed_i = 1
        self.cam_mode = 1             # start with the ORBIT overview
        self.orbit_yaw = 0.0
        self.orbit_pitch = 48.0
        self.zoom = 1.0
        self.show_beams = True
        self.clock_last = pytime.perf_counter()
        self.acc = 0.0
        self.t = 0.0
        self.hud_timer = 1.0
        self.led_state = None
        self.storm_vis = []
        self.mini = None
        self.mini_box = None
        self.hud_open = True          # dashboard expanded?
        self.hud_more = False         # full telemetry table shown (T)?
        self.plan_key = None          # last route A* handed to the rover
        self.plan_flash = 0.0         # seconds the A* chip stays lit
        self.plan_info = ("planned", 0, 0.0)
        self.map_open = True          # minimap expanded?
        self._hud_cache = {}

        self.path_layer = PathLayer()
        self.trail = Trail()
        self.dust = DustPuffs()
        self.ghost = Ghost()
        self.sfx = SoundFx()
        self.last_collected = 0
        self.low_flag = False
        self.crit_clock = -99.0
        self.build_scene()
        self.build_rover()
        self.build_beacon()
        self.build_hud()
        self.build_minimap()
        self.set_camera_mode(self.cam_mode)

    # ------------------------------------------------------------- helpers
    def mk(self, **kw):
        """Create an Entity and remember it if it belongs to the map."""
        e = Entity(**kw)
        if "parent" not in kw:
            self.static.append(e)
        return e

    # --------------------------------------------------------------- scene
    def build_scene(self):
        for e in self.static:
            destroy(e)
        self.static = []
        self.sample_vis = []
        sim = self.sim
        terrain = sim.terrain
        cols, rows = terrain.cols, terrain.rows
        rng = random.Random(sim.seed)

        try:
            tex = make_ground_texture(cols, rows, sim.seed, terrain.surface)
        except Exception as err:             # never let cosmetics crash the sim
            print("Ground texture unavailable, using flat colour:", err)
            tex = None
        self.ground = self.mk(
            model="plane", texture=tex,
            color=Color(1, 1, 1, 1) if tex else rgb(176, 104, 62),
            scale=(cols, 1, rows), position=(cols / 2, 0, -rows / 2),
            collider="mesh", double_sided=True)
        self.mk(model="plane", scale=(300, 1, 300), double_sided=True,
                position=(cols / 2, -0.06, -rows / 2), color=rgb(150, 88, 52))

        # canyon walls around the arena
        t, h = 0.6, 0.5
        wall = rgb(122, 70, 42)
        self.mk(model="cube", color=wall, scale=(cols + 2 * t, h, t),
                position=(cols / 2, h / 2, t / 2))
        self.mk(model="cube", color=wall, scale=(cols + 2 * t, h, t),
                position=(cols / 2, h / 2, -rows - t / 2))
        self.mk(model="cube", color=wall, scale=(t, h, rows),
                position=(-t / 2, h / 2, -rows / 2))
        self.mk(model="cube", color=wall, scale=(t, h, rows),
                position=(cols + t / 2, h / 2, -rows / 2))

        for r in range(rows):
            for c in range(cols):
                kind = terrain.map[r][c]
                if kind == "rock":
                    self.build_rock(c, r, rng)
                elif kind == "crater":
                    self.build_crater(c, r)

        self.build_charger()
        for s in sim.samples:
            self.build_sample(s)
        self.build_storms()

    # ---------------------------------------------------------------- rocks
    # Three different rock designs. Which one sits on a cell depends only on
    # (map seed, cell), so the same seed always gives the same rocks.
    #   boulder - round, grey-brown, medium height
    #   crag    - tall, sharp, dark blue-grey shards
    #   slab    - low, wide, layered rusty sandstone ledges
    ROCK_TYPES = ("boulder", "crag", "slab")

    def rock_kind(self, c, r):
        pick = random.Random(self.sim.seed * 7919 + r * 131 + c).random()
        return self.ROCK_TYPES[min(2, int(pick * 3))]

    def build_rock(self, c, r, rng):
        kind = self.rock_kind(c, r)
        if kind == "crag":
            self.rock_crag(c, r, random.Random(self.sim.seed * 104729 + r * 977 + c * 31 + 5))
        elif kind == "slab":
            self.rock_slab(c, r, random.Random(self.sim.seed * 130363 + r * 463 + c * 17 + 9))
        else:
            self.rock_boulder(c, r, rng)

    def rock_boulder(self, c, r, rng):
        """Type 1 - BOULDER: round, weathered grey-brown stone (original design)."""
        g = rng.randint(92, 124)
        base = rgb(g, g - 6, g - 14)
        mid = rgb(g + 16, g + 8, g - 2)
        top = rgb(g + 36, g + 28, g + 16)
        glint = rgb(g + 58, g + 48, g + 34)
        dark = rgb(max(g - 34, 0), max(g - 40, 0), max(g - 48, 0))
        h = rng.uniform(0.6, 1.05)
        root = self.mk(position=(c + 0.5, 0, -(r + 0.5)),
                       rotation_y=rng.uniform(0, 360))
        Entity(parent=root, model="circle", rotation_x=90, scale=1.30,
               y=0.012, color=Color(0, 0, 0, 0.30), double_sided=True)
        Entity(parent=root, model="sphere", color=dark,
               scale=(1.04, 0.26, 1.00), y=0.07)
        Entity(parent=root, model="sphere", color=base,
               scale=(0.95, h, 0.9), y=h * 0.32)
        Entity(parent=root, model="sphere", color=mid,
               scale=(0.70, h * 0.75, 0.80), x=0.14, z=0.10, y=h * 0.30,
               rotation_y=rng.uniform(0, 60))
        Entity(parent=root, model="sphere", color=top,
               scale=(0.62, h * 0.5, 0.58), y=h * 0.62, x=-0.04)
        Entity(parent=root, model="sphere", color=glint,
               scale=(0.26, h * 0.20, 0.24), y=h * 0.86, x=-0.10, z=-0.06)
        Entity(parent=root, model="sphere", color=base,
               scale=(0.40, 0.30, 0.40), x=0.30, z=-0.25, y=0.10)
        for _ in range(3 if FANCY_PROPS else 0):             # loose pebbles
            a = rng.uniform(0, 2 * math.pi)
            d = rng.uniform(0.36, 0.46)
            pr = rng.uniform(0.08, 0.15)
            Entity(parent=root, model="sphere", color=mid,
                   scale=(pr * 1.2, pr * 0.7, pr),
                   position=(math.cos(a) * d, pr * 0.3, math.sin(a) * d))

    def rock_crag(self, c, r, rng):
        """Type 2 - CRAG: three tall, sharp, faceted shards of dark basalt."""
        g = rng.randint(58, 80)
        base = rgb(g, g + 2, g + 14)                  # cool blue-grey basalt
        mid = rgb(g + 22, g + 24, g + 38)
        top = rgb(g + 52, g + 54, g + 70)             # pale, frosted tips
        dark = rgb(max(g - 12, 0), max(g - 12, 0), max(g - 2, 0))
        root = self.mk(position=(c + 0.5, 0, -(r + 0.5)),
                       rotation_y=rng.uniform(0, 360))
        Entity(parent=root, model="circle", rotation_x=90, scale=1.25,
               y=0.012, color=Color(0, 0, 0, 0.30), double_sided=True)
        Entity(parent=root, model="cube", color=dark,
               scale=(0.84, 0.12, 0.78), y=0.06,
               rotation_y=rng.uniform(-12, 12))
        shards = ((0.00, 0.02, 0.42, rng.uniform(0.80, 0.98), base),
                  (-0.27, -0.20, 0.30, rng.uniform(0.52, 0.68), mid),
                  (0.26, 0.20, 0.28, rng.uniform(0.42, 0.56), mid))
        for sx, sz, w, hgt, col in shards:
            lean_x, lean_z = rng.uniform(-9, 9), rng.uniform(-9, 9)
            turn = rng.uniform(0, 90)
            Entity(parent=root, model="cube", color=col,
                   scale=(w, hgt, w * 0.85),
                   position=(sx, hgt / 2 + 0.06, sz),
                   rotation=(lean_x, turn, lean_z))
            # chisel point: a cube turned 45 deg on its side, sitting on top
            Entity(parent=root, model="cube", color=top,
                   scale=(w * 0.95, w * 0.62, w * 0.62),
                   position=(sx, hgt + 0.02, sz),
                   rotation=(lean_x + 45, turn, lean_z))
        for _ in range(3 if FANCY_PROPS else 0):             # loose scree
            a = rng.uniform(0, 2 * math.pi)
            d = rng.uniform(0.36, 0.46)
            pr = rng.uniform(0.07, 0.13)
            Entity(parent=root, model="cube", color=mid,
                   scale=(pr, pr * 0.8, pr * 1.2),
                   rotation=(rng.uniform(0, 60), rng.uniform(0, 90), 0),
                   position=(math.cos(a) * d, pr * 0.35, math.sin(a) * d))

    def rock_slab(self, c, r, rng):
        """Type 3 - SLAB: low, wide, layered sandstone ledges + a leaning plate."""
        k = rng.randint(-8, 8)
        bands = (rgb(168 + k, 104 + k, 70 + k),       # rusty ochre
                 rgb(138 + k, 78 + k, 54 + k),        # dark rust
                 rgb(190 + k, 130 + k, 88 + k),       # pale sand band
                 rgb(150 + k, 92 + k, 62 + k))
        root = self.mk(position=(c + 0.5, 0, -(r + 0.5)),
                       rotation_y=rng.uniform(0, 360))
        Entity(parent=root, model="circle", rotation_x=90, scale=1.35,
               y=0.012, color=Color(0, 0, 0, 0.30), double_sided=True)
        y = 0.0
        for i, (w, d, t) in enumerate(((0.98, 0.94, 0.17), (0.86, 0.80, 0.15),
                                       (0.70, 0.66, 0.14), (0.50, 0.46, 0.12))):
            jitter = 0.0 if i == 0 else 0.05
            Entity(parent=root, model="cube", color=bands[i],
                   scale=(w, t, d),
                   position=(rng.uniform(-jitter, jitter), y + t / 2,
                             rng.uniform(-jitter, jitter)),
                   rotation_y=rng.uniform(-14, 14))
            y += t
        Entity(parent=root, model="cube", color=bands[2],       # leaning plate
               scale=(0.46, 0.50, 0.07), position=(0.26, 0.30, -0.26),
               rotation=(0, 35, 22))
        for _ in range(3 if FANCY_PROPS else 0):             # flat flakes
            a = rng.uniform(0, 2 * math.pi)
            d = rng.uniform(0.38, 0.48)
            pr = rng.uniform(0.07, 0.12)
            Entity(parent=root, model="cube", color=bands[rng.randrange(4)],
                   scale=(pr * 1.4, pr * 0.4, pr), rotation_y=rng.uniform(0, 180),
                   position=(math.cos(a) * d, pr * 0.2, math.sin(a) * d))

    def build_crater(self, c, r):
        root = self.mk(position=(c + 0.5, 0, -(r + 0.5)))
        for size, y, col in ((1.08, 0.010, rgb(164, 104, 66)),
                             (1.05, 0.012, rgb(150, 92, 56)),
                             (0.88, 0.016, rgb(96, 56, 34)),
                             (0.62, 0.020, rgb(52, 32, 22)),
                             (0.30, 0.024, rgb(28, 18, 14))):
            Entity(parent=root, model="circle", rotation_x=90, scale=size,
                   y=y, color=col, double_sided=True)
        for i in range(14 if FANCY_PROPS else 0):            # raised rim
            a = i / 14 * 2 * math.pi
            rim = (rgb(204, 144, 96) if i % 2 == 0 else rgb(178, 120, 78))
            sz = 0.15 + 0.03 * (i % 3)
            Entity(parent=root, model="sphere", color=rim,
                   scale=(sz, 0.07, sz),
                   position=(math.cos(a) * 0.45, 0.035, math.sin(a) * 0.45))

    def build_charger(self):
        sim = self.sim
        root = self.mk(position=(sim.charger_x, 0, -sim.charger_y))
        Entity(parent=root, model="cube", color=rgb(240, 200, 40),
               scale=(1.12, 0.04, 1.12), y=0.02)
        Entity(parent=root, model="cube", color=rgb(55, 65, 88),
               scale=(1.0, 0.06, 1.0), y=0.045)
        Entity(parent=root, model="cube", color=rgb(240, 200, 40),
               scale=(0.55, 0.02, 0.10), y=0.085)
        Entity(parent=root, model="cube", color=rgb(240, 200, 40),
               scale=(0.10, 0.02, 0.55), y=0.085)
        Entity(parent=root, model="cube", color=rgb(190, 190, 200),
               scale=(0.06, 0.9, 0.06), position=(0.42, 0.45, 0.42))
        Entity(parent=root, model="cube", color=rgb(30, 70, 150),
               scale=(0.7, 0.04, 0.5), position=(0.42, 0.95, 0.42),
               rotation_x=-25)
        self.charger_light = Entity(
            parent=root, model="sphere", color=rgb(90, 90, 90),
            scale=0.13, position=(0.42, 1.02, 0.42))

    def build_storms(self):
        self.storm_vis = []
        for si, st in enumerate(self.sim.storms):
            srng = random.Random(self.sim.seed * 31 + si)
            root = self.mk(position=(st.x, 0, -st.y))
            # soft ground haze: stacked discs, bigger = fainter (fuzzy edge)
            for k, (f, a) in enumerate(((1.00, 26), (0.82, 30),
                                        (0.64, 34), (0.46, 38))):
                Entity(parent=root, model="circle", rotation_x=90,
                       scale=st.radius * 2 * f, y=0.03 + 0.003 * k,
                       color=rgb(225, 170, 110, a), double_sided=True)
            # dust vortex: rings of puffs, narrower and higher each level,
            # every ring spins at its own speed (and direction)
            levels = ((0.16, 0.95, 11, 55, 125), (0.38, 0.78, 9, -42, 105),
                      (0.62, 0.58, 7, 70, 85), (0.88, 0.38, 5, -60, 65))
            for lvl, (y, rf, n, spin, alpha) in enumerate(levels):
                if not FANCY_PROPS:
                    n = max(3, n // 2)
                swirl = Entity(parent=root)
                swirl.spin = spin
                for i in range(n):
                    a = i / n * 2 * math.pi + srng.uniform(-0.25, 0.25)
                    rr = st.radius * rf * srng.uniform(0.55, 1.0)
                    sz = srng.uniform(0.50, 0.90) * (1.0 - 0.10 * lvl)
                    Entity(parent=swirl, model="sphere",
                           scale=(sz * 1.1, sz * 0.45, sz),
                           position=(math.cos(a) * rr,
                                     y + srng.uniform(-0.04, 0.04),
                                     math.sin(a) * rr),
                           color=rgb(205 + 6 * lvl, 150 + 8 * lvl,
                                     95 + 9 * lvl, alpha))
                self.storm_vis.append(swirl)

    def build_sample(self, s):
        """Science sample site: marker plate, sealed sample case, survey flag.

        Nothing floats or spins: the flag sways, the status light blinks and
        a detection ring pulses on the ground (all animated in sync_visuals).
        """
        root = self.mk(position=(s.x, 0, -s.y))
        Entity(parent=root, model="cube", color=rgb(225, 225, 228),
               scale=(0.52, 0.030, 0.52), y=0.015)                 # plate
        Entity(parent=root, model="cube", color=rgb(235, 105, 25),
               scale=(0.42, 0.034, 0.42), y=0.017)                 # inset
        Entity(parent=root, model="cube", color=rgb(62, 70, 82),
               scale=(0.24, 0.15, 0.17), position=(0.02, 0.11, 0.04))
        Entity(parent=root, model="cube", color=rgb(185, 190, 200),
               scale=(0.25, 0.025, 0.18), position=(0.02, 0.20, 0.04))
        Entity(parent=root, model="cube", color=rgb(235, 105, 25),
               scale=(0.10, 0.04, 0.012), position=(0.02, 0.12, -0.05))
        led = Entity(parent=root, model="sphere", color=rgb(255, 170, 40),
                     scale=0.045, position=(0.10, 0.225, 0.04))
        Entity(parent=root, model="cube", color=rgb(200, 200, 205),
               scale=(0.018, 0.62, 0.018), position=(-0.18, 0.31, -0.14))
        pivot = Entity(parent=root, position=(-0.18, 0.55, -0.14))
        Entity(parent=pivot, model="cube", color=rgb(255, 110, 20),
               scale=(0.20, 0.11, 0.010), x=0.10)                  # flag
        ring = self.mk(model="circle", rotation_x=90, scale=0.5, y=0.03,
                       position=(s.x, 0.03, -s.y),
                       color=rgb(255, 150, 40, 120), double_sided=True)
        mark = self.mk(model="circle", rotation_x=90, scale=0.34,
                       position=(s.x, 0.014, -s.y), color=rgb(92, 56, 34),
                       double_sided=True, enabled=False)           # dug spot
        self.sample_vis.append((s, root, ring, pivot, led, mark))

    # --------------------------------------------------------------- rover
    def build_rover(self):
        rv = Entity()
        self.rv = rv

        def part(model, col, scale, pos, **kw):
            return Entity(parent=rv, model=model, color=col, scale=scale,
                          position=pos, **kw)

        part("circle", Color(0, 0, 0, 0.35), 0.95, (0, 0.014, 0),
             rotation_x=90, double_sided=True)                  # shadow blob
        part("cube", rgb(236, 236, 240), (0.50, 0.13, 0.80), (0, 0.30, 0))
        part("cube", rgb(210, 120, 40), (0.34, 0.05, 0.72), (0, 0.37, 0))
        part("cube", rgb(30, 60, 135), (0.68, 0.025, 0.50), (0, 0.42, -0.05))
        part("cube", rgb(205, 205, 210), (0.04, 0.36, 0.04), (0.14, 0.58, 0.22))
        part("cube", rgb(40, 40, 48), (0.20, 0.09, 0.10), (0.14, 0.78, 0.22))
        part("sphere", rgb(120, 200, 255), 0.05, (0.09, 0.78, 0.28))
        part("sphere", rgb(120, 200, 255), 0.05, (0.19, 0.78, 0.28))
        part("cube", rgb(255, 140, 30), (0.30, 0.07, 0.06), (0, 0.30, 0.42))
        self.led = part("sphere", rgb(60, 255, 90), 0.07, (0.14, 0.86, 0.22))

        self.wheels = []
        for sx in (-0.33, 0.33):
            for sz in (-0.30, 0.0, 0.30):
                w = Entity(parent=rv, model="cube", color=rgb(38, 38, 44),
                           scale=(0.10, 0.24, 0.24), position=(sx, 0.12, sz))
                Entity(parent=w, model="cube", color=rgb(165, 165, 175),
                       scale=(1.2, 0.25, 0.25), position=(0, 0.38, 0))
                self.wheels.append((sx, w))

        self.beams = [Entity(model="cube", color=COL_BEAM_OK,
                             scale=(0.025, 0.025, 1))
                      for _ in self.sim.rover.sensor_angles]

    def build_beacon(self):
        self.beacon = Entity(enabled=False)
        Entity(parent=self.beacon, model="cube", color=rgb(235, 235, 240),
               scale=(0.035, 1.2, 0.035), y=0.6)
        self.beacon_top = Entity(parent=self.beacon, model="sphere",
                                 color=rgb(255, 235, 60), scale=0.22, y=1.25)
        Entity(parent=self.beacon, model="circle", rotation_x=90, scale=0.9,
               y=0.03, color=rgb(255, 235, 60, 150), double_sided=True)

    # ----------------------------------------------------------------- HUD
    def build_hud(self):
        # night overlay: sits behind every HUD element (larger z = farther)
        self.night = Entity(parent=camera.ui, model="quad", scale=(4, 4),
                            z=0.5, color=Color(0.02, 0.04, 0.16, 0))

        W, HH = HUD_W, HUD_HEAD_H
        self.bar_w = W - 0.030
        # vertical layout, measured down from the top of the panel
        by = -(HH + 0.008 + 0.010)                    # battery bar centre
        chip_h, chip_y = 0.032, -0.092                # pipeline chips
        y_here = -0.110                               # "you are here" marker
        y_now = -0.130                                # plain-language sentence
        y_div1, y_exp = -0.176, -0.184                # what each part does
        y_div2, y_dat = -0.246, -0.254                # live numbers
        y_btn = -0.322                                # "more details" button
        y_more = -0.345                               # full telemetry table
        self.hud_h_min = 0.343
        self.hud_h_more = -y_more + len(HUD_LABELS) * HUD_LH + 0.014

        root = Entity(parent=camera.ui,
                      position=window.top_left + Vec2(0.010, -0.010))
        self.hud_root = root
        self.hud_bg = Entity(parent=root, model="quad", origin=(-0.5, 0.5),
                             z=0.01, scale=(W, self.hud_h_min),
                             color=COL_PANEL)
        # header strip = the click target that collapses / expands
        self.hud_head = Entity(parent=root, model="quad", collider="box",
                               z=0.009, scale=(W, HH),
                               position=(W / 2, -HH / 2), color=COL_HEAD)
        Entity(parent=root, model="quad", z=0.008, scale=(W, 0.003),
               position=(W / 2, -HH), color=COL_ACCENT)
        Text(parent=root, text="A.R.E.S  DASHBOARD", origin=(-0.5, 0),
             position=(0.012, -HH / 2), scale=0.85, color=COL_ACCENT)
        self.hud_right = Text(parent=root, text="", origin=(0.5, 0),
                              position=(W - 0.058, -HH / 2), scale=0.75,
                              color=COL_VALUE)
        self.hud_icon = Text(parent=root, text="[-]", origin=(0.5, 0),
                             position=(W - 0.010, -HH / 2), scale=0.85,
                             color=COL_ACCENT)

        body = Entity(parent=root)
        self.hud_body = body

        # --- battery bar (unchanged) ---
        Entity(parent=body, model="quad", z=0.006, scale=(self.bar_w, 0.020),
               position=(W / 2, by), color=COL_BAR_BG)
        self.hud_bar = Entity(parent=body, model="quad", z=0.005,
                              origin=(-0.5, 0), scale=(self.bar_w, 0.020),
                              position=(0.015, by),
                              color=rgb(90, 220, 110))
        self.hud_batt = Text(parent=body, text="", origin=(-0.5, 0),
                             position=(0.024, by), scale=0.70,
                             color=rgb(255, 255, 255))

        # --- pipeline: planner -> PID -> RAYS, the lit chip is "you are here" ---
        gap = 0.030
        chip_w = (W - 0.030 - 2 * gap) / 3
        self.chip_x = [0.015 + chip_w / 2 + i * (chip_w + gap) for i in range(3)]
        self.chip_bg, self.chip_txt = [], []
        for i, cx in enumerate(self.chip_x):
            self.chip_bg.append(Entity(parent=body, model="quad", z=0.006,
                                       scale=(chip_w, chip_h),
                                       position=(cx, chip_y), color=COL_BAR_BG))
            self.chip_txt.append(Text(parent=body, text=CHIP_TEXT[i],
                                      origin=(0, 0), position=(cx, chip_y),
                                      scale=HUD_TS, color=COL_LABEL))
            if i < 2:
                Text(parent=body, text=">", origin=(0, 0), scale=0.9,
                     position=(cx + chip_w / 2 + gap / 2, chip_y),
                     color=COL_LABEL)
        self.you_here = Text(parent=body, text="^ you are here",
                             origin=(0, 0.5), position=(self.chip_x[0], y_here),
                             scale=0.60, color=COL_ACCENT)
        self.now_txt = Text(parent=body, text="", origin=(-0.5, 0.5),
                            position=(0.016, y_now), scale=HUD_TS,
                            color=COL_VALUE)

        # --- what each part does (the active one is highlighted) ---
        Entity(parent=body, model="quad", z=0.008, scale=(self.bar_w, 0.002),
               position=(W / 2, y_div1), color=COL_BAR_BG)
        self.exp_lbl = []
        self.exp_desc = []
        for i, (name, desc) in enumerate(EXPLAIN):
            y = y_exp - i * HUD_TL
            self.exp_lbl.append(Text(parent=body, text=name, origin=(-0.5, 0.5),
                                     position=(0.016, y), scale=HUD_TS,
                                     color=COL_LABEL))
            self.exp_desc.append(
                Text(parent=body, text=desc, origin=(-0.5, 0.5),
                     position=(0.092, y), scale=HUD_TS,
                     color=rgb(205, 214, 228)))

        # --- where the robot is, right now ---
        Entity(parent=body, model="quad", z=0.008, scale=(self.bar_w, 0.002),
               position=(W / 2, y_div2), color=COL_BAR_BG)
        Text(parent=body, text="Robot\nRoute\nMission", origin=(-0.5, 0.5),
             position=(0.016, y_dat), scale=HUD_TS, color=COL_LABEL)
        self.dat_txt = Text(parent=body, text="", origin=(-0.5, 0.5),
                            position=(0.092, y_dat), scale=HUD_TS,
                            color=COL_VALUE)

        # --- button: show / hide the full telemetry table (key T) ---
        self.hud_more_btn = Entity(parent=body, model="quad", collider="box",
                                   z=0.009, scale=(self.bar_w, 0.026),
                                   position=(W / 2, y_btn), color=COL_HEAD)
        self.more_txt = Text(parent=body, text="", origin=(0, 0),
                             position=(W / 2, y_btn), scale=0.68,
                             color=COL_ACCENT)

        # --- full telemetry table (the original dashboard rows) ---
        more = Entity(parent=body)
        self.hud_more_body = more
        Text(parent=more, text="\n".join(HUD_LABELS), origin=(-0.5, 0.5),
             position=(0.016, y_more), scale=HUD_SCALE, color=COL_LABEL)
        self.hud_vals = Text(parent=more, text="", origin=(-0.5, 0.5),
                             position=(0.158, y_more), scale=HUD_SCALE,
                             color=COL_VALUE)

        self.status = Text(text="", origin=(0, 0), scale=1.3,
                           position=(0, -0.43))
        Text(text=("Click: go   E: auto-explore   C: charger   1-4: camera   "
                   "arrows/wheel: view   Space: pause   Z/X: speed   "
                   "B: beams   G: ghost   M: mute\n"
                   "H: dashboard   T: details   N: map   R: new map"),
             origin=(-0.5, -0.5), scale=0.7,
             position=window.bottom_left + Vec2(0.01, 0.008),
             color=rgb(255, 245, 230))
        self.apply_hud_state()

    def apply_hud_state(self):
        self.hud_body.enabled = self.hud_open
        self.hud_more_body.enabled = self.hud_open and self.hud_more
        if self.hud_open:
            self.hud_bg.scale_y = (self.hud_h_more if self.hud_more
                                   else self.hud_h_min)
        else:
            self.hud_bg.scale_y = HUD_HEAD_H
        self.hud_icon.text = "[-]" if self.hud_open else "[+]"
        self.more_txt.text = ("[-] hide full telemetry   (T)" if self.hud_more
                              else "[+] full telemetry   (T)")
        self._hud_cache = {}                          # force a text refresh

    def toggle_hud(self):
        self.hud_open = not self.hud_open
        self.apply_hud_state()
        self.update_hud()

    def toggle_more(self):
        self.hud_more = not self.hud_more
        self.hud_open = True                          # T also opens the panel
        self.apply_hud_state()
        self.update_hud()

    def set_text(self, widget, key, value):
        """Only rebuild a Text when its string really changed (cheaper)."""
        if self._hud_cache.get(key) != value:
            self._hud_cache[key] = value
            widget.text = value

    # ------------------------------------------------------------- minimap
    def mini_xy(self, x, y):
        t = self.sim.terrain
        return (x / t.cols - 0.5) * self.mini_w, (0.5 - y / t.rows) * self.mini_h

    def build_minimap(self):
        if self.mini_box is not None:
            destroy(self.mini_box)
        sim = self.sim
        t = sim.terrain
        cols, rows = t.cols, t.rows
        W = 0.30
        H = W * rows / cols
        self.mini_w, self.mini_h = W, H
        MH = 0.032                                    # header height

        box = Entity(parent=camera.ui)
        self.mini_box = box
        self.map_x = window.bottom_right.x - 0.02 - W / 2
        self.map_y = window.bottom_right.y + 0.05

        # header strip (always visible) = click target to collapse / expand
        self.map_head = Entity(parent=box, model="quad", collider="box",
                               z=0.009, scale=(W + 0.008, MH),
                               color=COL_HEAD)
        self.map_title = Text(parent=box, text="MAP", origin=(-0.5, 0),
                              scale=0.85, color=COL_ACCENT)
        self.map_icon = Text(parent=box, text="[-]", origin=(0.5, 0),
                             scale=0.85, color=COL_ACCENT)

        root = Entity(parent=box)
        self.mini = root
        # The terrain is drawn from flat coloured tiles (no texture needed).
        Entity(parent=root, model="quad", color=rgb(70, 80, 100),
               scale=(W + 0.008, H + 0.008), z=0.02)
        Entity(parent=root, model="quad", color=rgb(176, 106, 64),
               scale=(W, H), z=0.015)

        tile_col = {"rock": rgb(112, 112, 118), "crater": rgb(34, 34, 40),
                    "soft": rgb(222, 176, 108), "slope": rgb(128, 74, 44)}

        def kind_at(r, c):
            k = t.map[r][c]
            if k in ("rock", "crater"):
                return k
            return t.surface[r][c]                    # "soft" / "slope" / None

        cw, ch = W / cols, H / rows
        for r in range(rows):
            c = 0
            while c < cols:
                k = kind_at(r, c)
                if k not in tile_col:
                    c += 1
                    continue
                c1 = c
                while c1 < cols and kind_at(r, c1) == k:
                    c1 += 1
                Entity(parent=root, model="quad", color=tile_col[k],
                       scale=((c1 - c) * cw, ch),
                       position=(((c + c1) / 2 / cols - 0.5) * W,
                                 (0.5 - (r + 0.5) / rows) * H, 0.01))
                c = c1

        cr, cc = sim.charger_cell
        Entity(parent=root, model="quad", color=rgb(240, 200, 40),
               scale=(cw * 0.85, ch * 0.85),
               position=((( cc + 0.5) / cols - 0.5) * W,
                         (0.5 - (cr + 0.5) / rows) * H, 0.005))

        def dot(color, size, z, model="circle"):
            return Entity(parent=root, model=model, color=color,
                          scale=size, z=z)

        for st in sim.storms:                    # translucent storm discs
            e = dot(rgb(235, 200, 120, 110),
                    (2 * st.radius / cols * W, 2 * st.radius / rows * H), 0)
            e.x, e.y = self.mini_xy(st.x, st.y)
        self.mini_samples = []
        for sm in sim.samples:
            e = dot(rgb(255, 150, 40), 0.014, -0.01)
            e.x, e.y = self.mini_xy(sm.x, sm.y)
            self.mini_samples.append((sm, e))
        self.mini_ghost = dot(rgb(235, 245, 255), 0.018, -0.02)
        self.mini_ghost.enabled = False
        self.mini_rover = dot(rgb(60, 150, 255), 0.022, -0.03)
        self.mini_head = dot(rgb(255, 255, 255), 0.010, -0.04)
        self.apply_map_state()

    def apply_map_state(self):
        W, H, MH = self.mini_w, self.mini_h, 0.032
        self.mini.enabled = self.map_open
        self.mini.position = (self.map_x, self.map_y + H / 2)
        hy = (self.map_y + H + 0.008 + MH / 2) if self.map_open \
            else (self.map_y + MH / 2)
        self.map_head.position = (self.map_x, hy)
        self.map_title.position = (self.map_x - W / 2 - 0.002, hy)
        self.map_icon.position = (self.map_x + W / 2 + 0.002, hy)
        self.map_icon.text = "[-]" if self.map_open else "[+]"

    def toggle_map(self):
        self.map_open = not self.map_open
        self.apply_map_state()

    def update_minimap(self, ghost_xy):
        r = self.sim.rover
        ux, uy = self.mini_xy(r.x, r.y)
        self.mini_rover.x, self.mini_rover.y = ux, uy
        self.mini_head.x = ux + math.cos(r.theta) * 0.016
        self.mini_head.y = uy - math.sin(r.theta) * 0.016
        for sm, e in self.mini_samples:
            e.enabled = not sm.collected
        if ghost_xy is None:
            self.mini_ghost.enabled = False
        else:
            self.mini_ghost.enabled = True
            self.mini_ghost.x, self.mini_ghost.y = self.mini_xy(*ghost_xy)

    # --------------------------------------- "where is the robot in this?"
    def track_plan(self, fdt):
        """Notice when the planner hands the rover a new route (first chip lights ~1.2 s)."""
        key = tuple(self.sim.rover.path)
        if key != self.plan_key:
            if key:
                length = sum(math.hypot(q[0] - p[0], q[1] - p[1])
                             for p, q in zip(key, key[1:]))
                self.plan_info = ("re-planned" if self.plan_key else "planned",
                                  len(key), length)
                self.plan_flash = 1.2
            self.plan_key = key
        self.plan_flash = max(0.0, self.plan_flash - fdt)

    def heading_error_deg(self):
        """Heading error the PID is working on (towards the next waypoint)."""
        r = self.sim.rover
        if not r.path:
            return 0.0
        row, col = r.path[min(r.path_index, len(r.path) - 1)]
        want = math.atan2(row + 0.5 - r.y, col + 0.5 - r.x)
        return math.degrees(r.normalize_angle(want - r.theta))

    def stage(self):
        """Which part of the pipeline is in control right now?

        Returns (chip index or None, one plain-language sentence):
        0 = the planner (A* / Theta* / RL), 1 = PID (steers), 2 = RAYS (sensor safety layer).
        """
        sim = self.sim
        r = sim.rover
        if not sim.battery.available():
            return None, "Battery empty - the rover has stopped."
        if r.collision_warning:
            return 2, "Safety stop: an obstacle is too close, so the sensors hold the wheels."
        if getattr(sim, "_recovering", False):
            return 2, f"Stuck. Backing to the cell centre, then {sim.planner_name} plans again."
        if r.avoidance_active:
            return 2, f"The sensors saw something the map missed - dodging it, then {sim.planner_name} re-plans."
        if r.replan_requested:
            return 0, f"{sim.planner_name} is re-planning a route around the obstacle."
        if self.plan_flash > 0.0:
            kind, n, length = self.plan_info
            note = f" ({sim.plan_note})" if sim.plan_note else ""
            return 0, (f"{sim.plan_used} {kind} a route: {n} waypoints, "
                       f"{length:.1f} m{note}.")
        if r.path:
            n = len(r.path)
            k = min(r.path_index, n - 1) + 1
            slow = " (dust storm: slowed)" if sim.in_storm else ""
            return 1, (f"PID steers to waypoint {k} of {n}: heading off by "
                       f"{self.heading_error_deg():+.0f} deg.{slow}")
        if sim.charging:
            return None, "Charging at base."
        if sim.message == "NO VALID PATH":
            return None, f"{sim.planner_name} found no safe route there. Try another spot."
        if sim.message.startswith("PLANNER:"):
            return None, (f"Planner: {sim.planner_name}. "
                          f"{PLANNER_EXPLAIN[sim.planner_name]}.")
        if sim.message:
            return None, sim.message.capitalize() + "."
        if sim.auto:
            if sim.samples_collected() == len(sim.samples):
                return None, "All samples collected - mission complete."
            return 0, f"{sim.planner_name} is choosing the next sample to reach."
        return None, f"Idle. Click the terrain: {sim.planner_name} plans a route, PID drives it."

    def apply_planner_labels(self):
        """Dashboard row 0 always names the planner that is selected (key P)."""
        name = self.sim.planner_name
        self.chip_txt[0].text = f"{name.upper()}  PLAN"
        self.exp_lbl[0].text = name
        self.exp_desc[0].text = PLANNER_EXPLAIN[name]

    def update_hud(self):
        sim = self.sim
        r, b, m = sim.rover, sim.battery, sim.mission
        pct = b.percentage()
        speed = "PAUSED" if sim.paused else f"{SPEEDS[self.speed_i]}x"

        if self._hud_cache.get("planner") != sim.planner_name:
            self._hud_cache["planner"] = sim.planner_name
            self.apply_planner_labels()

        if self.hud_open:
            self.set_text(self.hud_right, "right",
                          f"seed {sim.seed}   {speed}"
                          + ("   ghost" if self.ghost.active else ""))
            frac = max(0.0, min(1.0, pct / 100.0))
            self.hud_bar.scale_x = max(1e-4, self.bar_w * frac)
            if pct < 15:
                self.hud_bar.color = rgb(235, 70, 60)
            elif pct < LOW_BATTERY:
                self.hud_bar.color = rgb(240, 170, 40)
            else:
                self.hud_bar.color = rgb(90, 220, 110)
            self.set_text(self.hud_batt, "batt",
                          f"BATTERY {pct:5.1f}%"
                          + ("    CHARGING" if sim.charging else ""))

            # pipeline strip: light the stage that is in control
            idx, sentence = self.stage()
            if self._hud_cache.get("stage") != idx:
                self._hud_cache["stage"] = idx
                for i in range(3):
                    on = (i == idx)
                    self.chip_bg[i].color = COL_ACCENT if on else COL_BAR_BG
                    self.chip_txt[i].color = rgb(20, 24, 32) if on else COL_LABEL
                    self.exp_lbl[i].color = COL_ACCENT if on else COL_LABEL
                self.you_here.enabled = idx is not None
                if idx is not None:
                    self.you_here.x = self.chip_x[idx]
            self.set_text(self.now_txt, "now", wrap(sentence, 50, 2))

            row, col = sim.rover_cell()
            n = len(r.path)
            if n:
                route = (f"waypoint {min(r.path_index, n - 1) + 1}/{n}   "
                         f"{r.get_distance_to_target():.1f} m to target")
            else:
                route = "no route"
            self.set_text(self.dat_txt, "dat", "\n".join((
                f"X {r.x:5.2f}  Y {r.y:5.2f} m   cell ({row},{col})   "
                f"{math.degrees(r.theta):4.0f} deg",
                route,
                f"{m.status}   {m.completed_missions} done / "
                f"{m.failed_missions} failed   samples "
                f"{sim.samples_collected()}/{len(sim.samples)}")))

            if self.hud_more:                         # the full telemetry table
                mode = "AUTO-EXPLORE" if sim.auto else "MANUAL"
                sun = sim.sun()
                solar = SOLAR_RATE * max(0.0, sun)
                rows = [
                    f"X {r.x:5.2f}   Y {r.y:5.2f} m",
                    f"{math.degrees(r.theta):6.1f} deg",
                    f"{r.v_left:5.2f} / {r.v_right:5.2f} m/s",
                    f"{r.get_distance_to_target():5.2f} m    waypoints {len(r.path)}",
                    f"{m.status}   {m.completed_missions} done / "
                    f"{m.failed_missions} failed",
                    f"{m.total_distance:6.1f} m    samples "
                    f"{sim.samples_collected()}/{len(sim.samples)}",
                    f"{mode}    camera {CAMERA_NAMES[self.cam_mode]}",
                    f"{sim.clock()} {'DAY' if sun > 0 else 'NIGHT'}    "
                    f"solar +{solar:4.2f}%/s",
                    "YES - slowed" if sim.in_storm else "no",
                    self.ghost.hud(),
                    f"{sim.telemetry.sample_count()} samples",
                ]
                self.set_text(self.hud_vals, "vals",
                              "\n".join(v[:38] for v in rows))
        else:
            self.set_text(self.hud_right, "right", f"battery {pct:4.1f}%")

        text = sim.status()
        self.status.text = text
        if "STOP" in text or "DEPLETED" in text:
            self.status.color = rgb(255, 90, 80)
        elif ("AVOIDANCE" in text or "STUCK" in text or "NO VALID" in text
              or "DUST" in text):
            self.status.color = rgb(255, 220, 70)
        elif "CHARGING" in text or "COLLECTED" in text:
            self.status.color = rgb(110, 255, 140)
        else:
            self.status.color = rgb(255, 255, 255)

    def update_hover(self):
        """Highlight the dashboard / minimap header under the mouse."""
        hov = mouse.hovered_entity
        self.hud_head.color = COL_HEAD_HOT if hov is self.hud_head else COL_HEAD
        self.hud_more_btn.color = (COL_HEAD_HOT if hov is self.hud_more_btn
                                   else COL_HEAD)
        self.map_head.color = COL_HEAD_HOT if hov is self.map_head else COL_HEAD

    # -------------------------------------------------------------- camera
    def set_camera_mode(self, mode):
        self.cam_mode = mode
        camera.fov = CAMERA_FOV[mode]

    def set_camera_mode(self, mode):
        self.cam_mode = mode
        camera.fov = CAMERA_FOV[mode]
        camera.rotation = Vec3(0, 0, 0)       # clear roll left by the last mode

    @staticmethod
    def aim_camera(pos, target):
        """Point the camera at `target` with ZERO roll (horizon always level)."""
        d = target - pos
        flat = math.hypot(d.x, d.z)
        yaw_deg = math.degrees(math.atan2(d.x, d.z))
        pitch_deg = math.degrees(math.atan2(-d.y, flat))   # +ve = look down
        camera.position = pos
        camera.rotation = Vec3(pitch_deg, yaw_deg, 0)

    def update_camera(self, fdt):
        sim = self.sim
        r = sim.rover
        cols, rows = sim.terrain.cols, sim.terrain.rows
        cx, cz = cols / 2, -rows / 2
        rx, rz = r.x, -r.y
        f = Vec3(math.cos(r.theta), 0, -math.sin(r.theta))   # heading, world

        if self.cam_mode == 0:                                # TOP
            camera.position = Vec3(cx, 36 * self.zoom, cz)
            camera.rotation = Vec3(90, 0, 0)
        elif self.cam_mode == 1:                              # ORBIT
            dist = 30 * self.zoom
            yr, pr = math.radians(self.orbit_yaw), math.radians(self.orbit_pitch)
            pos = Vec3(cx + dist * math.cos(pr) * math.sin(yr),
                       dist * math.sin(pr),
                       cz - dist * math.cos(pr) * math.cos(yr))
            self.aim_camera(pos, Vec3(cx, 0, cz))
        elif self.cam_mode == 2:                              # CHASE
            want = Vec3(rx, 0, rz) - f * (3.4 * self.zoom) + Vec3(0, 2.3 * self.zoom, 0)
            k = min(1.0, 5.0 * fdt)
            pos = camera.position + (want - camera.position) * k
            self.aim_camera(pos, Vec3(rx, 0.3, rz) + f * 1.2)
        else:                                                 # COCKPIT
            pos = Vec3(rx, 0.62, rz) + f * 0.30
            self.aim_camera(pos, Vec3(rx, 0.35, rz) + f * 5)

    # ---------------------------------------------------------- per frame
    def sync_visuals(self, fdt, sim_dt):
        sim = self.sim
        r = sim.rover
        self.t += fdt

        self.rv.position = Vec3(r.x, 0, -r.y)
        self.rv.rotation_y = yaw(r.theta)
        for sx, wheel in self.wheels:
            # sim "right" wheel appears on the left of the model because the
            # 2D sim frame has y pointing down, so swap them here.
            v = r.v_right if sx < 0 else r.v_left
            wheel.rotation_x += math.degrees(v / WHEEL_RADIUS) * sim_dt

        # status LED
        if not sim.battery.available() or r.collision_warning:
            state = "red"
        elif sim.charging:
            state = "blue"
        elif r.avoidance_active:
            state = "yellow"
        else:
            state = "green"
        if state != self.led_state:
            self.led_state = state
            self.led.color = {"red": rgb(255, 60, 60), "blue": rgb(80, 160, 255),
                              "yellow": rgb(255, 220, 50),
                              "green": rgb(60, 255, 90)}[state]
        self.charger_light.color = (rgb(90, 255, 130) if sim.charging
                                    else rgb(90, 90, 90))

        # sensor beams
        for i, rel in enumerate(r.sensor_angles):
            beam = self.beams[i]
            beam.enabled = self.show_beams
            phi = r.theta + rel
            length = r.sensor_hit_distances[i]
            if r.sensor_obstacle_hits[i]:
                length = max(0.0, length - r.sensor_visual_padding)
            length = max(length, 0.01)
            beam.position = Vec3(r.x + math.cos(phi) * length / 2, 0.34,
                                 -(r.y + math.sin(phi) * length / 2))
            beam.rotation_y = yaw(phi)
            beam.scale_z = length
            clear = r.sensor_distances[i]
            if clear <= r.sensor_emergency_distance:
                beam.color = COL_BEAM_STOP
            elif clear <= r.sensor_warning_distance:
                beam.color = COL_BEAM_WARN
            elif clear <= SAFE_DISTANCE:
                beam.color = COL_BEAM_NEAR
            else:
                beam.color = COL_BEAM_OK

        # path, target beacon, tracks
        self.path_layer.sync(r.path, r.path_index)
        if r.path:
            grow, gcol = r.path[-1]
            self.beacon.enabled = True
            self.beacon.position = Vec3(gcol + 0.5, 0, -(grow + 0.5))
            self.beacon_top.y = 1.25 + 0.08 * math.sin(self.t * 4)
        else:
            self.beacon.enabled = False
        self.trail.update(r.x, r.y, r.theta, abs(r.linear_velocity) > 0.05)

        # dust puffs, storm swirls, ghost, minimap
        self.dust.update(sim_dt, r.x, r.y, r.theta, r.linear_velocity,
                         sim.in_storm)
        for swirl in self.storm_vis:
            swirl.rotation_y += getattr(swirl, "spin", 45) * sim_dt
        self.update_minimap(self.ghost.update(sim_dt))

        # samples: flag sways, light blinks, detection ring pulses
        for i, (s, root, ring, flag, led, mark) in enumerate(self.sample_vis):
            if s.collected:
                root.enabled = False
                ring.enabled = False
                mark.enabled = True
            else:
                flag.rotation_y = 14 * math.sin(self.t * 2.6 + i)
                on = math.sin(self.t * 6.0 + i * 1.7) > 0
                led.color = rgb(255, 190, 70) if on else rgb(110, 70, 25)
                f = (self.t * 0.45 + i * 0.37) % 1.0
                ring.scale = 0.35 + 1.25 * f
                ring.color = rgb(255, 150, 40, int(150 * (1.0 - f)))

    def update_sound(self):
        sim = self.sim
        n = sim.samples_collected()
        if n > self.last_collected:
            self.sfx.play("pickup")
        self.last_collected = n
        pct = sim.battery.percentage()
        if pct < LOW_BATTERY and not self.low_flag:
            self.low_flag = True
            self.sfx.play("low")
        elif pct > LOW_BATTERY + 10:
            self.low_flag = False
        if (pct < 10 and not sim.paused and not sim.charging
                and self.t - self.crit_clock > 5.0):
            self.crit_clock = self.t
            self.sfx.play("critical")

    def update(self):
        now = pytime.perf_counter()
        fdt = min(now - self.clock_last, 0.1)
        self.clock_last = now
        sim = self.sim

        sim_dt = 0.0
        if not sim.paused:
            sim_dt = fdt * SPEEDS[self.speed_i]
            self.acc += sim_dt
            n = 0
            while self.acc >= STEP and n < 40:
                sim.step(STEP)
                self.acc -= STEP
                n += 1
            if n == 40:
                self.acc = 0.0

        # orbit-camera keys
        if held_keys["left arrow"]:
            self.orbit_yaw -= 60 * fdt
        if held_keys["right arrow"]:
            self.orbit_yaw += 60 * fdt
        if held_keys["up arrow"]:
            self.orbit_pitch = min(85.0, self.orbit_pitch + 40 * fdt)
        if held_keys["down arrow"]:
            self.orbit_pitch = max(10.0, self.orbit_pitch - 40 * fdt)

        self.sync_visuals(fdt, sim_dt)
        self.update_camera(fdt)
        self.night.color = Color(0.02, 0.04, 0.16,
                                 (1.0 - sim.daylight()) * 0.55)
        self.update_sound()
        self.update_hover()
        self.track_plan(fdt)

        self.hud_timer += fdt
        if self.hud_timer >= 0.1:
            self.hud_timer = 0.0
            self.update_hud()

    # --------------------------------------------------------------- input
    def input(self, key):
        sim = self.sim
        if key == "left mouse down":
            hov = mouse.hovered_entity
            if hov is self.hud_head:               # dashboard header icon
                self.toggle_hud()
                return
            if hov is self.hud_more_btn:           # "full telemetry" button
                self.toggle_more()
                return
            if hov is self.map_head:               # minimap header icon
                self.toggle_map()
                return
            p = mouse.world_point
            if mouse.hovered_entity is self.ground and p is not None:
                col, row = int(p.x), int(-p.z)
                if 0 <= row < sim.terrain.rows and 0 <= col < sim.terrain.cols:
                    sim.auto = False
                    sim.go_to((row, col), "TARGET")
        elif key in ("1", "2", "3", "4"):
            self.set_camera_mode(int(key) - 1)
        elif key == "p":
            sim.cycle_planner()
        elif key == "e":
            sim.auto = not sim.auto
        elif key == "c":
            sim.auto = False
            sim.go_to_charger()
        elif key == "space":
            sim.paused = not sim.paused
        elif key == "z":
            self.speed_i = max(0, self.speed_i - 1)
        elif key == "x":
            self.speed_i = min(len(SPEEDS) - 1, self.speed_i + 1)
        elif key == "b":
            self.show_beams = not self.show_beams
        elif key == "g":
            self.ghost.toggle(sim.seed)
        elif key == "m":
            self.sfx.muted = not self.sfx.muted
        elif key == "h":
            self.toggle_hud()
        elif key == "t":
            self.toggle_more()
        elif key == "n":
            self.toggle_map()
        elif key == "scroll up":
            self.zoom = max(0.4, self.zoom * 0.92)
        elif key == "scroll down":
            self.zoom = min(2.5, self.zoom * 1.08)
        elif key == "r":
            sim.reset()
            self.path_layer.clear()
            self.trail.clear()
            self.dust.clear()
            self.ghost.stop()
            self.plan_key, self.plan_flash = None, 0.0
            self.build_scene()
            self.build_minimap()
            self.last_collected = 0
            self.low_flag = False
            self.acc = 0.0
        elif key == "escape":
            application.quit()


if __name__ == "__main__":
    seed_arg = int(sys.argv[1]) if len(sys.argv) > 1 else None
    app = Ursina(title="ARES 3D - Autonomous Robotic Exploration System",
                 borderless=False)
    window.color = rgb(214, 160, 122)
    try:
        window.exit_button.visible = False
    except Exception:
        pass
    camera.clip_plane_far = 300
    for _name in ("entity_counter", "collider_counter"):   # clipped dev text
        try:
            getattr(window, _name).enabled = False
        except Exception:
            pass
    game = Game(seed_arg)
    app.run()