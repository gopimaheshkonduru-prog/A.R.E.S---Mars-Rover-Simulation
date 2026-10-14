"""
ARES - headless simulation layer (no graphics).

Wraps the original ARES modules (Terrain, Rover, AStar, Battery, Mission,
TelemetryLogger) into one object so that any front-end (the 2D pygame
window, the 3D Ursina window, a unit test) drives the exact same logic.

New, simple, free add-ons that live here:
  * science samples to collect         * charging station
  * auto-explore mode (nearest-sample, return to charge when battery low)
  * stuck watchdog                     * reproducible maps via a seed
"""
import math
import random
import time
from collections import deque

from environment.terrain import Terrain
from rover.rover import Rover
from navigation.astar import AStar
from navigation.rl_planner import RLPlanner
from navigation.theta_star import ThetaStar
from battery.battery import Battery
from mission.mission import Mission
from telemetry.logger import TelemetryLogger

# ---------------------------------------------------------------- tunables
DRAIN_MULTIPLIER = 3.0   # >1 makes the battery drain faster so the demo shows
                         # "return to charger" within ~2 minutes. Set 1.0 for
                         # the original energy model.
CHARGE_RATE = 6.0        # battery % per second while sitting on the pad
CHARGER_RADIUS = 0.8     # metres from pad centre that count as "on the pad"
LOW_BATTERY = 30.0       # auto mode heads to the charger below this %
FULL_BATTERY = 95.0      # auto mode leaves the charger above this %
SAMPLE_COUNT = 6
SAMPLE_PICKUP_RADIUS = 0.6
STUCK_WINDOW = 6.0       # seconds without progress before recovery kicks in
STUCK_DISTANCE = 0.25    # metres that count as "progress" inside that window
RECOVERY_TIMEOUT = 3.0   # max seconds spent driving back to the cell centre

STORM_COUNT = 2          # dust-storm zones per map
STORM_SLOW = 0.55        # wheel-speed multiplier inside a storm
STORM_DRAIN = 2.0        # battery load multiplier inside a storm
SURFACE_DRAIN = {"soft": 1.5, "slope": 1.3}   # extra drain on costly tiles
PLANNERS = ("A*", "Theta*", "RL")   # route planners, cycled with P in the 3D view
# Run A*'s string-pulling over the planner's route? A* needs it (grid
# staircase). Theta* already takes its shortcuts during the search. For RL,
# True = rover drives the straightened version, False = the raw learned
# grid route (you see exactly what the agent learned).
SMOOTH_ROUTE = {"A*": True, "Theta*": False, "RL": False}
DAY_LENGTH = 240.0       # simulated seconds for one full day/night cycle
SOLAR_RATE = 0.30        # battery % per second at noon (daylight only)


class Sample:
    """A science sample (rock core / mineral) the rover can collect."""

    def __init__(self, cell):
        self.cell = cell                 # (row, col)
        self.x = cell[1] + 0.5
        self.y = cell[0] + 0.5
        self.collected = False
        self.fails = 0                   # failed attempts to reach it


class Storm:
    """Circular dust-storm zone (metres): slows and drains the rover."""

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def contains(self, x, y):
        return math.hypot(x - self.x, y - self.y) < self.radius


class World:
    def __init__(self, seed=None):
        self.telemetry = None
        self.planner_name = PLANNERS[0]     # survives reset() (new map)
        self.plan_note = ""                 # one-line stats of the last plan
        self.plan_used = self.planner_name
        self.reset(seed)

    # ------------------------------------------------------------- setup
    @staticmethod
    def _reachable(astar, start):
        """All cells the rover can actually reach from `start` (BFS)."""
        seen = {start}
        queue = deque([start])
        while queue:
            node = queue.popleft()
            for nb in astar.get_neighbors(node):
                if nb not in seen:
                    seen.add(nb)
                    queue.append(nb)
        return seen

    def reset(self, seed=None):
        """Build a brand-new map + rover. Same seed -> same map."""
        if self.telemetry is not None:
            self.save_telemetry()

        if seed is None:
            seed = random.randrange(1, 1_000_000)

        # Some random maps wall the rover in; try following seeds until
        # the rover has a decent amount of free space to explore.
        for attempt in range(200):
            random.seed(seed)
            terrain = Terrain()
            astar = AStar(terrain)
            rover = Rover()
            start = astar.find_nearest_safe_cell((int(rover.y), int(rover.x)))
            if start is not None:
                reachable = self._reachable(astar, start)
                if len(reachable) >= 120:
                    break
            seed += 1

        self.seed = seed
        self.terrain = terrain
        self.astar = astar
        # All planners share the same terrain, safety rules and tile costs;
        # `astar` stays the reference planner (reachability, sample picking).
        self.planners = {
            "A*": astar,
            "Theta*": ThetaStar(terrain),
            "RL": RLPlanner(terrain, seed=seed),
        }
        self.rover = rover
        self.rover.x = start[1] + 0.5      # same safe-cell start as main.py
        self.rover.y = start[0] + 0.5
        self.battery = Battery(capacity=100.0)
        self.mission = Mission()
        self.telemetry = TelemetryLogger()

        # Samples/charger go in "roomy" cells only: the reactive avoidance
        # layer cannot squeeze into one-cell pockets beside rocks or walls.
        roomy = {c for c in reachable if self._roomy(astar, c)} or reachable

        # Charging station = roomy cell closest to the top-left corner.
        self.charger_cell = min(
            roomy, key=lambda c: (c[0] - 1) ** 2 + (c[1] - 1) ** 2
        )
        self.charger_x = self.charger_cell[1] + 0.5
        self.charger_y = self.charger_cell[0] + 0.5

        # Samples: reachable, spread out, not on top of the rover/charger.
        rng = random.Random(seed)
        candidates = sorted(roomy)
        rng.shuffle(candidates)
        self.samples = []
        for cell in candidates:
            if len(self.samples) >= SAMPLE_COUNT:
                break
            if self._cell_dist(cell, start) < 3:
                continue
            if self._cell_dist(cell, self.charger_cell) < 3:
                continue
            if any(self._cell_dist(cell, s.cell) < 3.5 for s in self.samples):
                continue
            self.samples.append(Sample(cell))

        # Dust storms: own RNG so maps / samples stay identical per seed.
        self.rover.noise_rng.seed(seed)
        srng = random.Random(seed + 17)
        spots = sorted(
            c for c in reachable
            if self._cell_dist(c, start) >= 4
            and self._cell_dist(c, self.charger_cell) >= 4
        )
        srng.shuffle(spots)
        self.storms = []
        for cell in spots:
            if len(self.storms) >= STORM_COUNT:
                break
            if any(self._cell_dist(cell, (s.y, s.x)) < 6
                   for s in self.storms):
                continue
            self.storms.append(
                Storm(cell[1] + 0.5, cell[0] + 0.5, srng.uniform(1.8, 2.8)))

        self.paused = False
        self.auto = False
        self.charging = False
        self.in_storm = False
        self.sim_time = 0.0
        self.target_cell = None
        self.target_label = ""
        self.message = ""
        self._auto_cooldown = 0.0
        self._stuck_timer = 0.0
        self._stuck_ref = (self.rover.x, self.rover.y)
        self._recovering = False
        self._recovery_time = 0.0
        self._recovered_once = False

    @staticmethod
    def _cell_dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _roomy(astar, cell):
        """True if all 8 neighbours are free sand (no tight pockets/edges)."""
        row, col = cell
        return all(
            astar.is_walkable(row + dr, col + dc)
            for dr in (-1, 0, 1) for dc in (-1, 0, 1)
        )

    # ----------------------------------------------------------- helpers
    def rover_cell(self):
        return (int(self.rover.y), int(self.rover.x))

    def on_charger(self):
        return math.hypot(
            self.rover.x - self.charger_x, self.rover.y - self.charger_y
        ) < CHARGER_RADIUS

    def samples_collected(self):
        return sum(1 for s in self.samples if s.collected)

    def storm_at(self, x, y):
        for s in self.storms:
            if s.contains(x, y):
                return s
        return None

    def surface_at(self, x, y):
        row, col = int(y), int(x)
        t = self.terrain
        if 0 <= row < t.rows and 0 <= col < t.cols:
            return t.surface[row][col]
        return None

    def sun(self):
        """Sun height -1..1 (>0 = daylight). Starts in the morning."""
        return math.cos(2 * math.pi * self.sim_time / DAY_LENGTH - math.pi / 3)

    def daylight(self):
        """0 (full night) .. 1 (full day), with a short dusk/dawn ramp."""
        return max(0.0, min(1.0, (self.sun() + 0.25) / 0.5))

    def clock(self):
        """Fake time of day, 'HH:MM' (noon = sun at its highest)."""
        hours = (12.0 + 24.0 * (self.sim_time - DAY_LENGTH / 6) / DAY_LENGTH) % 24
        return f"{int(hours):02d}:{int((hours % 1) * 60):02d}"

    def _reset_stuck(self):
        self._stuck_timer = 0.0
        self._stuck_ref = (self.rover.x, self.rover.y)

    # ---------------------------------------------------------- planners
    def set_planner(self, name):
        if name in self.planners:
            self.planner_name = name
            self.message = f"PLANNER: {name}"

    def cycle_planner(self):
        i = PLANNERS.index(self.planner_name)
        self.set_planner(PLANNERS[(i + 1) % len(PLANNERS)])
        return self.planner_name

    def plan(self, start, goal):
        """Plan start -> goal with the selected planner.

        SMOOTH_ROUTE decides, per planner, whether A*'s string-pulling is
        applied afterwards (A*: yes, it needs it; Theta*: no, it already
        shortcuts while searching; RL: no, so the learned route stays
        visible). If the chosen planner returns nothing, plain A* is tried
        so the rover is never left without a route - unless the planner
        already proved the goal unreachable, in which case A* is skipped.
        """
        name = self.planner_name
        planner = self.planners[name]
        t0 = time.perf_counter()
        raw = planner.find_path(start, goal)
        stats = getattr(planner, "last_stats", None) or {}
        used = name
        if name == "RL" and stats.get("fallback"):
            used = "A* (RL fallback)"
        elif (not raw and planner is not self.astar
                and not stats.get("unreachable")):
            raw = self.astar.find_path(start, goal)
            used = f"A* ({name} fallback)"
        ms = (time.perf_counter() - t0) * 1000.0

        if name == "RL" and not stats.get("fallback"):
            self.plan_note = (f"learned in {stats['episodes']} episodes "
                              f"({stats['total_episodes']} total, {ms:.0f} ms)")
        elif name == "Theta*" and used == name:
            self.plan_note = f"any-angle search, {stats.get('expanded', 0)} nodes, {ms:.0f} ms"
        else:
            self.plan_note = f"{ms:.0f} ms"
        self.plan_used = used
        # A fallback route is plain A* (a grid staircase), so it is always
        # string-pulled, whatever SMOOTH_ROUTE says about the chosen planner.
        smooth = SMOOTH_ROUTE.get(name, True) or used.startswith("A*")
        return self.astar.smooth_path(raw) if smooth else raw

    # ----------------------------------------------------------- mission
    def go_to(self, cell, label="TARGET"):
        """Plan a path to `cell` (selected planner) and start a mission."""
        path = self.plan(self.rover_cell(), cell)
        if not path:
            self.message = "NO VALID PATH"
            return False
        self.rover.set_path(path)
        self.mission.start(cell)
        self.target_cell = cell
        self.target_label = label
        self.message = ""
        self._recovered_once = False
        self._reset_stuck()
        return True

    def go_to_charger(self):
        return self.go_to(self.charger_cell, "CHARGER")

    # -------------------------------------------------------- simulation
    def step(self, dt):
        """Advance the simulation by dt seconds (same order as main.py)."""
        if self.paused:
            return
        r, b, m = self.rover, self.battery, self.mission
        self.sim_time += dt

        if self._recovering and b.available():
            self._recover_step(dt)
        elif b.available():
            r.autonomous_control(dt, self.terrain)
            if r.replan_requested:
                r.replan_requested = False
                if r.goal is not None:
                    new_path = self.plan(self.rover_cell(), r.goal)
                    if new_path:
                        r.set_path(new_path)
        else:
            r.v_left = 0.0
            r.v_right = 0.0

        # Ground effects: storms slow the wheels; storms, soft sand and
        # slopes all raise the battery load (one multiplier in consume()).
        self.in_storm = self.storm_at(r.x, r.y) is not None
        load = 1.0
        if self.in_storm:
            r.v_left *= STORM_SLOW
            r.v_right *= STORM_SLOW
            load *= STORM_DRAIN
        load *= SURFACE_DRAIN.get(self.surface_at(r.x, r.y), 1.0)

        r.update(dt)
        b.consume(r.v_left, r.v_right, dt * DRAIN_MULTIPLIER, load)

        # Solar panels: charge only while the sun is up.
        sun = self.sun()
        if sun > 0.0:
            b.level = min(b.capacity,
                          b.level + SOLAR_RATE * sun * dt)

        self.charging = self.on_charger() and b.level < b.capacity
        if self.charging:
            b.level = min(b.capacity, b.level + CHARGE_RATE * dt)

        m.update_distance(r.x, r.y)
        self.telemetry.record(r, b, m)

        if r.target_reached and r.path:
            m.complete()
            r.path = []

        for s in self.samples:
            if (not s.collected
                    and math.hypot(s.x - r.x, s.y - r.y) < SAMPLE_PICKUP_RADIUS):
                s.collected = True

        self._watchdog(dt)
        if self.auto:
            self._auto_explore(dt)

    # ---------------------------------------------------------- watchdog
    def _watchdog(self, dt):
        r = self.rover
        if not r.path or self.on_charger():
            self._reset_stuck()
            return
        self._stuck_timer += dt
        if self._stuck_timer < STUCK_WINDOW:
            return
        moved = math.hypot(
            r.x - self._stuck_ref[0], r.y - self._stuck_ref[1]
        )
        self._reset_stuck()
        if moved < STUCK_DISTANCE and not self._recovered_once:
            self._recovered_once = True
            self._recovering = True
            self._recovery_time = 0.0
            self.message = "STUCK - RECOVERING"
        elif moved < STUCK_DISTANCE:
            self.mission.fail()
            r.path = []
            r.replan_requested = False
            for s in self.samples:
                if s.cell == self.target_cell and not s.collected:
                    s.fails += 1
            self.message = "STUCK - MISSION ABORTED"

    def _recover_step(self, dt):
        """Drive slowly back to the centre of the current cell, then replan.

        The original reactive controller can deadlock when the rover sits
        exactly at its clearance limit next to a rock corner. A cell centre
        is always clear of every neighbour (that is how A* defines a safe
        cell), so heading there frees the rover without any collision risk.
        """
        r = self.rover
        row, col = self.rover_cell()
        dx = col + 0.5 - r.x
        dy = row + 0.5 - r.y
        dist = math.hypot(dx, dy)
        self._recovery_time += dt

        if dist < 0.06 or self._recovery_time > RECOVERY_TIMEOUT:
            self._recovering = False
            r.v_left = r.v_right = 0.0
            r.pid.reset()
            if self.target_cell is not None:
                recovered = self.target_cell
                label = self.target_label
                self.go_to(recovered, label)
                self._recovered_once = True      # only one recovery per mission
            return

        err = r.normalize_angle(math.atan2(dy, dx) - r.theta)
        if abs(err) > 0.35:                       # rotate in place first
            turn = 0.5 if err > 0 else -0.5
            r.v_left, r.v_right = -turn, turn
        else:                                     # then creep to the centre
            speed = min(0.4, 1.0 * dist)
            omega = 2.0 * err
            r.v_left = speed - omega * r.wheel_base / 2
            r.v_right = speed + omega * r.wheel_base / 2

    # ------------------------------------------------------ auto-explore
    def _next_sample(self):
        best, best_len = None, None
        for s in self.samples:
            if s.collected or s.fails >= 2:
                continue
            path = self.astar.find_path(self.rover_cell(), s.cell)
            if not path:
                s.fails = 2
                continue
            if best is None or len(path) < best_len:
                best, best_len = s, len(path)
        return best

    def _auto_explore(self, dt):
        r = self.rover
        if r.path or not self.battery.available():
            return
        self._auto_cooldown = max(0.0, self._auto_cooldown - dt)
        if self._auto_cooldown > 0.0:
            return

        pct = self.battery.percentage()
        on_pad = self.on_charger()

        if on_pad and pct < FULL_BATTERY:
            return                                   # keep charging
        if pct < LOW_BATTERY and not on_pad:
            if self.go_to_charger():
                return
            self._auto_cooldown = 1.0

        target = self._next_sample()
        if target is not None:
            if not self.go_to(target.cell, "SAMPLE"):
                target.fails += 1
                self._auto_cooldown = 1.0
        elif not on_pad:                             # all done: park & charge
            if not self.go_to_charger():
                self._auto_cooldown = 1.0

    # ------------------------------------------------------------ status
    def status(self):
        r = self.rover
        if not self.battery.available():
            return "BATTERY DEPLETED"
        if r.collision_warning:
            return "SAFETY STOP - OBSTACLE"
        if self.charging:
            return "CHARGING AT BASE"
        if self._recovering:
            return "STUCK - RECOVERING"
        if r.avoidance_active:
            return "OBSTACLE AVOIDANCE"
        if self.in_storm:
            return "DUST STORM - SLOWED"
        if r.replan_requested:
            return f"REPLANNING {self.planner_name}"
        if r.path:
            return f"{self.planner_name} + PID NAVIGATION -> {self.target_label}"
        if self.message:
            return self.message
        if self.auto:
            if self.samples_collected() == len(self.samples):
                return "AUTO-EXPLORE: ALL SAMPLES COLLECTED"
            return "AUTO-EXPLORE: PLANNING"
        return "CLICK THE TERRAIN TO SET A TARGET"

    # --------------------------------------------------------- telemetry
    def save_telemetry(self, filename="reports/telemetry.csv"):
        if self.telemetry is not None and self.telemetry.data:
            self.telemetry.save_csv(filename)
