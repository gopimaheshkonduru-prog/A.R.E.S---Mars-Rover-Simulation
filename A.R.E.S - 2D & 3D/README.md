# A.R.E.S - Autonomous Robotic Exploration System

A rover simulation: route planning (A*, Theta*, Q-learning RL), PID heading control, ray-sensor obstacle
avoidance, battery model, missions and telemetry reports.

## Run

    pip install -r requirements.txt     # Python 3.9 - 3.12 recommended

    python main.py            # original 2D view (pygame)
    python main_3d.py         # 3D view (Ursina)  - random Mars map
    python main_3d.py 42      # 3D view with a reproducible map (seed 42)
    python reports/analyze.py # plots from reports/telemetry.csv

Both views drive the same simulation code (rover/, navigation/, controller/,
battery/, mission/, telemetry/). `main_3d.py` only adds a 3D renderer.

## 3D controls

| Key / mouse   | Action                                                |
|---------------|-------------------------------------------------------|
| Left click    | Send the rover to that spot                           |
| P             | Switch route planner: A* -> Theta* -> RL (Q-learning) |
| E             | Auto-explore: collect samples, recharge when battery < 30% |
| C             | Drive to the charging station                         |
| 1 2 3 4       | Camera: Top / Orbit / Chase / Cockpit                 |
| Arrows, wheel | Orbit / tilt, zoom                                    |
| Space         | Pause                                                 |
| Z / X         | Slower / faster simulation (0.5x - 8x)                |
| B             | Show / hide sensor beams                              |
| G             | Ghost replay of `reports/telemetry.csv` (toggle)      |
| M             | Mute / unmute sound effects                           |
| H             | Collapse / expand the dashboard (or click its header) |
| T             | Dashboard: show / hide the full telemetry table       |
| N             | Collapse / expand the minimap                         |
| R             | New random map                                        |
| Esc           | Quit (telemetry saved, `reports/analyze.py` runs)     |

## What is new

- `simulation/world.py` - headless simulation wrapper (no graphics):
  science samples, charging station, auto-explore mode, stuck-recovery
  manoeuvre, reproducible seeded maps.
- `main_3d.py` - 3D renderer: rover model with spinning wheels and status LED,
  boulders, craters, canyon walls, A* path + target beacon, wheel tracks,
  coloured sensor beams, four cameras, HUD.
- `DRAIN_MULTIPLIER` in `simulation/world.py` speeds up battery drain so the
  "return to charger" behaviour shows up in a short demo (set it to 1.0 for the
  original energy model).

## Add-ons (latest)

- `dashboard/dashboard.py` - on quit, saves telemetry and runs `reports/analyze.py`
  (plots in `reports/*.png`). Run it by hand with `python dashboard/dashboard.py`.
- Dust storms (`Storm` in `simulation/world.py`): wheel speed x0.55, battery load x2
  via `Battery.consume(..., load)`. Soft sand / slope tiles add a smaller load.
- `Terrain.cost` / `Terrain.surface`: soft sand (cost 4) and slope (cost 2) tiles;
  `AStar` multiplies the step cost by the tile cost so the planner detours.
- `AStar.smooth_path`: string-pulling with a clearance check (0.20 m extra margin),
  used by `World.go_to` and the replan step.
- Minimap (bottom-right), dust puffs, day/night (`DAY_LENGTH`, `SOLAR_RATE`),
  sensor noise (`Rover.sensor_noise`, 0 = off), ghost replay, sound effects
  (`simulation/sfx.py` writes the .wav files into `assets/sfx/` on first run).
- Ghost replay: start the app with the SAME seed that produced the telemetry
  (`python main_3d.py 42`), then press G.

## Route planners (A* / Theta* / RL)

Press **P** (3D and 2D window) to switch the planner. The dashboard row, the
status bar and the "now" sentence show which one is in control. Every planner
uses the same safe cells, corner-cutting rule and soft-sand / slope costs, so
the PID, ray sensors, storms, battery and waypoint slow-down are unchanged.

| Planner | File | Idea |
|---------|------|------|
| A*      | `navigation/astar.py` | Cheapest route on the grid, then string-pulling (unchanged). |
| Theta*  | `navigation/theta_star.py` | Any-angle A*: links a cell straight to its grandparent when the line keeps the rover's clearance. Straighter routes, fewer waypoints. |
| RL      | `navigation/rl_planner.py` | Tabular Q-learning. State = cell, actions = the safe 8-way moves, reward = -(distance x tile cost). The greedy policy is the route. One Q-table per goal is kept per map, so re-plans and repeat trips (charger) keep learning instead of starting over. |

Safety nets: if RL or Theta* ever return no route, `World.plan()` falls back to
A* and the status line says so. `SMOOTH_ROUTE` in `simulation/world.py`
controls whether A*'s string-pulling is applied to each planner's route (RL is
shown raw by default so you can see what the agent learned; set it to `True`
for a smoother drive).

In the 2D window the same **P** key cycles the planner; the telemetry panel
shows `Planner: <name> (<stats>)` and the status bar names the planner in
control. (The 2D view drives each planner's route as-is, exactly as it always
drove A*'s.)

Extra RL details:

* The map is split into connected regions once. A goal the rover cannot reach
  is refused immediately (`last_stats["unreachable"]`) instead of spending the
  whole 0.35 s training budget, and training only starts from cells that can
  actually reach the goal.
* With no training at all the greedy policy reaches the goal on only about
  half of the routes (and is optimal on about a quarter); after training it
  reaches it - optimally - on every route tested. That gap is the learning.
* If RL (or Theta*) ever falls back to A*, that route is string-pulled like a
  normal A* route, whatever `SMOOTH_ROUTE` says.

Compare them without graphics:

    python navigation/compare_planners.py          # 20 maps x 6 routes
    python navigation/compare_planners.py 50 10
    python navigation/compare_planners.py --curve  # + RL learning curve

`--curve` trains a cold RL agent for every route and saves
`reports/rl_learning_curve.png`: the share of routes the greedy policy already
solves, and its extra cost over the A* optimum, after N training episodes.

## Rock types (3 designs)

Every rock cell is drawn as one of three designs. Which one sits where depends
only on the map seed and the cell, so `python main_3d.py 42` always shows the
same rocks. Only the looks differ - collisions, sensors and A* are unchanged.

| Type    | Look                                            |
|---------|-------------------------------------------------|
| Boulder | round, weathered grey-brown stone (original)    |
| Crag    | tall, sharp, dark blue-grey shards, pale tips   |
| Slab    | low, wide, layered rusty sandstone + leaning plate |

Code: `rock_kind`, `rock_boulder`, `rock_crag`, `rock_slab` in `main_3d.py`.

## Dashboard (top-left)

Minimal by default: battery, then the robot's "brain" as a pipeline
**A\* -> PID -> RAYS** with the stage that is in control right now lit up
("you are here"), one plain sentence about what it is doing, and where the
robot is (position, cell, heading, waypoint k/N, mission, samples).

| Part  | What it does                                                        |
|-------|---------------------------------------------------------------------|
| A*    | Plans the cheapest safe route on the grid: around rocks and craters, avoiding soft sand / slopes. Runs when a target is set and again when the rover has to re-plan. |
| PID   | Follows that route: turns the heading error to the next waypoint into left/right wheel speeds. |
| Rays  | Sensor safety layer: brakes or dodges anything the map did not show; if it takes over, A* re-plans afterwards. |

Press **T** (or click the bar at the bottom of the panel) for the full
telemetry table: position, heading, wheel speeds, target error, mission,
distance, mode, clock/solar, storm, ghost replay, telemetry samples.
**H** collapses the whole dashboard.

## Run in Cursor

1. Install Python 3.10 - 3.12 (python.org). Unzip the project.
2. Cursor: **File > Open Folder...** and pick the folder that contains
   `main_3d.py` (the `A.R.E.S` folder). Accept the prompt to install the
   Python extension.
3. Open a terminal: **Ctrl+`** (Mac: **Cmd+`**) and create an environment:

       python -m venv .venv                     # Windows: py -3.11 -m venv .venv
       # Windows (PowerShell):  .venv\Scripts\Activate.ps1
       # Windows (cmd):         .venv\Scripts\activate.bat
       # Mac / Linux:           source .venv/bin/activate
       pip install -r requirements.txt

   If PowerShell blocks the activation script, skip activating and call the
   environment directly: `.venv\Scripts\python -m pip install -r requirements.txt`
   and `.venv\Scripts\python main_3d.py`.
4. Tell Cursor which Python to use: **Ctrl+Shift+P > Python: Select
   Interpreter > .venv**.
5. Run it: type `python main_3d.py` in the terminal (or `python main_3d.py 42`),
   or press **F5** and choose a launch entry from `.vscode/launch.json`
   (3D random map, 3D seed 42, 2D pygame, reports only).

Run it on your own computer (Windows / macOS / Linux desktop) - the 3D window
needs a display, so it will not open inside WSL, SSH or a web IDE.
