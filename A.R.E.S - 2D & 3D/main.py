import pygame

import sys

import math

import time

from environment.terrain import Terrain

from rover.rover import Rover

from navigation.astar import AStar
from navigation.theta_star import ThetaStar
from navigation.rl_planner import RLPlanner

from battery.battery import Battery

from mission.mission import Mission

from telemetry.logger import TelemetryLogger



# =====================================================

# INITIALIZATION

# =====================================================

pygame.init()

WIDTH = 1000

HEIGHT = 700

screen = pygame.display.set_mode(

    (

        WIDTH,

        HEIGHT

    )

)

pygame.display.set_caption(

    "ARES - Autonomous Robotic Exploration System"

)

clock = pygame.time.Clock()



# =====================================================

# FONTS

# =====================================================

font = pygame.font.Font(

    None,

    24

)

small_font = pygame.font.Font(

    None,

    20

)



# =====================================================

# SYSTEM OBJECTS

# =====================================================

terrain = Terrain()

rover = Rover()

astar = AStar(

    terrain

)


# Route planners: press P in the window to switch A* -> Theta* -> RL.
planners = {
    "A*": astar,
    "Theta*": ThetaStar(terrain),
    "RL": RLPlanner(terrain),
}
planner_names = list(planners)
planner_name = "A*"
plan_note = ""          # one-line stats of the last plan (shown in the HUD)


def plan_route(start, goal):
    """Plan with the selected planner and remember what it did.

    RLPlanner already falls back to A* by itself; Theta* returns [] only when
    the goal cannot be reached, so no extra fallback is needed here.
    """
    global plan_note

    planner = planners[planner_name]
    t0 = time.perf_counter()
    path = planner.find_path(start, goal)
    ms = (time.perf_counter() - t0) * 1000.0
    stats = getattr(planner, "last_stats", None) or {}

    if planner_name == "RL" and stats.get("fallback"):
        plan_note = f"RL failed -> A* fallback, {ms:.0f} ms"
    elif planner_name == "RL":
        plan_note = f"{stats.get('episodes', 0)} episodes, {ms:.0f} ms"
    elif planner_name == "Theta*":
        plan_note = f"{stats.get('expanded', 0)} nodes, {ms:.0f} ms"
    else:
        plan_note = f"{ms:.0f} ms"

    return path


# Place the initial rover pose at the centre of a clearance-safe

# cell. The default coordinates lie on a grid intersection, where an

# adjacent obstacle can overlap the rover's physical collision circle.

initial_cell = astar.find_nearest_safe_cell(

    (

        int(rover.y),

        int(rover.x)

    )

)

if initial_cell is not None:

    initial_row, initial_col = initial_cell

    rover.x = initial_col + 0.5

    rover.y = initial_row + 0.5

battery = Battery(

    capacity=100.0

)

mission = Mission()

telemetry = TelemetryLogger()



# =====================================================

# CLICKED TARGET

# =====================================================

clicked_target = None



# =====================================================

# HELPER

# =====================================================

def rover_grid_position():

    col = int(

        rover.x

    )

    row = int(

        rover.y

    )

    return (

        row,

        col

    )



# =====================================================

# MAIN LOOP

# =====================================================

running = True

while running:

    # =================================================

    # EVENTS

    # =================================================

    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            running = False

        # ---------------------------------------------
        # P = SWITCH ROUTE PLANNER (A* / Theta* / RL)
        # ---------------------------------------------

        if event.type == pygame.KEYDOWN and event.key == pygame.K_p:

            planner_name = planner_names[
                (planner_names.index(planner_name) + 1) % len(planner_names)
            ]

            pygame.display.set_caption(
                "ARES - Autonomous Robotic Exploration System"
                f"  [planner: {planner_name}]"
            )

            plan_note = ""

            print("Planner:", planner_name)

        # ---------------------------------------------

        # LEFT CLICK = NEW MISSION

        # ---------------------------------------------

        if event.type == pygame.MOUSEBUTTONDOWN:

            if event.button == 1:

                mouse_x, mouse_y = event.pos

                # -----------------------------------------

                # SCREEN -> GRID

                # -----------------------------------------

                target_col = mouse_x // 40

                target_row = mouse_y // 40

                # -----------------------------------------

                # VALID TARGET

                # -----------------------------------------

                if (

                    0 <= target_row < terrain.rows

                    and

                    0 <= target_col < terrain.cols

                ):

                    clicked_target = (

                        target_row,

                        target_col

                    )

                    # -------------------------------------

                    # CURRENT ROVER CELL

                    # -------------------------------------

                    start = (

                        rover_grid_position()

                    )

                    # -------------------------------------

                    # REQUESTED GOAL

                    # -------------------------------------

                    goal = (

                        target_row,

                        target_col

                    )

                    # -------------------------------------

                    # PATH PLANNING (planner selected with P)

                    # -------------------------------------

                    path = plan_route(

                        start,

                        goal

                    )

                    if path:

                        rover.set_path(

                            path

                        )

                        mission.start(

                            goal

                        )

                        print(

                            "Path found:",

                            len(path),

                            "waypoints"

                        )

                    else:

                        print(

                            "No valid path found!"

                        )

    # =================================================

    # TIME STEP

    # =================================================

    dt = (

        clock.tick(60)

        /

        1000.0

    )

    # Prevent unusually large timestep

    dt = min(

        dt,

        0.05

    )

    # =================================================

    # AUTONOMOUS CONTROL

    # =================================================

    if battery.available():

        rover.autonomous_control(

            dt,

            terrain

        )

        # =================================================

        # REPLAN AFTER OBSTACLE AVOIDANCE

        # =================================================

        if rover.replan_requested:

            rover.replan_requested = False

            if rover.goal is not None:

                # -----------------------------------------

                # CURRENT ROVER POSITION

                # -----------------------------------------

                start = (

                    rover_grid_position()

                )

                # -----------------------------------------

                # NEW PATH (same planner)

                # -----------------------------------------

                new_path = plan_route(

                    start,

                    rover.goal

                )

                if new_path:

                    rover.set_path(

                        new_path

                    )

                    print(

                        "Replanned:",

                        len(new_path),

                        "waypoints"

                    )

                else:

                    print(

                        "Replanning failed!"

                    )

    else:

        rover.v_left = 0.0

        rover.v_right = 0.0

    # =================================================

    # ROVER PHYSICS

    # =================================================

    rover.update(

        dt

    )

    # =================================================

    # BATTERY

    # =================================================

    battery.consume(

        rover.v_left,

        rover.v_right,

        dt

    )

    # =================================================

    # MISSION DISTANCE

    # =================================================

    mission.update_distance(

        rover.x,

        rover.y

    )

    # =================================================

    # TELEMETRY

    # =================================================

    telemetry.record(

        rover,

        battery,

        mission

    )

    # =================================================

    # MISSION COMPLETE

    # =================================================

    if rover.target_reached:

        if rover.path:

            mission.complete()

            rover.path = []

    # =================================================

    # DRAW TERRAIN

    # =================================================

    terrain.draw(

        screen

    )

    # =================================================

    # DRAW A* PATH

    # =================================================

    if rover.path:

        for row, col in rover.path:

            center_x = (

                col * 40 +

                20

            )

            center_y = (

                row * 40 +

                20

            )

            pygame.draw.circle(

                screen,

                (

                    255,

                    220,

                    0

                ),

                (

                    center_x,

                    center_y

                ),

                4

            )

        # ---------------------------------------------

        # PATH LINES

        # ---------------------------------------------

        for i in range(

            len(rover.path) - 1

        ):

            row1, col1 = (

                rover.path[i]

            )

            row2, col2 = (

                rover.path[i + 1]

            )

            pygame.draw.line(

                screen,

                (

                    255,

                    220,

                    0

                ),

                (

                    col1 * 40 + 20,

                    row1 * 40 + 20

                ),

                (

                    col2 * 40 + 20,

                    row2 * 40 + 20

                ),

                3

            )

    # =================================================

    # DRAW EXACT CLICKED TARGET

    # =================================================

    if clicked_target is not None:

        target_row, target_col = (

            clicked_target

        )

        target_x = (

            target_col * 40 +

            20

        )

        target_y = (

            target_row * 40 +

            20

        )

        pygame.draw.circle(

            screen,

            (

                255,

                255,

                0

            ),

            (

                target_x,

                target_y

            ),

            11,

            3

        )

        pygame.draw.circle(

            screen,

            (

                255,

                255,

                0

            ),

            (

                target_x,

                target_y

            ),

            3

        )

    # =================================================

    # DRAW ROVER

    # =================================================

    rover.draw(

        screen

    )

    # =================================================

    # TELEMETRY VALUES

    # =================================================

    distance = (

        rover.get_distance_to_target()

    )

    telemetry_text = [

        f"X: {rover.x:.2f} m",

        f"Y: {rover.y:.2f} m",

        f"Theta: "

        f"{math.degrees(rover.theta):.1f} deg",

        f"Left Wheel: "

        f"{rover.v_left:.2f} m/s",

        f"Right Wheel: "

        f"{rover.v_right:.2f} m/s",

        f"Target Error: "

        f"{distance:.2f} m",

        f"{planner_name} Waypoints: "

        f"{len(rover.path)}",

        f"Planner: {planner_name}"
        + (f" ({plan_note})" if plan_note else ""),

        f"Samples: "

        f"{telemetry.sample_count()}"

    ]

    # =================================================

    # DRAW TELEMETRY

    # =================================================

    y = 10

    for text in telemetry_text:

        surface = font.render(

            text,

            True,

            (

                255,

                255,

                255

            )

        )

        screen.blit(

            surface,

            (

                10,

                y

            )

        )

        y += 25

    # =================================================

    # SYSTEM PANEL

    # =================================================

    panel_x = 760

    panel_y = 10

    pygame.draw.rect(

        screen,

        (

            20,

            20,

            20

        ),

        (

            panel_x - 10,

            panel_y - 5,

            235,

            145

        )

    )

    battery_text = (

        f"Battery: "

        f"{battery.percentage():.1f}%"

    )

    distance_text = (

        f"Mission Distance: "

        f"{mission.total_distance:.2f} m"

    )

    mission_text = (

        f"Mission: "

        f"{mission.status}"

    )

    completed_text = (

        f"Completed: "

        f"{mission.completed_missions}"

    )

    panel_lines = [

        battery_text,

        distance_text,

        mission_text,

        completed_text

    ]

    y = panel_y

    for text in panel_lines:

        surface = small_font.render(

            text,

            True,

            (

                255,

                255,

                255

            )

        )

        screen.blit(

            surface,

            (

                panel_x,

                y

            )

        )

        y += 30

    # =================================================

    # STATUS

    # =================================================

    if rover.collision_warning:

        status = (

            "SAFETY STOP - OBSTACLE"

        )

    elif not battery.available():

        status = (

            "BATTERY DEPLETED"

        )

    elif rover.target_reached:

        status = (

            "MISSION COMPLETE"

        )

    elif rover.avoidance_active:

        status = (

            "OBSTACLE AVOIDANCE"

        )

    elif rover.replan_requested:

        status = (

            f"REPLANNING {planner_name}"

        )

    elif rover.path:

        status = (

            f"{planner_name} + PID AUTONOMOUS NAVIGATION"

        )

    else:

        status = (

            "CLICK TO SET MISSION TARGET"

        )

    status_surface = font.render(

        status,

        True,

        (

            255,

            255,

            255

        )

    )

    screen.blit(

        status_surface,

        (

            10,

            HEIGHT - 35

        )

    )

    # =================================================

    # DISPLAY

    # =================================================

    pygame.display.update()



# =====================================================

# SAVE TELEMETRY

# =====================================================

telemetry.save_csv(

    "reports/telemetry.csv"

)

print(

    "Telemetry saved to "

    "reports/telemetry.csv"

)



# =====================================================

# EXIT

# =====================================================

pygame.quit()

sys.exit()