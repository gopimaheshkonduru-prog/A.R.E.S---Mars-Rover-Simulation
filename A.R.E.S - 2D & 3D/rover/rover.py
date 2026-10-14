import pygame
import math
import random

from controller.pid import PIDController


class Rover:

    def __init__(self):

        # =================================================
        # POSITION
        # =================================================

        self.x = 10.0
        self.y = 8.0
        self.theta = 0.0

        # =================================================
        # DIFFERENTIAL DRIVE
        # =================================================

        self.v_left = 0.0
        self.v_right = 0.0

        self.wheel_base = 1.0
        self.max_wheel_speed = 2.0

        # =================================================
        # ROBOT DYNAMICS
        # =================================================

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.dx_dt = 0.0
        self.dy_dt = 0.0
        self.dtheta_dt = 0.0

        # =================================================
        # PID CONTROLLER
        # =================================================

        self.pid = PIDController(
            kp=3.0,
            ki=0.01,
            kd=0.5,
            output_limit=2.0
        )

        # =================================================
        # PATH / MISSION
        # =================================================

        self.path = []
        self.path_index = 0
        self.goal = None
        self.replan_requested = False

        # =================================================
        # STATUS
        # =================================================

        self.target_reached = False
        self.collision_warning = False

        # =================================================
        # ROVER PHYSICAL PARAMETERS
        # =================================================

        self.rover_radius = 0.35
        self.safety_margin = 0.05

        # Total collision clearance = 0.40 m.
        self.collision_radius = (
            self.rover_radius + self.safety_margin
        )

        # =================================================
        # OBSTACLE AVOIDANCE STATE
        # =================================================

        self.avoidance_active = False
        self.avoidance_direction = 0.0
        self.avoidance_timer = 0.0

        # Rotate in place first when trapped in a three-sided pocket.
        self.escape_turning = False
        self.escape_direction_locked = False

        # =================================================
        # SIMULATED OBSTACLE SENSORS
        # =================================================

        self.sensor_range = 1.6

        # Gaussian range error (metres, 1 sigma) added when a ray
        # hits something. Set to 0.0 for perfect sensors.
        self.sensor_noise = 0.015
        self.noise_rng = random.Random()

        # Slightly wider coverage helps when approaching
        # an obstacle at an angle.
        self.sensor_angles = [
            math.radians(-60),
            math.radians(-30),
            0.0,
            math.radians(30),
            math.radians(60)
        ]

        self.sensor_distances = [
            self.sensor_range
            for _ in self.sensor_angles
        ]

        # Keep raw ray-hit distances separate from clearance
        # distances. Navigation uses the latter; rendering uses
        # the former so rays can end at the sensed obstacle edge.
        self.sensor_hit_distances = [
            self.sensor_range
            for _ in self.sensor_angles
        ]

        self.sensor_obstacle_hits = [
            False
            for _ in self.sensor_angles
        ]

        # Covers the ray-march increment and the two-pixel line
        # width so a rendered ray never enters an obstacle cell.
        self.sensor_visual_padding = 0.05

        self.sensor_warning = False

        # IMPORTANT:
        # sensor_distances are CLEARANCE values, not raw
        # ray-hit distances. 0 means the rover is touching
        # / inside the physical clearance boundary.

        # Keep warning below the corridor half-width so a
        # one-cell (~1 m) corridor does not constantly
        # trigger avoidance.
        self.sensor_warning_distance = 0.28
        self.sensor_emergency_distance = 0.12

        # =================================================
        # DISPLAY
        # =================================================

        self.color = (0, 150, 255)
        self.radius = 15
        self.pixels_per_meter = 40

    # =====================================================
    # SET PATH
    # =====================================================

    def set_path(self, path):

        self.path = path
        self.path_index = 0

        if path:

            self.goal = path[-1]

            # A* includes the rover's current grid cell as its
            # first waypoint. It is behind the actual route and can
            # require an unnecessary turn back to the cell centre.

            # Start at the first outgoing waypoint when one exists.
            current_cell = (
                int(self.y),
                int(self.x)
            )

            if (
                len(path) > 1
                and path[0] == current_cell
            ):

                self.path_index = 1

        self.replan_requested = False
        self.target_reached = False
        self.collision_warning = False

        self.avoidance_active = False
        self.avoidance_direction = 0.0
        self.avoidance_timer = 0.0

        self.escape_turning = False
        self.escape_direction_locked = False

        self.sensor_warning = False

        self.pid.reset()

    # =====================================================
    # NORMALIZE ANGLE
    # =====================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle

    # =====================================================
    # DISTANCE TO CELL
    # =====================================================

    def distance_to_cell(
        self,
        x,
        y,
        row,
        col
    ):

        left = col
        right = col + 1

        top = row
        bottom = row + 1

        closest_x = max(
            left,
            min(x, right)
        )

        closest_y = max(
            top,
            min(y, bottom)
        )

        dx = x - closest_x
        dy = y - closest_y

        return math.sqrt(
            dx * dx
            +
            dy * dy
        )

    # =====================================================
    # COLLISION CHECK
    # =====================================================

    def is_collision(
        self,
        terrain,
        test_x=None,
        test_y=None
    ):

        if test_x is None:
            test_x = self.x

        if test_y is None:
            test_y = self.y

        current_row = int(test_y)
        current_col = int(test_x)

        # -------------------------------------------------
        # OUTSIDE MAP
        # -------------------------------------------------

        if (
            current_row < 0
            or current_row >= terrain.rows
            or current_col < 0
            or current_col >= terrain.cols
        ):

            return True

        # Keep the rover's collision circle inside the terrain,
        # not only its centre point.
        if (
            test_x < self.collision_radius
            or test_x > terrain.cols - self.collision_radius
            or test_y < self.collision_radius
            or test_y > terrain.rows - self.collision_radius
        ):

            return True

        # -------------------------------------------------
        # CHECK NEARBY CELLS
        # -------------------------------------------------

        check_radius = 1

        for row in range(
            current_row - check_radius,
            current_row + check_radius + 1
        ):

            for col in range(
                current_col - check_radius,
                current_col + check_radius + 1
            ):

                if (
                    row < 0
                    or row >= terrain.rows
                    or col < 0
                    or col >= terrain.cols
                ):

                    continue

                if terrain.map[row][col] == "sand":
                    continue

                distance = self.distance_to_cell(
                    test_x,
                    test_y,
                    row,
                    col
                )

                if distance < self.collision_radius:
                    return True

        return False

    # =====================================================
    # OBSTACLE SENSOR
    # =====================================================

    def sense_obstacles(self, terrain):

        distances = []
        hit_distances = []
        obstacle_hits = []

        # Smaller step = more accurate obstacle boundary.
        step = 0.025

        for relative_angle in self.sensor_angles:

            sensor_angle = (
                self.theta
                +
                relative_angle
            )

            # Start slightly away from the rover centre.
            # The rover body is accounted for separately below.
            ray_distance = 0.05

            detected_distance = self.sensor_range
            hit_distance = self.sensor_range
            obstacle_hit = False

            while ray_distance <= self.sensor_range:

                test_x = (
                    self.x
                    +
                    math.cos(sensor_angle)
                    *
                    ray_distance
                )

                test_y = (
                    self.y
                    +
                    math.sin(sensor_angle)
                    *
                    ray_distance
                )

                if self.sensor_hits_obstacle(
                    terrain,
                    test_x,
                    test_y
                ):

                    # Convert raw ray distance into the
                    # remaining clearance outside the rover.
                    #
                    # Example:
                    # obstacle hit at 0.75 m
                    # rover radius = 0.35 m
                    # clearance = 0.40 m
                    #
                    # This makes the displayed sensor line
                    # stop BEFORE the obstacle instead of
                    # visually entering it.

                    noisy_distance = max(
                        0.01,
                        ray_distance
                        + self.noise_rng.gauss(
                            0.0,
                            self.sensor_noise
                        )
                    )

                    detected_distance = max(
                        0.0,
                        noisy_distance - self.rover_radius
                    )

                    hit_distance = noisy_distance
                    obstacle_hit = True

                    break

                ray_distance += step

            distances.append(detected_distance)
            hit_distances.append(hit_distance)
            obstacle_hits.append(obstacle_hit)

        self.sensor_distances = distances
        self.sensor_hit_distances = hit_distances
        self.sensor_obstacle_hits = obstacle_hits

        # -------------------------------------------------
        # FRONT SENSOR GROUP
        # -------------------------------------------------

        front_distance = min(
            distances[1],
            distances[2],
            distances[3]
        )

        self.sensor_warning = (
            front_distance
            <=
            self.sensor_warning_distance
        )

        return distances

    # =====================================================
    # SENSOR TERRAIN TEST
    # =====================================================

    def sensor_hits_obstacle(
        self,
        terrain,
        x,
        y
    ):

        row = int(y)
        col = int(x)

        if (
            row < 0
            or row >= terrain.rows
            or col < 0
            or col >= terrain.cols
        ):

            return True

        return terrain.map[row][col] != "sand"

    # =====================================================
    # GET SENSOR DATA
    # =====================================================

    def get_sensor_data(self):

        return {
            "left_sensor":
                self.sensor_distances[0],

            "left_front_sensor":
                self.sensor_distances[1],

            "front_sensor":
                self.sensor_distances[2],

            "right_front_sensor":
                self.sensor_distances[3],

            "right_sensor":
                self.sensor_distances[4],

            "warning":
                self.sensor_warning
        }

    # =====================================================
    # PREDICT NEXT POSITION
    # =====================================================

    def predicted_position(self, dt):

        next_x, next_y, _ = self.predicted_pose(dt)

        return next_x, next_y

    # =====================================================
    # PREDICT DIFFERENTIAL-DRIVE POSE
    # =====================================================

    def predicted_pose(self, dt):

        v = (
            self.v_right
            +
            self.v_left
        ) / 2

        omega = (
            self.v_right
            -
            self.v_left
        ) / self.wheel_base

        next_theta = self.normalize_angle(
            self.theta
            +
            omega * dt
        )

        if abs(omega) < 1e-6:

            next_x = (
                self.x
                +
                v
                *
                math.cos(self.theta)
                *
                dt
            )

            next_y = (
                self.y
                +
                v
                *
                math.sin(self.theta)
                *
                dt
            )

            return next_x, next_y, next_theta

        turn_radius = v / omega

        next_x = (
            self.x
            +
            turn_radius
            *
            (
                math.sin(next_theta)
                -
                math.sin(self.theta)
            )
        )

        next_y = (
            self.y
            -
            turn_radius
            *
            (
                math.cos(next_theta)
                -
                math.cos(self.theta)
            )
        )

        return next_x, next_y, next_theta

    # =====================================================
    # PREDICT SWEPT BODY COLLISION
    # =====================================================

    def predicted_collision(
        self,
        terrain,
        dt
    ):

        v = abs(
            (
                self.v_right
                +
                self.v_left
            ) / 2
        )

        omega = abs(
            (
                self.v_right
                -
                self.v_left
            )
            /
            self.wheel_base
        )

        # Sample both translation and rotation. The rover body is
        # circular, so testing its collision circle at each sampled
        # centre covers the swept body during an arc.

        distance_steps = math.ceil(
            v * dt / 0.04
        )

        turn_steps = math.ceil(
            omega * dt / math.radians(5)
        )

        steps = max(
            1,
            distance_steps,
            turn_steps
        )

        for step in range(
            1,
            steps + 1
        ):

            sample_dt = (
                dt
                *
                step
                /
                steps
            )

            test_x, test_y, _ = self.predicted_pose(
                sample_dt
            )

            if self.is_collision(
                terrain,
                test_x,
                test_y
            ):

                return True

        return False

    # =====================================================
    # WAYPOINT ACCEPTANCE
    # =====================================================

    def waypoint_acceptance_distance(self):

        if self.path_index >= len(self.path) - 1:
            return 0.20

        if (
            self.path_index == 0
            or self.path_index >= len(self.path) - 1
        ):
            return 0.45

        previous_row, previous_col = self.path[
            self.path_index - 1
        ]

        current_row, current_col = self.path[
            self.path_index
        ]

        next_row, next_col = self.path[
            self.path_index + 1
        ]

        incoming = (
            current_row - previous_row,
            current_col - previous_col
        )

        outgoing = (
            next_row - current_row,
            next_col - current_col
        )

        # Straight runs retain the original generous handoff.
        # At a corner, capture the waypoint more closely before
        # steering toward the next segment.

        if incoming == outgoing:
            return 0.45

        return 0.28

    # =====================================================
    # FIND SAFE ESCAPE DIRECTION
    # =====================================================

    def find_escape_direction(self, terrain):

        test_angles = [
            0.0,

            math.pi / 8,
            -math.pi / 8,

            math.pi / 4,
            -math.pi / 4,

            3 * math.pi / 8,
            -3 * math.pi / 8,

            math.pi / 2,
            -math.pi / 2,

            5 * math.pi / 8,
            -5 * math.pi / 8,

            3 * math.pi / 4,
            -3 * math.pi / 4,

            7 * math.pi / 8,
            -7 * math.pi / 8,

            math.pi
        ]

        best_angle = None
        best_score = -float("inf")

        target_angle = None

        if self.goal is not None:

            goal_row, goal_col = self.goal

            target_angle = math.atan2(
                goal_row + 0.5 - self.y,
                goal_col + 0.5 - self.x
            )

        for relative_angle in test_angles:

            angle = (
                self.theta
                +
                relative_angle
            )

            free_distance = 0.0

            test_step = 0.05

            while free_distance < 1.50:

                test_distance = (
                    free_distance
                    +
                    test_step
                )

                test_x = (
                    self.x
                    +
                    math.cos(angle)
                    *
                    test_distance
                )

                test_y = (
                    self.y
                    +
                    math.sin(angle)
                    *
                    test_distance
                )

                if self.is_collision(
                    terrain,
                    test_x,
                    test_y
                ):

                    break

                free_distance = test_distance

            if free_distance < 0.20:
                continue

            target_score = 0.0

            if target_angle is not None:

                target_error = abs(
                    self.normalize_angle(
                        target_angle
                        -
                        angle
                    )
                )

                target_score = (
                    math.pi
                    -
                    target_error
                ) * 0.75

            score = (
                free_distance * 4.0
                +
                target_score
                -
                abs(relative_angle) * 0.15
            )

            if score > best_score:

                best_score = score
                best_angle = angle

        return best_angle

    # =====================================================
    # CHECK DIRECTION SAFETY
    # =====================================================

    def direction_is_safe(
        self,
        terrain,
        angle,
        distance=0.6
    ):

        steps = 12

        for i in range(
            1,
            steps + 1
        ):

            test_distance = (
                distance
                *
                i
                /
                steps
            )

            test_x = (
                self.x
                +
                math.cos(angle)
                *
                test_distance
            )

            test_y = (
                self.y
                +
                math.sin(angle)
                *
                test_distance
            )

            if self.is_collision(
                terrain,
                test_x,
                test_y
            ):

                return False

        return True

    # =====================================================
    # OBSTACLE AVOIDANCE
    # =====================================================

    def obstacle_avoidance(
        self,
        dt,
        terrain
    ):

        self.collision_warning = True

        self.sense_obstacles(
            terrain
        )

        # Three front sensors blocked = pocket/dead-end geometry.
        pocket = (
            self.sensor_distances[1]
            <=
            self.sensor_warning_distance

            and

            self.sensor_distances[2]
            <=
            self.sensor_warning_distance

            and

            self.sensor_distances[3]
            <=
            self.sensor_warning_distance
        )

        if not self.avoidance_active:

            escape_angle = self.find_escape_direction(
                terrain
            )

            # =================================================
            # NEW STUCK-STATE HANDLING
            # =================================================
            #
            # If no translational escape is available, rotate
            # in place toward the side with more clearance.
            # This prevents the rover from freezing when it is
            # trapped with three sides blocked.
            #

            if escape_angle is None:

                turn_speed = 0.30

                if (
                    self.sensor_distances[1]
                    >=
                    self.sensor_distances[3]
                ):

                    self.v_left = -turn_speed
                    self.v_right = turn_speed

                else:

                    self.v_left = turn_speed
                    self.v_right = -turn_speed

                self.escape_turning = True
                self.escape_direction_locked = True

                self.avoidance_timer += dt

                return

            self.avoidance_active = True

            self.avoidance_direction = escape_angle

            self.avoidance_timer = 0.0

            self.escape_direction_locked = True

            self.escape_turning = pocket

            self.pid.reset()

        self.avoidance_timer += dt

        # Do not continuously pick a new direction. That creates
        # left/right oscillation in tight spaces.

        if (
            self.escape_direction_locked

            and

            not self.direction_is_safe(
                terrain,
                self.avoidance_direction,
                0.35
            )
        ):

            new_direction = self.find_escape_direction(
                terrain
            )

            if new_direction is not None:

                self.avoidance_direction = new_direction

                self.escape_turning = True

                self.pid.reset()

            else:

                # =================================================
                # NEW STUCK-STATE HANDLING
                # =================================================
                #
                # The locked escape route became blocked.
                # Keep rotating toward the more open side rather
                # than stopping and getting stuck.
                #

                turn_speed = 0.30

                if (
                    self.sensor_distances[1]
                    >=
                    self.sensor_distances[3]
                ):

                    self.v_left = -turn_speed
                    self.v_right = turn_speed

                else:

                    self.v_left = turn_speed
                    self.v_right = -turn_speed

                self.escape_turning = True

                return

        angle_error = self.normalize_angle(
            self.avoidance_direction
            -
            self.theta
        )

        left_distance = self.sensor_distances[1]
        front_distance = self.sensor_distances[2]
        right_distance = self.sensor_distances[3]

        # -------------------------------------------------
        # ROTATE FIRST IN A THREE-SIDED POCKET
        # -------------------------------------------------

        if self.escape_turning:

            turn_tolerance = math.radians(10)

            if abs(angle_error) > turn_tolerance:

                turn_speed = 0.38

                if angle_error > 0:

                    self.v_left = -turn_speed
                    self.v_right = turn_speed

                else:

                    self.v_left = turn_speed
                    self.v_right = -turn_speed

                # No translation while turning. This is important:
                # the circular rover body stays at its already-safe
                # centre instead of sweeping a corner into an obstacle.

                return

            self.escape_turning = False

        # -------------------------------------------------
        # ADAPTIVE FORWARD SPEED
        # -------------------------------------------------

        if (
            front_distance
            <=
            self.sensor_emergency_distance
        ):

            forward_speed = 0.04

        elif (
            front_distance
            <=
            self.sensor_warning_distance
        ):

            forward_speed = 0.12

        elif front_distance <= 0.50:

            forward_speed = 0.28

        else:

            forward_speed = 0.55

        turn_amount = abs(angle_error)

        if turn_amount > math.radians(45):

            forward_speed = min(
                forward_speed,
                0.12
            )

        elif turn_amount > math.radians(20):

            forward_speed = min(
                forward_speed,
                0.25
            )

        turn_gain = 1.10

        correction = max(
            -0.55,
            min(
                0.55,
                angle_error
                *
                turn_gain
            )
        )

        # Wall balancing is disabled in pocket mode so it cannot
        # fight the selected escape direction.

        if (
            not pocket

            and

            left_distance < 0.55

            and

            right_distance < 0.55
        ):

            side_error = (
                right_distance
                -
                left_distance
            )

            wall_correction = max(
                -0.10,
                min(
                    0.10,
                    side_error
                    *
                    0.25
                )
            )

            correction += wall_correction

        self.v_left = (
            forward_speed
            -
            correction
        )

        self.v_right = (
            forward_speed
            +
            correction
        )

        # Never push a predicted rover-body collision.

        if self.predicted_collision(
            terrain,
            min(
                dt * 2.0,
                0.10
            )
        ):

            if abs(angle_error) > math.radians(12):

                self.escape_turning = True

                if angle_error > 0:

                    self.v_left = -0.30
                    self.v_right = 0.30

                else:

                    self.v_left = 0.30
                    self.v_right = -0.30

            else:

                self.v_left = 0.0
                self.v_right = 0.0

        self.v_left = max(
            -self.max_wheel_speed,
            min(
                self.max_wheel_speed,
                self.v_left
            )
        )

        self.v_right = max(
            -self.max_wheel_speed,
            min(
                self.max_wheel_speed,
                self.v_right
            )
        )

        # -------------------------------------------------
        # LEAVE AVOIDANCE ONLY AFTER A REAL ESCAPE
        # -------------------------------------------------

        self.sense_obstacles(
            terrain
        )

        front_clear = (
            self.sensor_distances[2]
            >
            0.50
        )

        direction_clear = self.direction_is_safe(
            terrain,
            self.theta,
            0.50
        )

        if (
            self.avoidance_timer > 0.60

            and

            not self.escape_turning

            and

            front_clear

            and

            direction_clear
        ):

            self.avoidance_active = False

            self.escape_turning = False

            self.escape_direction_locked = False

            self.avoidance_timer = 0.0

            self.collision_warning = False

            self.pid.reset()

            self.replan_requested = True

    # =====================================================
    # AUTONOMOUS CONTROL
    # =====================================================

    def autonomous_control(
        self,
        dt,
        terrain
    ):

        self.collision_warning = False

        self.sense_obstacles(
            terrain
        )

        # -------------------------------------------------
        # NO PATH
        # -------------------------------------------------

        if not self.path:

            self.v_left = 0.0
            self.v_right = 0.0

            return

        # -------------------------------------------------
        # CURRENT PHYSICAL COLLISION
        # -------------------------------------------------

        if self.is_collision(
            terrain
        ):

            self.obstacle_avoidance(
                dt,
                terrain
            )

            return

        # -------------------------------------------------
        # ACTIVE AVOIDANCE
        # -------------------------------------------------

        if self.avoidance_active:

            self.obstacle_avoidance(
                dt,
                terrain
            )

            return

        # -------------------------------------------------
        # SENSOR WARNING
        # -------------------------------------------------

        if self.sensor_warning:

            self.obstacle_avoidance(
                dt,
                terrain
            )

            return

        # -------------------------------------------------
        # PATH COMPLETE
        # -------------------------------------------------

        if (
            self.path_index
            >=
            len(self.path)
        ):

            self.v_left = 0.0
            self.v_right = 0.0

            self.target_reached = True

            return

        # -------------------------------------------------
        # CURRENT WAYPOINT
        # -------------------------------------------------

        row, col = self.path[
            self.path_index
        ]

        # -------------------------------------------------
        # SEGMENT TRACKING (slow-down / speed-up ramp)
        # Remember where this leg of the path started.
        # -------------------------------------------------

        segment_key = (
            id(self.path),
            len(self.path),
            self.path_index
        )

        if getattr(self, "_segment_key", None) != segment_key:

            self._segment_key = segment_key

            self._segment_start = (
                self.x,
                self.y
            )

        waypoint_x = col + 0.5
        waypoint_y = row + 0.5

        dx = (
            waypoint_x
            -
            self.x
        )

        dy = (
            waypoint_y
            -
            self.y
        )

        distance = math.sqrt(
            dx * dx
            +
            dy * dy
        )

        # -------------------------------------------------
        # WAYPOINT REACHED
        # -------------------------------------------------

        if (
            distance
            <
            self.waypoint_acceptance_distance()
        ):

            if (
                self.path_index
                <
                len(self.path) - 1
            ):

                self.path_index += 1

                self.pid.reset()

                return

            else:

                self.v_left = 0.0
                self.v_right = 0.0

                self.target_reached = True

                return

        # -------------------------------------------------
        # DESIRED HEADING
        # -------------------------------------------------

        desired_theta = math.atan2(
            dy,
            dx
        )

        # -------------------------------------------------
        # HEADING ERROR
        # -------------------------------------------------

        heading_error = self.normalize_angle(
            desired_theta
            -
            self.theta
        )

        # -------------------------------------------------
        # PID
        # -------------------------------------------------

        omega = self.pid.update(
            heading_error,
            dt
        )

        # -------------------------------------------------
        # LINEAR VELOCITY
        # -------------------------------------------------

        # Waypoint speed profile:
        #   * leaving a point -> start slow, speed up along the leg
        #   * nearing a point -> slow down, nearly crawling at it
        #   * last point      -> slow down smoothly and stop
        #
        # Tune these two numbers if you want a different feel:
        #   POINT_SPEED = speed at a waypoint (m/s)
        #   RAMP_RATE   = how quickly it speeds up / slows down

        POINT_SPEED = 0.15
        RAMP_RATE = 1.6

        accept = self.waypoint_acceptance_distance()

        from_start = math.hypot(
            self.x - self._segment_start[0],
            self.y - self._segment_start[1]
        )

        is_last_point = (
            self.path_index
            >=
            len(self.path) - 1
        )

        if is_last_point:

            approach_speed = (
                0.08
                +
                0.90 * max(0.0, distance - accept)
            )

        else:

            approach_speed = (
                POINT_SPEED
                +
                RAMP_RATE * max(0.0, distance - accept)
            )

        leave_speed = (
            POINT_SPEED
            +
            RAMP_RATE * from_start
        )

        linear_velocity = min(
            1.20,
            approach_speed,
            leave_speed
        )

        # Slow down when heading is significantly wrong.

        heading_factor = max(
            0.20,
            math.cos(
                min(
                    abs(heading_error),
                    math.pi / 2
                )
            )
        )

        linear_velocity *= heading_factor

        # -------------------------------------------------
        # DIFFERENTIAL DRIVE
        # -------------------------------------------------

        self.v_left = (
            linear_velocity
            -
            omega
            *
            self.wheel_base
            /
            2
        )

        self.v_right = (
            linear_velocity
            +
            omega
            *
            self.wheel_base
            /
            2
        )

        # -------------------------------------------------
        # SPEED LIMIT
        # -------------------------------------------------

        self.v_left = max(
            -self.max_wheel_speed,
            min(
                self.max_wheel_speed,
                self.v_left
            )
        )

        self.v_right = max(
            -self.max_wheel_speed,
            min(
                self.max_wheel_speed,
                self.v_right
            )
        )

        # -------------------------------------------------
        # PREDICTIVE SAFETY
        # -------------------------------------------------

        if self.predicted_collision(
            terrain,
            dt * 1.5
        ):

            self.obstacle_avoidance(
                dt,
                terrain
            )

    # =====================================================
    # UPDATE KINEMATICS
    # =====================================================

    def update(self, dt):

        # -------------------------------------------------
        # DIFFERENTIAL DRIVE
        # -------------------------------------------------

        self.linear_velocity = (
            self.v_right
            +
            self.v_left
        ) / 2

        self.angular_velocity = (
            self.v_right
            -
            self.v_left
        ) / self.wheel_base

        # -------------------------------------------------
        # ROVER KINEMATICS
        # -------------------------------------------------

        self.dx_dt = (
            self.linear_velocity
            *
            math.cos(self.theta)
        )

        self.dy_dt = (
            self.linear_velocity
            *
            math.sin(self.theta)
        )

        self.dtheta_dt = (
            self.angular_velocity
        )

        # -------------------------------------------------
        # POSITION UPDATE
        # -------------------------------------------------

        self.x, self.y, self.theta = self.predicted_pose(
            dt
        )

    # =====================================================
    # GET DYNAMICS
    # =====================================================

    def get_dynamics(self):

        return {

            "linear_velocity":
                self.linear_velocity,

            "angular_velocity":
                self.angular_velocity,

            "dx_dt":
                self.dx_dt,

            "dy_dt":
                self.dy_dt,

            "dtheta_dt":
                self.dtheta_dt,

            "left_wheel":
                self.v_left,

            "right_wheel":
                self.v_right,

            "theta":
                self.theta
        }

    # =====================================================
    # DRAW ROVER
    # =====================================================

    def draw(self, screen):

        screen_x = int(
            self.x
            *
            self.pixels_per_meter
        )

        screen_y = int(
            self.y
            *
            self.pixels_per_meter
        )

        # -------------------------------------------------
        # ROVER
        # -------------------------------------------------

        pygame.draw.circle(
            screen,
            self.color,
            (
                screen_x,
                screen_y
            ),
            self.radius
        )

        # -------------------------------------------------
        # SAFETY CIRCLE
        # -------------------------------------------------

        safety_pixels = int(
            self.collision_radius
            *
            self.pixels_per_meter
        )

        pygame.draw.circle(
            screen,
            (
                255,
                255,
                255
            ),
            (
                screen_x,
                screen_y
            ),
            safety_pixels,
            1
        )

        # -------------------------------------------------
        # DIRECTION
        # -------------------------------------------------

        direction_length = 28

        end_x = (
            screen_x
            +
            int(
                math.cos(self.theta)
                *
                direction_length
            )
        )

        end_y = (
            screen_y
            +
            int(
                math.sin(self.theta)
                *
                direction_length
            )
        )

        pygame.draw.line(
            screen,
            (
                255,
                255,
                255
            ),
            (
                screen_x,
                screen_y
            ),
            (
                end_x,
                end_y
            ),
            4
        )

        # -------------------------------------------------
        # SENSOR VISUALIZATION
        #
        # Green = clear
        # Yellow = obstacle detected / warning distance
        # Red = obstacle very close
        # -------------------------------------------------

        for i, relative_angle in enumerate(
            self.sensor_angles
        ):

            sensor_angle = (
                self.theta
                +
                relative_angle
            )

            sensor_distance = (
                self.sensor_hit_distances[i]
            )

            if self.sensor_obstacle_hits[i]:

                sensor_distance = max(
                    0.0,
                    sensor_distance
                    -
                    self.sensor_visual_padding
                )

            sensor_end_x = (
                screen_x
                +
                int(
                    math.cos(sensor_angle)
                    *
                    sensor_distance
                    *
                    self.pixels_per_meter
                )
            )

            sensor_end_y = (
                screen_y
                +
                int(
                    math.sin(sensor_angle)
                    *
                    sensor_distance
                    *
                    self.pixels_per_meter
                )
            )

            # Use the clearance distance for the colour logic.
            clearance_distance = (
                self.sensor_distances[i]
            )

            if (
                clearance_distance
                <=
                self.sensor_emergency_distance
            ):

                sensor_color = (
                    255,
                    60,
                    60
                )

            elif (
                clearance_distance
                <=
                self.sensor_warning_distance
            ):

                sensor_color = (
                    255,
                    220,
                    50
                )

            else:

                sensor_color = (
                    100,
                    255,
                    100
                )

            pygame.draw.line(
                screen,
                sensor_color,
                (
                    screen_x,
                    screen_y
                ),
                (
                    sensor_end_x,
                    sensor_end_y
                ),
                2
            )

    # =====================================================
    # DISTANCE TO TARGET
    # =====================================================

    def get_distance_to_target(self):

        if not self.path:

            return 0.0

        row, col = self.path[-1]

        target_x = col + 0.5
        target_y = row + 0.5

        return math.sqrt(
            (
                target_x
                -
                self.x
            ) ** 2
            +
            (
                target_y
                -
                self.y
            ) ** 2
        )