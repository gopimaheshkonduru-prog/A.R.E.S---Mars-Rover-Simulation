import heapq
import math


class AStar:

    def __init__(self, terrain):

        self.terrain = terrain

        # Match the rover's physical collision clearance:
        # 0.35 m body radius + 0.05 m safety margin.
        # Grid cells are one metre wide in the simulator.
        self.clearance_radius = 0.40


    # =====================================================
    # WALKABLE CELL
    # =====================================================

    def is_walkable(self, row, col):

        if row < 0 or row >= self.terrain.rows:
            return False

        if col < 0 or col >= self.terrain.cols:
            return False

        return self.terrain.map[row][col] == "sand"


    # =====================================================
    # SAFE CELL
    # =====================================================

    def is_safe_cell(self, row, col):

        if not self.is_walkable(
            row,
            col
        ):
            return False

        # A* waypoints are cell centres. Reject a waypoint when
        # the rover's clearance circle would overlap a rock/crater.
        center_x = col + 0.5
        center_y = row + 0.5

        for check_row in range(row - 1, row + 2):
            for check_col in range(col - 1, col + 2):
                if not self.is_walkable(
                    check_row,
                    check_col
                ):
                    if self.distance_to_cell(
                        center_x,
                        center_y,
                        check_row,
                        check_col
                    ) < self.clearance_radius:
                        return False

        return True


    # =====================================================
    # DISTANCE TO CELL
    # =====================================================

    def distance_to_cell(self, x, y, row, col):

        left = col
        right = col + 1
        top = row
        bottom = row + 1

        closest_x = max(left, min(x, right))
        closest_y = max(top, min(y, bottom))

        dx = x - closest_x
        dy = y - closest_y

        return math.sqrt(dx * dx + dy * dy)


    # =====================================================
    # FIND NEAREST WALKABLE CELL
    # =====================================================

    def find_nearest_safe_cell(self, position):

        row, col = position


        if self.is_safe_cell(
            row,
            col
        ):

            return position


        for radius in range(1, 6):

            for dr in range(
                -radius,
                radius + 1
            ):

                for dc in range(
                    -radius,
                    radius + 1
                ):

                    new_row = row + dr
                    new_col = col + dc


                    if self.is_safe_cell(
                        new_row,
                        new_col
                    ):

                        return (
                            new_row,
                            new_col
                        )


        return None


    # =====================================================
    # HEURISTIC
    # =====================================================

    def heuristic(self, a, b):

        dx = abs(
            a[1] - b[1]
        )

        dy = abs(
            a[0] - b[0]
        )

        return max(
            dx,
            dy
        )


    # =====================================================
    # NEIGHBORS
    # =====================================================

    def get_neighbors(self, node):

        row, col = node


        directions = [

            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),

            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1)

        ]


        neighbors = []


        for dr, dc in directions:

            new_row = row + dr
            new_col = col + dc


            if not self.is_safe_cell(
                new_row,
                new_col
            ):

                continue


            # -------------------------------------------------
            # IMPORTANT:
            # Prevent diagonal corner cutting.
            #
            # Example:
            #
            #   R X
            #   X .
            #
            # The rover cannot squeeze diagonally through X.
            # -------------------------------------------------

            if dr != 0 and dc != 0:

                if not self.is_safe_cell(
                    row + dr,
                    col
                ):

                    continue


                if not self.is_safe_cell(
                    row,
                    col + dc
                ):

                    continue


            neighbors.append(
                (
                    new_row,
                    new_col
                )
            )


        return neighbors


    # =====================================================
    # A* SEARCH
    # =====================================================

    def find_path(
        self,
        start,
        goal
    ):

        # -------------------------------------------------
        # VALIDATE START
        # -------------------------------------------------

        safe_start = self.find_nearest_safe_cell(
            start
        )


        if safe_start is None:

            return []


        # -------------------------------------------------
        # VALIDATE GOAL
        # -------------------------------------------------

        safe_goal = self.find_nearest_safe_cell(
            goal
        )


        if safe_goal is None:

            return []


        # -------------------------------------------------
        # OPEN SET
        # -------------------------------------------------

        open_set = []


        heapq.heappush(
            open_set,
            (
                0,
                safe_start
            )
        )


        came_from = {}


        cost_so_far = {

            safe_start: 0

        }


        # -------------------------------------------------
        # A* LOOP
        # -------------------------------------------------

        while open_set:

            current_priority, current = (
                heapq.heappop(
                    open_set
                )
            )


            if current == safe_goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )


            for neighbor in self.get_neighbors(
                current
            ):

                # Straight movement

                if (
                    neighbor[0] == current[0]
                    or
                    neighbor[1] == current[1]
                ):

                    movement_cost = 1.0


                # Diagonal movement

                else:

                    movement_cost = math.sqrt(2)


                # Soft sand / slope tiles cost more to enter.
                movement_cost *= self.terrain.tile_cost(
                    neighbor[0],
                    neighbor[1]
                )


                new_cost = (
                    cost_so_far[current]
                    + movement_cost
                )


                if (
                    neighbor not in cost_so_far
                    or
                    new_cost
                    < cost_so_far[neighbor]
                ):

                    cost_so_far[
                        neighbor
                    ] = new_cost


                    priority = (
                        new_cost
                        +
                        self.heuristic(
                            neighbor,
                            safe_goal
                        )
                    )


                    heapq.heappush(
                        open_set,
                        (
                            priority,
                            neighbor
                        )
                    )


                    came_from[
                        neighbor
                    ] = current


        return []


    # =====================================================
    # RECONSTRUCT PATH
    # =====================================================

    def reconstruct_path(
        self,
        came_from,
        current
    ):

        path = [

            current

        ]


        while current in came_from:

            current = came_from[
                current
            ]


            path.append(
                current
            )


        path.reverse()


        return path


    # =====================================================
    # PATH SMOOTHING (string pulling)
    # =====================================================

    def smooth_path(self, path, margin=0.20):

        # Drop every waypoint that can be skipped by driving in a
        # straight line. A shortcut is accepted only if the whole
        # line keeps clearance_radius + margin away from every
        # rock / crater / wall and stays on normal sand, so
        # smoothing never pulls the rover closer to an obstacle
        # than the original A* path.

        if len(path) < 3:

            return path

        smoothed = [path[0]]

        i = 0

        while i < len(path) - 1:

            j = len(path) - 1

            while (
                j > i + 1
                and not self.line_is_clear(
                    path[i],
                    path[j],
                    margin
                )
            ):

                j -= 1

            smoothed.append(path[j])

            i = j

        return smoothed


    def line_is_clear(self, a, b, margin):

        need = self.clearance_radius + margin

        x0, y0 = a[1] + 0.5, a[0] + 0.5
        x1, y1 = b[1] + 0.5, b[0] + 0.5

        steps = max(
            1,
            int(math.hypot(x1 - x0, y1 - y0) / 0.1)
        )

        for k in range(steps + 1):

            t = k / steps

            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t

            row, col = int(y), int(x)

            if self.terrain.tile_cost(row, col) > 1.0:

                return False

            for check_row in range(row - 1, row + 2):

                for check_col in range(col - 1, col + 2):

                    if (
                        not self.is_walkable(check_row, check_col)
                        and self.distance_to_cell(
                            x, y, check_row, check_col
                        ) < need
                    ):

                        return False

        return True
