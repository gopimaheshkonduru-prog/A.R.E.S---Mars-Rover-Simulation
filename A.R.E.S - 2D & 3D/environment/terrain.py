import pygame
import random

WIDTH = 1000
HEIGHT = 700
TILE_SIZE = 40

MARS = (139, 69, 19)
GRID = (110, 50, 10)

SAND = (139, 69, 19)
ROCK = (90, 90, 90)
CRATER = (40, 40, 40)

# A* movement-cost multipliers for drivable-but-bad ground (1.0 = normal sand)
SOFT_SAND_COST = 4.0
SLOPE_COST = 2.0


class Terrain:

    def __init__(self):

        self.rows = HEIGHT // TILE_SIZE
        self.cols = WIDTH // TILE_SIZE

        self.map = []

        for row in range(self.rows):

            current_row = []

            for col in range(self.cols):

                chance = random.randint(1, 100)

                if chance <= 10:
                    current_row.append("rock")

                elif chance <= 15:
                    current_row.append("crater")

                else:
                    current_row.append("sand")

            self.map.append(current_row)

        # Extra layer: drivable tiles that cost more to cross. The sensors
        # and collision code still see them as plain sand; only A* cares.
        # Drawn AFTER the map so the same seed still gives the same rocks.
        self.cost = [[1.0] * self.cols for _ in range(self.rows)]
        self.surface = [[None] * self.cols for _ in range(self.rows)]
        self._add_patches("soft", 5, 1.5, 3.0, SOFT_SAND_COST)
        self._add_patches("slope", 3, 1.5, 2.5, SLOPE_COST)

    def _add_patches(self, kind, count, rmin, rmax, cost):

        for _ in range(count):

            cr = random.uniform(0, self.rows)
            cc = random.uniform(0, self.cols)
            radius = random.uniform(rmin, rmax)

            for row in range(self.rows):

                for col in range(self.cols):

                    if self.map[row][col] != "sand":
                        continue

                    if self.surface[row][col] is not None:
                        continue

                    d = ((row + 0.5 - cr) ** 2 + (col + 0.5 - cc) ** 2) ** 0.5

                    if d <= radius:
                        self.surface[row][col] = kind
                        self.cost[row][col] = cost

    def tile_cost(self, row, col):

        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.cost[row][col]

        return 1.0

    def draw(self, screen):

        for row in range(self.rows):

            for col in range(self.cols):

                x = col * TILE_SIZE
                y = row * TILE_SIZE

                tile = self.map[row][col]

                if tile == "sand":
                    color = SAND

                elif tile == "rock":
                    color = ROCK

                elif tile == "crater":
                    color = CRATER

                pygame.draw.rect(
                    screen,
                    color,
                    (x, y, TILE_SIZE, TILE_SIZE)
                )

                pygame.draw.rect(
                    screen,
                    GRID,
                    (x, y, TILE_SIZE, TILE_SIZE),
                    1
                )