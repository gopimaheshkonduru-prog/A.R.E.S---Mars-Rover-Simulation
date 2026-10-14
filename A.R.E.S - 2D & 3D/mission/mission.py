class Mission:

    def __init__(self):

        self.status = "IDLE"

        self.target = None

        self.mission_count = 0

        self.completed_missions = 0

        self.failed_missions = 0

        self.total_distance = 0.0

        self.previous_x = None
        self.previous_y = None

    # --------------------------------------------------
    # START MISSION
    # --------------------------------------------------

    def start(self, target):

        self.target = target

        self.status = "NAVIGATING"

        self.mission_count += 1

        self.previous_x = None
        self.previous_y = None

    # --------------------------------------------------
    # UPDATE DISTANCE
    # --------------------------------------------------

    def update_distance(self, x, y):

        if self.previous_x is not None:

            dx = x - self.previous_x
            dy = y - self.previous_y

            distance = (
                dx * dx
                + dy * dy
            ) ** 0.5

            self.total_distance += distance

        self.previous_x = x
        self.previous_y = y

    # --------------------------------------------------
    # COMPLETE
    # --------------------------------------------------

    def complete(self):

        self.status = "MISSION COMPLETE"

        self.completed_missions += 1

    # --------------------------------------------------
    # FAIL
    # --------------------------------------------------

    def fail(self):

        self.status = "MISSION FAILED"

        self.failed_missions += 1

    # --------------------------------------------------
    # RESET
    # --------------------------------------------------

    def reset(self):

        self.status = "IDLE"

        self.target = None