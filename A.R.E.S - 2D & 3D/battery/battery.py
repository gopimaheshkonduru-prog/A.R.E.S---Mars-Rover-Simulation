class Battery:

    def __init__(self, capacity=100.0):

        self.capacity = capacity
        self.level = capacity

        self.total_energy_used = 0.0

    # --------------------------------------------------
    # CONSUME ENERGY
    # --------------------------------------------------

    def consume(self, left_speed, right_speed, dt, load=1.0):

        # load > 1 = extra drain (dust storm, soft sand, ...)

        movement = (
            abs(left_speed)
            + abs(right_speed)
        ) / 2

        # Energy consumption model
        consumption_rate = (
            0.08
            + 0.12 * movement
        )

        energy_used = (
            consumption_rate * dt * load
        )

        self.level -= energy_used

        self.total_energy_used += energy_used

        self.level = max(
            0.0,
            self.level
        )

    # --------------------------------------------------
    # BATTERY AVAILABLE?
    # --------------------------------------------------

    def available(self):

        return self.level > 1.0

    # --------------------------------------------------
    # PERCENTAGE
    # --------------------------------------------------

    def percentage(self):

        return (
            self.level
            / self.capacity
            * 100
        )

    # --------------------------------------------------
    # RESET
    # --------------------------------------------------

    def reset(self):

        self.level = self.capacity

        self.total_energy_used = 0.0