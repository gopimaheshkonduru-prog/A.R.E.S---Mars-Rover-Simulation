class PIDController:

    def __init__(self, kp, ki, kd, output_limit):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_limit = output_limit

        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):

        if dt <= 0:
            return 0.0

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # Derivative term
        if self.previous_error is None:
            derivative = 0.0

        else:
            derivative = self.kd * (error - self.previous_error) / dt

        self.previous_error = error

        # PID output
        output = proportional + integral + derivative

        # Limit output
        output = max(
            -self.output_limit,
            min(self.output_limit, output)
        )

        return output

    def reset(self):

        self.integral = 0.0
        # The next update establishes a baseline instead of
        # differentiating from an unrelated pre-reset error.
        self.previous_error = None
