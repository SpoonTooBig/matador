import time

class PIDController:

    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, current_value, current_time=None):
        # Determine time step (dt)
        if current_time is None:
            current_time = time.time()

        if self.prev_time is None:
            self.prev_time = current_time
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0.0:
            return 0.0  # Avoid division by zero if updates happen instantly

        # Calculate error
        error = self.setpoint - current_value

        # Proportional term
        p_out = self.kp * error

        # Integral term with anti-windup (if limits are set)
        self.integral += error * dt
        i_out = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_out = self.kd * derivative

        # Total output
        output = p_out + i_out + d_out

        # Apply output limits if defined
        min_limit, max_limit = self.output_limits
        if min_limit is not None or max_limit is not None:
        # Clamp output
            output = max(
                min_limit if min_limit is not None else float('-inf'),
                min(output, max_limit if max_limit is not None else float('inf')),
            )
        # Basic anti-windup: adjust integral if we hit a limit
        if output == min_limit or output == max_limit:
            self.integral -= error * dt

        # Save state for next iteration
        self.prev_error = error
        self.prev_time = current_time

        return output

    def change_setpoint(self, new_setpoint):
        """Change the target setpoint dynamically."""
        self.setpoint = new_setpoint

    def reset(self):
        """Reset the controller states to prevent sudden jumps if paused."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None