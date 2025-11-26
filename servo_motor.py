from machine import PWM
import math

# --------- Adjustable configuration values ---------

# Default servo timing (tweak these)
DEFAULT_PULSE_MIN_MS = 1.0     # ms at minimum angle
DEFAULT_PULSE_MAX_MS = 2.0     # ms at maximum angle
DEFAULT_FRAME_MS = 20.0        # ms per period (50 Hz)

# Default arm geometry (edit to match hardware)
DEFAULT_LINK1_LENGTH_MM = 90.0
DEFAULT_LINK2_LENGTH_MM = 90.0

# Default allowed angle range for the servos
DEFAULT_MIN_ANGLE_DEG = 0.0
DEFAULT_MAX_ANGLE_DEG = 180.0


class servo_motor:
    """
    Low-level driver for a 2-link arm on the Pico.

    - Takes a target (x, y) in millimetres.
    - Uses inverse kinematics to find joint angles.
    - Converts angles to PWM duty_u16 for the servos.
    """

    def __init__(self,
                 shoulder_output: PWM,
                 elbow_output: PWM,
                 pwm_hz: int,
                 link1_length_mm: float = DEFAULT_LINK1_LENGTH_MM,
                 link2_length_mm: float = DEFAULT_LINK2_LENGTH_MM,
                 pulse_min_ms: float = DEFAULT_PULSE_MIN_MS,
                 pulse_max_ms: float = DEFAULT_PULSE_MAX_MS,
                 frame_ms: float = DEFAULT_FRAME_MS,
                 min_angle_deg: float = DEFAULT_MIN_ANGLE_DEG,
                 max_angle_deg: float = DEFAULT_MAX_ANGLE_DEG):
        """
        Args:
            shoulder_output: PWM connected to the shoulder servo.
            elbow_output:    PWM connected to the elbow servo.
            pwm_hz:          PWM frequency (usually 50 Hz).
            link1_length_mm: Length of first arm segment.
            link2_length_mm: Length of second arm segment.
            pulse_min_ms:    Pulse width at the minimum angle.
            pulse_max_ms:    Pulse width at the maximum angle.
            frame_ms:        Period of the PWM signal.
            min_angle_deg:   Smallest angle allowed for the servos.
            max_angle_deg:   Largest angle allowed for the servos.
        """
        self.shoulder_pwm = shoulder_output
        self.elbow_pwm = elbow_output

        self.link1 = float(link1_length_mm)
        self.link2 = float(link2_length_mm)

        self.pulse_min_ms = float(pulse_min_ms)
        self.pulse_max_ms = float(pulse_max_ms)
        self.frame_ms = float(frame_ms)

        self.min_angle_deg = float(min_angle_deg)
        self.max_angle_deg = float(max_angle_deg)

        self.shoulder_pwm.freq(pwm_hz)
        self.elbow_pwm.freq(pwm_hz)

    # ---------- Inverse kinematics ----------

    def compute_joint_angles(self, target_x_mm: float, target_y_mm: float):
        """
        For a given (x, y) in mm, compute the shoulder and elbow angles in degrees.

        Returns:
            (shoulder_deg, elbow_deg) clamped between min_angle_deg and max_angle_deg.
        """
        x_pos = float(target_x_mm)
        y_pos = float(target_y_mm)

        l1 = self.link1
        l2 = self.link2

        # Distance from shoulder to target
        dist_sq = x_pos * x_pos + y_pos * y_pos
        dist = math.sqrt(dist_sq) if dist_sq > 0.0 else 0.0

        # Restrict target to reachable radius if necessary
        max_span = l1 + l2
        if dist > max_span and dist > 0.0:
            scale_factor = max_span / dist
            x_pos *= scale_factor
            y_pos *= scale_factor
            dist_sq = x_pos * x_pos + y_pos * y_pos

        # Elbow using law of cosines
        cos_elbow = (dist_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_rad = math.acos(cos_elbow)
        elbow_deg = math.degrees(elbow_rad)

        # Shoulder angle
        k1 = l1 + l2 * math.cos(elbow_rad)
        k2 = l2 * math.sin(elbow_rad)
        shoulder_rad = math.atan2(y_pos, x_pos) - math.atan2(k2, k1)
        shoulder_deg = math.degrees(shoulder_rad)

        # Clamp angles using configured range
        shoulder_deg = max(self.min_angle_deg,
                           min(self.max_angle_deg, shoulder_deg))
        elbow_deg = max(self.min_angle_deg,
                        min(self.max_angle_deg, elbow_deg))

        return shoulder_deg, elbow_deg

    # ---------- Angle -> duty conversion ----------

    def _angle_deg_to_duty(self, angle_deg: float) -> int:
        """
        Turn an angle in degrees into a duty_u16 value according to the
        configured timing and angle range.
        """
        if angle_deg < self.min_angle_deg:
            angle_deg = self.min_angle_deg
        if angle_deg > self.max_angle_deg:
            angle_deg = self.max_angle_deg

        angle_span = self.max_angle_deg - self.min_angle_deg
        if angle_span == 0.0:
            ratio = 0.0
        else:
            ratio = (angle_deg - self.min_angle_deg) / angle_span

        pulse_span_ms = self.pulse_max_ms - self.pulse_min_ms
        pulse_ms = self.pulse_min_ms + ratio * pulse_span_ms
        duty_ratio = pulse_ms / self.frame_ms
        duty_raw = int(duty_ratio * 65535.0)
        return duty_raw

    # ---------- Servo output helpers ----------

    def apply_joint_angles(self, shoulder_angle_deg: float, elbow_angle_deg: float):
        """
        Send explicit joint angles to the shoulder and elbow servos.
        """
        duty_shoulder = self._angle_deg_to_duty(shoulder_angle_deg)
        duty_elbow = self._angle_deg_to_duty(elbow_angle_deg)

        self.shoulder_pwm.duty_u16(duty_shoulder)
        self.elbow_pwm.duty_u16(duty_elbow)

    def move_tip_to(self, x_mm: float, y_mm: float):
        """
        Given (x, y) in mm, calculate joint angles and drive the servos.
        """
        shoulder_deg, elbow_deg = self.compute_joint_angles(x_mm, y_mm)
        self.apply_joint_angles(shoulder_deg, elbow_deg)
