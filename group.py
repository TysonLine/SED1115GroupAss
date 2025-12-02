# ---- AnalogPositionReader.py ----
from machine import ADC


# ADC input range
ADC_MIN_VALUE = 0
ADC_MAX_VALUE = 65535

# Default physical ranges in mm (tweak to match workspace)
DEFAULT_X_MIN_MM = 30.0
DEFAULT_X_MAX_MM = 240.0
DEFAULT_Y_MIN_MM = 40.0
DEFAULT_Y_MAX_MM = 200.0


class AnalogPositionReader:
    """
    Uses two ADC channels to measure control inputs and map them to
    physical coordinates in millimetres (or any numeric range).
    """

    def __init__(self,
                 adc_x_channel: ADC,
                 adc_y_channel: ADC,
                 x_min_mm: float = DEFAULT_X_MIN_MM,
                 x_max_mm: float = DEFAULT_X_MAX_MM,
                 y_min_mm: float = DEFAULT_Y_MIN_MM,
                 y_max_mm: float = DEFAULT_Y_MAX_MM,
                 adc_min_value: int = ADC_MIN_VALUE,
                 adc_max_value: int = ADC_MAX_VALUE):
        """
        Args:
            adc_x_channel: ADC used for the X input.
            adc_y_channel: ADC used for the Y input.
            x_min_mm:      Minimum X coordinate (or min output).
            x_max_mm:      Maximum X coordinate (or max output).
            y_min_mm:      Minimum Y coordinate (or min output).
            y_max_mm:      Maximum Y coordinate (or max output).
            adc_min_value: Lowest ADC reading used in mapping.
            adc_max_value: Highest ADC reading used in mapping.
        """
        self.adc_x = adc_x_channel
        self.adc_y = adc_y_channel

        self.x_min_mm = float(x_min_mm)
        self.x_max_mm = float(x_max_mm)
        self.y_min_mm = float(y_min_mm)
        self.y_max_mm = float(y_max_mm)

        self.adc_min_value = int(adc_min_value)
        self.adc_max_value = int(adc_max_value)

    @staticmethod
    def remap_interval(raw_value: int,
                       in_min: int, in_max: int,
                       out_min: float, out_max: float) -> float:
        """
        Convert a value from one numeric span to another using linear scaling.
        """
        if in_max == in_min:
            return out_min

        clamped = raw_value
        if clamped < in_min:
            clamped = in_min
        if clamped > in_max:
            clamped = in_max

        in_span = in_max - in_min
        out_span = out_max - out_min
        fraction = (clamped - in_min) / in_span
        return out_min + fraction * out_span

    def sample_x_mm(self) -> float:
        """
        Read the X input and return the mapped X position in mm (or mapped value).
        """
        adc_count_x = self.adc_x.read_u16()
        x_mm = self.remap_interval(
            adc_count_x,
            self.adc_min_value,
            self.adc_max_value,
            self.x_min_mm,
            self.x_max_mm,
        )
        return x_mm

    def sample_y_mm(self) -> float:
        """
        Read the Y input and return the mapped Y position in mm (or mapped value).
        """
        adc_count_y = self.adc_y.read_u16()
        y_mm = self.remap_interval(
            adc_count_y,
            self.adc_min_value,
            self.adc_max_value,
            self.y_min_mm,
            self.y_max_mm,
        )
        return y_mm


# ---- peninfo.py ----
from machine import PWM, Pin
import time

# --------- Adjustable pen configuration ---------

# Pin and frequency for the pen servo
PEN_PWM_PIN_NUMBER = 2
PEN_PWM_FREQUENCY_HZ = 50

# Timing settings for the pen servo signal
PEN_PULSE_MIN_MS = 1.0
PEN_PULSE_MAX_MS = 2.0
PEN_FRAME_MS = 20.0  # 50 Hz

# Angles for pen states (tweak these to tune the arm)
PEN_LIFT_ANGLE_DEG = 50.0    # pen off the page
PEN_TOUCH_ANGLE_DEG = 0.0    # pen on the page


# Create the PWM driver for the pen servo based on config above
pen_pwm_pin = Pin(PEN_PWM_PIN_NUMBER)
pen_pwm = PWM(pen_pwm_pin)
pen_pwm.freq(PEN_PWM_FREQUENCY_HZ)


def _pen_angle_to_duty(angle_deg: float) -> int:
    """
    Translate a pen angle (degrees) into a duty_u16 value using
    the configured timing and angle limits.
    """
    if angle_deg < 0.0:
        angle_deg = 0.0
    if angle_deg > 180.0:
        angle_deg = 180.0

    pulse_span_ms = PEN_PULSE_MAX_MS - PEN_PULSE_MIN_MS
    pulse_ms = PEN_PULSE_MIN_MS + (angle_deg / 180.0) * pulse_span_ms
    duty_ratio = pulse_ms / PEN_FRAME_MS
    duty_raw = int(duty_ratio * 65535.0)
    return duty_raw


def set_pen_contact(down: bool):
    """
    Move the pen to either drawing or lifted position.

    Args:
        down: True -> pen touches the surface, False -> pen is raised.
    """
    target_angle = PEN_TOUCH_ANGLE_DEG if down else PEN_LIFT_ANGLE_DEG
    duty_val = _pen_angle_to_duty(target_angle)
    pen_pwm.duty_u16(duty_val)


# ---- servo_motor.py ----
from machine import PWM
import math

# --------- Adjustable configuration values ---------

# Default servo timing (tweak these)
DEFAULT_PULSE_MIN_MS = 1.0     # ms at minimum angle
DEFAULT_PULSE_MAX_MS = 2.0     # ms at maximum angle
DEFAULT_FRAME_MS = 20.0        # ms per period (50 Hz)

# Default arm geometry (edit to match hardware)
DEFAULT_LINK1_LENGTH_MM = 147.0
DEFAULT_LINK2_LENGTH_MM = 147.0

# Default allowed angle range for the servos
DEFAULT_MIN_ANGLE_DEG = -20.0
DEFAULT_MAX_ANGLE_DEG = 250.0
# 180


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

    # ---------- NEW: forward kinematics ----------

    def forward_kinematics(self, shoulder_deg: float, elbow_deg: float):
        """
        Given shoulder and elbow angles in degrees, return tip (x, y) in mm
        in the SAME coordinate system used by compute_joint_angles.
        """
        q1 = math.radians(shoulder_deg)
        q2 = math.radians(elbow_deg)

        x = self.link1 * math.cos(q1) + self.link2 * math.cos(q1 + q2)
        y = self.link1 * math.sin(q1) + self.link2 * math.sin(q1 + q2)
        return x, y

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


# ----------------- Canvas → arm coordinate transform -----------------
# Tune these so that (0,0) in the file corresponds to the top-left of your page
# in the robot's coordinate system.

# Position of the TOP-LEFT corner of the canvas relative to the shoulder (in mm)
CANVAS_TOPLEFT_X_ARM_MM = -155.0   # mm to the right of the shoulder
CANVAS_TOPLEFT_Y_ARM_MM = 35.0   # mm above the shoulder (if +Y is "up" in your setup)

# Old guess:
# CANVAS_TOPLEFT_X_ARM_MM = -290.0
# CANVAS_TOPLEFT_Y_ARM_MM = -63.0

# Scaling from file units to millimetres
# If your coords in coords_test.txt are already in mm, leave these as 1.0.
CANVAS_SCALE_X = 1.0              # mm per unit of file X
CANVAS_SCALE_Y = 1.0              # mm per unit of file Y


def canvas_to_arm(u: float, v: float):
    """
    Convert canvas/file coordinates (u, v) to arm coordinates (x, y) in mm.

    File coords:
        u: 0 at LEFT,  increases to the RIGHT
        v: 0 at TOP,   increases DOWNWARDS

    Arm coords:
        x: 0 at SHOULDER, positive in the arm's +X direction
        y: 0 at SHOULDER, positive in the arm's +Y direction (usually "up")

    We:
        - Shift by (CANVAS_TOPLEFT_X_ARM_MM, CANVAS_TOPLEFT_Y_ARM_MM)
        - Flip Y because file v grows DOWN, but arm y grows UP
    """
    x_arm = CANVAS_TOPLEFT_X_ARM_MM - (u * CANVAS_SCALE_X)
    y_arm = CANVAS_TOPLEFT_Y_ARM_MM - (v * CANVAS_SCALE_Y)  # minus: v down → y up
    return x_arm, y_arm


# ----------------- Main that reads coordinates from a file -----------------

def main_from_file():
    """
    Read (u, v) coordinates from a text file and move the arm to each point
    using inverse kinematics, after converting them to arm coordinates.

    File format (coords_test.txt):
        # comment lines start with '#'
        u, v        # u = x from left, v = y from top (both ≥ 0)

        Example:
        0,0
        60,40
        90,60
    """
    from machine import Pin, PWM
    import time

    # <<< EDIT THESE TWO PINS FOR YOUR HARDWARE >>>
    SHOULDER_PIN_NUMBER = 0
    ELBOW_PIN_NUMBER = 1
    PWM_FREQUENCY_HZ = 50

    COORD_FILE = "coords_test.txt"

    # Set up PWM for the two arm servos
    shoulder_pwm = PWM(Pin(SHOULDER_PIN_NUMBER))
    elbow_pwm = PWM(Pin(ELBOW_PIN_NUMBER))
    shoulder_pwm.freq(PWM_FREQUENCY_HZ)
    elbow_pwm.freq(PWM_FREQUENCY_HZ)

    # Create arm driver (uses 147 mm links from your class above)
    arm = servo_motor(
        shoulder_output=shoulder_pwm,
        elbow_output=elbow_pwm,
        pwm_hz=PWM_FREQUENCY_HZ,
    )

    # Helper: move to one file point (u, v) after converting to arm coords
    def move_to_point(u: float, v: float):
        x_mm, y_mm = canvas_to_arm(u, v)
        shoulder_deg, elbow_deg = arm.compute_joint_angles(x_mm, y_mm)
        print(
            "  canvas ({:.1f}, {:.1f}) -> arm ({:.1f}, {:.1f}) "
            "-> shoulder = {:.1f} deg, elbow = {:.1f} deg".format(
                u, v, x_mm, y_mm, shoulder_deg, elbow_deg
            )
        )
        arm.apply_joint_angles(shoulder_deg, elbow_deg)

    # Read all coordinates from file
    coords = []
    try:
        with open(COORD_FILE, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue  # skip blank / comment lines

                parts = line.split(",")
                if len(parts) < 2:
                    continue

                u = float(parts[0].strip())
                v = float(parts[1].strip())
                coords.append((u, v))
    except OSError as e:
        print("Could not open coordinate file:", COORD_FILE, "error:", e)
        return

    if not coords:
        print("No coordinates found in", COORD_FILE)
        return

    print("Loaded", len(coords), "points from", COORD_FILE)

    # 1) Go to the first point with pen UP
    first_u, first_v = coords[0]
    print("Moving to first point with pen UP (canvas):", first_u, first_v)
    set_pen_contact(False)
    move_to_point(first_u, first_v)
    time.sleep(1.5)

    if len(coords) == 1:
        print("Only one point in file; stopping here.")
        return

    # 2) Lower the pen and follow the rest of the path
    print("Pen DOWN, drawing path...")
    set_pen_contact(True)
    time.sleep(0.5)

    for (u, v) in coords[1:]:
        print("Moving to (canvas):", u, v)
        move_to_point(u, v)
        time.sleep(0.7)

    # 3) Lift the pen at the end
    print("Path done. Lifting pen.")
    set_pen_contact(False)


# ----------------- NEW: manual calibration with pots A0/A1 -----------------

def manual_calibrate_with_pots():
    """
    Use two potentiometers on A0 (ADC0, pin 26) and A1 (ADC1, pin 27)
    to directly control the shoulder and elbow servos.

    The pots are mapped to the servo angle range [DEFAULT_MIN_ANGLE_DEG, DEFAULT_MAX_ANGLE_DEG].

    While you move the pots, this prints:
        Shoulder angle, Elbow angle, and tip (x, y) in arm coordinates.

    When the pen is exactly at the point you want to be canvas (0,0),
    copy that tip (x, y) and paste them into:

        CANVAS_TOPLEFT_X_ARM_MM = tip_x
        CANVAS_TOPLEFT_Y_ARM_MM = tip_y
    """
    from machine import Pin, PWM, ADC
    import time

    # <<< SAME SERVO PINS AS main_from_file >>> 
    SHOULDER_PIN_NUMBER = 0
    ELBOW_PIN_NUMBER = 1
    PWM_FREQUENCY_HZ = 50

    # Set up PWM for the two arm servos
    shoulder_pwm = PWM(Pin(SHOULDER_PIN_NUMBER))
    elbow_pwm = PWM(Pin(ELBOW_PIN_NUMBER))
    shoulder_pwm.freq(PWM_FREQUENCY_HZ)
    elbow_pwm.freq(PWM_FREQUENCY_HZ)

    # Create arm driver
    arm = servo_motor(
        shoulder_output=shoulder_pwm,
        elbow_output=elbow_pwm,
        pwm_hz=PWM_FREQUENCY_HZ,
    )

    # Pots on A0/A1 (GP26/GP27)
    pot_shoulder = ADC(0)  # A0 / GP26 / pin 31
    pot_elbow    = ADC(1)  # A1 / GP27 / pin 32

    # Map ADC readings to angles in the same range the servos use
    pot_reader = AnalogPositionReader(
        pot_shoulder,
        pot_elbow,
        x_min_mm=DEFAULT_MIN_ANGLE_DEG,   # used as min angle
        x_max_mm=DEFAULT_MAX_ANGLE_DEG,   # used as max angle
        y_min_mm=DEFAULT_MIN_ANGLE_DEG,
        y_max_mm=DEFAULT_MAX_ANGLE_DEG,
    )

    print("Manual calibration mode with pots on A0/A1.")
    print("Turn the pots to move the arm.")
    print("When the pen is at the point you want as canvas (0,0),")
    print("note the tip coordinates below and paste them into CANVAS_TOPLEFT_X/Y_ARM_MM.")
    time.sleep(1.0)

    while True:
        # 1) Pot -> angles (degrees)
        shoulder_angle = pot_reader.sample_x_mm()
        elbow_angle    = pot_reader.sample_y_mm()

        # 2) Move servos
        arm.apply_joint_angles(shoulder_angle, elbow_angle)

        # 3) Compute current tip position
        tip_x, tip_y = arm.forward_kinematics(shoulder_angle, elbow_angle)

        # 4) Print info
        print(
            "Shoulder = {:.1f}°, Elbow = {:.1f}°  ->  tip = ({:.1f} mm, {:.1f} mm)".format(
                shoulder_angle, elbow_angle, tip_x, tip_y
            )
        )

        time.sleep(0.1)


# ----------------- Entry point -----------------

if __name__ == "__main__":
    # NORMAL DRAWING FROM FILE:
    main_from_file()
    #manual_calibrate_with_pots()

    # FOR MANUAL CALIBRATION WITH POTS:
    # Comment out the line above and uncomment the line below:
    # manual_calibrate_with_pots()
