from machine import PWM, Pin
import time

# --------- Adjustable pen configuration ---------

# Pin and frequency for the pen servo
PEN_PWM_PIN_NUMBER = 22
PEN_PWM_FREQUENCY_HZ = 50

# Timing settings for the pen servo signal
PEN_PULSE_MIN_MS = 1.0
PEN_PULSE_MAX_MS = 2.0
PEN_FRAME_MS = 20.0  # 50 Hz

# Angles for pen states (tweak these to tune the arm)
PEN_LIFT_ANGLE_DEG = 0.0    # pen off the page
PEN_TOUCH_ANGLE_DEG = 90.0  # pen on the page


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


if __name__ == "__main__":
    print("Program start")
    
    while True:
        # Pen down
        print("DOWN")
        set_pen_contact(True)
        time.sleep(2)
        
        # Pen up
        print("UP")
        set_pen_contact(False)
        time.sleep(2)
