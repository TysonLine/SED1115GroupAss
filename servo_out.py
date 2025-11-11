import config
from geometry import clamp

def angle_to_us(angle_deg, min_deg, max_deg, min_us, max_us):
    """Linear map angle (deg) to microseconds (µs)."""
    a = clamp(angle_deg, min_deg, max_deg)
    if max_deg == min_deg:
        t = 0.0
    else:
        t = (a - min_deg) / (max_deg - min_deg)
    us = int(round(min_us + t * (max_us - min_us)))
    return us

def set_servos(shoulder_us, elbow_us, pen_us):
    """Write pulses to servos (stub)."""
    print("SERVOS:", shoulder_us, elbow_us, pen_us)
