import math
import config

def clamp(v, lo, hi):
    """Keep v within [lo, hi]."""
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def ik_two_link(x, y, L1, L2):
    """Simple 2-link IK returning (shoulder_deg, elbow_deg)."""
    r2 = x*x + y*y
    c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)
    c2 = clamp(c2, -1.0, 1.0)
    s2 = math.sqrt(1.0 - c2*c2)
    theta2 = math.atan2(s2, c2)
    phi = math.atan2(y, x)
    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = phi - math.atan2(k2, k1)
    shoulder_deg = math.degrees(theta1)
    elbow_deg = math.degrees(theta2)
    return shoulder_deg, elbow_deg

def limit_angles(shoulder_deg, elbow_deg):
    """Enforce joint limits from config."""
    s = clamp(shoulder_deg, config.SHOULDER_MIN, config.SHOULDER_MAX)
    e = clamp(elbow_deg, config.ELBOW_MIN, config.ELBOW_MAX)
    return s, e
