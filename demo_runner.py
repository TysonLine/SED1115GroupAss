"""Demo runner for Deliverable C (desktop simulation).
Generates fake potentiometer inputs and toggles the pen.
This lets you show the pipeline works without real Pico hardware.
Run: python demo_runner.py
"""
import time, math
import config
from mapping_io import map_norm_to_xy
from geometry import ik_two_link, limit_angles
from servo_out import angle_to_us, set_servos

def fake_inputs(t):
    # Normalized 0..1 sweep
    nx = 0.5 + 0.45*math.sin(2*math.pi*0.2*t)   # slow X sweep
    ny = 0.5 + 0.45*math.cos(2*math.pi*0.15*t)  # slow Y sweep
    pen_down = int(t) % 2 == 0  # toggle every second
    return nx, ny, pen_down

def step(t):
    nx, ny, pen_down = fake_inputs(t)
    x, y = map_norm_to_xy(nx, ny)
    s_deg, e_deg = ik_two_link(x, y, config.L1_MM, config.L2_MM)
    s_deg, e_deg = limit_angles(s_deg, e_deg)
    s_us = angle_to_us(s_deg, 0.0, 180.0, config.SHOULDER_MIN_US, config.SHOULDER_MAX_US)
    e_us = angle_to_us(e_deg, 0.0, 180.0, config.ELBOW_MIN_US,    config.ELBOW_MAX_US)
    p_us = config.PEN_MAX_US if pen_down else config.PEN_MIN_US
    set_servos(s_us, e_us, p_us)

def main():
    dt = 1.0 / config.CONTROL_HZ
    t0 = time.time()
    for _ in range(200):   # run ~4 seconds at 50Hz
        now = time.time() - t0
        step(now)
        time.sleep(dt)

if __name__ == "__main__":
    main()
