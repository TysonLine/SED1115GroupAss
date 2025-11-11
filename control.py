import time
import config
from io_hw import init_hardware, read_x_adc, read_y_adc, read_pen_switch
from mapping_io import adc_to_01, map_norm_to_xy
from geometry import ik_two_link, limit_angles
from servo_out import angle_to_us, set_servos

def one_step():
    """One control tick: read -> map -> IK -> limit -> convert -> output."""
    rx = read_x_adc()
    ry = read_y_adc()
    pen_down = read_pen_switch()
    nx = adc_to_01(rx)
    ny = adc_to_01(ry)
    x, y = map_norm_to_xy(nx, ny)
    shoulder_deg, elbow_deg = ik_two_link(x, y, config.L1_MM, config.L2_MM)
    shoulder_deg, elbow_deg = limit_angles(shoulder_deg, elbow_deg)
    s_us = angle_to_us(shoulder_deg, 0.0, 180.0, config.SHOULDER_MIN_US, config.SHOULDER_MAX_US)
    e_us = angle_to_us(elbow_deg, 0.0, 180.0, config.ELBOW_MIN_US, config.ELBOW_MAX_US)
    p_us = config.PEN_MAX_US if pen_down else config.PEN_MIN_US
    set_servos(s_us, e_us, p_us)

def main():
    """Tiny loop calling one_step() at CONTROL_HZ."""
    init_hardware()
    dt = 1.0 / config.CONTROL_HZ
    while True:
        one_step()
        time.sleep(dt)

if __name__ == "__main__":
    main()
