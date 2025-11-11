import config

def adc_to_01(raw):
    """Map raw (0..65535) to 0.0..1.0"""
    if raw < 0:
        raw = 0
    if raw > 65535:
        raw = 65535
    return raw / 65535.0

def map_norm_to_xy(nx, ny):
    """Map normalized 0..1 to (x,y) in mm using config workspace."""
    x = config.X_MIN + nx * (config.X_MAX - config.X_MIN)
    y = config.Y_MIN + ny * (config.Y_MAX - config.Y_MIN)
    return x, y
