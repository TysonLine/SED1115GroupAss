from machine import ADC

# --------- Adjustable mapping values ---------

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
    physical coordinates in millimetres.
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
            x_min_mm:      Minimum X coordinate.
            x_max_mm:      Maximum X coordinate.
            y_min_mm:      Minimum Y coordinate.
            y_max_mm:      Maximum Y coordinate.
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
        Read the X input and return the mapped X position in mm.
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
        Read the Y input and return the mapped Y position in mm.
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
