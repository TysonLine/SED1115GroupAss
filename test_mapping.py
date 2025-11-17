import unittest, config
from mapping_io import map_norm_to_xy

class TestMapping(unittest.TestCase):
    def test_center_maps_to_center(self):
        x, y = map_norm_to_xy(0.5, 0.5)
        self.assertAlmostEqual(x, (config.X_MIN+config.X_MAX)/2, places=3)
        self.assertAlmostEqual(y, (config.Y_MIN+config.Y_MAX)/2, places=3)

if __name__ == "__main__":
    unittest.main()
