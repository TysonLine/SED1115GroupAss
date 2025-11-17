import unittest
from geometry import ik_two_link

class TestIK(unittest.TestCase):
    def test_reachable(self):
        s,e = ik_two_link(50.0, 100.0, 90.0, 90.0)
        self.assertTrue(0.0 <= s <= 180.0)
        self.assertTrue(0.0 <= e <= 180.0)

if __name__ == "__main__":
    unittest.main()
