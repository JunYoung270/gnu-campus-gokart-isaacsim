import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros2_speed_limit import clamp_speed, parse_speed_limit


class Ros2SpeedLimitTest(unittest.TestCase):
    def test_sixty_kilometres_per_hour(self):
        limit = parse_speed_limit(60.0 / 3.6)
        self.assertAlmostEqual(clamp_speed(30.0, limit), 60.0 / 3.6)
        self.assertAlmostEqual(clamp_speed(-30.0, limit), -(60.0 / 3.6))

    def test_commands_inside_limit_are_unchanged(self):
        self.assertEqual(clamp_speed(0.5, 60.0 / 3.6), 0.5)

    def test_missing_limit_keeps_backward_compatibility(self):
        self.assertIsNone(parse_speed_limit(None))
        self.assertEqual(clamp_speed(25.0, None), 25.0)

    def test_invalid_values_are_rejected(self):
        for value in (0.0, -1.0, float("inf"), float("nan")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    parse_speed_limit(value)
        with self.assertRaises(ValueError):
            clamp_speed(float("nan"), 10.0)


if __name__ == "__main__":
    unittest.main()
