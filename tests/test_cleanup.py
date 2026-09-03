from pathlib import Path
import unittest


class CleanupTest(unittest.TestCase):
    def test_leg_body_angle_lock_code_is_removed(self):
        source = (Path(__file__).parents[1] / "Simulation.py").read_text(encoding="utf-8")
        for symbol in (
            "LOCK_LEG_BODY_ANGLE_ZERO",
            "LEG_BODY_ANGLE_KP",
            "LEG_BODY_ANGLE_KD",
        ):
            self.assertNotIn(symbol, source)


if __name__ == "__main__":
    unittest.main()
