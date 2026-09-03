from pathlib import Path
import unittest


class CleanupTest(unittest.TestCase):
    SOURCE_FILES = (
        "Simulation.py",
        "Controller.py",
        "environment.py",
        "VMC.py",
        "lqr_controller.py",
        "keyboard.py",
        "caculation.py",
    )

    def source_text(self):
        project_root = Path(__file__).parents[1]
        return "\n".join(
            (project_root / filename).read_text(encoding="utf-8")
            for filename in self.SOURCE_FILES
        )

    def test_leg_body_angle_lock_code_is_removed(self):
        source = (Path(__file__).parents[1] / "Simulation.py").read_text(encoding="utf-8")
        for symbol in (
            "LOCK_LEG_BODY_ANGLE_ZERO",
            "LEG_BODY_ANGLE_KP",
            "LEG_BODY_ANGLE_KD",
        ):
            self.assertNotIn(symbol, source)

    def test_legacy_english_comments_are_removed(self):
        source = self.source_text()
        for fragment in (
            "model timestep",
            "Virtual-leg force controller",
            "Roll PID output",
            "Anti-split PID",
            "Keep the original",
            "The wheel (Wt)",
            "The left XML joints",
            "Positive MATLAB",
            "Direct length PID",
            "Run the MuJoCo",
            "The imported MuJoCo",
            "Positive wheel_vel",
        ):
            with self.subTest(fragment=fragment):
                self.assertNotIn(fragment, source)

    def test_commented_out_code_is_removed(self):
        self.assertNotIn("#robot.wheel_torque", self.source_text())

    def test_wildcard_calculation_import_is_removed(self):
        self.assertNotIn("from caculation import *", self.source_text())


if __name__ == "__main__":
    unittest.main()
