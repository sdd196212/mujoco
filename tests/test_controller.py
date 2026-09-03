from types import SimpleNamespace
import unittest

import Controller


class ControllerTest(unittest.TestCase):
    def test_roll_pid_clamps_output_and_reset_clears_integral(self):
        RollPID = getattr(Controller, "RollPID", None)
        self.assertIsNotNone(RollPID)
        pid = RollPID(kp=10.0, ki=2.0, kd=0.0, output_limit=3.0, integral_limit=1.0)
        self.assertEqual(pid.update(1.0, 0.0, 0.1), 3.0)
        self.assertNotEqual(pid.integral, 0.0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)

    def test_leg_force_pid_resets_when_disabled(self):
        leg_force_f0 = getattr(Controller, "leg_force_f0", None)
        self.assertIsNotNone(leg_force_f0)
        vmc = SimpleNamespace(L0=0.2, d_L0=0.0)
        force = leg_force_f0(vmc, 0.01, enabled=True)
        self.assertGreater(force, 0.0)
        self.assertEqual(leg_force_f0(vmc, 0.01, enabled=False), 0.0)
        self.assertEqual(vmc.length_integral, 0.0)

    def test_split_pid_returns_zero_and_resets_when_disabled(self):
        split_torque_pid = getattr(Controller, "split_torque_pid", None)
        self.assertIsNotNone(split_torque_pid)
        robot = SimpleNamespace()
        right = SimpleNamespace(theta=0.1, d_theta=0.0)
        left = SimpleNamespace(theta=0.0, d_theta=0.0)
        torque, error = split_torque_pid(robot, right, left, 0.01, enabled=True)
        self.assertGreater(torque, 0.0)
        self.assertAlmostEqual(error, 0.1)
        self.assertEqual(split_torque_pid(robot, right, left, 0.01, enabled=False), (0.0, 0.0))
        self.assertEqual(robot.split_integral, 0.0)


if __name__ == "__main__":
    unittest.main()
