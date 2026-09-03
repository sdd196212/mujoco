from types import SimpleNamespace
import unittest

import Controller


class RecordingVMC(SimpleNamespace):
    """记录虚拟腿位置更新参数的测试替身。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.position_calls = []

    def vmc_calc_pos(self, **kwargs):
        self.position_calls.append(kwargs)


class RecordingLQR:
    """返回固定输出并记录输入状态的测试替身。"""

    def __init__(self):
        self.calls = []

    def control(self, *args):
        self.calls.append(args)
        return [args[0] + 10.0, args[1] + 20.0]


class ControllerTest(unittest.TestCase):
    def test_update_vmc_positions_preserves_joint_and_imu_signs(self):
        update_vmc_positions = getattr(Controller, "update_vmc_positions", None)
        self.assertIsNotNone(update_vmc_positions)
        robot = SimpleNamespace(
            euler=[0.11, 0.22, 0.33],
            gyro=[0.44, 0.55, 0.66],
            joint_pos=[1.0, 2.0, 3.0, 4.0],
        )
        vmc_r = RecordingVMC()
        vmc_l = RecordingVMC()

        update_vmc_positions(robot, vmc_r, vmc_l, 0.004)

        self.assertEqual(vmc_r.position_calls, [{
            "dt": 0.004,
            "phi1": 1.0 + 3.141592653589793,
            "phi4": 2.0,
            "pitch": 0.22,
            "gyro": 0.55,
        }])
        self.assertEqual(vmc_l.position_calls, [{
            "dt": 0.004,
            "phi1": 4.0 + 3.141592653589793,
            "phi4": 3.0,
            "pitch": -0.22,
            "gyro": -0.55,
        }])

    def test_compute_lqr_outputs_preserves_row_specific_sign_mapping(self):
        compute_lqr_outputs = getattr(Controller, "compute_lqr_outputs", None)
        self.assertIsNotNone(compute_lqr_outputs)
        robot = SimpleNamespace(euler=[0.1, 0.2, 0.3], gyro=[0.4, 0.5, 0.6], x=1.5, d_x=0.7)
        vmc_r = SimpleNamespace(theta=0.11, d_theta=0.12, L0=0.28)
        vmc_l = SimpleNamespace(theta=-0.21, d_theta=-0.22, L0=0.29)
        lqr = RecordingLQR()

        outputs = compute_lqr_outputs(robot, vmc_r, vmc_l, lqr, enabled=True)

        self.assertEqual(outputs, ((9.89, 20.12), (9.79, 20.22), 0.11, -0.21))
        self.assertEqual(lqr.calls, [
            (-0.11, -0.12, 1.5, 1.4, 0.2, 0.5, 0.28),
            (0.11, 0.12, -1.5, -0.7, -0.2, -0.5, 0.28),
            (-0.21, -0.22, 1.5, 1.4, 0.2, 0.5, 0.29),
            (0.21, 0.22, -1.5, -0.7, -0.2, -0.5, 0.29),
        ])

    def test_compute_lqr_outputs_disabled_returns_zero_outputs_and_theta(self):
        compute_lqr_outputs = getattr(Controller, "compute_lqr_outputs", None)
        self.assertIsNotNone(compute_lqr_outputs)
        robot = SimpleNamespace(euler=[0.0, 0.0, 0.0], gyro=[0.0, 0.0, 0.0], x=0.0, d_x=0.0)
        vmc_r = SimpleNamespace(theta=0.11, d_theta=0.12, L0=0.28)
        vmc_l = SimpleNamespace(theta=-0.21, d_theta=-0.22, L0=0.29)
        lqr = RecordingLQR()

        outputs = compute_lqr_outputs(robot, vmc_r, vmc_l, lqr, enabled=False)

        self.assertEqual(outputs, ((0.0, 0.0), (0.0, 0.0), 0.0, 0.0))
        self.assertEqual(lqr.calls, [])

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
