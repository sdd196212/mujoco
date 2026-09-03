from types import SimpleNamespace
import sys
import unittest
from types import ModuleType
from unittest.mock import patch


def _module_stub(name, class_name):
    """构造仅满足 Simulation 导入所需的最小模块占位。"""
    module = ModuleType(name)
    setattr(module, class_name, type(class_name, (), {}))
    return module


# Simulation 的编排逻辑不需要实例化 MuJoCo 依赖；导入时临时隔离这些模块。
_import_stubs = {
    "environment": _module_stub("environment", "LegWheelRobot"),
    "VMC": _module_stub("VMC", "leg_VMC"),
    "lqr_controller": _module_stub("lqr_controller", "LQRController"),
}
with patch.dict(sys.modules, _import_stubs):
    import Simulation


class FakeVMC:
    """用于验证控制编排的轻量虚拟腿替身。"""

    def __init__(self, theta, d_theta, length, torque_set):
        self.theta = theta
        self.d_theta = d_theta
        self.alpha = 0.0
        self.L0 = length
        self.d_L0 = 0.0
        self.torque_set = list(torque_set)
        self.position_calls = 0
        self.torque_calls = 0

    def vmc_calc_pos(self, **kwargs):
        self.position_calls += 1

    def vmc_calc_torque(self):
        self.torque_calls += 1


class RecordingLQR:
    """记录 LQR 状态并返回固定的 [轮端力矩, 虚拟腿力矩]。"""

    def __init__(self):
        self.calls = []

    def control(self, *args):
        self.calls.append(args)
        return [100.0 + args[0], 200.0 + args[1]]


class FakeRobot:
    """提供 apply_lqr 所需最小接口，不启动 MuJoCo viewer。"""

    def __init__(self):
        self.euler = [0.03, 0.2, 0.0]
        self.gyro = [0.04, 0.5, 0.0]
        self.joint_pos = [1.0, 2.0, 3.0, 4.0]
        self.sensor_T = 0.001
        self.x = 1.5
        self.d_x = 0.7
        self.joint_torque = []
        self.wheel_torque = []
        self.data = SimpleNamespace(ctrl=[0.0] * 6)
        self.actuator_calls = 0

    def actuator_set_torque(self):
        self.actuator_calls += 1
        self.data.ctrl[0:4] = self.joint_torque
        self.data.ctrl[4:6] = self.wheel_torque


class SimulationTest(unittest.TestCase):
    def test_apply_lqr_force_theta_zero_reaches_all_lqr_calls(self):
        robot = FakeRobot()
        vmc_r = FakeVMC(theta=0.11, d_theta=0.12, length=0.28, torque_set=[1.0, 2.0])
        vmc_l = FakeVMC(theta=-0.21, d_theta=-0.22, length=0.29, torque_set=[3.0, 4.0])
        lqr = RecordingLQR()
        previous = getattr(Simulation, "FORCE_LQR_THETA_ZERO", False)
        Simulation.FORCE_LQR_THETA_ZERO = True
        try:
            result = Simulation.apply_lqr(robot, vmc_r, vmc_l, lqr, enabled=True)
        finally:
            Simulation.FORCE_LQR_THETA_ZERO = previous

        self.assertEqual(len(lqr.calls), 4)
        self.assertTrue(all(call[0] == 0.0 and call[1] == 0.0 for call in lqr.calls))
        self.assertEqual(robot.actuator_calls, 1)
        self.assertEqual(robot.wheel_torque, [100.0, 100.0])
        self.assertEqual(result, ((100.0, 200.0), (100.0, 200.0)))
        self.assertEqual(robot.joint_torque, [2.0, 1.0, -3.0, -4.0])


if __name__ == "__main__":
    unittest.main()
