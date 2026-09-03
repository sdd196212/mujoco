"""运行基于 MATLAB LQR 控制链的 MuJoCo 轮腿仿真。"""

from pathlib import Path

from environment import LegWheelRobot
from VMC import leg_VMC
from lqr_controller import LQRController
from Controller import (
    F0_MAX,
    RollPID,
    compute_lqr_outputs,
    leg_force_f0,
    map_vmc_torques,
    split_torque_pid,
    update_vmc_positions,
)



CONTROL_DIVIDER = 4  # 模型步长为 1 ms，控制更新周期为 4 ms


def apply_lqr(robot, vmc_r, vmc_l, lqr, enabled=True, roll_pid=None):
    """计算标准力矩并映射到 XML 中的六个执行器。"""
    pitch = float(robot.euler[1])
    control_dt = CONTROL_DIVIDER * robot.sensor_T

    update_vmc_positions(robot, vmc_r, vmc_l, control_dt)
    (u_r, u_l, theta_r_lqr, theta_l_lqr) = compute_lqr_outputs(
        robot, vmc_r, vmc_l, lqr, enabled=enabled
    )

    T_r, Tp_r = map(float, u_r)
    T_l, Tp_l = map(float, u_l)
    split_tp, split_error = split_torque_pid(
        robot, vmc_r, vmc_l, control_dt, enabled=enabled
    )

    # Keep one PID instance across control cycles when the caller does not
    # explicitly provide one.  This preserves the integral state in scripts
    # that use apply_lqr() directly.
    if roll_pid is None:
        roll_pid = getattr(robot, "_roll_pid", None)
        if roll_pid is None:
            roll_pid = RollPID()
            robot._roll_pid = roll_pid
    if enabled:
        roll_f0 = roll_pid.update(
            roll=float(robot.euler[0]),
            roll_rate=float(robot.gyro[0]),
            dt=control_dt,
        )
    else:
        roll_pid.reset()
        roll_f0 = 0.0

    # Direct length PID, followed by the existing roll differential.
    base_f0_r = leg_force_f0(vmc_r, control_dt, enabled=enabled)
    base_f0_l = leg_force_f0(vmc_l, control_dt, enabled=enabled)
    vmc_r.F0 = max(-F0_MAX, min(F0_MAX, base_f0_r - roll_f0))
    vmc_l.F0 =-max(-F0_MAX, min(F0_MAX, base_f0_l + roll_f0))
    # The left VMC plane is mirrored before actuator mapping.
 
    vmc_r.Tp = Tp_r - split_tp
    vmc_l.Tp = Tp_l - split_tp
    
    
    map_vmc_torques(robot, vmc_r, vmc_l)

    robot.wheel_torque = [T_r, T_l]
    robot.actuator_set_torque()
    print(
        f"[LQR] F0(R/L)=({vmc_r.F0:+.6f}, {vmc_l.F0:+.6f}) N, "
        f"L0(R/L)=({vmc_r.L0:+.6f}, {vmc_l.L0:+.6f}) m, "
        f"roll dF0={roll_f0:+.6f} N, "
        f"pitch={pitch:+.6f} rad, "
        f"theta(R/L)=({vmc_r.theta:+.6f}, {vmc_l.theta:+.6f}) rad, "
        f"alpha(R/L)=({vmc_r.alpha:+.6f}, {vmc_l.alpha:+.6f}) rad, "
        f"theta_lqr(R/L)=({theta_r_lqr:+.6f}, {theta_l_lqr:+.6f}) rad, "
        f"split={split_error:+.6f} rad, split Tp={split_tp:+.6f} Nm, "
        f"Tp(R/L)=({vmc_r.Tp:+.6f}, {vmc_l.Tp:+.6f}) Nm, "
        f"wheel T(R/L)=({T_r:+.6f}, {T_l:+.6f}) Nm, "
        f"actuator ctrl(R/L)=({robot.data.ctrl[4]:+.6f}, {robot.data.ctrl[5]:+.6f}) Nm"
    )
    return (T_r, Tp_r), (T_l, Tp_l)


def main():
    model_path = Path(__file__).with_name("MJCF") / "env.xml"
    robot = LegWheelRobot(str(model_path), visualize=True)
    vmc_r = leg_VMC()
    vmc_l = leg_VMC()
    lqr = LQRController()
#
    robot.sensor_read_data()
    step_count = 0
    try:
        while robot.viewer is None or robot.viewer.is_running():
            robot.sensor_read_data()
            if step_count % CONTROL_DIVIDER == 0:
                apply_lqr(robot, vmc_r, vmc_l, lqr, enabled=True)
            robot.step()
            step_count += 1
    finally:
        if robot.viewer is not None:
            robot.viewer.close()


if __name__ == "__main__":
    main()
