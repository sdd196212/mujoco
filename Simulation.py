"""Run the MuJoCo wheel-leg simulation with the MATLAB LQR chain."""

import math
from pathlib import Path

from environment import LegWheelRobot
from VMC import leg_VMC
from lqr_controller import LQRController



CONTROL_DIVIDER = 4  # model timestep is 1 ms; control update is 4 ms


def apply_lqr(robot, vmc_r, vmc_l, lqr, enabled=True):
    """Compute canonical torques and map them to the six XML actuators."""
    pitch = float(robot.euler[1])
    gyro = float(robot.gyro[1])

    # Keep the original MuJoCo VMC input convention.
    vmc_r.vmc_calc_pos(
        dt=CONTROL_DIVIDER * robot.sensor_T,
        phi1=float(robot.joint_pos[0] + math.pi),
        phi4=float(robot.joint_pos[1]),
        pitch=pitch,
        gyro=gyro,
    )
    vmc_l.vmc_calc_pos(
        dt=CONTROL_DIVIDER * robot.sensor_T,
        # The left XML joints use the opposite axis and are mirrored in the
        # physical model.  Convert them to the same canonical plane as the
        # right leg before running the unchanged VMC geometry.
        phi1=float(-robot.joint_pos[3] + math.pi),
        phi4=float(-robot.joint_pos[2]),
        pitch=pitch,
        gyro=gyro,
    )
    if enabled:
        # The MATLAB plant uses the opposite pitch orientation from MuJoCo.
        # Its control subsystem applies the exported gain as K @ state.
        u_r = lqr.control(vmc_r.theta, vmc_r.d_theta, robot.x, robot.d_x,
                          -pitch, -gyro, vmc_r.L0)
        u_l = lqr.control(vmc_l.theta, vmc_l.d_theta, robot.x, robot.d_x,
                          -pitch, -gyro, vmc_l.L0)
    else:
        u_r = u_l = (0.0, 0.0)

    T_r, Tp_r = map(float, u_r)
    T_l, Tp_l = map(float, u_l)

    # Map the virtual leg torque through the current mechanism Jacobian.
    vmc_r.F0, vmc_r.Tp = 0.0, Tp_r
    vmc_l.F0, vmc_l.Tp = 0.0, Tp_l
    vmc_r.vmc_calc_torque()
    vmc_l.vmc_calc_torque()

    # VMC torque_set is [phi4, phi1], matching the existing actuator order.
    robot.joint_torque = [
        vmc_r.torque_set[1],   # right jAB / phi1
        vmc_r.torque_set[0],   # right jAG / phi4
        -vmc_l.torque_set[0],   # left jIJ / -phi4
        -vmc_l.torque_set[1],   # left jIO / -phi1
    ]
    # Positive MATLAB T drives +x. Both wheel axes require a negative control.
    robot.wheel_torque = [-T_r, -T_l]
    robot.actuator_set_torque()
    print(
        f"[LQR] F0(R/L)=({vmc_r.F0:+.6f}, {vmc_l.F0:+.6f}) N, "
        f"Tp(R/L)=({Tp_r:+.6f}, {Tp_l:+.6f}) Nm, "
        f"wheel T(R/L)=({T_r:+.6f}, {T_l:+.6f}) Nm, "
        f"actuator(R/L)=({robot.wheel_torque[0]:+.6f}, {robot.wheel_torque[1]:+.6f}) Nm"
    )
    return (T_r, Tp_r), (T_l, Tp_l)


def main():
    model_path = Path(__file__).with_name("MJCF") / "env.xml"
    robot = LegWheelRobot(str(model_path), visualize=True)
    vmc_r = leg_VMC()
    vmc_l = leg_VMC()
    lqr = LQRController()

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
