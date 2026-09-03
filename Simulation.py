"""Run the MuJoCo wheel-leg simulation with the MATLAB LQR chain."""

import math
from pathlib import Path

from environment import LegWheelRobot
from VMC import leg_VMC
from lqr_controller import LQRController
from Controller import F0_MAX, RollPID, leg_force_f0, split_torque_pid



CONTROL_DIVIDER = 4  # model timestep is 1 ms; control update is 4 ms

# Step-by-step diagnosis mode: hold the leg-to-body angle alpha at zero with
# a local PD loop instead of the LQR Tp output. Theta itself is not modified.
FORCE_LQR_THETA_ZERO = False
THETA_INSTALL_OFFSET = math.pi / 4.0
LOCK_LEG_BODY_ANGLE_ZERO = False
#LOCK_LEG_BODY_ANGLE_ZERO = True
LEG_BODY_ANGLE_KP = 20.0   # Nm/rad

LEG_BODY_ANGLE_KD = 2.0    # Nm/(rad/s)


def apply_lqr(robot, vmc_r, vmc_l, lqr, enabled=True, roll_pid=None):
    """Compute canonical torques and map them to the six XML actuators."""
    pitch = float(robot.euler[1])
    gyro = float(robot.gyro[1])
    control_dt = CONTROL_DIVIDER * robot.sensor_T

    # Keep the original MuJoCo VMC input convention.
    vmc_r.vmc_calc_pos(
        dt=control_dt,
        phi1=float(robot.joint_pos[0] + math.pi),
        phi4=float(robot.joint_pos[1]),
        pitch=pitch,
        gyro=gyro,
    )
    vmc_l.vmc_calc_pos(
        dt=control_dt,
        phi1=float(robot.joint_pos[3] + math.pi),
        phi4=float(robot.joint_pos[2]),
        pitch=-pitch,
        gyro=-gyro,
    )
    # Keep printable LQR inputs defined even when the controller is disabled.
    theta_r_lqr = 0.0
    dtheta_r_lqr = 0.0
    theta_l_lqr = 0.0
    dtheta_l_lqr = 0.0
    if enabled:
        # Use +pitch here so the wheel-torque pitch feedback is opposite to
        # the previous convention. Its control subsystem applies K @ state.
        theta_r_lqr = (
            0.0 if FORCE_LQR_THETA_ZERO
            else vmc_r.theta #- THETA_INSTALL_OFFSET/3
        )
        dtheta_r_lqr = 0.0 if FORCE_LQR_THETA_ZERO else vmc_r.d_theta
        theta_l_lqr = (
            0.0 if FORCE_LQR_THETA_ZERO
            else vmc_l.theta #+ THETA_INSTALL_OFFSET/3
        )
        dtheta_l_lqr = 0.0 if FORCE_LQR_THETA_ZERO else vmc_l.d_theta
        # The wheel (Wt) and virtual-leg (Tp) rows use different measured
        # sign conventions in this MuJoCo model. Compute each row with its
        # own state instead of correcting the combined output afterward.
        u_r_wheel = lqr.control(-theta_r_lqr, -vmc_r.d_theta, robot.x, robot.d_x*2,
                                pitch, gyro, vmc_r.L0)
        u_r_tp = lqr.control(vmc_r.theta, vmc_r.d_theta, -robot.x, -robot.d_x,
                            -pitch, -gyro, vmc_r.L0)
        u_r = (u_r_wheel[0], u_r_tp[1])
        # The left XML joints use the opposite rotational axes.  Keep the
        # left VMC geometry in its physical coordinates, but mirror theta
        # into the same LQR coordinate as the right leg.  Without this,
        # a symmetric pose appears as theta_R=-theta_L and produces
        # opposite wheel torques.
        u_l_wheel = lqr.control(theta_l_lqr, vmc_l.d_theta,robot.x, robot.d_x*2,
                                pitch, gyro, vmc_l.L0)
        u_l_tp = lqr.control(-vmc_l.theta, -vmc_l.d_theta, -robot.x, -robot.d_x,
                            -pitch, -gyro, vmc_l.L0)
        u_l = (u_l_wheel[0], u_l_tp[1])
    else:
        u_r = u_l = (0.0, 0.0)

    T_r, Tp_r = map(float, u_r)
    T_l, Tp_l = map(float, u_l)
    if LOCK_LEG_BODY_ANGLE_ZERO and enabled:
        # Alpha is the leg angle in the body frame. Mirror the left-leg
        # coordinate before applying the same restoring law to both legs.
        alpha_r_ctrl = float(vmc_r.alpha)
        dalpha_r_ctrl = float(vmc_r.d_alpha)
        alpha_l_ctrl = float(-vmc_l.alpha)
        dalpha_l_ctrl = float(-vmc_l.d_alpha)
        Tp_r = -LEG_BODY_ANGLE_KP * alpha_r_ctrl - LEG_BODY_ANGLE_KD * dalpha_r_ctrl
        Tp_l = -LEG_BODY_ANGLE_KP * alpha_l_ctrl - LEG_BODY_ANGLE_KD * dalpha_l_ctrl

        split_tp, split_error = 0.0, alpha_r_ctrl + alpha_l_ctrl
    else:
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
    
    robot.wheel_torque = [T_r, T_l]
    #robot.wheel_torque = [0, 0]
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
