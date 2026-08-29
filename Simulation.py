"""Run the MuJoCo wheel-leg simulation with the MATLAB LQR chain."""

import math
from pathlib import Path

from environment import LegWheelRobot
from VMC import leg_VMC
from lqr_controller import LQRController



CONTROL_DIVIDER = 4  # model timestep is 1 ms; control update is 4 ms

# Virtual-leg force controller. F0 is a radial force in the VMC leg plane.
LEG_LENGTH_TARGET = 0.25
LEG_LENGTH_KP = 400.0       # N/m
LEG_LENGTH_KI = 300.0       # N/(m*s)
LEG_LENGTH_KD = 20.0        # N/(m/s)
LEG_LENGTH_INTEGRAL_LIMIT = 0.15  # m*s
F0_MAX = 120.0              # N, before the existing motor torque limits

# Roll PID output is a differential radial force.  With the model's actual
# IMU/actuator convention, a positive roll correction subtracts dF0 from the
# right leg and adds it to the left leg.
ROLL_TARGET = 0.0           # rad
ROLL_KP = 35.0              # N/rad
ROLL_KI = 2.0               # N/(rad*s)
ROLL_KD = 3.0               # N*s/rad
ROLL_F0_MAX = 30.0           # N, differential force limit

# Anti-split PID. In mirrored VMC coordinates, symmetric legs satisfy
# theta_R + theta_L = 0.
SPLIT_KP = 18.0             # Nm/rad
SPLIT_KI = 1.0              # Nm/(rad*s)
SPLIT_KD = 1.5              # Nm*s/rad
SPLIT_INTEGRAL_LIMIT = 0.5  # rad*s
SPLIT_TP_MAX = 8.0          # Nm, correction only


class RollPID:
    """Roll-angle PID whose output is a differential virtual-leg force."""

    def __init__(self, kp=ROLL_KP, ki=ROLL_KI, kd=ROLL_KD,
                 output_limit=ROLL_F0_MAX, integral_limit=5.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit))
        self.integral_limit = abs(float(integral_limit))
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, roll, roll_rate, dt, target=ROLL_TARGET):
        # Positive correction is applied through the reversed left/right F0
        # mapping below, so keep the PID error in the intuitive direction.
        error = float(roll) - float(target)
        self.integral += error * max(float(dt), 0.0)
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        correction = self.kp * error + self.ki * self.integral
        correction += self.kd * float(roll_rate)
        return float(max(-self.output_limit,
                         min(self.output_limit, correction)))


def leg_force_f0(vmc, dt, enabled=True):
    """Return the direct leg-length PID output in newtons."""
    first_update = not hasattr(vmc, "length_integral")
    if first_update:
        vmc.length_integral = 0.0
    if not enabled:
        vmc.length_integral = 0.0
        return 0.0

    error = LEG_LENGTH_TARGET - vmc.L0
    vmc.length_integral += error * max(float(dt), 0.0)
    vmc.length_integral = max(
        -LEG_LENGTH_INTEGRAL_LIMIT,
        min(LEG_LENGTH_INTEGRAL_LIMIT, vmc.length_integral),
    )
    length_rate = 0.0 if first_update else vmc.d_L0
    force = (
        LEG_LENGTH_KP * error
        + LEG_LENGTH_KI * vmc.length_integral
        - LEG_LENGTH_KD * length_rate
    )
    vmc.length_error = float(error)
    return float(max(-F0_MAX, min(F0_MAX, force)))


def split_torque_pid(robot, vmc_r, vmc_l, dt, enabled=True):
    """Return opposite-leg Tp correction that suppresses leg splitting."""
    if not hasattr(robot, "split_integral"):
        robot.split_integral = 0.0
    if not enabled:
        robot.split_integral = 0.0
        return 0.0, 0.0

    error = vmc_r.theta + vmc_l.theta
    error_rate = vmc_r.d_theta + vmc_l.d_theta
    robot.split_integral += error * max(float(dt), 0.0)
    robot.split_integral = max(
        -SPLIT_INTEGRAL_LIMIT,
        min(SPLIT_INTEGRAL_LIMIT, robot.split_integral),
    )
    torque = (
        SPLIT_KP * error
        + SPLIT_KI * robot.split_integral
        + SPLIT_KD * error_rate
    )
    torque = max(-SPLIT_TP_MAX, min(SPLIT_TP_MAX, torque))
    return float(torque), float(error)


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
    vmc_l.F0 = max(-F0_MAX, min(F0_MAX, base_f0_l + roll_f0))
    # The left VMC plane is mirrored before actuator mapping.
    vmc_l.F0 = -vmc_l.F0
    vmc_r.Tp = Tp_r + split_tp
    vmc_l.Tp = -Tp_l - split_tp
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
    robot.wheel_torque = [0, 0]
    robot.actuator_set_torque()
    print(
        f"[LQR] F0(R/L)=({vmc_r.F0:+.6f}, {vmc_l.F0:+.6f}) N, "
        f"L0(R/L)=({vmc_r.L0:+.6f}, {vmc_l.L0:+.6f}) m, "
        f"roll dF0={roll_f0:+.6f} N, "
        f"pitch={pitch:+.6f} N, "
        f"theta(R/L)=({vmc_r.theta:+.6f}, {vmc_l.theta:+.6f}) rad, "
        f"split={split_error:+.6f} rad, split Tp={split_tp:+.6f} Nm, "
        f"Tp(R/L)=({vmc_r.Tp:+.6f}, {vmc_l.Tp:+.6f}) Nm, "
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
