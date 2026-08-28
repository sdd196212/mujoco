# mujoco

## LQR and F0 control

`Simulation.py` loads the 31-point MATLAB gain table and computes the wheel
torque `T` and virtual hip torque `Tp`. The `leg_force_f0()` controller adds
radial VMC force `F0` for gravity compensation and leg-length regulation:

- nominal leg length: `0.143 m`
- length gains: `Kp = 800 N/m`, `Kd = 35 N/(m/s)`
- radial force limit: `+/-120 N`

The force is split between the two legs using the MuJoCo model mass and the
current leg-to-vertical projection, then `F0` and `Tp` are mapped through the
existing VMC Jacobian to the four leg actuators. The wheel actuator receives
the existing sign-corrected LQR `T` output.

Roll control uses `RollPID` in `Simulation.py`. Its output is a differential
force (`+dF0` on the right leg and `-dF0` on the left leg), based on the IMU
roll angle `euler[0]` and roll rate `gyro[0]`. The default gains are `Kp=35
N/rad`, `Ki=2 N/(rad*s)`, `Kd=3 N*s/rad`, with a `+/-30 N` differential-force
limit and integral anti-windup.

Actuator limits are defined in `MJCF/robot.xml`: the four leg joint motors are
limited to `+/-35 Nm` and the two wheel motors to `+/-4 Nm`. `Tp` has no
software limit; after VMC mapping, `environment.py` applies only these final
actuator limits.
