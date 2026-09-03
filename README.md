# mujoco

## LQR and F0 control

`Simulation.py` loads the 31-point MATLAB gain table and orchestrates the wheel
torque `T` and virtual hip torque `Tp` calculation. `Controller.py` provides
`leg_force_f0()`, which adds radial VMC force `F0` for leg-length regulation:

- nominal leg length: `0.28 m`
- length gains: `Kp = 400 N/m`, `Ki = 300 N/(m*s)`, `Kd = 20 N/(m/s)`
- radial force limit: `+/-120 N`

Each leg runs its own leg-length PID, then the roll differential is applied to
the two radial-force commands. `F0` and `Tp` are mapped through the existing
VMC Jacobian to the four leg actuators. The wheel actuator receives the
existing sign-corrected LQR `T` output.

Roll control uses `RollPID` in `Controller.py`. Its output is a differential
force (`-dF0` on the right leg and `+dF0` on the left leg for a positive
correction), based on the IMU
roll angle `euler[0]` and roll rate `gyro[0]`. The default gains are `Kp=40
N/rad`, `Ki=1 N/(rad*s)`, `Kd=3 N*s/rad`, with a `+/-30 N` differential-force
limit and integral anti-windup.

Actuator limits are defined in `MJCF/robot.xml`: the four leg joint motors are
limited to `+/-35 Nm` and the two wheel motors to `+/-3 Nm`. `Tp` has no
software limit; after VMC mapping, `environment.py` applies only these final
actuator limits.
