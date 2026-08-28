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
