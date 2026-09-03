"""控制器参数与通用控制辅助函数。"""


# 虚拟腿长度 PID 和径向力限幅
LEG_LENGTH_TARGET = 0.28
LEG_LENGTH_KP = 400.0
LEG_LENGTH_KI = 300.0
LEG_LENGTH_KD = 20.0
LEG_LENGTH_INTEGRAL_LIMIT = 0.15
F0_MAX = 120.0

# 滚转 PID 输出为左右腿之间的差分径向力
ROLL_TARGET = 0.0
ROLL_KP = 40.0
ROLL_KI = 1.0
ROLL_KD = 3.0
ROLL_F0_MAX = 30.0

# 分腿抑制 PID。镜像 VMC 坐标下，对称姿态满足 theta_R + theta_L = 0
SPLIT_KP = 18.0
SPLIT_KI = 2.0
SPLIT_KD = 1.5
SPLIT_INTEGRAL_LIMIT = 0.5
SPLIT_TP_MAX = 20.0


class RollPID:
    """输出差分虚拟腿径向力的滚转角 PID。"""

    def __init__(self, kp=ROLL_KP, ki=ROLL_KI, kd=ROLL_KD,
                 output_limit=ROLL_F0_MAX, integral_limit=5.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit))
        self.integral_limit = abs(float(integral_limit))
        self.integral = 0.0

    def reset(self):
        """清除积分项。"""
        self.integral = 0.0

    def update(self, roll, roll_rate, dt, target=ROLL_TARGET):
        """根据滚转角、角速度和目标值计算限幅后的差分力。"""
        # 左右腿 F0 映射方向相反，因此误差保持直观的正负方向。
        error = float(roll) - float(target)
        self.integral += error * max(float(dt), 0.0)
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        correction = self.kp * error + self.ki * self.integral
        correction += self.kd * float(roll_rate)
        return float(max(-self.output_limit,
                         min(self.output_limit, correction)))


def leg_force_f0(vmc, dt, enabled=True):
    """计算直接腿长 PID 径向力，单位为牛顿。"""
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
    """计算抑制分腿的反向虚拟腿力矩修正及分腿误差。"""
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


class PID:
    def __init__(self, p,i,d):
        # self.kp, self.ki, self.kd = pid_params
        self.kp = p
        self.ki = i
        self.kd = d
        self.integral = 0
        self.prev_error = 0
    
    def calc(self, current, target):
        error = target - current
        
        p_term = self.kp * error
        self.integral += error
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error)
        self.prev_error = error
        
        return p_term + i_term + d_term
