import mujoco
import mujoco.viewer
import numpy as np
from caculation import orientation2euler


class LegWheelRobot:
    """腿轮机器人仿真类"""
    
    def __init__(self, model_path: str = 'legwheel_robot1.xml', visualize=True):
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.sensor_T = float(self.model.opt.timestep)
        self.sensor_f = 1/self.sensor_T 
        # 导入的 MuJoCo 车轮网格半径为 77 mm；qpos/qvel 使用 SI 单位，因此换算为米。
        self.wheel_r = 0.077

        self.gyro = []
        self.accel = []
        self.orien = []
        self.euler = []

        self.joint_pos = []
        self.wheel_vel = [0,0]

        self.x = 0 #整车位移
        self.d_x = 0 #整车速度

        self.sensor_data = []

        self.left_wheel_pos = 0
        self.right_wheel_pos = 0
        
        self.last_left_wheel_pos = 0
        self.last_right_wheel_pos = 0

        self.wheel_torque = [0,0]#顺序：右、左
        self.joint_torque = [0,0,0,0]#顺序：右前、右后、左前、左后

        # 启动可视化界面
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if visualize else None
        if self.viewer is not None:
            print("MuJoCo界面已启动！按ESC退出")
    
    def sensor_read_data(self):
        """读取传感器数据"""
        # 更新传感器数据
        mujoco.mj_forward(self.model, self.data)
        
        # 四元数+欧拉角
        self.orien = self.data.sensor('orientation').data.copy()
        self.euler = orientation2euler(self.orien)
        # 陀螺仪（角速度）
        self.gyro = self.data.sensor('gyro').data.copy()
        # 轮速（官方给的轮速好像有问题，这边直接用当前位置与上一次位置做差，实车还是要用LK电机的轮速数据）
        self.right_wheel_pos = self.data.sensor('Right_Wheel_pos').data.copy()[0]
        self.left_wheel_pos =  self.data.sensor('Left_Wheel_pos').data.copy()[0]
        right_qdot = (self.right_wheel_pos - self.last_right_wheel_pos) * self.sensor_f
        left_qdot = (self.left_wheel_pos - self.last_left_wheel_pos) * self.sensor_f
        # 两侧 wheel_vel 为正均表示车身向前运动。
        self.wheel_vel[0] = -float(right_qdot)
        self.wheel_vel[1] = float(left_qdot)
        self.last_right_wheel_pos = self.right_wheel_pos
        self.last_left_wheel_pos = self.left_wheel_pos
        
        self.d_x = (self.wheel_vel[0] + self.wheel_vel[1]) * 0.5 * self.wheel_r
        self.x = self.x + self.d_x*self.sensor_T

        right_front_pos = self.data.sensor('Right_front_joint_pos').data.copy()[0] + 0.027  # jAB
        right_rear_pos = self.data.sensor('Right_rear_joint_pos').data.copy()[0] + 1.3       # jAG
        left_front_pos = self.data.sensor('Left_front_joint_pos').data.copy()[0] + 0.003     # jIJ
        left_rear_pos = self.data.sensor('Left_rear_joint_pos').data.copy()[0] - 1.3         # jIO
        self.joint_pos = np.array([right_front_pos, right_rear_pos, left_front_pos, left_rear_pos])

    def actuator_set_torque(self):
        """设置执行器力矩"""
        controls = np.asarray(self.joint_torque + self.wheel_torque, dtype=float)
        if self.model.nu == controls.size:
            controls = np.clip(
                controls,
                self.model.actuator_ctrlrange[:, 0],
                self.model.actuator_ctrlrange[:, 1],
            )
        # 设置关节力矩
        self.data.ctrl[0:4] = controls[0:4]
        
        # 设置轮子力矩
        self.data.ctrl[4:6] = controls[4:6]

    def set_joint_positions(self, joint_angles):

        # 获取关节索引
        joint_indices = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'jAG'),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'jGH'),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'jIO'),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'jOP')
        ]
        
        # 设置关节位置和速度
        for i, idx in enumerate(joint_indices):
            if idx != -1 and i < len(joint_angles):
                self.data.qpos[self.model.jnt_qposadr[idx]] = joint_angles[i]
                self.data.qvel[self.model.jnt_dofadr[idx]] = 0.0
        
        # 更新模型状态
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        """执行一步仿真"""
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()
    
    def reset(self):
        """重置机器人状态"""
        mujoco.mj_resetData(self.model, self.data)
        self.x = 0.0
        self.last_left_wheel_pos = 0.0
        self.last_right_wheel_pos = 0.0
