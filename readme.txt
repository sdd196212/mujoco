选择mujoco，给你舒适的仿真调车环境
本项目为串联腿空白仿真环境，预装VMC解算，可以轻松部署控制算法

环境：python3.10，ubuntu22.04

前置：
    conda create -n py310 python=3.10
    conda activate py310
    pip3 install mujoco==3.4.0
    pip3 install pynput
    pip3 install numpy

运行：
    python3 Simulation.py

LQR 部署说明：
    lqr_gains.mat/csv 由 MATLAB 的 get_k.m 生成，包含 leg 和 k11-k16、k21-k26
    共 31 个腿长采样点。安装 scipy 时优先读取 mat；未安装 scipy 时自动读取
    同目录的 csv 备份。Simulation.py 按实时 L0 线性插值 K，并使用
    [theta,dtheta,x,dx,phi,dphi] 状态、按 MATLAB control 子系统实际连线
    使用 u=Kx 计算 [T,Tp]。MuJoCo 的 pitch/gyro 输入在送入 LQR 前取负号；
    左腿关节按 XML 轴向先镜像到右腿的统一平面，再将 VMC 力矩镜像回执行器。
    若重新计算了 MATLAB 增益，请在 MATLAB 工程目录运行
    并改串非跳跃上台阶/export_lqr_gains.m，然后覆盖本目录的 lqr_gains.mat。
