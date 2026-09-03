"""兼容 MATLAB 增益表的 LQR 控制器。"""

from pathlib import Path

import numpy as np

try:
    from scipy.io import loadmat
except ImportError:  # CSV 回退方案可避免 MuJoCo 运行环境强制依赖 SciPy。
    loadmat = None


GAIN_NAMES = (
    ("k11", "k12", "k13", "k14", "k15", "k16"),
    ("k21", "k22", "k23", "k24", "k25", "k26"),
)


class LQRGainTable:
    """加载 31 组 MATLAB 增益样本并进行线性插值。"""

    def __init__(self, path=None):
        self.path = Path(path or Path(__file__).with_name("lqr_gains.mat"))
        csv_fallback = self.path.with_suffix(".csv")
        if not self.path.is_file() and not csv_fallback.is_file():
            raise FileNotFoundError(
                f"LQR gain table not found: {self.path}. "
                "Export leg/k11-k16/k21-k26 from MATLAB to this path."
            )
        if self.path.suffix.lower() == ".csv" or loadmat is None:
            csv_path = self.path if self.path.suffix.lower() == ".csv" else self.path.with_suffix(".csv")
            if not csv_path.is_file():
                raise ImportError(
                    "SciPy is not installed and no CSV gain table was found. "
                    f"Install scipy or provide {csv_path}."
                )
            table = np.loadtxt(csv_path, delimiter=",")
            if table.ndim != 2 or table.shape[1] != 13:
                raise ValueError(f"{csv_path} must have 13 columns: leg plus 12 gain values")
            raw = {"leg": table[:, 0]}
            for index, name in enumerate(sum(GAIN_NAMES, ()), start=1):
                raw[name] = table[:, index]
            self.source = csv_path
        else:
            raw = loadmat(self.path, squeeze_me=True, struct_as_record=False)
            if "leg" not in raw:
                raise ValueError(f"{self.path} does not contain variable 'leg'")
            self.source = self.path
        self.leg = np.asarray(raw["leg"], dtype=float).reshape(-1)
        if self.leg.size < 2 or not np.all(np.isfinite(self.leg)):
            raise ValueError("'leg' must contain at least two finite samples")
        order = np.argsort(self.leg)
        self.leg = self.leg[order]
        rows = []
        for names in GAIN_NAMES:
            row = []
            for name in names:
                if name not in raw:
                    raise ValueError(f"{self.path} does not contain variable '{name}'")
                values = np.asarray(raw[name], dtype=float).reshape(-1)
                if values.size != self.leg.size:
                    raise ValueError(f"{name} has {values.size} samples, expected {self.leg.size}")
                row.append(values[order])
            rows.append(np.vstack(row))
        self.gains = np.stack(rows, axis=0).transpose(2, 0, 1)  # 样本、输入、状态

    def at(self, leg_length):
        """返回 2x6 矩阵，并将超出导出范围的腿长截断至边界。"""
        length = float(np.clip(leg_length, self.leg[0], self.leg[-1]))
        return np.asarray(
            [[np.interp(length, self.leg, self.gains[:, r, c]) for c in range(6)]
             for r in range(2)], dtype=float)


class LQRController:
    """按 MATLAB 状态顺序计算 [T, Tp]。

    导出的增益已包含 Simulink ``control`` 子系统采用的符号约定，
    其矩阵乘法为 ``K @ state``。
    """

    def __init__(self, gain_table=None, path=None):
        self.gain_table = gain_table or LQRGainTable(path)

    def control(self, theta, dtheta, x, dx, phi, dphi, leg_length):
        state = np.asarray([theta, dtheta, x, dx, phi, dphi], dtype=float)
        if not np.all(np.isfinite(state)):
            raise ValueError(f"non-finite LQR state: {state}")
        return self.gain_table.at(leg_length).dot(state)
