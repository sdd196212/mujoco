"""MATLAB-compatible gain-table LQR controller."""

from pathlib import Path

import numpy as np

try:
    from scipy.io import loadmat
except ImportError:  # CSV fallback keeps the MuJoCo runtime lightweight.
    loadmat = None


GAIN_NAMES = (
    ("k11", "k12", "k13", "k14", "k15", "k16"),
    ("k21", "k22", "k23", "k24", "k25", "k26"),
)


class LQRGainTable:
    """Load and linearly interpolate the 31 MATLAB gain samples."""

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
        self.gains = np.stack(rows, axis=0).transpose(2, 0, 1)  # sample, input, state

    def at(self, leg_length):
        """Return a 2x6 matrix, clipping outside the exported length range."""
        length = float(np.clip(leg_length, self.leg[0], self.leg[-1]))
        return np.asarray(
            [[np.interp(length, self.leg, self.gains[:, r, c]) for c in range(6)]
             for r in range(2)], dtype=float)


class LQRController:
    """Compute [T, Tp] using the MATLAB state order.

    The exported gains already include the sign convention used by the
    Simulink ``control`` subsystem, whose matrix product is ``K @ state``.
    """

    def __init__(self, gain_table=None, path=None):
        self.gain_table = gain_table or LQRGainTable(path)

    def control(self, theta, dtheta, x, dx, phi, dphi, leg_length):
        state = np.asarray([theta, dtheta, x, dx, phi, dphi], dtype=float)
        if not np.all(np.isfinite(state)):
            raise ValueError(f"non-finite LQR state: {state}")
        return self.gain_table.at(leg_length).dot(state)
