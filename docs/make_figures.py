"""Build the host simulation of lib/Pid/Pid.h and plot its step response.

    pip install matplotlib
    python docs/make_figures.py      # needs g++
"""

import csv
import io
import subprocess
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "images"
plt.rcParams.update({"figure.dpi": 150, "axes.grid": True, "grid.alpha": 0.3, "axes.spines.top": False, "axes.spines.right": False})


def simulate() -> dict[str, list[float]]:
    with tempfile.TemporaryDirectory() as tmp:
        exe = Path(tmp) / "step"
        subprocess.run(["g++", "-std=c++17", "-O2", f"-I{ROOT / 'lib' / 'Pid'}", str(ROOT / "docs" / "step_response.cpp"), "-o", str(exe)], check=True)
        text = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
    rows = list(csv.DictReader(io.StringIO(text)))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def main() -> None:
    d = simulate()
    t = d["time"]
    sp = [100 if x < 1 else 400 if x < 2 else 150 for x in t]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    a1.plot(t, sp, "k--", lw=1, label="setpoint")
    labels = {"P": "P, Kp 0.5 (steady-state error)", "PI": "PI, Kp 0.5 Ki 5",
              "PID": "PID, Kp 1.2 Ki 8 Kd 0.02", "PI_no_antiwindup": "PI without anti-windup"}
    for key, label in labels.items():
        style = ":" if key == "PI_no_antiwindup" else "-"
        a1.plot(t, d[f"{key}_rpm"], style, label=label)
        a2.plot(t, d[f"{key}_pwm"], style)
    a1.axvspan(1, 2, color="orange", alpha=0.08)
    a1.text(1.5, 360, "400 rpm is out of reach:\nactuator saturates", ha="center", fontsize=8)
    a1.set(ylabel="speed (rpm)", title="Pid.h in closed loop with a first-order motor model (tau 150 ms, 1.3 rpm/PWM)")
    a1.legend(fontsize=8, loc="upper right")
    a2.set(xlabel="time (s)", ylabel="PWM command", ylim=(-270, 270))
    fig.tight_layout()
    fig.savefig(OUT / "step_response.png")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    main()
