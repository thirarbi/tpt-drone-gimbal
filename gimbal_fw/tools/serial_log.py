"""serial_log.py -- record gimbal_fw "D," lines to CSV, then plot them (T11/T17 plots, poster numbers).

Set LOG_CSV_HZ (e.g. 50) in main/config_defaults.h, flash, close idf.py monitor (one program per port), then:

    python gimbal_fw/tools/serial_log.py record /dev/ttyUSB0 --seconds 30 --out run1.csv
    python gimbal_fw/tools/serial_log.py plot run1.csv           # writes run1.png, prints error RMS/max
"""

import argparse
import time

import numpy as np

COLUMNS = ["t_ms", "cam_roll", "cam_pitch", "qcmd_yaw", "qcmd_roll", "qcmd_pitch",
           "qmeas_yaw", "qmeas_roll", "qmeas_pitch", "flags"]       # degrees, see print_csv() in app_main.c


def record(port, baud, seconds, out):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.1
    s.dtr = s.rts = False          # DevKit auto-reset lines: don't reset the ESP32 on open
    s.open()
    n = 0
    with s, open(out, "w") as f:
        f.write(",".join(COLUMNS) + "\n")
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            line = s.readline().decode(errors="ignore").strip()
            if line.startswith("D,") and line.count(",") == len(COLUMNS):
                f.write(line[2:] + "\n")
                n += 1
    print(f"{n} rows -> {out}" + ("" if n else "  (none: is LOG_CSV_HZ > 0? monitor closed? baud?)"))


def plot(path, target_pitch, target_roll):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = np.genfromtxt(path, delimiter=",", names=True)
    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    e_pitch = d["cam_pitch"] - target_pitch
    e_roll = d["cam_roll"] - target_roll
    for name, e in (("pitch", e_pitch), ("roll", e_roll)):
        print(f"camera {name} error: RMS {np.sqrt(np.mean(e ** 2)):.2f} deg, max {np.max(np.abs(e)):.2f} deg")

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    ax[0].plot(t, e_pitch, label="camera pitch error")
    ax[0].plot(t, e_roll, label="camera roll error")
    ax[0].set_ylabel("deg (IMU)")
    for j in ("roll", "pitch"):
        ax[1].plot(t, d[f"qcmd_{j}"], label=f"q_cmd {j}")
        ax[1].plot(t, d[f"qmeas_{j}"], "--", label=f"q_meas {j}")
    ax[1].set_ylabel("joint deg")
    ax[1].set_xlabel("s")
    for a in ax:
        a.grid(True)
        a.legend(loc="upper right")
    out = path.rsplit(".", 1)[0] + ".png"
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"plot -> {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("port")
    r.add_argument("--baud", type=int, default=921600)
    r.add_argument("--seconds", type=float, default=30.0)
    r.add_argument("--out", default="gimbal_log.csv")
    p = sub.add_parser("plot")
    p.add_argument("csv")
    p.add_argument("--target-pitch", type=float, default=-90.0)
    p.add_argument("--target-roll", type=float, default=0.0)
    args = ap.parse_args()
    if args.cmd == "record":
        record(args.port, args.baud, args.seconds, args.out)
    else:
        plot(args.csv, args.target_pitch, args.target_roll)


if __name__ == "__main__":
    main()
