"""gimbal_console.py -- talk to gimbal_fw over UART (T13/T14 bench test, runs on the Pi
or on a laptop with a 3.3 V USB-serial adapter on ESP32 GPIO25/26).

    python -m landing_sw.gimbal_console [--port /dev/ttyAMA0] [--watch]

Commands:  n = NADIR   p = PARK (torque off)   h PITCH ROLL YAW = HOLD (deg)
           s = status  q = quit
Ctrl+C or q leaves the gimbal in whatever mode it was; the firmware watchdog
drops it to NADIR 200 ms after the resends stop.
"""

from __future__ import annotations

import argparse
import math
import threading
import time

from . import config
from . import gimbal_proto as gp
from .gimbal_iface import ESP32Gimbal

MODE_NAMES = {gp.MODE_PARK: "PARK", gp.MODE_HOLD: "HOLD", gp.MODE_NADIR: "NADIR"}


def status_line(g: ESP32Gimbal) -> str:
    t = g.telem()
    if t is None:
        return "no TELEM yet: TX/RX swapped? common GND? firmware in APP_GIMBAL? (TESTING.md step 7)"
    yaw, roll, pitch = (math.degrees(a) for a in t["q_rad"])
    return (f"{MODE_NAMES.get(t['mode'], t['mode'])}  joints p/r/y {pitch:+6.1f} {roll:+6.1f} {yaw:+6.1f} deg"
            f" | cam roll {math.degrees(t['cam_roll_rad']):+6.1f} pitch {math.degrees(t['cam_pitch_rad']):+6.1f}"
            f" | {','.join(gp.flag_names(t['flags'])) or '-'} | servo_err {t['servo_err']}"
            f" | loop max {t['loop_max_us']} us | rtt {g.latency_s * 1e3:.0f} ms | age {t['age_s'] * 1e3:.0f} ms")


def main(argv=None):
    ap = argparse.ArgumentParser(description="gimbal_fw UART console")
    ap.add_argument("--port", default=config.ESP32_GIMBAL_PORT)
    ap.add_argument("--baud", type=int, default=config.ESP32_GIMBAL_BAUD)
    ap.add_argument("--watch", action="store_true", help="print status every second")
    args = ap.parse_args(argv)

    g = ESP32Gimbal(args.port, args.baud)
    stop = threading.Event()
    if args.watch:
        def watch():
            while not stop.wait(1.0):
                print(status_line(g))
        threading.Thread(target=watch, daemon=True).start()

    time.sleep(0.3)
    print(status_line(g))
    try:
        while True:
            cmd = input("> ").split()
            if not cmd:
                continue
            if cmd[0] == "q":
                break
            if cmd[0] == "n":
                g.nadir()
            elif cmd[0] == "p":
                g.park()
            elif cmd[0] == "h" and len(cmd) == 4:
                g.set_angles(*(float(x) for x in cmd[1:]))
            elif cmd[0] != "s":
                print(__doc__.split("Commands:")[1].split("Ctrl+C")[0])
                continue
            time.sleep(0.1)
            print(status_line(g))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        stop.set()
        g.close()


if __name__ == "__main__":
    main()
