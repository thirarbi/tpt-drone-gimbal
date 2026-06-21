"""telemetry_db.py -- SQLite flight logger.

One `flight` row per arming/run, many `tick` rows (per-frame state of the whole
perception->control chain), and discrete `event` rows (state transitions, faults).
This is the off-hot-path telemetry sink (principle 7): the full-rate detection
loop logs here for post-flight analysis without blocking control.
"""

from __future__ import annotations

import sqlite3
import time

from . import config

TICK_COLS = [
    "flight_id", "t", "state", "marker_id",
    "pnp_x", "pnp_y", "pnp_z", "reproj_err",
    "meas_x", "meas_y", "meas_z", "meas_accepted", "innov_chi2",
    "ekf_px", "ekf_py", "ekf_pz", "ekf_vx", "ekf_vy", "ekf_vz",
    "g_pitch", "g_roll", "g_yaw", "sp_vx", "sp_vy", "sp_vz", "agl", "loop_ms",
]


class TelemetryDB:
    def __init__(self, path: str = None):
        path = path or config.results_path(config.TELEMETRY_DB_PATH)
        self.conn = sqlite3.connect(path)
        self.conn.row_factory = sqlite3.Row
        self._init_schema()

    def _init_schema(self):
        c = self.conn.cursor()
        c.execute("""CREATE TABLE IF NOT EXISTS flight (
            flight_id INTEGER PRIMARY KEY AUTOINCREMENT,
            started_at REAL, scenario TEXT, gps TEXT, notes TEXT)""")
        c.execute(f"""CREATE TABLE IF NOT EXISTS tick (
            tick_id INTEGER PRIMARY KEY AUTOINCREMENT,
            {', '.join(col + ' REAL' if col not in ('state',) else col + ' TEXT'
                       for col in TICK_COLS)})""")
        c.execute("""CREATE TABLE IF NOT EXISTS event (
            event_id INTEGER PRIMARY KEY AUTOINCREMENT,
            flight_id INTEGER, t REAL, kind TEXT, detail TEXT)""")
        self.conn.commit()

    def start_flight(self, scenario: str, gps: str = "", notes: str = "") -> int:
        cur = self.conn.execute(
            "INSERT INTO flight(started_at, scenario, gps, notes) VALUES (?,?,?,?)",
            (time.time(), scenario, gps, notes))
        self.conn.commit()
        return int(cur.lastrowid)

    def log_tick(self, *values):
        if len(values) != len(TICK_COLS):
            raise ValueError(f"log_tick expects {len(TICK_COLS)} values, "
                             f"got {len(values)}")
        ph = ",".join("?" * len(TICK_COLS))
        self.conn.execute(
            f"INSERT INTO tick({','.join(TICK_COLS)}) VALUES ({ph})",
            tuple(None if v is None else (str(v) if isinstance(v, str) else float(v))
                  for v in values))

    def log_event(self, flight_id: int, t: float, kind: str, detail: str = ""):
        self.conn.execute(
            "INSERT INTO event(flight_id, t, kind, detail) VALUES (?,?,?,?)",
            (flight_id, t, kind, detail))

    def commit(self):
        self.conn.commit()

    def close(self):
        self.conn.commit()
        self.conn.close()
