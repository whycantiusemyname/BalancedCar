"""Headless tuning CLI: send bracket frames to BalanceCar, drive it, log telemetry.

Examples:
    python BalanceTuner.py --port COM7 --seconds 10
    python BalanceTuner.py --port COM7 --send "[slider,OffsetBand,0]" --seconds 45 --label band0
    python BalanceTuner.py --port COM7 --drive "20,0,1.5" --drive "0,30,1.0" --seconds 10
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import serial

from GamepadBridge import (
    PARAMETERS_TYPE,
    SAMPLE_NAMES,
    SAMPLE_TYPE,
    STATE_NAMES,
    TelemetryParser,
)

# MG310: 13 PPR x4 quadrature x20.4 gearbox = 1060.8 counts/rev; wheel D=48mm.
COUNTS_PER_CM = 1060.8 / (math.pi * 4.8)
DRIVE_SEND_PERIOD_S = 0.05


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--send", action="append", default=[],
                        help="bracket frame to send after link-up; repeatable")
    parser.add_argument("--drive", action="append", default=[],
                        help="LV,RH,seconds joystick segment (percent -100..100); "
                             "segments run in order after --send frames")
    parser.add_argument("--seconds", type=float, default=15.0,
                        help="total session duration including drive segments")
    parser.add_argument("--label", default="session")
    parser.add_argument("--log-dir", default=str(Path(__file__).parent.parent / "Logs"))
    parser.add_argument("--keep-binary", action="store_true",
                        help="leave the car in binary telemetry mode on exit")
    return parser.parse_args()


class Session:
    def __init__(self, connection: serial.Serial, writer: csv.DictWriter) -> None:
        self.connection = connection
        self.writer = writer
        self.parser = TelemetryParser()
        self.samples: list[dict[str, float]] = []
        self.states: list[int] = []
        self.host_times: list[float] = []
        self.parameters: dict[str, float] | None = None

    def pump(self) -> None:
        waiting = self.connection.in_waiting
        if not waiting:
            return
        for message in self.parser.feed(self.connection.read(waiting)):
            if message["type"] == PARAMETERS_TYPE:
                self.parameters = message["parameters"]
                continue
            if message["type"] != SAMPLE_TYPE:
                continue
            now = time.monotonic()
            self.samples.append(message["values"])
            self.states.append(int(message["state"]))
            self.host_times.append(now)
            self.writer.writerow({
                "host_s": f"{now:.3f}",
                "mcu_ms": message["timestamp_ms"],
                "state": message["state"],
                "flags": message["flags"],
                **message["values"],
            })

    def balancing(self) -> bool:
        return not self.states or self.states[-1] == 1

    def send(self, frame: bytes) -> None:
        self.connection.write(frame)
        self.connection.flush()

    def stop_motion(self) -> None:
        for _ in range(3):
            self.send(b"[joystick,0,0,0,0]")
            time.sleep(0.03)

    def drive(self, lv: float, rh: float, seconds: float) -> bool:
        """Stream one joystick segment; returns False if the car left BALANCING."""
        frame = f"[joystick,0,{lv:.0f},{rh:.0f},0]".encode()
        first_sample = len(self.samples)
        deadline = time.monotonic() + seconds
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.send(frame)
                next_send = now + DRIVE_SEND_PERIOD_S
            self.pump()
            if not self.balancing():
                self.stop_motion()
                print(f"  ABORT drive({lv:.0f},{rh:.0f}): state="
                      f"{STATE_NAMES.get(self.states[-1], self.states[-1])}")
                return False
            time.sleep(0.005)
        self.stop_motion()

        # dead-reckon the segment from telemetry between its first/last samples
        distance_cm = 0.0
        yaw_deg = 0.0
        span = range(max(first_sample, 1), len(self.samples))
        for index in span:
            dt = self.host_times[index] - self.host_times[index - 1]
            if dt <= 0 or dt > 0.5:
                continue
            distance_cm += self.samples[index]["forward_speed_counts_s"] * dt / COUNTS_PER_CM
            yaw_deg += self.samples[index]["yaw_rate_dps"] * dt
        print(f"  drive(LV={lv:.0f},RH={rh:.0f},{seconds:.1f}s): "
              f"moved {distance_cm:+.1f} cm, turned {yaw_deg:+.0f} deg")
        return True


def summarize(session: Session) -> None:
    samples, states = session.samples, session.states
    if not samples:
        print("no samples decoded")
        return
    counts: dict[int, int] = {}
    for s in states:
        counts[s] = counts.get(s, 0) + 1
    print("states:", {STATE_NAMES.get(k, k): v for k, v in sorted(counts.items())})

    drift_cm = 0.0
    for index in range(1, len(samples)):
        dt = session.host_times[index] - session.host_times[index - 1]
        if 0 < dt <= 0.5 and states[index] == 1:
            drift_cm += samples[index]["forward_speed_counts_s"] * dt / COUNTS_PER_CM
    print(f"net drift: {drift_cm:+.1f} cm")

    idle = [s for s, st in zip(samples, states) if st == 1
            and s["target_speed_counts_s"] == 0 and s["target_yaw_rate_dps"] == 0]
    if not idle:
        last = samples[-1]
        print("no idle-balancing samples; last sample:")
        print({k: round(last[k], 2) for k in
               ("pitch_deg", "pitch_rate_dps", "left_motor_command", "right_motor_command")})
        return

    def series(name: str) -> list[float]:
        return [s[name] for s in idle]

    def sd(xs: list[float]) -> float:
        m = sum(xs) / len(xs)
        return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))

    motor = series("left_motor_command")
    jumps = [abs(a - b) for a, b in zip(motor, motor[1:])]
    reversals = sum(1 for a, b in zip(motor, motor[1:]) if a * b < 0)
    pairs = max(1, len(motor) - 1)
    pitch = series("pitch_deg")
    pitch_mean = sum(pitch) / len(pitch)
    crossings = sum(1 for a, b in zip(pitch, pitch[1:])
                    if (a - pitch_mean) * (b - pitch_mean) < 0)
    span_s = max(0.06 * len(idle), 1e-6)
    print(f"idle n={len(idle)}")
    print(f"  pitch osc  ~{crossings / 2 / span_s:.1f} Hz (zero-cross, aliased above 8Hz)")
    print(f"  pitch      mean={sum(series('pitch_deg'))/len(idle):+.2f} sd={sd(series('pitch_deg')):.2f} deg")
    print(f"  pitch_rate sd={sd(series('pitch_rate_dps')):.1f} dps")
    print(f"  speed      sd={sd(series('forward_speed_counts_s')):.0f} counts/s")
    print(f"  integral   mean={sum(series('speed_integral_pitch_deg'))/len(idle):+.2f} deg")
    print(f"  motor L    sd={sd(motor):.0f} mean|jump|={sum(jumps)/max(1,len(jumps)):.0f} "
          f"reversals={100*reversals/pairs:.0f}% jumps>=90: {100*sum(1 for j in jumps if j >= 90)/max(1,len(jumps)):.0f}%")
    print(f"  turn_out   sd={sd(series('turn_output')):.1f}")


def main() -> int:
    args = parse_args()
    log_dir = Path(args.log_dir) / (datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + args.label)
    log_dir.mkdir(parents=True, exist_ok=True)
    csv_path = log_dir / "telemetry.csv"

    connection = serial.Serial(args.port, args.baud, timeout=0, write_timeout=1.0)
    print(f"connected {args.port} @ {args.baud}")
    try:
        with csv_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(
                file, fieldnames=("host_s", "mcu_ms", "state", "flags", *SAMPLE_NAMES))
            writer.writeheader()
            session = Session(connection, writer)

            session.send(b"[telemetry,binary]")
            time.sleep(0.3)
            for frame in args.send:
                session.send(frame.encode())
                print("sent", frame)
                time.sleep(0.15)

            end = time.monotonic() + args.seconds
            for spec in args.drive:
                lv_text, rh_text, sec_text = spec.split(",")
                session.pump()
                if not session.drive(float(lv_text), float(rh_text), float(sec_text)):
                    break
                settle = time.monotonic() + 0.5
                while time.monotonic() < settle:
                    session.pump()
                    time.sleep(0.01)
            while time.monotonic() < end:
                session.pump()
                time.sleep(0.01)
    finally:
        try:
            connection.write(b"[joystick,0,0,0,0]")
            if not args.keep_binary:
                connection.write(b"[telemetry,text]")
            connection.flush()
        except serial.SerialException:
            pass
        connection.close()

    print(f"log: {csv_path}")
    print(f"frames={session.parser.valid_frames} crc_err={session.parser.crc_errors} "
          f"len_err={session.parser.length_errors} lost={session.parser.missing_samples}")
    if session.parameters is not None:
        print("parameters:", {k: round(v, 5) for k, v in session.parameters.items()})
    summarize(session)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
