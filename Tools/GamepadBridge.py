"""Forward a pygame controller to BalanceCar and record binary telemetry."""

from __future__ import annotations

import argparse
import asyncio
import csv
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time

import pygame
import serial
from bleak import BleakClient, BleakScanner
from bleak.backends.winrt.util import allow_sta, uninitialize_sta
from serial.tools import list_ports


MAGIC = b"\xA5\x5A"
PROTOCOL_VERSION = 1
SAMPLE_TYPE = 1
PARAMETERS_TYPE = 2
HEADER_FORMAT = "<2sBBBBBBHI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
SAMPLE_FORMAT = "<17h"
PARAMETERS_FORMAT = "<21f"
SAMPLE_NAMES = (
    "pitch_deg",
    "accel_pitch_deg",
    "pitch_rate_dps",
    "yaw_rate_dps",
    "yaw_bias_dps",
    "left_speed_counts_s",
    "right_speed_counts_s",
    "forward_speed_counts_s",
    "turn_speed_counts_s",
    "target_speed_counts_s",
    "target_pitch_deg",
    "target_yaw_rate_dps",
    "speed_integral_pitch_deg",
    "balance_output",
    "turn_output",
    "left_motor_command",
    "right_motor_command",
)
SAMPLE_SCALES = (
    100.0, 100.0, 100.0, 100.0, 100.0,
    1.0, 1.0, 1.0, 1.0, 1.0,
    100.0, 100.0, 1000.0,
    1.0, 1.0, 1.0, 1.0,
)
PARAMETER_NAMES = (
    "angle_kp", "angle_ki", "angle_kd",
    "speed_kp", "speed_ki", "speed_kd",
    "turn_kp", "turn_ki", "turn_kd",
    "balance_trim_deg", "motor_deadzone_offset",
    "speed_integral_limit_deg", "target_pitch_limit_deg",
    "motor_output_limit", "turn_output_limit", "turn_integral_limit",
    "joystick_speed_limit", "joystick_yaw_rate_limit_dps",
    "command_timeout_ms", "motor_deadzone_band", "position_hold_kp",
)
STATE_NAMES = {0: "CALIBRATING", 1: "BALANCING", 2: "FALLEN", 3: "FAULT"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward a gamepad to BalanceCar and log full telemetry."
    )
    parser.add_argument("--port", help="Car Bluetooth serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--ble-name", default="HC-04BLE")
    parser.add_argument("--ble-address", help="BLE address; skips name discovery")
    parser.add_argument(
        "--ble-characteristic",
        default="0000ffe1-0000-1000-8000-00805f9b34fb",
    )
    parser.add_argument("--controller", type=int, default=0)
    parser.add_argument("--lv-axis", type=int, default=1)
    parser.add_argument("--rh-axis", type=int, default=2)
    parser.add_argument("--single-rh-axis", type=int, default=0)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--deadzone", type=float, default=0.05)
    parser.add_argument("--expo", type=float, default=0.45)
    parser.add_argument("--accel-rate", type=float, default=120.0)
    parser.add_argument("--brake-rate", type=float, default=300.0)
    parser.add_argument("--turn-rate", type=float, default=360.0)
    parser.add_argument("--high-speed-steer-scale", type=float, default=0.55)
    parser.add_argument("--quit-button", type=int, default=6)
    parser.add_argument("--recover-button", type=int, default=2)
    parser.add_argument("--mode-button", type=int, default=3)
    parser.add_argument(
        "--log", default="Logs",
        help="Log root directory or an explicit .csv path (default: Logs)",
    )
    parser.add_argument("--list", action="store_true")
    return parser.parse_args()


def crc16_ccitt(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class TelemetryParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bytes_received = 0
        self.valid_frames = 0
        self.crc_errors = 0
        self.length_errors = 0
        self.discarded_bytes = 0
        self.missing_samples = 0
        self.last_sample_sequence: int | None = None

    def feed(self, data: bytes) -> list[dict[str, object]]:
        self.bytes_received += len(data)
        self.buffer.extend(data)
        decoded: list[dict[str, object]] = []
        while True:
            magic_index = self.buffer.find(MAGIC)
            if magic_index < 0:
                keep = 1 if self.buffer.endswith(MAGIC[:1]) else 0
                discard = len(self.buffer) - keep
                self.discarded_bytes += discard
                if discard:
                    del self.buffer[:discard]
                break
            if magic_index:
                self.discarded_bytes += magic_index
                del self.buffer[:magic_index]
            if len(self.buffer) < HEADER_SIZE:
                break
            header = struct.unpack_from(HEADER_FORMAT, self.buffer)
            _, frame_type, version, frame_length, state, flags, _, sequence, timestamp_ms = header
            if version != PROTOCOL_VERSION or frame_length < HEADER_SIZE + 2 or frame_length > 128:
                self.length_errors += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_length:
                break
            frame = bytes(self.buffer[:frame_length])
            expected_crc = struct.unpack_from("<H", frame, frame_length - 2)[0]
            if crc16_ccitt(frame[:-2]) != expected_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:frame_length]
            payload = frame[HEADER_SIZE:-2]
            message: dict[str, object] = {
                "type": frame_type,
                "sequence": sequence,
                "timestamp_ms": timestamp_ms,
                "state": state,
                "flags": flags,
            }
            if frame_type == SAMPLE_TYPE and len(payload) == struct.calcsize(SAMPLE_FORMAT):
                raw_values = struct.unpack(SAMPLE_FORMAT, payload)
                message["values"] = {
                    name: raw / scale
                    for name, raw, scale in zip(SAMPLE_NAMES, raw_values, SAMPLE_SCALES)
                }
                if self.last_sample_sequence is None:
                    gap = 0
                else:
                    gap = (sequence - self.last_sample_sequence - 1) & 0xFFFF
                    if gap > 0x7FFF:
                        gap = 0
                self.missing_samples += gap
                self.last_sample_sequence = sequence
                message["sample_gap"] = gap
            elif frame_type == PARAMETERS_TYPE and len(payload) == struct.calcsize(PARAMETERS_FORMAT):
                values = struct.unpack(PARAMETERS_FORMAT, payload)
                message["parameters"] = dict(zip(PARAMETER_NAMES, values))
            else:
                self.length_errors += 1
                continue
            self.valid_frames += 1
            decoded.append(message)
        return decoded


class TelemetryLogger:
    def __init__(self, requested_path: str, link: str, baud: int) -> None:
        requested = Path(requested_path)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        if requested.suffix.lower() == ".csv":
            self.session_dir = requested.parent
            self.csv_path = requested
        else:
            self.session_dir = requested / stamp
            self.csv_path = self.session_dir / "telemetry.csv"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.file = self.csv_path.open("w", newline="", encoding="utf-8", buffering=1)
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=(
                "host_time_s", "host_monotonic_s", "raw_lv", "raw_rh",
                "command_lv", "command_rh", "joystick_mode",
                "sequence", "sample_gap", "mcu_time_ms", "state", "state_name",
                "flags", *SAMPLE_NAMES,
            ),
        )
        self.writer.writeheader()
        self.link = link
        self.baud = baud
        self.started = time.monotonic()
        self.first_sample_at: float | None = None
        self.last_sample_at: float | None = None
        self.sample_count = 0

    def handle(self, message: dict[str, object], controls: dict[str, object]) -> None:
        if message["type"] == PARAMETERS_TYPE:
            payload = {
                "captured_at": datetime.now().isoformat(timespec="seconds"),
                "link": self.link,
                "baud": self.baud,
                "protocol_version": PROTOCOL_VERSION,
                "parameters": message["parameters"],
            }
            (self.session_dir / "parameters.json").write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            return
        now = time.monotonic()
        if self.first_sample_at is None:
            self.first_sample_at = now
        self.last_sample_at = now
        self.sample_count += 1
        values = message["values"]
        row = {
            "host_time_s": f"{time.time():.6f}",
            "host_monotonic_s": f"{now:.6f}",
            "raw_lv": controls["raw_lv"],
            "raw_rh": controls["raw_rh"],
            "command_lv": controls["command_lv"],
            "command_rh": controls["command_rh"],
            "joystick_mode": controls["mode"],
            "sequence": message["sequence"],
            "sample_gap": message["sample_gap"],
            "mcu_time_ms": message["timestamp_ms"],
            "state": message["state"],
            "state_name": STATE_NAMES.get(int(message["state"]), "UNKNOWN"),
            "flags": message["flags"],
            **values,
        }
        self.writer.writerow(row)

    def close(self, parser: TelemetryParser) -> dict[str, object]:
        self.file.close()
        elapsed = max(0.0, time.monotonic() - self.started)
        sample_elapsed = 0.0
        if self.first_sample_at is not None and self.last_sample_at is not None:
            sample_elapsed = max(0.0, self.last_sample_at - self.first_sample_at)
        effective_hz = (
            (self.sample_count - 1) / sample_elapsed
            if self.sample_count > 1 and sample_elapsed > 0.0 else 0.0
        )
        total_expected = self.sample_count + parser.missing_samples
        stats: dict[str, object] = {
            "link": self.link,
            "configured_baud": self.baud,
            "run_seconds": elapsed,
            "bytes_received": parser.bytes_received,
            "valid_frames": parser.valid_frames,
            "sample_frames": self.sample_count,
            "missing_sample_sequences": parser.missing_samples,
            "sample_loss_percent": (
                100.0 * parser.missing_samples / total_expected if total_expected else 0.0
            ),
            "crc_errors": parser.crc_errors,
            "length_errors": parser.length_errors,
            "discarded_nonbinary_bytes": parser.discarded_bytes,
            "effective_sample_hz": effective_hz,
            "csv": str(self.csv_path.resolve()),
        }
        (self.session_dir / "link-stats.json").write_text(
            json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return stats


def axis_to_percent(value: float, deadzone: float) -> int:
    value = max(-1.0, min(1.0, value))
    if abs(value) <= deadzone:
        return 0
    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    return round((magnitude if value > 0.0 else -magnitude) * 100.0)


def init_controllers() -> list[pygame.joystick.Joystick]:
    pygame.init()
    pygame.joystick.init()
    controllers = []
    for index in range(pygame.joystick.get_count()):
        controller = pygame.joystick.Joystick(index)
        controller.init()
        controllers.append(controller)
    return controllers


def print_devices(controllers: list[pygame.joystick.Joystick]) -> None:
    print("Serial ports:")
    for port in list_ports.comports():
        print(f"  {port.device}: {port.description}")
    print("Controllers:")
    for index, controller in enumerate(controllers):
        print(f"  {index}: {controller.get_name()} ({controller.get_numaxes()} axes)")


def apply_expo(value: int, expo: float) -> float:
    normalized = value / 100.0
    return ((1.0 - expo) * normalized + expo * normalized ** 3) * 100.0


def move_toward(current: float, target: float, maximum_delta: float) -> float:
    if current < target:
        return min(current + maximum_delta, target)
    if current > target:
        return max(current - maximum_delta, target)
    return current


def update_motion_command(
    controller: pygame.joystick.Joystick,
    args: argparse.Namespace,
    rh_axis: int,
    current_lv: float,
    current_rh: float,
    dt_seconds: float,
) -> tuple[bytes, int, int, float, float, int, int]:
    raw_lv = axis_to_percent(-controller.get_axis(args.lv_axis), args.deadzone)
    raw_rh = axis_to_percent(controller.get_axis(rh_axis), args.deadzone)
    target_lv = apply_expo(raw_lv, args.expo)
    if current_lv * target_lv < 0.0:
        next_lv = move_toward(current_lv, 0.0, args.brake_rate * dt_seconds)
    else:
        rate = args.accel_rate if abs(target_lv) > abs(current_lv) else args.brake_rate
        next_lv = move_toward(current_lv, target_lv, rate * dt_seconds)
    target_rh = apply_expo(raw_rh, args.expo)
    steer_scale = 1.0 - (1.0 - args.high_speed_steer_scale) * abs(next_lv) / 100.0
    next_rh = move_toward(current_rh, target_rh * steer_scale, args.turn_rate * dt_seconds)
    lv, rh = round(next_lv), round(next_rh)
    return f"[joystick,0,{lv},{rh},0]".encode(), lv, rh, next_lv, next_rh, raw_lv, raw_rh


def send_serial_safely(connection: serial.Serial, frame: bytes) -> None:
    connection.write(frame)
    connection.flush()


def finish_serial(connection: serial.Serial) -> None:
    for frame in (b"[joystick,0,0,0,0]",) * 3 + (b"[telemetry,text]",):
        try:
            send_serial_safely(connection, frame)
        except serial.SerialException:
            break
        time.sleep(0.02)


def run_serial(args: argparse.Namespace, controller: pygame.joystick.Joystick) -> None:
    connection = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.2)
    parser = TelemetryParser()
    logger = TelemetryLogger(args.log, f"serial:{args.port}", args.baud)
    controls: dict[str, object] = {
        "raw_lv": 0, "raw_rh": 0, "command_lv": 0, "command_rh": 0, "mode": "TWO"
    }
    clock = pygame.time.Clock()
    single_stick = False
    recover_was_pressed = False
    mode_was_pressed = False
    neutral_until = 0.0
    command_lv = command_rh = 0.0
    last_update = time.monotonic()
    send_serial_safely(connection, b"[telemetry,binary]")
    print(f"Serial: {args.port} @ {args.baud}; log: {logger.session_dir.resolve()}")
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEREMOVED:
                    raise RuntimeError("Controller disconnected")
            if args.quit_button >= 0 and controller.get_button(args.quit_button):
                raise KeyboardInterrupt
            recover_pressed = controller.get_button(args.recover_button) != 0
            mode_pressed = controller.get_button(args.mode_button) != 0
            if mode_pressed and not mode_was_pressed:
                single_stick = not single_stick
                controls["mode"] = "ONE" if single_stick else "TWO"
                print(f"\nJoystick mode: {controls['mode']}")
            if recover_pressed and not recover_was_pressed:
                send_serial_safely(connection, b"[key,Recover,down]")
                neutral_until = time.monotonic() + 3.0
                print("\nRecover sent; controls neutralized for 3 seconds")
            recover_was_pressed, mode_was_pressed = recover_pressed, mode_pressed
            now = time.monotonic()
            dt_seconds = min(0.1, max(0.0, now - last_update))
            last_update = now
            if now < neutral_until:
                frame, lv, rh, command_lv, command_rh, raw_lv, raw_rh = (
                    b"[joystick,0,0,0,0]", 0, 0, 0.0, 0.0, 0, 0
                )
            else:
                rh_axis = args.single_rh_axis if single_stick else args.rh_axis
                frame, lv, rh, command_lv, command_rh, raw_lv, raw_rh = update_motion_command(
                    controller, args, rh_axis, command_lv, command_rh, dt_seconds
                )
            controls.update(raw_lv=raw_lv, raw_rh=raw_rh, command_lv=lv, command_rh=rh)
            connection.write(frame)
            if connection.in_waiting:
                for message in parser.feed(connection.read(connection.in_waiting)):
                    logger.handle(message, controls)
            print(
                f"\r{controls['mode']} TX {lv:+4d}/{rh:+4d} "
                f"RX {logger.sample_count:6d} lost {parser.missing_samples:4d} "
                f"crc {parser.crc_errors:3d}",
                end="", flush=True,
            )
            clock.tick(args.rate)
    finally:
        finish_serial(connection)
        connection.close()
        stats = logger.close(parser)
        print(f"\nLink result: {stats['effective_sample_hz']:.2f} Hz, "
              f"loss {stats['sample_loss_percent']:.2f}%, CRC {stats['crc_errors']}")


async def write_ble_frame(client: BleakClient, characteristic: str, frame: bytes) -> None:
    attribute = client.services.get_characteristic(characteristic)
    if attribute is None:
        raise RuntimeError(f"BLE characteristic not found: {characteristic}")
    chunk_size = max(1, attribute.max_write_without_response_size)
    for offset in range(0, len(frame), chunk_size):
        await client.write_gatt_char(attribute, frame[offset:offset + chunk_size], response=False)


async def run_ble(args: argparse.Namespace, controller: pygame.joystick.Joystick) -> None:
    if args.ble_address:
        device = await BleakScanner.find_device_by_address(args.ble_address, timeout=10.0)
    else:
        device = await BleakScanner.find_device_by_name(args.ble_name, timeout=10.0)
    if device is None:
        raise RuntimeError(f"BLE device not found: {args.ble_address or args.ble_name}")
    parser = TelemetryParser()
    logger = TelemetryLogger(args.log, f"ble:{device.address}", args.baud)
    controls: dict[str, object] = {
        "raw_lv": 0, "raw_rh": 0, "command_lv": 0, "command_rh": 0, "mode": "TWO"
    }

    def on_notification(_: object, data: bytearray) -> None:
        for message in parser.feed(bytes(data)):
            logger.handle(message, controls)

    single_stick = False
    recover_was_pressed = mode_was_pressed = False
    neutral_until = 0.0
    command_lv = command_rh = 0.0
    last_update = time.monotonic()
    async with BleakClient(device, timeout=10.0) as client:
        await client.start_notify(args.ble_characteristic, on_notification)
        await write_ble_frame(client, args.ble_characteristic, b"[telemetry,binary]")
        print(f"BLE: {device.name} ({device.address}); log: {logger.session_dir.resolve()}")
        period = 1.0 / args.rate
        next_send = asyncio.get_running_loop().time()
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.JOYDEVICEREMOVED:
                        raise RuntimeError("Controller disconnected")
                if args.quit_button >= 0 and controller.get_button(args.quit_button):
                    raise KeyboardInterrupt
                recover_pressed = controller.get_button(args.recover_button) != 0
                mode_pressed = controller.get_button(args.mode_button) != 0
                if mode_pressed and not mode_was_pressed:
                    single_stick = not single_stick
                    controls["mode"] = "ONE" if single_stick else "TWO"
                    print(f"\nJoystick mode: {controls['mode']}")
                if recover_pressed and not recover_was_pressed:
                    await write_ble_frame(client, args.ble_characteristic, b"[key,Recover,down]")
                    neutral_until = time.monotonic() + 3.0
                    print("\nRecover sent; controls neutralized for 3 seconds")
                recover_was_pressed, mode_was_pressed = recover_pressed, mode_pressed
                now = time.monotonic()
                dt_seconds = min(0.1, max(0.0, now - last_update))
                last_update = now
                if now < neutral_until:
                    frame, lv, rh, command_lv, command_rh, raw_lv, raw_rh = (
                        b"[joystick,0,0,0,0]", 0, 0, 0.0, 0.0, 0, 0
                    )
                else:
                    rh_axis = args.single_rh_axis if single_stick else args.rh_axis
                    frame, lv, rh, command_lv, command_rh, raw_lv, raw_rh = update_motion_command(
                        controller, args, rh_axis, command_lv, command_rh, dt_seconds
                    )
                controls.update(raw_lv=raw_lv, raw_rh=raw_rh, command_lv=lv, command_rh=rh)
                await write_ble_frame(client, args.ble_characteristic, frame)
                print(
                    f"\r{controls['mode']} TX {lv:+4d}/{rh:+4d} "
                    f"RX {logger.sample_count:6d} lost {parser.missing_samples:4d} "
                    f"crc {parser.crc_errors:3d}", end="", flush=True,
                )
                next_send += period
                await asyncio.sleep(max(0.0, next_send - asyncio.get_running_loop().time()))
        finally:
            for frame in (b"[joystick,0,0,0,0]",) * 3 + (b"[telemetry,text]",):
                if client.is_connected:
                    await write_ble_frame(client, args.ble_characteristic, frame)
                    await asyncio.sleep(0.02)
            stats = logger.close(parser)
            print(f"\nLink result: {stats['effective_sample_hz']:.2f} Hz, "
                  f"loss {stats['sample_loss_percent']:.2f}%, CRC {stats['crc_errors']}")


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.deadzone <= 0.5:
        raise SystemExit("--deadzone must be between 0 and 0.5")
    if not 0.0 <= args.expo <= 1.0:
        raise SystemExit("--expo must be between 0 and 1")
    if not 0.0 < args.high_speed_steer_scale <= 1.0:
        raise SystemExit("--high-speed-steer-scale must be between 0 and 1")
    if min(args.rate, args.accel_rate, args.brake_rate, args.turn_rate) <= 0.0:
        raise SystemExit("rates must be greater than zero")
    controllers = init_controllers()
    if args.list:
        print_devices(controllers)
        return 0
    if args.controller >= len(controllers):
        print_devices(controllers)
        raise SystemExit(f"Controller index {args.controller} is not available")
    controller = controllers[args.controller]
    required_axis = max(args.lv_axis, args.rh_axis, args.single_rh_axis)
    if min(args.lv_axis, args.rh_axis, args.single_rh_axis) < 0 or required_axis >= controller.get_numaxes():
        raise SystemExit(f"Controller has {controller.get_numaxes()} axes; check mappings")
    required_button = max(args.quit_button, args.recover_button, args.mode_button)
    if required_button >= controller.get_numbuttons():
        raise SystemExit(f"Controller has {controller.get_numbuttons()} buttons; check mappings")
    print(f"Controller: {controller.get_name()}")
    try:
        if args.port:
            run_serial(args, controller)
        else:
            uninitialize_sta()
            allow_sta()
            asyncio.run(run_ble(args, controller))
    except (KeyboardInterrupt, RuntimeError, serial.SerialException) as exc:
        print(f"\nStopping: {exc}")
    finally:
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
