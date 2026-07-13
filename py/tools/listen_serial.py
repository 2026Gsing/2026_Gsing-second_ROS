#!/usr/bin/env python3
"""
UART7 wheel-leg support debug monitor.

Firmware frame:
  AA 55 | time_us u32 | frame_type u8 | leg_id u8 | flags u8 | seq u8 |
  value[16] float32 | 55 AA
"""

import argparse
import csv
import struct
import sys
import time
from typing import Dict, Iterable, List, Optional, Tuple

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:
    print("pyserial is required: pip install pyserial", file=sys.stderr)
    raise SystemExit(2) from exc


HEADER = b"\xAA\x55"
TAIL = b"\x55\xAA"
FRAME_FMT = "<2sIBBBB16f2s"
FRAME_SIZE = struct.calcsize(FRAME_FMT)

FRAME_TYPE_LEG = 1
FRAME_TYPE_GLOBAL = 2

FLAG_SWING = 0x01
FLAG_LIMITED = 0x02
FLAG_RESCUE = 0x04

LEG_NAMES = {
    0: "leg0_12_LF",
    1: "leg1_78_RR",
    2: "leg2_56_LR",
    3: "leg3_34_RF",
}

LEG_FIELDS = [
    "raw_target_x_cm",
    "raw_target_y_cm",
    "target_x_cm",
    "target_y_cm",
    "actual_x_cm",
    "actual_y_cm",
    "error_y_cm",
    "fy_total_n",
    "fy_up_n",
    "fy_sag_n",
    "swing_unload_n",
    "support_y_bias_cm",
    "stance_entry_ramp",
    "stance_support_ramp",
    "sag_rescue",
    "tau_ff_knee_nm",
]

GLOBAL_FIELDS = [
    "gait_cmd_vx_mps",
    "gait_cmd_wz_radps",
    "gait_target_stride_cm",
    "gait_target_lift_cm",
    "gait_stride_cm",
    "gait_lift_cm",
    "wheel_leg_base_mps",
    "wheel_leg_turn_mps",
    "roll_err_filt_deg",
    "pitch_err_filt_deg",
    "comp_scale",
    "vibration_scale",
    "support_force_bias_leg0_n",
    "support_force_bias_leg1_n",
    "support_force_bias_leg2_n",
    "support_force_bias_leg3_n",
]


def iter_ports() -> Iterable[str]:
    for port in serial.tools.list_ports.comports():
        yield port.device


def auto_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        text = f"{port.device} {port.description} {port.hwid}".lower()
        if "stm32" in text or "0483:5740" in text:
            return port.device
    for port in ports:
        text = f"{port.device} {port.description} {port.hwid}".lower()
        if "usb" in text or "ch340" in text or "cp210" in text or "ftdi" in text:
            return port.device
    if sys.platform.startswith("win"):
        return "COM3"
    return "/dev/ttyUSB0"


def parse_frame(raw: bytes) -> Optional[dict]:
    if len(raw) != FRAME_SIZE:
        return None
    unpacked = struct.unpack(FRAME_FMT, raw)
    header = unpacked[0]
    tail = unpacked[-1]
    if header != HEADER or tail != TAIL:
        return None
    return {
        "time_us": unpacked[1],
        "frame_type": unpacked[2],
        "leg_id": unpacked[3],
        "flags": unpacked[4],
        "seq": unpacked[5],
        "values": list(unpacked[6:22]),
    }


def pop_frames(buffer: bytearray) -> List[dict]:
    frames = []
    while True:
        start = buffer.find(HEADER)
        if start < 0:
            del buffer[:-1]
            return frames
        if start > 0:
            del buffer[:start]
        if len(buffer) < FRAME_SIZE:
            return frames
        raw = bytes(buffer[:FRAME_SIZE])
        frame = parse_frame(raw)
        if frame is None:
            del buffer[0]
            continue
        frames.append(frame)
        del buffer[:FRAME_SIZE]


def flag_text(flags: int) -> str:
    text = []
    if flags & FLAG_SWING:
        text.append("SW")
    else:
        text.append("ST")
    if flags & FLAG_LIMITED:
        text.append("LIM")
    if flags & FLAG_RESCUE:
        text.append("RES")
    return "|".join(text)


def frame_row(frame: dict) -> Dict[str, object]:
    row = {
        "host_time_s": f"{time.time():.3f}",
        "time_us": frame["time_us"],
        "seq": frame["seq"],
        "frame_type": frame["frame_type"],
        "leg_id": frame["leg_id"],
        "flags": frame["flags"],
    }
    if frame["frame_type"] == FRAME_TYPE_LEG:
        for name, value in zip(LEG_FIELDS, frame["values"]):
            row[name] = value
    elif frame["frame_type"] == FRAME_TYPE_GLOBAL:
        for name, value in zip(GLOBAL_FIELDS, frame["values"]):
            row[name] = value
    return row


def print_snapshot(latest_legs: Dict[int, dict], latest_global: Optional[dict]) -> None:
    if latest_global is not None:
        g = latest_global["values"]
        print(
            f"\nGLOBAL t={latest_global['time_us'] / 1e6:8.3f}s "
            f"vx={g[0]:+.3f} wz={g[1]:+.3f} "
            f"stride={g[4]:.2f}cm lift={g[5]:.2f}cm "
            f"roll={g[8]:+.2f} pitch={g[9]:+.2f} "
            f"comp={g[10]:.2f} vib={g[11]:.2f}"
        )
        print(
            "att_bias[N] "
            f"L0={g[12]:+.2f} L1={g[13]:+.2f} "
            f"L2={g[14]:+.2f} L3={g[15]:+.2f}"
        )

    print(
        "leg          st  ty/ay(cm)  ey(cm)  FyTot  FyUp  FySag  "
        "Unld  yBias  entry support rescue tauK"
    )
    for leg_id in range(4):
        frame = latest_legs.get(leg_id)
        if frame is None:
            print(f"{LEG_NAMES[leg_id]:12s} --  no data")
            continue
        v = frame["values"]
        print(
            f"{LEG_NAMES[leg_id]:12s} {flag_text(frame['flags']):6s} "
            f"{v[3]:5.2f}/{v[5]:5.2f} {v[6]:+6.2f} "
            f"{v[7]:+6.1f} {v[8]:+5.1f} {v[9]:+5.1f} "
            f"{v[10]:+5.1f} {v[11]:+5.2f} "
            f"{v[12]:.2f}   {v[13]:.2f}    {v[14]:.2f}  {v[15]:+5.2f}"
        )


def write_csv_row(writer: csv.DictWriter, row: Dict[str, object]) -> None:
    safe = {field: row.get(field, "") for field in writer.fieldnames or []}
    writer.writerow(safe)


def build_csv_writer(path: str) -> Tuple[object, csv.DictWriter]:
    fields = [
        "host_time_s",
        "time_us",
        "seq",
        "frame_type",
        "leg_id",
        "flags",
        *LEG_FIELDS,
        *GLOBAL_FIELDS,
    ]
    handle = open(path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    return handle, writer


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Decode STM32 UART7 wheel-leg support debug frames."
    )
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM7 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--print-rate", type=float, default=5.0, help="Snapshot print rate in Hz")
    parser.add_argument("--csv", default=None, help="Optional CSV log path")
    parser.add_argument("--raw", action="store_true", help="Print raw hex instead of decoding")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args()

    if args.list_ports:
        for port in serial.tools.list_ports.comports():
            print(f"{port.device}\t{port.description}\t{port.hwid}")
        return 0

    port = args.port or auto_port()
    print(f"Opening {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=0.05)

    csv_handle = None
    csv_writer = None
    if args.csv:
        csv_handle, csv_writer = build_csv_writer(args.csv)
        print(f"CSV logging to {args.csv}")

    latest_legs: Dict[int, dict] = {}
    latest_global: Optional[dict] = None
    buffer = bytearray()
    last_print = 0.0
    print_period = 1.0 / max(args.print_rate, 0.1)

    try:
        while True:
            data = ser.read(max(1, ser.in_waiting))
            if not data:
                continue
            if args.raw:
                print(data.hex(" "))
                continue

            buffer.extend(data)
            for frame in pop_frames(buffer):
                if csv_writer is not None:
                    write_csv_row(csv_writer, frame_row(frame))
                if frame["frame_type"] == FRAME_TYPE_LEG and frame["leg_id"] in LEG_NAMES:
                    latest_legs[frame["leg_id"]] = frame
                elif frame["frame_type"] == FRAME_TYPE_GLOBAL:
                    latest_global = frame

            now = time.time()
            if now - last_print >= print_period:
                last_print = now
                print_snapshot(latest_legs, latest_global)
                if csv_handle is not None:
                    csv_handle.flush()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
        if csv_handle is not None:
            csv_handle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
