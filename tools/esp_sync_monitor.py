#!/usr/bin/env python3
"""
ESP32 serial sync monitor for rider_driver.

This tool tails the ESP32 log console, keeps a small live state model, and
extracts structured events that are useful when reverse-engineering how the
stock firmware drives motors.

Typical usage:
  python tools/esp_sync_monitor.py --port COM3
  python tools/esp_sync_monitor.py --port COM3 --raw-log logs\\esp_raw.log --event-log logs\\esp_events.jsonl
  python tools/esp_sync_monitor.py --replay logs\\esp_raw.log
"""

from __future__ import annotations

import argparse
import builtins
import json
import os
import re
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Deque, Dict, Iterable, Iterator, List, Optional, Tuple

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError as exc:  # pragma: no cover - user-facing dependency check
    print(
        "pyserial is required. Install it with `pip install pyserial` and retry.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


def print(*args, **kwargs):  # type: ignore[no-redef]
    try:
        builtins.print(*args, **kwargs)
    except UnicodeEncodeError:
        sep = kwargs.get("sep", " ")
        end = kwargs.get("end", "\n")
        file = kwargs.get("file", sys.stdout)
        flush = kwargs.get("flush", False)
        text = sep.join(str(arg) for arg in args) + end
        encoding = getattr(file, "encoding", None) or "utf-8"
        buffer = getattr(file, "buffer", None)
        if buffer is not None:
            buffer.write(text.encode(encoding, errors="backslashreplace"))
            if flush:
                file.flush()
            return
        builtins.print(
            text.encode(encoding, errors="backslashreplace").decode(encoding, errors="replace"),
            end="",
            file=file,
            flush=flush,
        )


ESP_LOG_RE = re.compile(
    r"^(?P<level>[IWEVD])\s+\((?P<tick>\d+)\)\s+(?P<tag>[^:]+):\s*(?P<msg>.*)$"
)
HEX_PAYLOAD_RE = re.compile(
    r"^(?P<label>TX|RX|ECHO_FLUSH|RX_NO_HDR|RX_ECHO_LEAK)\s+\((?P<count>\d+)\):\s+(?P<hex>[0-9A-Fa-f ]+)(?P<more>\.\.\.)?$"
)
UART_APP_RAW_RE = re.compile(
    r"^\[bus(?P<bus>\d+)\]\s+RAW\s+(?P<count>\d+)\s+bytes:\s+(?P<hex>[0-9A-Fa-f ]+)(?P<more>\.\.\.)?$"
)
UART_PROTOCOL_READ_RE = re.compile(
    r"^\[bus(?P<bus>\d+)\]\s+READ\s+addr=0x(?P<addr>[0-9A-Fa-f]{2})\s+len=(?P<read_len>\d+)$"
)
UART_PROTOCOL_WRITE_RE = re.compile(
    r"^\[bus(?P<bus>\d+)\]\s+WRITE\s+addr=0x(?P<addr>[0-9A-Fa-f]{2})\s+data_len=(?P<data_len>\d+)$"
)
PROFILE_RE = re.compile(r"selected profile:\s+(?P<profile>.+)$")
PROFILE_SCORE_RE = re.compile(r"profile score:\s+SCSCL=(?P<scscl>-?\d+)\s+legacy=(?P<legacy>-?\d+)")
TORQUE_RE = re.compile(
    r"ID\s+(?P<id>\d+):\s+torque\s+(?P<state>ON|OFF)\s+OK\s+\(addr=(?P<addr>\d+),\s+profile=(?P<profile>.+)\)"
)
START_TARGET_RE = re.compile(
    r"ID\s+(?P<id>\d+)\s+start=(?P<start>\d+)\s+target=(?P<target>\d+)\s+speed=(?P<speed>\d+)"
)
MOVE_OUT_BACK_RE = re.compile(r"ID\s+(?P<id>\d+)\s+move\s+(?P<phase>out|back):\s+(?P<result>[A-Z_]+)")
MID_FINAL_RE = re.compile(
    r"ID\s+(?P<id>\d+)\s+(?P<phase>mid|final)\s+pos=(?P<pos>\d+)\s+load=(?P<load>\d+)\s+moving=(?P<moving>\d+)"
)
BEFORE_AFTER_RE = re.compile(
    r"(?P<phase>before|after):\s+pos=(?P<pos>\d+)\s+\(0x(?P<pos_hex>[0-9A-Fa-f]+)\)(?:\s+from\s+addr=(?P<addr>\d+))?"
)
SIMPLE_TUNER_RE = re.compile(
    r"ID\s+(?P<id>\d+)\s+start=(?P<start>\d+)\s+goal=(?P<goal>\d+)\s+delta=(?P<delta>-?\d+)\s+speed=(?P<speed>\d+)"
)
STATUS_PARAMS_RE = re.compile(
    r"(?P<label>[A-Z0-9_() ,]+)\s+ID=(?P<id>\d+)\s+err=0x(?P<error>[0-9A-Fa-f]{2})\s+params\((?P<count>\d+)\):\s*(?P<hex>[0-9A-Fa-f ]*)"
)
KEY_VALUE_RE = re.compile(r"^(?P<key>[A-Z_]+)=(?P<value>\d+)$")
LED_RE = re.compile(r"^LED\[(?P<idx>\d+)\]\s+R=(?P<r>\d+)\s+G=(?P<g>\d+)\s+B=(?P<b>\d+)$")
ACTION_RE = re.compile(r"^ACTION=(?P<value>RESET|\d+)$")


PROTO1_INST_NAMES = {
    0x01: "PING",
    0x02: "READ",
    0x03: "WRITE",
    0x04: "REG_WRITE",
    0x05: "ACTION",
    0x83: "SYNC_WRITE",
}

XGO_ADDR_NAMES = {
    0x01: "BATTERY",
    0x03: "PERFORM",
    0x04: "CALIBRATION",
    0x05: "UPGRADE",
    0x06: "SET_ORIGIN",
    0x07: "FIRMWARE_VERSION",
    0x09: "GAIT_TYPE",
    0x20: "LOAD_UNLOAD_MOTOR",
    0x30: "VX",
    0x31: "VY",
    0x32: "VYAW",
    0x35: "TRANSLATION_Z",
    0x36: "ATTITUDE_R",
    0x39: "PERIODIC_ROT_R",
    0x3E: "ACTION",
    0x50: "MOTOR_ANGLE",
    0x5C: "MOTOR_SPEED",
    0x61: "IMU_BALANCE",
    0x62: "ROLL",
    0x63: "PITCH",
    0x64: "YAW",
    0x65: "IMU_BLOCK",
    0x66: "IMU_ROLL_I16",
    0x67: "IMU_PITCH_I16",
    0x68: "IMU_YAW_I16_OR_LED_BASE",
    0x69: "LED_1",
    0x6A: "LED_2",
    0x6B: "LED_3",
    0x6C: "LED_4",
    0x71: "CLAW",
    0x73: "ARM_X",
    0x74: "ARM_Z",
    0x75: "ARM_SPEED",
    0x80: "PERIODIC_TRAN_X",
    0x81: "PERIODIC_TRAN_Y",
    0x82: "PERIODIC_TRAN_Z",
}


@dataclass
class Proto1Request:
    device: str
    inst: int
    inst_name: str
    target_id: int
    addr: Optional[int] = None
    read_len: Optional[int] = None
    data: List[int] = field(default_factory=list)
    detail: str = ""


@dataclass
class MotorState:
    motor_id: int
    profile: Optional[str] = None
    torque_enabled: Optional[bool] = None
    last_position: Optional[int] = None
    last_speed: Optional[int] = None
    last_load: Optional[int] = None
    moving: Optional[int] = None
    last_goal: Optional[int] = None
    last_goal_speed: Optional[int] = None
    last_voltage: Optional[int] = None
    last_temperature: Optional[int] = None
    last_current: Optional[int] = None
    last_seen_host_ts: Optional[str] = None


def host_ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def hex_to_bytes(hex_text: str) -> bytes:
    cleaned = "".join(hex_text.split())
    if not cleaned:
        return b""
    return bytes.fromhex(cleaned)


def le_u16(data: List[int]) -> int:
    if len(data) < 2:
        return 0
    return data[0] | (data[1] << 8)


def auto_pick_port() -> Optional[str]:
    ports = list(list_ports.comports())
    if not ports:
        return None

    preferred_terms = ("cp210", "ch340", "usb", "uart", "esp", "silicon labs")
    for port in ports:
        hay = " ".join(
            filter(
                None,
                [
                    getattr(port, "device", ""),
                    getattr(port, "description", ""),
                    getattr(port, "manufacturer", ""),
                ],
            )
        ).lower()
        if any(term in hay for term in preferred_terms):
            return port.device

    if len(ports) == 1:
        return ports[0].device
    return None


def decode_proto1_frame(frame: bytes) -> Optional[dict]:
    if len(frame) < 6 or frame[0:2] != b"\xFF\xFF":
        return None

    target_id = frame[2]
    length_field = frame[3]
    if len(frame) < 4 + length_field:
        return None

    if len(frame) == 4 + length_field and length_field >= 2:
        opcode = frame[4]
        params = list(frame[5:-1])
        checksum = frame[-1]
        out = {
            "target_id": target_id,
            "length": length_field,
            "opcode": opcode,
            "opcode_name": PROTO1_INST_NAMES.get(opcode, f"0x{opcode:02X}"),
            "params": params,
            "checksum": checksum,
        }
        if opcode == 0x02 and len(params) >= 2:
            out["addr"] = params[0]
            out["read_len"] = params[1]
        elif opcode == 0x03 and len(params) >= 1:
            out["addr"] = params[0]
            out["data"] = params[1:]
        elif opcode == 0x83 and len(params) >= 2:
            start_addr = params[0]
            data_len = params[1]
            out["addr"] = start_addr
            out["data_len"] = data_len
            records = []
            record_width = 1 + data_len
            records_raw = params[2:]
            for i in range(0, len(records_raw), record_width):
                chunk = records_raw[i : i + record_width]
                if len(chunk) != record_width:
                    break
                record = {"target_id": chunk[0], "data": chunk[1:]}
                if start_addr == 0x1E and data_len >= 4:
                    record["goal_position"] = chunk[1] | (chunk[2] << 8)
                    record["goal_speed"] = chunk[3] | (chunk[4] << 8)
                records.append(record)
            out["records"] = records
        return out

    error = frame[4]
    params = list(frame[5:-1])
    checksum = frame[-1]
    return {
        "target_id": target_id,
        "length": length_field,
        "error": error,
        "params": params,
        "checksum": checksum,
        "is_status": True,
    }


def decode_xgo_frame(frame: bytes) -> Optional[dict]:
    if len(frame) < 8 or frame[0:2] != b"\x55\x00" or frame[-2:] != b"\x00\xAA":
        return None
    length = frame[2]
    if length != len(frame):
        return None
    frame_type = frame[3]
    addr = frame[4]
    checksum = frame[-3]
    data = list(frame[5:-3])
    out = {
        "length": length,
        "type": frame_type,
        "type_name": {0x01: "WRITE", 0x02: "READ"}.get(frame_type, f"0x{frame_type:02X}"),
        "addr": addr,
        "addr_name": XGO_ADDR_NAMES.get(addr, f"0x{addr:02X}"),
        "data": data,
        "checksum": checksum,
    }
    if frame_type == 0x02 and data:
        out["read_len"] = data[0]
    return out


class SyncMonitor:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.motors: Dict[int, MotorState] = {}
        self.pending_proto1: Deque[Proto1Request] = deque()
        self._ensure_parent_dir(args.raw_log)
        self._ensure_parent_dir(args.event_log)
        self.raw_log_fp = open(args.raw_log, "a", encoding="utf-8") if args.raw_log else None
        self.event_log_fp = open(args.event_log, "a", encoding="utf-8") if args.event_log else None
        self.next_snapshot_at = time.monotonic() + args.summary_every

    @staticmethod
    def _ensure_parent_dir(path: Optional[str]) -> None:
        if not path:
            return
        parent = os.path.dirname(os.path.abspath(path))
        if parent:
            os.makedirs(parent, exist_ok=True)

    def close(self) -> None:
        if self.raw_log_fp:
            self.raw_log_fp.close()
        if self.event_log_fp:
            self.event_log_fp.close()

    def get_motor(self, motor_id: int) -> MotorState:
        state = self.motors.get(motor_id)
        if state is None:
            state = MotorState(motor_id=motor_id)
            self.motors[motor_id] = state
        state.last_seen_host_ts = host_ts()
        return state

    def emit_event(self, event: dict) -> None:
        if self.event_log_fp:
            self.event_log_fp.write(json.dumps(event, ensure_ascii=True) + "\n")
            self.event_log_fp.flush()

    def write_raw(self, source: str, line: str) -> None:
        if self.raw_log_fp:
            self.raw_log_fp.write(f"{host_ts()} [{source}] {line}\n")
            self.raw_log_fp.flush()

    def maybe_snapshot(self) -> None:
        if time.monotonic() < self.next_snapshot_at:
            return
        self.next_snapshot_at = time.monotonic() + self.args.summary_every
        if not self.motors:
            print(f"[{host_ts()}] SNAPSHOT no motor state yet")
            return
        parts = []
        for motor_id in sorted(self.motors):
            m = self.motors[motor_id]
            parts.append(
                "ID{mid}(profile={profile},torque={torque},pos={pos},spd={spd},load={load},moving={moving},goal={goal},goal_spd={goal_spd})".format(
                    mid=motor_id,
                    profile=m.profile or "?",
                    torque=("on" if m.torque_enabled else "off") if m.torque_enabled is not None else "?",
                    pos=m.last_position if m.last_position is not None else "?",
                    spd=m.last_speed if m.last_speed is not None else "?",
                    load=m.last_load if m.last_load is not None else "?",
                    moving=m.moving if m.moving is not None else "?",
                    goal=m.last_goal if m.last_goal is not None else "?",
                    goal_spd=m.last_goal_speed if m.last_goal_speed is not None else "?",
                )
            )
        print(f"[{host_ts()}] SNAPSHOT {'; '.join(parts)}")

    def handle_proto1_tx(self, device: str, decoded: dict) -> None:
        inst = decoded["opcode"]
        req = Proto1Request(
            device=device,
            inst=inst,
            inst_name=decoded["opcode_name"],
            target_id=decoded["target_id"],
            addr=decoded.get("addr"),
            read_len=decoded.get("read_len"),
            data=decoded.get("data", []),
            detail="",
        )

        if inst == 0x03 and decoded.get("addr") == 0x18 and decoded.get("data"):
            motor = self.get_motor(decoded["target_id"])
            motor.torque_enabled = bool(decoded["data"][0])
            req.detail = f"torque={'on' if motor.torque_enabled else 'off'}"
        elif inst == 0x03 and decoded.get("addr") == 0x1E and len(decoded.get("data", [])) >= 4:
            motor = self.get_motor(decoded["target_id"])
            data = decoded["data"]
            motor.last_goal = data[0] | (data[1] << 8)
            motor.last_goal_speed = data[2] | (data[3] << 8)
            req.detail = f"goal={motor.last_goal} speed={motor.last_goal_speed}"
        elif inst == 0x83:
            for record in decoded.get("records", []):
                motor = self.get_motor(record["target_id"])
                if "goal_position" in record:
                    motor.last_goal = record["goal_position"]
                if "goal_speed" in record:
                    motor.last_goal_speed = record["goal_speed"]
            req.detail = f"sync_write_records={len(decoded.get('records', []))}"

        if decoded["target_id"] != 0xFE:
            self.pending_proto1.append(req)

        summary = f"{device} TX {decoded['opcode_name']} id={decoded['target_id']}"
        if decoded.get("addr") is not None:
            summary += f" addr=0x{decoded['addr']:02X}"
        if req.detail:
            summary += f" {req.detail}"
        print(f"[{host_ts()}] {summary}")
        self.emit_event(
            {
                "host_ts": host_ts(),
                "kind": "proto1_tx",
                "device": device,
                "summary": summary,
                "decoded": decoded,
            }
        )

    def handle_proto1_rx(self, device: str, decoded: dict) -> None:
        matched: Optional[Proto1Request] = None
        for req in list(self.pending_proto1):
            if req.target_id == decoded["target_id"]:
                matched = req
                self.pending_proto1.remove(req)
                break

        summary = f"{device} RX status id={decoded['target_id']} error=0x{decoded['error']:02X}"
        data = decoded.get("params", [])
        if matched and matched.inst == 0x02 and matched.addr == 0x24 and len(data) >= 6:
            motor = self.get_motor(decoded["target_id"])
            motor.last_position = le_u16(data[0:2])
            motor.last_speed = le_u16(data[2:4])
            motor.last_load = le_u16(data[4:6])
            summary += f" pos={motor.last_position} speed={motor.last_speed} load={motor.last_load}"
        elif matched and matched.inst == 0x02 and matched.addr in {0x2A, 0x2B, 0x2E} and data:
            motor = self.get_motor(decoded["target_id"])
            if matched.addr == 0x2A:
                motor.last_voltage = data[0]
                summary += f" voltage={data[0]}"
            elif matched.addr == 0x2B:
                motor.last_temperature = data[0]
                summary += f" temperature={data[0]}"
            elif matched.addr == 0x2E:
                motor.moving = data[0]
                summary += f" moving={data[0]}"
        elif matched and matched.inst == 0x02 and matched.addr is not None:
            summary += f" from READ addr=0x{matched.addr:02X} params={data}"
        elif data:
            summary += f" params={data}"

        print(f"[{host_ts()}] {summary}")
        self.emit_event(
            {
                "host_ts": host_ts(),
                "kind": "proto1_rx",
                "device": device,
                "summary": summary,
                "decoded": decoded,
                "matched_request": asdict(matched) if matched else None,
            }
        )

    def parse_component_message(self, tag: str, msg: str) -> bool:
        if tag in {"scs", "dxl"}:
            m = HEX_PAYLOAD_RE.match(msg)
            if not m:
                return False
            frame = hex_to_bytes(m.group("hex"))
            decoded = decode_proto1_frame(frame)
            if decoded is None:
                print(f"[{host_ts()}] {tag} {m.group('label')} raw={frame.hex(' ')}")
                return True
            if decoded.get("is_status"):
                self.handle_proto1_rx(tag, decoded)
            else:
                self.handle_proto1_tx(tag, decoded)
            return True

        if tag == "uart_app":
            m = UART_APP_RAW_RE.match(msg)
            if not m:
                return False
            bus_id = int(m.group("bus"))
            frame = hex_to_bytes(m.group("hex"))
            xgo = decode_xgo_frame(frame)
            summary = f"uart_app bus{bus_id} RAW {len(frame)}B"
            event = {
                "host_ts": host_ts(),
                "kind": "uart_app_raw",
                "bus_id": bus_id,
                "raw_hex": frame.hex(" "),
            }
            if xgo:
                summary += f" {xgo['type_name']} {xgo['addr_name']}"
                if xgo["type_name"] == "READ" and xgo.get("read_len") is not None:
                    summary += f" read_len={xgo['read_len']}"
                elif xgo["data"]:
                    summary += f" data={xgo['data']}"
                event["decoded"] = xgo
            print(f"[{host_ts()}] {summary}")
            self.emit_event(event)
            return True

        if tag == "uart_protocol":
            read_match = UART_PROTOCOL_READ_RE.match(msg)
            if read_match:
                addr = int(read_match.group("addr"), 16)
                read_len = int(read_match.group("read_len"))
                summary = f"uart_protocol bus{read_match.group('bus')} READ {XGO_ADDR_NAMES.get(addr, f'0x{addr:02X}')} len={read_len}"
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "uart_protocol_read",
                        "bus_id": int(read_match.group("bus")),
                        "addr": addr,
                        "addr_name": XGO_ADDR_NAMES.get(addr, f"0x{addr:02X}"),
                        "read_len": read_len,
                    }
                )
                return True

            write_match = UART_PROTOCOL_WRITE_RE.match(msg)
            if write_match:
                addr = int(write_match.group("addr"), 16)
                data_len = int(write_match.group("data_len"))
                summary = f"uart_protocol bus{write_match.group('bus')} WRITE {XGO_ADDR_NAMES.get(addr, f'0x{addr:02X}')} data_len={data_len}"
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "uart_protocol_write",
                        "bus_id": int(write_match.group("bus")),
                        "addr": addr,
                        "addr_name": XGO_ADDR_NAMES.get(addr, f"0x{addr:02X}"),
                        "data_len": data_len,
                    }
                )
                return True

            kv = KEY_VALUE_RE.match(msg)
            if kv:
                summary = f"uart_protocol {kv.group('key')}={kv.group('value')}"
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "uart_protocol_value",
                        "key": kv.group("key"),
                        "value": int(kv.group("value")),
                    }
                )
                return True

            led = LED_RE.match(msg)
            if led:
                summary = "uart_protocol LED[{idx}] rgb=({r},{g},{b})".format(
                    idx=led.group("idx"),
                    r=led.group("r"),
                    g=led.group("g"),
                    b=led.group("b"),
                )
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "uart_protocol_led",
                        "index": int(led.group("idx")),
                        "r": int(led.group("r")),
                        "g": int(led.group("g")),
                        "b": int(led.group("b")),
                    }
                )
                return True

            action = ACTION_RE.match(msg)
            if action:
                summary = f"uart_protocol ACTION={action.group('value')}"
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "uart_protocol_action",
                        "value": action.group("value"),
                    }
                )
                return True

            return False

        if tag in {"motor_test", "motor_tuner"}:
            match = PROFILE_RE.search(msg)
            if match:
                profile = match.group("profile").strip()
                print(f"[{host_ts()}] {tag} selected profile {profile}")
                self.emit_event(
                    {"host_ts": host_ts(), "kind": "profile", "source": tag, "profile": profile}
                )
                return True

            match = PROFILE_SCORE_RE.search(msg)
            if match:
                print(
                    f"[{host_ts()}] {tag} profile_score scscl={match.group('scscl')} legacy={match.group('legacy')}"
                )
                return True

            match = TORQUE_RE.search(msg)
            if match:
                motor = self.get_motor(int(match.group("id")))
                motor.profile = match.group("profile").strip()
                motor.torque_enabled = match.group("state") == "ON"
                summary = f"{tag} ID{motor.motor_id} torque={'on' if motor.torque_enabled else 'off'} profile={motor.profile}"
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "motor_torque",
                        "source": tag,
                        "motor_id": motor.motor_id,
                        "profile": motor.profile,
                        "torque_enabled": motor.torque_enabled,
                    }
                )
                return True

            match = START_TARGET_RE.search(msg)
            if match:
                motor = self.get_motor(int(match.group("id")))
                motor.last_position = int(match.group("start"))
                motor.last_goal = int(match.group("target"))
                motor.last_goal_speed = int(match.group("speed"))
                summary = (
                    f"{tag} ID{motor.motor_id} start={motor.last_position} "
                    f"target={motor.last_goal} speed={motor.last_goal_speed}"
                )
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "motor_goal",
                        "source": tag,
                        "motor_id": motor.motor_id,
                        "start": motor.last_position,
                        "target": motor.last_goal,
                        "speed": motor.last_goal_speed,
                    }
                )
                return True

            match = SIMPLE_TUNER_RE.search(msg)
            if match:
                motor = self.get_motor(int(match.group("id")))
                motor.last_position = int(match.group("start"))
                motor.last_goal = int(match.group("goal"))
                motor.last_goal_speed = int(match.group("speed"))
                summary = (
                    f"{tag} ID{motor.motor_id} start={motor.last_position} "
                    f"goal={motor.last_goal} delta={match.group('delta')} speed={motor.last_goal_speed}"
                )
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "motor_goal",
                        "source": tag,
                        "motor_id": motor.motor_id,
                        "start": motor.last_position,
                        "goal": motor.last_goal,
                        "delta": int(match.group("delta")),
                        "speed": motor.last_goal_speed,
                    }
                )
                return True

            match = MID_FINAL_RE.search(msg)
            if match:
                motor = self.get_motor(int(match.group("id")))
                motor.last_position = int(match.group("pos"))
                motor.last_load = int(match.group("load"))
                motor.moving = int(match.group("moving"))
                summary = (
                    f"{tag} ID{motor.motor_id} {match.group('phase')} pos={motor.last_position} "
                    f"load={motor.last_load} moving={motor.moving}"
                )
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "motor_feedback",
                        "source": tag,
                        "phase": match.group("phase"),
                        "motor_id": motor.motor_id,
                        "position": motor.last_position,
                        "load": motor.last_load,
                        "moving": motor.moving,
                    }
                )
                return True

            match = BEFORE_AFTER_RE.search(msg)
            if match:
                addr = match.group("addr")
                summary = f"{tag} {match.group('phase')} pos={match.group('pos')}"
                if addr:
                    summary += f" addr={addr}"
                print(f"[{host_ts()}] {summary}")
                return True

            match = STATUS_PARAMS_RE.search(msg)
            if match:
                params = list(hex_to_bytes(match.group("hex")))
                summary = (
                    f"{tag} {match.group('label').strip()} ID{match.group('id')} "
                    f"err=0x{match.group('error')} params={params}"
                )
                print(f"[{host_ts()}] {summary}")
                self.emit_event(
                    {
                        "host_ts": host_ts(),
                        "kind": "status_params",
                        "source": tag,
                        "label": match.group("label").strip(),
                        "motor_id": int(match.group("id")),
                        "error": int(match.group("error"), 16),
                        "params": params,
                    }
                )
                return True

            return False

        return False

    def handle_line(self, source: str, line: str) -> None:
        line = line.rstrip("\r\n")
        if not line:
            return
        self.write_raw(source, line)

        m = ESP_LOG_RE.match(line)
        if not m:
            if not self.args.quiet_unparsed:
                print(f"[{host_ts()}] RAW {line}")
            return

        parsed = self.parse_component_message(m.group("tag"), m.group("msg"))
        if not parsed and not self.args.quiet_unparsed:
            print(f"[{host_ts()}] {m.group('tag')}: {m.group('msg')}")

    def run_serial(self, port: str, baud: int) -> None:
        print(f"[{host_ts()}] opening serial port {port} @ {baud}")
        ser = serial.Serial(port, baud, timeout=0.2)
        try:
            while True:
                raw = ser.readline()
                if raw:
                    try:
                        line = raw.decode("utf-8", errors="replace")
                    except Exception:
                        line = raw.decode(errors="replace")
                    self.handle_line("serial", line)
                self.maybe_snapshot()
        finally:
            ser.close()

    def run_replay(self, path: str) -> None:
        print(f"[{host_ts()}] replaying log file {path}")
        with open(path, "r", encoding="utf-8", errors="replace") as fp:
            for line in fp:
                self.handle_line("replay", line)
                self.maybe_snapshot()
        self.maybe_snapshot()


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tail ESP32 serial logs and extract structured motor/UART events."
    )
    parser.add_argument("--port", help="Serial port, for example COM3.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate. Default: 115200")
    parser.add_argument(
        "--replay",
        help="Replay a previously captured text log instead of opening a serial port.",
    )
    parser.add_argument("--raw-log", help="Append raw lines to this file.")
    parser.add_argument("--event-log", help="Append structured events as JSONL to this file.")
    parser.add_argument(
        "--summary-every",
        type=float,
        default=15.0,
        help="Print a compact motor snapshot every N seconds. Default: 15",
    )
    parser.add_argument(
        "--quiet-unparsed",
        action="store_true",
        help="Hide lines that do not match a known parser rule.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    monitor = SyncMonitor(args)
    try:
        if args.replay:
            monitor.run_replay(args.replay)
            return 0

        port = args.port or auto_pick_port()
        if not port:
            print(
                "No serial port selected. Pass --port COMx explicitly, or connect a single ESP32 serial device.",
                file=sys.stderr,
            )
            return 2

        monitor.run_serial(port, args.baud)
        return 0
    except KeyboardInterrupt:
        print(f"\n[{host_ts()}] stopped by user")
        return 0
    finally:
        monitor.close()


if __name__ == "__main__":
    raise SystemExit(main())
