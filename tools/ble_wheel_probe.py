#!/usr/bin/env python3
"""
Windows BLE probe tool for RiderDriver.

This tool talks to the ESP32 BLE debug service directly from Windows, so we can
iterate on wheel-node reverse engineering without depending on the Android app.

Examples:
  python tools/ble_wheel_probe.py scan
  python tools/ble_wheel_probe.py send --command "WGET"
  python tools/ble_wheel_probe.py probe-wheel --modes I16 DXL --values -100 -40 -20 0 20 40 100
  python tools/ble_wheel_probe.py interactive
"""

from __future__ import annotations

import argparse
import asyncio
import builtins
import json
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Iterable, List, Optional, Sequence

try:
    from bleak import BleakClient, BleakScanner
except ImportError as exc:  # pragma: no cover - dependency check
    print(
        "bleak is required. Install it with `python -m pip install --user bleak`.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


DEVICE_NAME = "RiderDriver"
SERVICE_UUID = "fedcba98-7654-3210-fedc-ba9876543210"
COMMAND_UUID = "fedcba98-7654-3210-fedc-ba9876543211"
STATUS_UUID = "fedcba98-7654-3210-fedc-ba9876543212"


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


def host_ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def ensure_parent_dir(path: Optional[str]) -> None:
    if not path:
        return
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def parse_json_status(text: str) -> Optional[dict]:
    text = text.strip()
    if not text.startswith("{"):
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


def classify_probe_direction(payload: dict) -> str:
    total = int(payload.get("sum", 0))
    pos_steps = int(payload.get("pos", 0))
    neg_steps = int(payload.get("neg", 0))
    if total < 0 or (neg_steps > pos_steps and neg_steps > 0):
        return "reverse"
    if total > 0 or (pos_steps > neg_steps and pos_steps > 0):
        return "forward"
    return "still"


@dataclass
class ProbeResult:
    host_ts: str
    command: str
    status_text: str
    payload: Optional[dict]


class RiderBleProbe:
    def __init__(self, address: Optional[str], timeout: float, log_path: Optional[str]):
        self.address = address
        self.timeout = timeout
        self.log_path = log_path
        ensure_parent_dir(log_path)
        self.log_fp = open(log_path, "a", encoding="utf-8") if log_path else None
        self.client: Optional[BleakClient] = None
        self.status_queue: asyncio.Queue[str] = asyncio.Queue()
        self.last_status_text: str = ""

    def close(self) -> None:
        if self.log_fp:
            self.log_fp.close()

    def emit_event(self, event: dict) -> None:
        if self.log_fp:
            self.log_fp.write(json.dumps(event, ensure_ascii=False) + "\n")
            self.log_fp.flush()

    async def scan(self, timeout: Optional[float] = None) -> List[object]:
        devices = await BleakScanner.discover(timeout=timeout or self.timeout)
        return list(devices)

    async def resolve_address(self) -> str:
        if self.address:
            return self.address

        devices = await self.scan()
        for device in devices:
            if getattr(device, "name", None) == DEVICE_NAME:
                return device.address
        raise RuntimeError(f"Did not find {DEVICE_NAME} during BLE scan")

    async def connect(self) -> None:
        if self.client and self.client.is_connected:
            return

        address = await self.resolve_address()
        self.client = BleakClient(address, timeout=self.timeout)
        await self.client.connect()
        await self.client.start_notify(STATUS_UUID, self._handle_notification)
        print(f"[{host_ts()}] connected to {DEVICE_NAME} {address}")
        self.emit_event(
            {
                "host_ts": host_ts(),
                "kind": "connect",
                "device_name": DEVICE_NAME,
                "address": address,
            }
        )

    async def disconnect(self) -> None:
        client = self.client
        if not client:
            return
        try:
            if client.is_connected:
                try:
                    await client.stop_notify(STATUS_UUID)
                except Exception:
                    pass
                await client.disconnect()
        finally:
            self.client = None

    def _handle_notification(self, _: object, data: bytearray) -> None:
        text = bytes(data).decode("utf-8", errors="replace").strip()
        self.last_status_text = text
        self.status_queue.put_nowait(text)
        self.emit_event(
            {
                "host_ts": host_ts(),
                "kind": "notify",
                "status": text,
                "payload": parse_json_status(text),
            }
        )
        print(f"[{host_ts()}] notify {text}")

    async def read_status(self) -> str:
        client = self.client
        if not client or not client.is_connected:
            raise RuntimeError("BLE client is not connected")
        data = await client.read_gatt_char(STATUS_UUID)
        text = bytes(data).decode("utf-8", errors="replace").strip()
        self.last_status_text = text
        self.emit_event(
            {
                "host_ts": host_ts(),
                "kind": "read_status",
                "status": text,
                "payload": parse_json_status(text),
            }
        )
        print(f"[{host_ts()}] read {text}")
        return text

    async def wait_for_status(
        self,
        accepted_kinds: Optional[Sequence[str]] = None,
        timeout: float = 3.0,
    ) -> ProbeResult:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for BLE status")
            text = await asyncio.wait_for(self.status_queue.get(), timeout=remaining)
            payload = parse_json_status(text)
            if accepted_kinds is None:
                return ProbeResult(host_ts(), "", text, payload)
            if payload and payload.get("kind") in accepted_kinds:
                return ProbeResult(host_ts(), "", text, payload)

    async def send_command(
        self,
        command: str,
        accepted_kinds: Optional[Sequence[str]] = None,
        wait_timeout: float = 3.0,
        settle_after_write: float = 0.2,
    ) -> ProbeResult:
        client = self.client
        if not client or not client.is_connected:
            raise RuntimeError("BLE client is not connected")

        while not self.status_queue.empty():
            try:
                self.status_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

        self.emit_event(
            {
                "host_ts": host_ts(),
                "kind": "command",
                "command": command,
            }
        )
        print(f"[{host_ts()}] send {command}")
        await client.write_gatt_char(COMMAND_UUID, command.encode("utf-8"), response=True)
        await asyncio.sleep(settle_after_write)

        try:
            result = await self.wait_for_status(accepted_kinds=accepted_kinds, timeout=wait_timeout)
            result.command = command
            return result
        except TimeoutError:
            text = await self.read_status()
            payload = parse_json_status(text)
            if accepted_kinds is not None and payload and payload.get("kind") not in accepted_kinds:
                raise
            return ProbeResult(host_ts(), command, text, payload)


async def run_scan(args: argparse.Namespace) -> int:
    probe = RiderBleProbe(address=args.address, timeout=args.timeout, log_path=args.log)
    try:
        devices = await probe.scan()
        for device in devices:
            print(f"{getattr(device, 'name', None)!r} {device.address}")
        return 0
    finally:
        probe.close()


async def run_send(args: argparse.Namespace) -> int:
    probe = RiderBleProbe(address=args.address, timeout=args.timeout, log_path=args.log)
    try:
        await probe.connect()
        accepted = args.accept_kind or None
        result = await probe.send_command(
            args.command,
            accepted_kinds=accepted,
            wait_timeout=args.wait_timeout,
            settle_after_write=args.settle_after_write,
        )
        print(result.status_text)
        return 0
    finally:
        await probe.disconnect()
        probe.close()


async def run_probe_wheel(args: argparse.Namespace) -> int:
    probe = RiderBleProbe(address=args.address, timeout=args.timeout, log_path=args.log)
    summary: Dict[str, List[dict]] = {}
    try:
        await probe.connect()
        for wheel_id in args.ids:
            key = f"id{wheel_id}"
            summary[key] = []
            for mode in args.modes:
                for value in args.values:
                    command = (
                        f"WPROBE {mode} {wheel_id} {value} "
                        f"{args.settle_ms} {args.samples} {args.interval_ms}"
                    )
                    result = await probe.send_command(
                        command,
                        accepted_kinds=["wprobe"],
                        wait_timeout=args.wait_timeout,
                        settle_after_write=args.settle_after_write,
                    )
                    payload = result.payload or {}
                    direction = classify_probe_direction(payload) if payload else "unknown"
                    row = {
                        "mode": mode,
                        "value": value,
                        "direction": direction,
                        "sum": payload.get("sum"),
                        "pos": payload.get("pos"),
                        "neg": payload.get("neg"),
                        "raw": payload.get("raw"),
                        "max_s": payload.get("max_s"),
                    }
                    summary[key].append(row)
                    print(
                        "[{ts}] {cmd} -> direction={direction} sum={sum_} raw={raw} max_s={max_s}".format(
                            ts=host_ts(),
                            cmd=command,
                            direction=direction,
                            sum_=row["sum"],
                            raw=row["raw"],
                            max_s=row["max_s"],
                        )
                    )
                    await asyncio.sleep(args.cooldown_ms / 1000.0)

        print("\n=== Wheel Probe Summary ===")
        for key, rows in summary.items():
            print(key)
            for row in rows:
                print(
                    "  {mode:>3} {value:>6} -> {direction:<7} sum={sum!s:<6} raw={raw!s:<6} max_s={max_s!s}".format(
                        **row
                    )
                )
        return 0
    finally:
        await probe.disconnect()
        probe.close()


async def run_interactive(args: argparse.Namespace) -> int:
    probe = RiderBleProbe(address=args.address, timeout=args.timeout, log_path=args.log)
    try:
        await probe.connect()
        print("Interactive mode. Type BLE commands such as GET, WGET, WPROBE I16 11 -40 120 10 80.")
        print("Type quit to exit.")
        while True:
            line = await asyncio.to_thread(input, "ble> ")
            command = line.strip()
            if not command:
                continue
            if command.lower() in {"quit", "exit"}:
                break
            kinds = ["wprobe"] if command.upper().startswith("WPROBE ") else None
            try:
                result = await probe.send_command(
                    command,
                    accepted_kinds=kinds,
                    wait_timeout=args.wait_timeout,
                    settle_after_write=args.settle_after_write,
                )
                print(result.status_text)
            except Exception as exc:
                print(f"command failed: {exc}")
        return 0
    finally:
        await probe.disconnect()
        probe.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Windows BLE probe for RiderDriver")
    parser.add_argument("--address", help="BLE MAC address override")
    parser.add_argument("--timeout", type=float, default=8.0, help="BLE connect/scan timeout seconds")
    parser.add_argument("--log", default="logs/ble_wheel_probe_events.jsonl", help="JSONL event log path")

    sub = parser.add_subparsers(dest="cmd", required=True)

    scan = sub.add_parser("scan", help="Scan for BLE devices")
    scan.set_defaults(func=run_scan)

    send = sub.add_parser("send", help="Send one BLE command")
    send.add_argument("--command", required=True, help="Command text, e.g. WGET")
    send.add_argument("--accept-kind", action="append", help="Expected JSON kind, can repeat")
    send.add_argument("--wait-timeout", type=float, default=4.0, help="Seconds to wait for status")
    send.add_argument("--settle-after-write", type=float, default=0.25, help="Delay after write before waiting")
    send.set_defaults(func=run_send)

    probe_wheel = sub.add_parser("probe-wheel", help="Batch probe wheel direction encodings")
    probe_wheel.add_argument("--ids", type=int, nargs="+", default=[11, 21], help="Wheel ids to test")
    probe_wheel.add_argument("--modes", nargs="+", default=["I16", "DXL"], help="Probe modes, e.g. I16 DXL RAW")
    probe_wheel.add_argument(
        "--values",
        type=int,
        nargs="+",
        default=[-180, -100, -60, -40, -30, -20, 0, 20, 30, 40, 60, 100, 180],
        help="Signed test values",
    )
    probe_wheel.add_argument("--settle-ms", type=int, default=120, help="Probe settle ms")
    probe_wheel.add_argument("--samples", type=int, default=10, help="Probe sample count")
    probe_wheel.add_argument("--interval-ms", type=int, default=80, help="Probe sample interval ms")
    probe_wheel.add_argument("--cooldown-ms", type=int, default=350, help="Pause between probes")
    probe_wheel.add_argument("--wait-timeout", type=float, default=5.0, help="Seconds to wait for probe result")
    probe_wheel.add_argument("--settle-after-write", type=float, default=0.15, help="Delay after write before waiting")
    probe_wheel.set_defaults(func=run_probe_wheel)

    interactive = sub.add_parser("interactive", help="Interactive BLE shell")
    interactive.add_argument("--wait-timeout", type=float, default=5.0, help="Seconds to wait for status")
    interactive.add_argument("--settle-after-write", type=float, default=0.25, help="Delay after write before waiting")
    interactive.set_defaults(func=run_interactive)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return asyncio.run(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
