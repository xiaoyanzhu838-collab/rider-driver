#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import json
import queue
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:
    from bleak import BleakClient, BleakScanner
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "缺少 bleak，请先运行: python -m pip install --user bleak"
    ) from exc


DEVICE_NAME = "RiderDriver"
SERVICE_UUID = "fedcba98-7654-3210-fedc-ba9876543210"
COMMAND_UUID = "fedcba98-7654-3210-fedc-ba9876543211"
STATUS_UUID = "fedcba98-7654-3210-fedc-ba9876543212"


def parse_json_status(text: str) -> Optional[dict]:
    text = text.strip()
    if not text.startswith("{"):
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


@dataclass
class DeviceInfo:
    name: str
    address: str
    rssi: int


class BleBridge:
    def __init__(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._event_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self._status_queue: Optional[asyncio.Queue[Tuple[str, Optional[dict]]]] = None
        self._client: Optional[BleakClient] = None
        self._thread.start()

    @property
    def event_queue(self) -> "queue.Queue[Tuple[str, Any]]":
        return self._event_queue

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._status_queue = asyncio.Queue()
        self._loop.run_forever()

    def submit(self, coro: asyncio.Future) -> "asyncio.Future[Any]":
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    async def scan(self, timeout: float = 5.0) -> List[DeviceInfo]:
        devices = await BleakScanner.discover(timeout=timeout)
        result: List[DeviceInfo] = []
        for item in devices:
            result.append(
                DeviceInfo(
                    name=getattr(item, "name", "") or "",
                    address=item.address,
                    rssi=getattr(item, "rssi", 0) or 0,
                )
            )
        return result

    async def connect(self, address: str, timeout: float = 8.0) -> str:
        await self.disconnect()
        client = BleakClient(address, timeout=timeout)
        await client.connect()
        self._client = client
        await self._ensure_gatt_ready()
        await client.start_notify(STATUS_UUID, self._handle_notification)
        self._event_queue.put(("connected", address))
        return address

    async def disconnect(self) -> None:
        client = self._client
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
            self._client = None
            self._event_queue.put(("disconnected", None))

    async def _ensure_gatt_ready(self) -> None:
        client = self._client
        if not client:
            raise RuntimeError("BLE 客户端未创建")

        last_error: Optional[Exception] = None
        for attempt in range(1, 6):
            try:
                get_services = getattr(client, "get_services", None)
                if callable(get_services):
                    await get_services()
                services = client.services
                if services is not None:
                    service = services.get_service(SERVICE_UUID)
                    if service is not None:
                        chars = {str(char.uuid).lower() for char in service.characteristics}
                        if COMMAND_UUID.lower() in chars and STATUS_UUID.lower() in chars:
                            return
                raise RuntimeError("未发现所需 BLE 特征")
            except Exception as exc:
                last_error = exc
                await asyncio.sleep(0.25 * attempt)
        raise RuntimeError(f"GATT 初始化失败: {last_error}")

    def _handle_notification(self, _: object, data: bytearray) -> None:
        text = bytes(data).decode("utf-8", errors="replace").strip()
        payload = parse_json_status(text)
        if self._status_queue is not None:
            self._status_queue.put_nowait((text, payload))
        self._event_queue.put(("notify", (text, payload)))

    async def send_command(
        self,
        command: str,
        accepted_kinds: Optional[Sequence[str]] = None,
        wait_timeout: float = 3.0,
        settle_after_write: float = 0.12,
    ) -> Tuple[str, Optional[dict]]:
        client = self._client
        if not client or not client.is_connected:
            raise RuntimeError("蓝牙未连接")
        if self._status_queue is None:
            raise RuntimeError("状态队列未初始化")

        while not self._status_queue.empty():
            try:
                self._status_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

        await client.write_gatt_char(COMMAND_UUID, command.encode("utf-8"), response=True)
        await asyncio.sleep(settle_after_write)

        deadline = self._loop.time() + wait_timeout
        while True:
            remain = deadline - self._loop.time()
            if remain <= 0:
                raise TimeoutError(f"等待指令响应超时: {command}")
            text, payload = await asyncio.wait_for(self._status_queue.get(), timeout=remain)
            if accepted_kinds is None:
                return text, payload
            if payload and payload.get("kind") in accepted_kinds:
                return text, payload


class BalanceGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("RiderDriver 蓝牙调参")
        self.root.geometry("1180x760")

        self.bridge = BleBridge()
        self.pending: List[Tuple[Any, Optional[Callable[[Any], None]], str]] = []
        self.scan_map: Dict[str, str] = {}
        self.connected_address: Optional[str] = None
        self.telemetry_poll_enabled = False
        self.telemetry_poll_inflight = False
        self.stream_started = False

        self.status_var = tk.StringVar(value="未连接")
        self.device_var = tk.StringVar()
        self.interval_var = tk.StringVar(value="100")

        self.kp_var = tk.StringVar(value="4.4")
        self.ki_var = tk.StringVar(value="0.0")
        self.kd_var = tk.StringVar(value="0.35")
        self.target_var = tk.StringVar(value="0.0")
        self.speed_limit_var = tk.StringVar(value="1000")
        self.tilt_cutoff_var = tk.StringVar(value="75")

        self.telemetry_vars: Dict[str, tk.StringVar] = {
            key: tk.StringVar(value="--")
            for key in (
                "theta", "target", "err", "zero", "roll", "pitch", "yaw",
                "ax", "ay", "az", "gx", "gy", "gz", "rate",
                "p", "i", "d", "w", "ws", "raw", "out", "ff", "yaw_out",
                "ls", "rs", "seq", "armed", "guard",
            )
        }

        self._build_ui()
        self._pump_async()
        self._pump_events()
        self._poll_telemetry()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        top = ttk.LabelFrame(main, text="连接")
        top.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(top, text="设备").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        self.device_box = ttk.Combobox(top, textvariable=self.device_var, width=48, state="readonly")
        self.device_box.grid(row=0, column=1, padx=6, pady=6, sticky="ew")
        ttk.Button(top, text="扫描", command=self.scan_devices).grid(row=0, column=2, padx=6, pady=6)
        ttk.Button(top, text="连接", command=self.connect_device).grid(row=0, column=3, padx=6, pady=6)
        ttk.Button(top, text="断开", command=self.disconnect_device).grid(row=0, column=4, padx=6, pady=6)
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=5, padx=6, pady=6, sticky="w")
        top.columnconfigure(1, weight=1)

        ctrl = ttk.Frame(main)
        ctrl.pack(fill=tk.X, pady=(0, 8))

        pid_frame = ttk.LabelFrame(ctrl, text="PID / 目标角")
        pid_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        entries = [
            ("Kp", self.kp_var),
            ("Ki", self.ki_var),
            ("Kd", self.kd_var),
            ("Target", self.target_var),
        ]
        for idx, (label, var) in enumerate(entries):
            ttk.Label(pid_frame, text=label).grid(row=0, column=idx * 2, padx=4, pady=6, sticky="e")
            ttk.Entry(pid_frame, textvariable=var, width=9).grid(row=0, column=idx * 2 + 1, padx=4, pady=6)

        ttk.Button(pid_frame, text="读取参数", command=self.read_pid).grid(row=1, column=0, columnspan=2, padx=4, pady=6, sticky="ew")
        ttk.Button(pid_frame, text="应用参数", command=self.apply_pid).grid(row=1, column=2, columnspan=2, padx=4, pady=6, sticky="ew")
        ttk.Button(pid_frame, text="关闭板端刷屏", command=lambda: self.set_board_log(False)).grid(row=1, column=4, columnspan=2, padx=4, pady=6, sticky="ew")
        ttk.Button(pid_frame, text="开启板端刷屏", command=lambda: self.set_board_log(True)).grid(row=1, column=6, columnspan=2, padx=4, pady=6, sticky="ew")

        tune_frame = ttk.LabelFrame(ctrl, text="基础限制")
        tune_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        tune_entries = [
            ("speed_limit", self.speed_limit_var),
            ("tilt_cutoff", self.tilt_cutoff_var),
        ]
        for idx, (label, var) in enumerate(tune_entries):
            row = 0
            col = idx * 2
            ttk.Label(tune_frame, text=label).grid(row=row, column=col, padx=4, pady=6, sticky="e")
            ttk.Entry(tune_frame, textvariable=var, width=8).grid(row=row, column=col + 1, padx=4, pady=6)

        ttk.Button(tune_frame, text="读取响应参数", command=self.read_runtime_cfg).grid(
            row=1, column=0, columnspan=2, padx=4, pady=6, sticky="ew"
        )
        ttk.Button(tune_frame, text="应用响应参数", command=self.apply_runtime_cfg).grid(
            row=1, column=2, columnspan=2, padx=4, pady=6, sticky="ew"
        )

        stream_frame = ttk.LabelFrame(ctrl, text="遥测")
        stream_frame.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(stream_frame, text="间隔 ms").grid(row=0, column=0, padx=6, pady=6)
        ttk.Entry(stream_frame, textvariable=self.interval_var, width=8).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(stream_frame, text="启动推流", command=self.start_stream).grid(row=1, column=0, padx=6, pady=6, sticky="ew")
        ttk.Button(stream_frame, text="停止推流", command=self.stop_stream).grid(row=1, column=1, padx=6, pady=6, sticky="ew")
        ttk.Button(stream_frame, text="抓一次", command=self.read_once).grid(row=2, column=0, columnspan=2, padx=6, pady=6, sticky="ew")

        body = ttk.Panedwindow(main, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True)

        telemetry_frame = ttk.LabelFrame(body, text="实时反馈")
        log_frame = ttk.LabelFrame(body, text="事件")
        body.add(telemetry_frame, weight=3)
        body.add(log_frame, weight=2)

        fields = [
            ("Seq", "seq"), ("Armed", "armed"), ("Guard", "guard"),
            ("theta", "theta"), ("target", "target"), ("err", "err"), ("zero", "zero"),
            ("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw"),
            ("ax", "ax"), ("ay", "ay"), ("az", "az"),
            ("gx", "gx"), ("gy", "gy"), ("gz", "gz"), ("rate", "rate"),
            ("P", "p"), ("I", "i"), ("D", "d"), ("W", "w"), ("wheel_fb", "ws"),
            ("raw", "raw"), ("out", "out"), ("ff", "ff"), ("yaw_out", "yaw_out"),
            ("left_speed", "ls"), ("right_speed", "rs"),
        ]

        for idx, (label, key) in enumerate(fields):
            row = idx // 4
            col = (idx % 4) * 2
            ttk.Label(telemetry_frame, text=label).grid(row=row, column=col, padx=6, pady=4, sticky="e")
            ttk.Label(telemetry_frame, textvariable=self.telemetry_vars[key], width=12).grid(
                row=row, column=col + 1, padx=6, pady=4, sticky="w"
            )

        self.log_text = tk.Text(log_frame, height=20, wrap="word")
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _submit(
        self,
        coro: asyncio.Future,
        callback: Optional[Callable[[Any], None]] = None,
        err_title: str = "操作失败",
        show_error: bool = True,
    ) -> None:
        self.pending.append((self.bridge.submit(coro), callback, err_title, show_error))

    def _pump_async(self) -> None:
        remain: List[Tuple[Any, Optional[Callable[[Any], None]], str, bool]] = []
        for future, callback, err_title, show_error in self.pending:
            if not future.done():
                remain.append((future, callback, err_title, show_error))
                continue
            try:
                result = future.result()
            except Exception as exc:
                if show_error:
                    messagebox.showerror(err_title, str(exc))
                continue
            if callback:
                callback(result)
        self.pending = remain
        self.root.after(100, self._pump_async)

    def _pump_events(self) -> None:
        while True:
            try:
                kind, payload = self.bridge.event_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "connected":
                self.connected_address = str(payload)
                self.telemetry_poll_enabled = False
                self.stream_started = False
                self.status_var.set(f"已连接 {payload}")
                self._append_log(f"已连接 {payload}")
                self.read_pid()
                self.read_runtime_cfg()
                self.set_board_log(False)
                self._submit(
                    self.bridge.send_command("BTELE 0 100", accepted_kinds=["btele"]),
                    lambda result: self._append_log(f"默认关闭推流: {result[0]}"),
                    "关闭默认推流失败",
                    show_error=False,
                )
            elif kind == "disconnected":
                self.connected_address = None
                self.telemetry_poll_enabled = False
                self.telemetry_poll_inflight = False
                self.stream_started = False
                self.status_var.set("已断开")
                self._append_log("蓝牙已断开")
            elif kind == "notify":
                text, data = payload
                if data and data.get("kind") == "telemetry":
                    self._update_telemetry(data)
                elif data and data.get("kind") == "pid":
                    self._update_pid(data)
                    self._append_log("PID 参数已更新")
                elif data and data.get("kind") == "cfg_balance":
                    self._update_runtime_cfg(data)
                    self._append_log("响应参数已更新")
                else:
                    self._append_log(text)
        self.root.after(80, self._pump_events)

    def _poll_telemetry(self) -> None:
        if (
            self.telemetry_poll_enabled
            and self.connected_address
            and not self.telemetry_poll_inflight
        ):
            self.telemetry_poll_inflight = True

            def done(result: Any) -> None:
                self.telemetry_poll_inflight = False
                if result and result[1]:
                    self._update_telemetry(result[1])

            self._submit(
                self.bridge.send_command(
                    "BGETTEL",
                    accepted_kinds=["telemetry"],
                    wait_timeout=1.0,
                    settle_after_write=0.02,
                ),
                done,
                "轮询遥测失败",
                show_error=False,
            )

        self.root.after(180, self._poll_telemetry)

    def _update_pid(self, payload: dict) -> None:
        self.kp_var.set(f"{float(payload.get('kp', 0.0)):.4f}")
        self.ki_var.set(f"{float(payload.get('ki', 0.0)):.4f}")
        self.kd_var.set(f"{float(payload.get('kd', 0.0)):.4f}")
        self.target_var.set(f"{float(payload.get('target', 0.0)):.3f}")

    def _update_runtime_cfg(self, payload: dict) -> None:
        self.speed_limit_var.set(str(int(payload.get("speed_limit", 0))))
        self.tilt_cutoff_var.set(f"{float(payload.get('tilt_cutoff', 0.0)):.2f}")

    def _set_tel(self, key: str, value: Any, digits: int = 3) -> None:
        if isinstance(value, float):
            self.telemetry_vars[key].set(f"{value:.{digits}f}")
        else:
            self.telemetry_vars[key].set(str(value))

    def _update_telemetry(self, payload: dict) -> None:
        mapping = {
            "seq": ("seq", 0),
            "arm": ("armed", 0),
            "guard": ("guard", 0),
            "theta": ("theta", 3),
            "target": ("target", 3),
            "err": ("err", 3),
            "zero": ("zero", 3),
            "roll": ("roll", 3),
            "pitch": ("pitch", 3),
            "yaw": ("yaw", 3),
            "ax": ("ax", 3),
            "ay": ("ay", 3),
            "az": ("az", 3),
            "gx": ("gx", 3),
            "gy": ("gy", 3),
            "gz": ("gz", 3),
            "rate": ("rate", 3),
            "p": ("p", 3),
            "i": ("i", 3),
            "d": ("d", 3),
            "w": ("w", 3),
            "ws": ("ws", 3),
            "raw": ("raw", 0),
            "out": ("out", 0),
            "ff": ("ff", 0),
            "yaw_out": ("yaw_out", 0),
            "ls": ("ls", 0),
            "rs": ("rs", 0),
        }
        for src, (dst, digits) in mapping.items():
            if src not in payload:
                continue
            value = payload[src]
            if digits == 0:
                self._set_tel(dst, value, 0)
            else:
                self._set_tel(dst, float(value), digits)

    def scan_devices(self) -> None:
        self.status_var.set("扫描中...")
        self._submit(self.bridge.scan(), self._on_scan_done, "扫描失败")

    def _on_scan_done(self, devices: List[DeviceInfo]) -> None:
        display_items: List[str] = []
        self.scan_map.clear()
        for item in devices:
            if item.name != DEVICE_NAME:
                continue
            label = f"{item.name} | {item.address} | RSSI {item.rssi}"
            display_items.append(label)
            self.scan_map[label] = item.address
        self.device_box["values"] = display_items
        if display_items:
            self.device_var.set(display_items[0])
            self.status_var.set(f"发现 {len(display_items)} 个 {DEVICE_NAME}")
        else:
            self.status_var.set("未发现 RiderDriver")

    def connect_device(self) -> None:
        label = self.device_var.get().strip()
        address = self.scan_map.get(label, "")
        if not address:
            messagebox.showwarning("提示", "请先扫描并选择设备")
            return
        self.status_var.set("连接中...")
        self._submit(self.bridge.connect(address), None, "连接失败")

    def disconnect_device(self) -> None:
        self._submit(self.bridge.disconnect(), None, "断开失败")

    def read_pid(self) -> None:
        self._submit(
            self.bridge.send_command("BGETPID", accepted_kinds=["pid"]),
            lambda result: self._update_pid(result[1] or {}),
            "读取 PID 失败",
        )

    def apply_pid(self) -> None:
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            target = float(self.target_var.get())
        except ValueError:
            messagebox.showwarning("提示", "PID 输入格式不正确")
            return
        cmd = f"BSETPID {kp:.4f} {ki:.4f} {kd:.4f} 0.0000 {target:.4f}"
        self._submit(
            self.bridge.send_command(cmd, accepted_kinds=["pid"]),
            lambda result: self._update_pid(result[1] or {}),
            "应用 PID 失败",
        )

    def read_runtime_cfg(self) -> None:
        self._submit(
            self.bridge.send_command("BGETCFG", accepted_kinds=["cfg_balance"]),
            lambda result: self._update_runtime_cfg(result[1] or {}),
            "读取响应参数失败",
        )

    def apply_runtime_cfg(self) -> None:
        try:
            speed_limit = int(self.speed_limit_var.get())
            tilt_cutoff = float(self.tilt_cutoff_var.get())
        except ValueError:
            messagebox.showwarning("提示", "响应参数格式不正确")
            return

        cmd = f"BSETCFG {speed_limit} {tilt_cutoff:.2f}"
        self._submit(
            self.bridge.send_command(cmd, accepted_kinds=["cfg_balance"]),
            lambda result: self._update_runtime_cfg(result[1] or {}),
            "应用响应参数失败",
        )

    def read_once(self) -> None:
        self._submit(
            self.bridge.send_command("BGETTEL", accepted_kinds=["telemetry"]),
            lambda result: self._update_telemetry(result[1] or {}),
            "读取遥测失败",
        )

    def start_stream(self) -> None:
        try:
            interval = int(self.interval_var.get())
        except ValueError:
            messagebox.showwarning("提示", "推流间隔必须是整数")
            return
        self.telemetry_poll_enabled = True
        self.stream_started = True
        self._submit(
            self.bridge.send_command(f"BTELE 1 {interval}", accepted_kinds=["btele"]),
            lambda result: self._append_log(f"已启动推流: {result[0]}"),
            "启动推流失败",
        )

    def stop_stream(self) -> None:
        self.telemetry_poll_enabled = False
        self.telemetry_poll_inflight = False
        self.stream_started = False
        self._submit(
            self.bridge.send_command("BTELE 0 100", accepted_kinds=["btele"]),
            lambda result: self._append_log(f"已停止推流: {result[0]}"),
            "停止推流失败",
        )

    def set_board_log(self, enabled: bool) -> None:
        cmd = f"BLOG {1 if enabled else 0}"
        self._submit(
            self.bridge.send_command(cmd, accepted_kinds=["blog"]),
            lambda result: self._append_log(f"板端日志开关: {result[0]}"),
            "设置板端日志失败",
        )

    def _on_close(self) -> None:
        try:
            self.bridge.submit(self.bridge.disconnect()).result(timeout=2)
        except Exception:
            pass
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    style = ttk.Style()
    if "vista" in style.theme_names():
        style.theme_use("vista")
    BalanceGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
