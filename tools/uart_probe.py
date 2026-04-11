#!/usr/bin/env python3
"""
最小 UART 探测脚本 —— 拷贝到 CM4 上运行:
    python3 uart_probe.py

功能:
  1) 向 /dev/ttyAMA0 发送一个 read firmware version 帧 (addr=0x07)
  2) 等待最多 2 秒，打印收到的所有原始字节 (hex)
  3) 如果什么都没收到，说明物理链路有问题（接线/方向/波特率）

协议帧格式:  55 00 LEN TYPE ADDR [DATA...] CHECK 00 AA
  LEN = 整帧长度（含帧头/帧尾）
  read firmware: 55 00 09 02 07 0A E5 00 AA
  CHECK = 255 - ((0x09 + 0x02 + 0x07 + 0x0A) % 256) = 0xE5
"""

import serial
import sys
import time

PORT = "/dev/ttyAMA0"
BAUD = 115200

# read firmware version 帧（与 CM4/xgolib 保持一致）
READ_FW_FRAME = bytes([0x55, 0x00, 0x09, 0x02, 0x07, 0x0A, 0xE5, 0x00, 0xAA])


def main():
    print(f"Opening {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"FAILED to open serial: {e}")
        sys.exit(1)

    # 清空缓冲区
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"TX ({len(READ_FW_FRAME)} bytes): {READ_FW_FRAME.hex(' ').upper()}")
    ser.write(READ_FW_FRAME)
    ser.flush()

    # 等待回复（最多 2 秒）
    print("Waiting for RX (2s timeout) ...")
    deadline = time.time() + 2.0
    rx_all = bytearray()
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            rx_all.extend(chunk)
            # 收到 AA 结尾可能就是完整帧了
            if 0xAA in chunk:
                break

    ser.close()

    if rx_all:
        print(f"RX ({len(rx_all)} bytes): {rx_all.hex(' ').upper()}")
        # 简单解析
        if rx_all[0:2] == b'\x55\x00' and rx_all[-1] == 0xAA:
            data_part = rx_all[5:-2]  # LEN TYPE ADDR 之后，CHECK 00 AA 之前
            print(f"  -> Firmware data (raw): {data_part.hex(' ').upper()}")
            try:
                print(f"  -> Firmware string: {data_part.decode('ascii', errors='replace')}")
            except:
                pass
        else:
            print("  -> Does NOT look like a valid protocol frame")
            print("     Check baud rate or wiring")
    else:
        print("RX: *** NOTHING received ***")
        print("  Possible causes:")
        print("  1. TX/RX wires swapped")
        print("  2. Wrong GPIO pins")
        print("  3. ESP32 not running / not booted")
        print("  4. Baud rate mismatch")


if __name__ == "__main__":
    main()
