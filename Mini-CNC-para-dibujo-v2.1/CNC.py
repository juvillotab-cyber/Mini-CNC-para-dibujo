#!/usr/bin/env python3

import serial
import time
import sys
import os

SERIAL_PORT   = "/dev/ttyUSB0"
BAUD_RATE     = 115200
TIMEOUT_LINE  = 60.0

def send_file(port, filepath):
    if not os.path.isfile(filepath):
        print(f"[ERROR] File not found: {filepath}")
        sys.exit(1)

    ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_LINE)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(1.5)

    print(f"[INFO] Connected to {port} at {BAUD_RATE} bps")
    print(f"[INFO] Sending file: {filepath}")

    with open(filepath, "r") as f:
        lines = f.readlines()

    total = len([l for l in lines if l.strip()])
    sent  = 0

    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith(";") or line.startswith("("):
            continue

        line_enc = (line + "\n").encode("ascii")
        ser.write(line_enc)
        ser.flush()
        sent += 1
        print(f"[{sent}/{total}] Sent: {line}")

        response = ser.readline().decode("ascii", errors="ignore").strip()
        if response:
            print(f"        Resp: {response}")

        if sent < total:
            time.sleep(0.01)

    ser.write(b"f\n")
    ser.flush()
    print("[INFO] Sent termination 'f'. Motors off.")
    time.sleep(0.1)
    ser.close()
    print("[INFO] Done.")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 CNC.py <file.nc> [serial_port]")
        print(f"  Default port: {SERIAL_PORT}")
        sys.exit(1)

    filepath = sys.argv[1]
    port     = sys.argv[2] if len(sys.argv) > 2 else SERIAL_PORT
    send_file(port, filepath)

if __name__ == "__main__":
    main()
