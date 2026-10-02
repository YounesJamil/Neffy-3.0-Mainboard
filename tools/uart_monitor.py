"""Live view of the UART traffic between the mainboard and the other board,
read back from the mainboard's own TX and RX pins (firmware debug command 'U').
The person reading repeats every 500 ms, so only its changes are shown. Every
frame is saved to logs/uart-<date>.txt. Ctrl+C ends.

Run from the project folder (cmd):
    "%USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe" tools\\uart_monitor.py [COMx]
"""
import os
import sys
import time

import serial
import serial.tools.list_ports

NAMES = {
    0x01: "MUTE", 0x03: "GET_PERSON_READING",
    0x04: "MOVE_BODY_IN_TIME", 0x05: "MOVE_HEAD_IN_TIME", 0x06: "MOVE_BODY_HEAD_IN_TIME",
    0x07: "DISABLE_MOTORS", 0x08: "PRESSURE_CLICK_L", 0x09: "BREATHING_RATE",
    0x0B: "PRESSURE_CLICK_R",
    0x0D: "BREATHING_DETECTION_NORMAL", 0x0E: "BREATHING_DETECTION_STRESSED",
    0x0F: "BREATHING_DETECTION_RELAXED", 0x10: "BREATHING_DETECTION_START",
}


def meaning(direction, cmd, payload):
    """Plain-text reading of the payload, for the commands where it carries one."""
    b = [int(x, 16) for x in payload]
    word = lambda i: (b[i] << 8) | b[i + 1]
    if cmd == 0x01 and b:
        return {0: "sound on", 1: "muted"}.get(b[0], f"? {b[0]}")
    if cmd == 0x03 and b:
        return {0: "nobody", 1: "someone there"}.get(b[0], f"? {b[0]}")
    if cmd in (0x04, 0x05):
        unit = "mm" if cmd == 0x04 else "deg"
        if direction == "rx" and len(b) >= 4:    # command: position, duration (x 0.1)
            return f"to {word(0) / 10:.1f} {unit} in {word(2) / 10:.1f} s"
        if direction == "tx" and len(b) >= 2:    # response: reached position (x 10)
            return f"reached {word(0) / 10:.1f} {unit}"
    if cmd == 0x06 and len(b) >= 8:
        return (f"body to {word(0) / 10:.1f} mm in {word(2) / 10:.1f} s, "
                f"head to {word(4) / 10:.1f} deg in {word(6) / 10:.1f} s")
    if cmd == 0x10:
        return "start 30 s measurement"
    return ""


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x303A and p.pid == 0x81BB:  # the firmware's USB port
            return p.device
    sys.exit("Mainboard firmware not found on USB")


port = sys.argv[1] if len(sys.argv) > 1 else find_port()
os.makedirs("logs", exist_ok=True)
path = os.path.join("logs", time.strftime("uart-%y%m%d-%H%M%S.txt"))

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.1
ser.rts, ser.dtr = False, True  # USB CDC only sends with DTR set; RTS would reset
ser.open()
ser.write(b"U")
print(f"UART monitor on {port} -> {path}   (Ctrl+C to stop)")
print("tx = mainboard sends, rx = mainboard receives\n", flush=True)

last_person = None
buf = b""
with open(path, "w", encoding="utf-8", buffering=1) as out:
    try:
        while True:
            buf += ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                parts = raw.decode(errors="replace").split()
                if len(parts) < 4 or parts[0] not in ("tx", "rx"):
                    continue
                out.write(" ".join(parts) + "\n")
                stamp = time.strftime("%H:%M:%S")
                if parts[2] == "stray":
                    print(f"{stamp}  {parts[0]}  stray byte {parts[3]}", flush=True)
                    continue
                cmd = int(parts[3], 16)
                payload = parts[5:]
                if parts[0] == "tx" and cmd == 0x03:  # repeats every 500 ms
                    if payload == last_person:
                        continue
                    last_person = payload
                name = NAMES.get(cmd, "?")
                text = meaning(parts[0], cmd, payload)
                print(f"{stamp}  {parts[0]}  {' '.join(parts[2:]):<30} {name:<28} {text}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"u")
        ser.close()
print(f"\nSaved to {path}")
