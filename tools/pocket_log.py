"""Records pocket voltages to logs/pocket-<date>.csv, from the pressure_click
test or from the firmware (neffy_esp32 streams them after receiving 'D'; its
own event lines, e.g. clicks, are kept as "# ..." comments).

Type a key and press Enter to mark the next sample (mark column); every mark is
confirmed on screen. q + Enter (or Ctrl+C) ends the recording.

Run from the project folder (cmd):
    "%USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe" tools\\pocket_log.py [COMx]
"""
import os
import sys
import threading
import time

import serial

KEYS = {
    "L": "left pocket", "R": "right pocket",
    "r": "rest", "h": "hand on (no press)", "c": "clicks with hand on",
    "p": "press + hold", "l": "let go", "k": "click", "x": "pressed with plastic",
}

port = sys.argv[1] if len(sys.argv) > 1 else "COM15"
os.makedirs("logs", exist_ok=True)
path = os.path.join("logs", time.strftime("pocket-%y%m%d-%H%M%S.csv"))

pending = ""
stop = threading.Event()

def read_keys():
    # line input works in every terminal; raw key reading (msvcrt) did not
    # reach the script inside the VS Code terminal
    global pending
    for line in sys.stdin:
        key = line.strip()[:1]
        if key == "q":
            break
        if key:
            pending = key
            print(f"\n>> mark '{key}'  {KEYS.get(key, '')}", flush=True)
    stop.set()

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.05
ser.rts, ser.dtr = False, True  # USB CDC only sends with DTR set; RTS would reset
ser.open()
ser.write(b"D")  # firmware: start pocket stream (the test sketch ignores it)

print(f"Recording {port} -> {path}")
print("Key + Enter: " + "   ".join(f"{k} = {v}" for k, v in KEYS.items()))
print("q + Enter to stop\n", flush=True)
threading.Thread(target=read_keys, daemon=True).start()

buf = b""
count = 0
with open(path, "w", encoding="utf-8", buffering=1) as out:
    out.write("t_ms,L_V,R_V,mark\n")
    try:
        while not stop.is_set():
            buf += ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                text = raw.decode(errors="replace").strip()
                parts = text.split(",")
                if len(parts) < 3 or not parts[0].isdigit():
                    if text.startswith("["):  # firmware event line
                        out.write(f"# {text}\n")
                        print(f"\n{text}", flush=True)
                    continue  # header or legend line
                t, lv, rv = parts[:3]
                mark, pending = pending, ""
                out.write(f"{t},{lv},{rv},{mark}\n")
                count += 1
                if count % 10 == 0:
                    print(f"\rL {lv} V   R {rv} V   ({count} samples)   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"d")  # firmware: stop pocket stream
        ser.close()

print(f"\n\nSaved {count} samples to {path}")
