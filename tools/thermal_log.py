"""Guided recording of the thermal (person) sensor for tuning the detector.
Shows what to do next with a countdown and writes the current step into every
line of logs/thermal-<date>.csv, so no keys are needed. Ctrl+C ends early.

Run from the project folder (cmd):
    "%USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe" tools\\thermal_log.py [COMx]
"""
import os
import sys
import threading
import time

import serial
import serial.tools.list_ports

# (label, seconds, instruction)
STEPS = [
    ("empty",    20, "Nobody in front of the sensor. Step out of view (at least 2 m away)."),
    ("approach", 10, "Walk up to Neffy slowly and stand in front of it as when using it."),
    ("stand",    20, "Stay in front, normally (moving a bit is fine)."),
    ("leave",    10, "Walk away, at least 2 m."),
    ("empty",    15, "Nobody in front."),
    ("pass",     20, "Walk past Neffy briskly without stopping, a few times back and forth."),
    ("empty",    15, "Nobody in front."),
    ("step_in",  10, "Step right in front of Neffy quickly and stay."),
    ("stand",    15, "Stay in front."),
    ("leave",    10, "Walk away."),
    ("empty",    15, "Nobody in front."),
    ("far",      15, "Stand about 1 m in front of Neffy."),
    ("leave",    10, "Walk away."),
    ("empty",    20, "Nobody in front. Almost done."),
]


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x303A and p.pid == 0x81BB:  # the firmware's USB port
            return p.device
    sys.exit("Mainboard firmware not found on USB")


port = sys.argv[1] if len(sys.argv) > 1 else find_port()
os.makedirs("logs", exist_ok=True)
path = os.path.join("logs", time.strftime("thermal-%y%m%d-%H%M%S.csv"))

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.1
ser.rts, ser.dtr = False, True  # USB CDC only sends with DTR set; RTS would reset
ser.open()
ser.write(b"T")

label = "start"
done = threading.Event()


def guide():
    global label
    print("Recording starts in 3 s ...", flush=True)
    time.sleep(3)
    for i, (name, secs, text) in enumerate(STEPS, 1):
        label = name
        print(f"\n[{i}/{len(STEPS)}] {text}", flush=True)
        for left in range(secs, 0, -1):
            print(f"\r   {left:2d} s left ", end="", flush=True)
            time.sleep(1)
    print("\n\nDone, thanks!", flush=True)
    done.set()


threading.Thread(target=guide, daemon=True).start()
buf = b""
count = 0
with open(path, "w", encoding="utf-8", buffering=1) as out:
    out.write("t_ms,temp_C,ptat_C,base_C,state,step\n")
    try:
        while not done.is_set():
            buf += ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                parts = raw.decode(errors="replace").strip().split(",")
                if len(parts) != 6 or parts[0] != "T":
                    continue
                out.write(",".join(parts[1:]) + f",{label}\n")
                count += 1
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"t")
        ser.close()
print(f"Saved {count} readings to {path}")
