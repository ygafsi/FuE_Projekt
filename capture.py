import serial
import sys
from datetime import datetime

PORT = "/dev/ttyUSB0"
BAUD = 115200
OUT = "capture_raw.log"

ser = serial.Serial(PORT, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.1)
print(f"Ecoute sur {PORT} @ {BAUD}. Ctrl+C pour arreter.")
print(f"Tout est logge dans {OUT}")

with open(OUT, "ab") as f:
    f.write(f"\n===== START {datetime.now()} =====\n".encode())
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                f.write(chunk)
                f.flush()
                sys.stdout.write(chunk.decode(errors="replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        f.write(f"\n===== STOP {datetime.now()} =====\n".encode())
        print("\nArrete.")
ser.close()