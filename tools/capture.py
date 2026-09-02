import serial, sys, time, os

out = "/tmp/boot2.txt"
port = "/dev/ttyACM0"
end = time.time() + 30

def try_open():
    for _ in range(60):
        try:
            return serial.Serial(port, 115200, timeout=0.5)
        except Exception:
            time.sleep(0.5)
    return None

ser = try_open()
if ser is None:
    print("never got port")
    sys.exit(1)

with open(out, "wb") as f:
    while time.time() < end:
        try:
            data = ser.read(4096)
            if data:
                f.write(data)
                f.flush()
        except Exception:
            # port dropped (re-enumeration); reopen
            try:
                ser.close()
            except Exception:
                pass
            ser = try_open()
            if ser is None:
                break

print("capture done, bytes:", os.path.getsize(out))
