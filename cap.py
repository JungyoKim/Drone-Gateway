import serial, time, sys
ser = serial.Serial("COM19", 115200, timeout=0.5)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.1); ser.setRTS(False)
time.sleep(0.05); ser.reset_input_buffer()
t0 = time.time()
with open("cap.log", "w", encoding="utf-8") as f:
    while time.time() - t0 < float(sys.argv[1] if len(sys.argv) > 1 else 40):
        ln = ser.readline().decode(errors="replace").rstrip()
        if ln:
            f.write(ln + "\n"); f.flush()
ser.close()
