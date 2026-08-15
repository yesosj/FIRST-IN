import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)

msg = "M 200 200\n"
ser.write(msg.encode("ascii"))
print("TX:", msg.strip())

rx = ser.readline().decode(errors="ignore").strip()
print("RX:", rx)

ser.close()
