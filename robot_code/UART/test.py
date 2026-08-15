import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 9600

def clamp(value, min_value=-255, max_value=255):
    return max(min_value, min(max_value, value))

def send_motor(ser, left, right):
    left = clamp(left)
    right = clamp(right)

    msg = f"M {left} {right}\n"
    ser.write(msg.encode("ascii"))
    print("TX:", msg.strip())

def main():
    print("Jetson UART motor test")
    print("example:")
    print("  run 80 80 3")
    print("  run -80 80 3")
    print("  0 0")
    print("  q")

    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        time.sleep(2)

        while True:
            raw = input("left right > ").strip()

            if raw.lower() in ["q", "quit", "exit"]:
                send_motor(ser, 0, 0)
                break

            parts = raw.split()

            try:
                if parts[0] == "run":
                    left = int(parts[1])
                    right = int(parts[2])
                    seconds = float(parts[3])

                    end_time = time.time() + seconds
                    while time.time() < end_time:
                        send_motor(ser, left, right)
                        time.sleep(0.1)

                    send_motor(ser, 0, 0)

                else:
                    left = int(parts[0])
                    right = int(parts[1])
                    send_motor(ser, left, right)

            except Exception:
                print("wrong input. example: 80 80 or run 80 80 3")

if __name__ == "__main__":
    main()
