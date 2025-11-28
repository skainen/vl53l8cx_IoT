import serial
import time

# Open serial port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

print("Reading sensor data... (Ctrl+C to stop)")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print(line)
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\nStopped")
    ser.close()
