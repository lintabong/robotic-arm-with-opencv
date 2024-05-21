import serial

ser = serial.Serial(
    port='COM3',
    baudrate=115200,
    timeout=1
)

try:
    while True:
        if ser.in_waiting > 0:
            result = ser.readline().decode('utf-8').rstrip()
            result = int(result)

            print(result)
except KeyboardInterrupt:
    print("Program dihentikan.")
finally:
    ser.close()
