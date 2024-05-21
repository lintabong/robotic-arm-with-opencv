import serial
import time
import random

def run():
    ser = serial.Serial('COM7', 115200)
    time.sleep(2)

    try:
        while True:
            ser.write("22.22.22.22.22.right.124\n".encode())

            time.sleep(2)

            ser.write("22.22.22.22.22.left.124\n".encode())

            time.sleep(2)

            ser.write("22.22.22.22.22.right.0\n".encode())

            time.sleep(2)

            ser.write("22.22.22.22.22.left.255\n".encode())

            time.sleep(2)

    except KeyboardInterrupt:
        ser.close()
        print("Koneksi serial ditutup.")

run()
