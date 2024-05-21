import serial
import time
import random

def run():
    ser = serial.Serial('COM7', 115200)
    time.sleep(2)

    try:
        while True:
            command = input("Masukkan perintah:")

            command = command.split('.')

            result = '.'.join([str(x) for x in command]) + '\n'

            print('data to send:', result)
            
            ser.write(result.encode())

            time.sleep(0.5)

    except KeyboardInterrupt:
        ser.close()
        print("Koneksi serial ditutup.")


run()
