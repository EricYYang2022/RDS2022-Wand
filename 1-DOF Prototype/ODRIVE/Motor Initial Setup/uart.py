import serial


def main():
    ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
    print(ser.name)


