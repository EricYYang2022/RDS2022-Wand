import serial


def main():
    o = serial.Serial("/dev/ttyACM0", baudrate=115200)
    print(o.name)
    o.write(b't 1 1')



