import odrive as o
from odrive.enums import *


def UART_setup(m):
    m.config.enable_uart_a = False
    m.config.gpio1_mode = GPIO_MODE_DIGITAL
    m.config.gpio2_mode = GPIO_MODE_DIGITAL
    m.config.uart_a_baudrate = 115200
    return 0


def main():
    print("Finding an ODRIVE...")
    m = o.find_any()
    m1 = m.axis1
    print("ODRIVE Connected...")
    UART_setup(m)

