import time
import odrive as o
from odrive.enums import *


def calibrate_motor(m):
    m.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    print("Starting Full Motor Calibration")
    time.sleep(20)
    m.motor.config.pre_calibrated = True
    print("Motor Calibrated")


def calibrate_encoder(m):
    print("Starting Encoder Calibration")
    m.encoder.config.use_index = True
    m.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    time.sleep(5)
    m.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(10)
    m.config.startup_encoder_index_search = True
    m.encoder.config.pre_calibrated = True
    m.motor.config.pre_calibrated = True
    print("Encoder Calibrated")


def main():
    print("Finding an ODRIVE...")
    m = o.find_any()
    m1 = m.axis1
    print("ODRIVE Connected...")

    while True:
        c = input("Do you want to calibrate the motor? (Y/N) \n")
        if c == "Y" or c == "N":
            break
    if c == "Y":
        print("Motor will be calibrated...")
        calibrate_motor(m1)
        calibrate_encoder(m1)
    elif c == "N":
        print("Motor will not be calibrated...")

    m1.controller.input_torque = 0.0
    m1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    m1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    while True:
        t = float(input("Input Torque Value T, 0<=T<=1: \n"))
        if 0.0 <= t <= 1.0:
            break
    m1.controller.input_torque = t
    time.sleep(5)
    m1.controller.input_torque = 0.0


main()

# Resources: https://github.com/but-i-love-pbj/odrive/blob/master/motor.py
# Resources: https://python.hotexamples.com/examples/odrive/-/find_any/python-find_any-function-examples.html
