import time
import odrive as o
from odrive.enums import *


def configure_motor_settings(m):
    """
    Function that allows for the configuration of motors based on various parameters
    :param m: ODRIVE Motor (odrv0)
    :return: Configured motors according to Maxon Motor
    """
    """
    # Current Limit:
    m.axis1.motor.config.current_lim = 9.28
    
    # Velocity Limit:
    m.axis1.controller.config.vel_limit = 0.001

    # Calibration current
    m.axis1.motor.config.calibration_current = 10

    # Brake Resistance:
    m.config.enable_brake_resistor = False

    # Max Negative Current:
    m.config.dc_max_negative_current = -0.04

    # Pole Pairs in motor
    m.axis1.motor.config.pole_pairs = 7

    # Torque Constant for the Motor (torque per motor amp)
    m.axis1.motor.config.torque_constant = 0.0525

    m.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

    m.axis1.encoder.config.cpr = 8192

    m.config_brake_resistance = 0.5
    """
    m.save_configuration()


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
    m1 = o.find_any()
    print("ODRIVE Connected...")
    print("ODRIVE Vbus Volage: " + str(m1.vbus_voltage))

    while True:
        configure = input("Do you want to configure the motor settings? (Y/N) \n")
        if configure == "Y" or configure == "N":
            break
    if configure == "Y":
        print("Motor will be configured...")
        configure_motor_settings(m1)
    elif configure == "N":
        print("Motor will not be configured...")

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


main()

# Resources: https://github.com/but-i-love-pbj/odrive/blob/master/motor.py
# Resources: https://python.hotexamples.com/examples/odrive/-/find_any/python-find_any-function-examples.html