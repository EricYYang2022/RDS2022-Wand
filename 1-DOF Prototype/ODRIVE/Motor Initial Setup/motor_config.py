import odrive as o
from odrive.enums import *


def configure_motor(m):
    """
    Function that allows for the configuration of motors based on various parameters
    :param m: ODRIVE Motor (odrv0)
    :return: Configured motors according to Maxon Motor
    """
    # Current Limit:
    # m1.axis0.motor.config.current_lim = 10

    # Velocity Limit:
    # m1.axis0.controller.config.vel_limit = 0.001

    # Calibration current
    # m1.axis0.motor.config.calibration_current = 10

    # Brake Resistance:
    m.config.enable_brake_resistor = False

    # Max Negative Current:
    m.config.dc_max_negative_current = 0.01

    # Pole Pairs in motor
    m.axis1.motor.config.pole_pairs = 7

    # Torque Constant for the Motor (torque per motor amp)
    m.axis1.motor.config.torque_constant = 0.0525

    m.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

    m.axis1.encoder.config.cpr = 8192

    m.save_configuration()


def main():
    print("Finding an ODRIVE...")
    m1 = o.find_any()
    print("ODRIVE Connected...")
    print("ODRIVE Vbus Volage: " + m1.vbus_voltage)

    while True:
        configure = input("Do you want to configure the motor? (Y/N)")
        if configure == "Y" or configure == "No":
            break
    if configure == "Y":
        print("Motor will be configured...")
        configure_motor(m1)
    elif configure == "N":
        print("Motor will not be configured...")

    m1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# Resources: https://github.com/but-i-love-pbj/odrive/blob/master/motor.py
# Resources: https://python.hotexamples.com/examples/odrive/-/find_any/python-find_any-function-examples.html
