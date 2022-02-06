import time
import odrive as o
from odrive.enums import *


def main():
    print("Finding an ODRIVE...")
    m = o.find_any()
    m1 = m.axis1
    print("ODRIVE Connected...")

    m1.controller.input_torque = 0.0
    m1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    m1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    while True:
        pos = m1.encoder.pos_estimate
        if pos >= 0.5:
            print(pos)
            if pos >= 0.6:
                m1.controller.input_torque = -0.2
            else:
                m1.controller.input_torque = -0.15
        else:
            m1.controller.input_torque = 0.0
        if pos <= -0.5:
            print(pos)
            if pos <= 0.6:
                m1.controller.input_torque = 0.2
            else:
                m1.controller.input_torque = 0.15
        else:
            m1.controller.input_torque = 0.0


main()

# Resources: https://github.com/but-i-love-pbj/odrive/blob/master/motor.py
# Resources: https://python.hotexamples.com/examples/odrive/-/find_any/python-find_any-function-examples.html
