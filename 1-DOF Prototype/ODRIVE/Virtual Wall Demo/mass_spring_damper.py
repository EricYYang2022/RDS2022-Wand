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

    wall = 0.5
    kp = 10
    kv = 0.05

    while True:
        p = m1.encoder.pos_estimate
        v = m1.encoder.vel_estimate
        print(p)
        if p >= wall:
            print(p)
            m1.controller.input_torque = -1 * kp * (p - wall) + -1 * kv * v
        elif p <= -1 * wall:
            print(p)
            m1.controller.input_torque = kp * (p - (-1 * wall)) + kv * v
        else:
            m1.controller.input_torque = 0.0


main()

# Resources: https://github.com/but-i-love-pbj/odrive/blob/master/motor.py
# Resources: https://python.hotexamples.com/examples/odrive/-/find_any/python-find_any-function-examples.html
