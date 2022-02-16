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

    wall = 0.6
    kp = 10
    kv = 0.05
    mass = 0.1
    dt = 0.01
    t = 0
    hit_wall = False
    impulse_time = 8192 * dt

    while True:
        p = m1.encoder.pos_estimate
        v = m1.encoder.vel_estimate
        if p >= wall:
            hit_wall = True
            print(p)
            torque = -1 * kp * (p - wall) + -1 * kv * v
            if t < impulse_time:
                torque += -1 * v * mass/dt
                t += 1
            m1.controller.input_torque = torque
        else:
            m1.controller.input_torque = 0.0
            if hit_wall:
                t = 0
                hit_wall = False


main()
