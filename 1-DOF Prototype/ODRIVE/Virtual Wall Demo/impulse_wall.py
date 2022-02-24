import time
import odrive as o
from odrive.enums import *


def main():
    global impulse_wall
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

    t = 0
    impulse_time = 0.05
    dt = 0.005

    impulse_wall = False
    impulse_start = False
    m = 0.003
    const_v = 0

    m1.controller.input_torque = 0.0
    torque = 0.0

    while True:
        p = m1.encoder.pos_estimate
        v = m1.encoder.vel_estimate
        print("pos: %f, vel: %f" % (p, v))
        torque = 0.0
        if p >= wall:
            torque = -1 * kp * (p - wall) + -1 * kv * v
            if not impulse_wall:
                impulse_wall = True
                impulse_start = True
                const_v = v + 0.0
                impulse_time = const_v / 8

            if impulse_start:
                if t < impulse_time:
                    t += dt
                    if v > 0.001:
                        torque += -1 * (m * const_v / dt)
                else:
                    impulse_start = False
        else:
            impulse_start = False
            t = 0
            torque = 0.0
            if p < wall - 0.05:
                impulse_wall = False

        m1.controller.input_torque = torque


main()
