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

    # Setting variables for mass_spring_damper
    wall = 0.5
    kp = 10
    kv = 0.05

    # Setting impulse wall force to be false
    impulse_wall = False

    # Setting variables for impulse
    m = 0.001
    const_v = 0
    t = 0
    impulse_time = 0.020
    dt = 0.005

    while True:
        p = m1.encoder.pos_estimate
        v = m1.encoder.vel_estimate
        print("pos: %f, vel: %f" % (p, v))
        torque = 0.0

        if p >= wall:
            # If within wall send the spring mass damper send spring mass torque
            torque += -1 * kp * (p - wall) + -1 * kv * v

            # Turn on Impulse force if needed and set vel constant
            if not impulse_wall and t == 0:
                impulse_wall = True
                const_v = v + 0.0

        if impulse_wall:
            # Send impulse force if impulse false is positive
            if t < impulse_time:
                t += dt
                torque += -1 * (m * const_v / dt)
            else:
                impulse_wall = False

        if p < wall - 0.1:
            # if pos away from wall, reset time to be 0 to allow for impulse force
            t = 0

        m1.controller.input_torque = torque


main()
