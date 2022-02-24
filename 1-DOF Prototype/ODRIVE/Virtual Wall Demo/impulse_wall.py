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
    kp = 7.5
    kv = 0.05

    t = 0
    impulse_time = 0.01
    dt = 0.005

    # Impulse force and impulse start is turned off
    impulse_wall = False
    impulse_enabled = False
    m = 0.00005
    const_v = 0

    while True:
        p = m1.encoder.pos_estimate
        v = m1.encoder.vel_estimate
        print("pos: %f, vel: %f" % (p, v))
        torque = 0.0
        if p >= wall:
            # If within wall send the spring mass damper, send spring mass torque
            torque = -1 * kp * (p - wall) + -1 * kv * v

            if not impulse_wall:
                # Turn on Impulse force if needed and set vel constant
                impulse_wall = True
                impulse_enabled = True
                const_v = v + 0.0
                # impulse_time = const_v / 8

            if impulse_enabled:
                # Send impulse force if impulse false is positive
                if t < impulse_time:
                    torque += -1 * (m * const_v / dt)
                else:
                    impulse_enabled = False
            t += dt
        else:
            # Resets impulse start when outside the wall
            impulse_enabled = False
            t = 0
            torque = 0.0
            if p < wall - 0.08:
                # Resets impulse a certain away from the wall
                impulse_wall = False

        m1.controller.input_torque = torque


main()
