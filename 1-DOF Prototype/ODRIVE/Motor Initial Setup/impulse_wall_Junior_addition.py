import odrive as o
from odrive.enums import *
import time


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
    mass = 0.005
    dt = 0.02
    t = 0
    hit_wall = False
    loop_time
    impulse_time = 50
    current_time = 0
    torque = 0
    
    #SMD stands for spring mass damper
    torqueSMD=0
    torque_prev=0
    torqueSMD_prev=0
    p_prev=0
   
   
   #need to find these values
    max_torque_change=0
    v_min=0
    p_min_change=0
    
    #f = open("test.csv", "w")
    start_time = time.time()
    while current_time<5:
        p = m1.encoder.pos_estimate
        v = m1.encoder.vel_estimate
        print(p)
        print("pos: %f, vel: %f" %(p,v))
        if p >= wall:
            if not hit_wall:
                hit_wall = True
                const_v = v + 0.0
            if v<v_min or abs(p-p_prev)<p_min_change
                torque=torque_prev
            else:
                torqueSMD = -1 * kp * (p - wall) + -1 * kv * v
                if abs(torqueSMD-torqueSMD_prev)>max_torque_change
                    if (torqueSMD-torqueSMD_prev)<0
                        torqueSMD= torqueSMD_prev-max_torque_change
                    if (torqueSMD-torqueSMD_prev)>0
                        torqueSMD= torqueSMD_prev+max_torque_change
                if t < impulse_time:
                    torque =torqueSMD -1 * const_v * mass/dt
                    t += 1
            m1.controller.input_torque = torque
            torqueSMD_prev=torqueSMD
            torque_prev=torque
        else:
            m1.controller.input_torque = 0.0
            if hit_wall:
                t = 0
                hit_wall = False
        current_time += 1
        elapsed_time = time.time()-start_time
        print(elapsed_time/elapsed_time)

        #f.write("%f,%f,%f,%f\n"%(current_time,p,v,torque))            
    #f.close()
    #print("csv file generated")
    

main()


# Resources: https://github.com/but-i-love-pbj/odrive/blob/master/motor.py
# Resources: https://python.hotexamples.com/examples/odrive/-/find_any/python-find_any-function-examples.html
