import numpy as np
import csv 

def extrapolate_position(p1, p2, dt, t):
    pdot = np.subtract(p2, p1)/dt
    pnew = np.add(p2, pdot*t)
    return pnew

def extrapolate_rotation(q0, q1, dt, t):
    phi = np.arccos(np.dot(q0, q1))
    t_star = t/dt
    q = np.add(np.multiply(np.sin((1-t_star)*phi)/np.sin(phi),q0), np.multiply(np.sin(t_star*phi)/np.sin(phi),q1))
    return q

def quaternion_to_transformation(q, p):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    T = [ [2*(q0*q0+q1*q1) - 1, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2), p[0]], 
        [ 2*(q1*q2+q0*q3), 2*(q0*q0+q2*q2)-1, 2*(q2*q3-q0*q1), p[1]], 
        [ 2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 2*(q0*q0+q3*q3)-1, p[2]],
        [0, 0, 0, 1]]
    return T

"""public final void set(Matrix4f m1) {
	w = Math.sqrt(1.0 + m1.m00 + m1.m11 + m1.m22) / 2.0;
	double w4 = (4.0 * w);
	x = (m1.m21 - m1.m12) / w4 ;
	y = (m1.m02 - m1.m20) / w4 ;
	z = (m1.m10 - m1.m01) / w4 ;
}"""

def transformation_to_quaternion(T):
    p = [row[3] for row in T][0:3]
    R = [row[0:3] for row in T][0:3]
    q0 = np.sqrt(1 + R[0][0] + R[1][1] + R[2][2])/2.0
    w4 = 4*q0
    q1 = (R[1][2] - R[2][1])/w4
    q2 = (R[2][0] - R[0][2])/w4
    q3 = (R[0][1] - R[1][0])/w4
    return [q0, q1, q2, q3], p

T1 = [[1, 0, 0, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]]
T2 = [[1, 0, 0, 0], [0, np.cos(0.01), -np.sin(0.01), np.cos(0.01)], [0, np.sin(0.01), np.cos(0.01), np.sin(0.01)], [0, 0, 0, 1]]

q_0, p_0 = transformation_to_quaternion(T1)
q_1, p_1 = transformation_to_quaternion(T2)

qnew = extrapolate_rotation(q_0, q_1, 0.1, 0.05)
pnew = extrapolate_position(p_0, p_1, 0.1, 0.05)

print(quaternion_to_transformation(qnew, pnew))
