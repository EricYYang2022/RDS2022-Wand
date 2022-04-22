#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <helpers.hpp>

using Eigen::MatrixXf;
using Eigen::VectorXf;

VectorXf whiteboard(VectorXf GR, float k, float c, float a, float theta_to_pos){
    MatrixXf trig_mat1 = trig_func(GR,theta_to_pos);
    MatrixXf trig_mat2 = trig_func(GR,theta_to_pos);

    VectorXf ee1 = ee_pos(trig_mat1, a);
    VectorXf ee2 = ee_pos(trig_mat2, a);
    VectorXf dist = normal_dist(ee2);
    VectorXf v_wt = vel_ee(ee1, ee2);
    VectorXf F = k*dist -c*v_wt.segment(1,3);
    VectorXf motor_torque =  jacobian_torque(trig_mat2, a, F, GR);
    return(motor_torque);
}


VectorXf inertia(VectorXf GR, float m, float a, float theta_to_pos){
    MatrixXf trig_mat1 = trig_func(GR,theta_to_pos);
    MatrixXf trig_mat2 = trig_func(GR,theta_to_pos);
    MatrixXf trig_mat3 = trig_func(GR,theta_to_pos);

    VectorXf ee1 = ee_pos(trig_mat1, a);
    VectorXf ee2 = ee_pos(trig_mat2, a);
    VectorXf ee3 = ee_pos(trig_mat3, a);

    VectorXf accel = accel_ee(ee1, ee2, ee3);
    VectorXf F_gobj {0,0,m*-9.81};
    VectorXf F = -(m*accel.segment(1,3) - F_gobj);

    VectorXf motor_torque =  jacobian_torque(trig_mat3, a, F, GR);
    return(motor_torque);
}
