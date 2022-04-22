#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

using Eigen::MatrixXf;
using Eigen::VectorXf;

VectorXf pull();

VectorXf wall_vec();

VectorXf theta_conv(VectorXf GR, float Theta_to_pos);

MatrixXf trig_func(VectorXf GR, float Theta_to_pos);

VectorXf ee_pos(MatrixXf trig_matrix, float a);

VectorXf p_vec(VectorXf p1, VectorXf p2);

VectorXf normal_dist(VectorXf e_e);

VectorXf vel_ee(VectorXf e_e1, VectorXf e_e2);

VectorXf accel_ee(VectorXf e_e1, VectorXf e_e2, VectorXf e_e3);

VectorXf jacobian_torque(MatrixXf trig_matrix, float a, VectorXf F,VectorXf GR);