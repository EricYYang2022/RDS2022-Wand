#ifndef HOMING_HPP
#define HOMING_HPP

MatrixXf FWD_Kinemeatic (float theta1, float theta2, float theta3);

MatrixXf computePseudoInverse(float theta1, float theta2, float theta3);

MatrixXf invJacobian( float theta1, float theta2);

MatrixXf applyAlgorithm(float Xd_x,float Xd_y, float Xd_z);

float homing();

#endif