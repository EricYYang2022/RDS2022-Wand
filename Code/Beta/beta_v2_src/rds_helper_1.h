#ifndef RDS_WAND_H
#define RDS_WAND_H

// includes
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ArduinoEigen.h>
#include <math.h>


using namespace Eigen;
using Eigen::MatrixXf;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::VectorXf;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::RowVector3f;
using Eigen:: Matrix;
using Eigen:: Vector;
using Eigen::Ref;


Vector<float, 6> wall_vec();
Vector<float, 4> theta_conv(Vector<float, 4> motor_pos, Vector<float, 3> GR, float Theta_to_pos);
Vector<float,1> normal_dist( Vector<float, 4>  e_e);
Vector<float, 4> vel_ee( Vector<float, 4> e_e1,  Vector<float, 4> e_e2);
Vector<float, 3> accel_ee( Vector<float, 4> e_e1,  Vector<float, 4> e_e2,  Vector<float, 4> e_e3);
Vector<float, 3> jacobian_torque(VectorXf trig_matrix, Vector<float, 3> F,Vector<float, 3> GR, float a);
Vector<float,7> trig_func(Vector<float, 4> motor_pos, Vector<float, 3> GR);
Vector<float, 4> ee_pos(VectorXf trig_mat, float a);
Vector<float, 3> whiteboard(Vector<float, 4> motor_pos, Vector<float, 3> GR, Ref<Vector<float, 4>> ee2, float k, float c, float a);
Vector<float, 3> interia(Vector<float, 4> motor_pos, Vector<float, 3> GR, Ref<Vector<float, 4>> ee2, Ref<Vector<float, 4>> ee3, float a, float m, int button);

#endif //RDS_WAND_H
