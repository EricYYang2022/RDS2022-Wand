// includes
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




//frame correction: 
Vector<float, 6> wall_vec();

Vector<float, 4> theta_conv(Vector3f GR, float Theta_to_pos, Vector<float, 4> motor_pos);


Vector<float,7> trig_func(Vector3f GR, float Theta_to_pos, Vector<float, 4> motor_pos);

Vector<float, 4> ee_pos(VectorXf trig_mat, float a);
 
 Vector<float,1> normal_dist( Vector<float, 4>  e_e);

Vector<float, 4> vel_ee( Vector<float, 4> e_e1,  Vector<float, 4> e_e2);

Vector<float, 3> accel_ee( Vector<float, 4> e_e1,  Vector<float, 4> e_e2,  Vector<float, 4> e_e3);

Vector<float, 3> jacobian_torque(VectorXf trig_matrix, float a, Vector<float, 3> F,Vector<float, 3> GR);


Vector<float, 3> whiteboard(Vector<float, 3> GR, float k, float c, float a, float theta_to_pos, Vector<float, 6> wall, Vector<float, 4> motor_pos);

Vector<float, 3> inertia(Vector<float, 3> GR, float m, float a, float theta_to_pos);
  