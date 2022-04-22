#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <helpers.hpp>

using Eigen::MatrixXf;
using Eigen::VectorXf;

VectorXf whiteboard(VectorXf GR, float k, float c, float a, float theta_to_pos);


VectorXf inertia(VectorXf GR, float m, float a, float theta_to_pos);
