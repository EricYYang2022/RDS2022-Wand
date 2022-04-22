
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <demos.hpp>


using Eigen::MatrixXf;
using Eigen::VectorXf;

VectorXf main(){
    VectorXf GR {1,1,1}; 
    float k = 1;
    float c = 1;
    float a = 1;
    float theta_to_pos = 1;
    float m = 10;

    VectorXf motor_torques1 = whiteboard(GR,k,c, a, theta_to_pos);
    VectorXf motor_torques2 = inertia(GR,m, a, theta_to_pos);

    return(motor_torques1,motor_torques2);
}
