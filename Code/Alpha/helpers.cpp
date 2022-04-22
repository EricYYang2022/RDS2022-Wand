#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

using Eigen::MatrixXf;
using Eigen::VectorXf;


VectorXf pull(){
    //arduino function to pull current encoder values w/ some timestamp
    //random values for testing
    VectorXf test {1,1,1,60};
    return test;

}



VectorXf wall_vec(){
    VectorXf test(6,0);
    test= {1,1,1,0,0,0};
    return test;

}

VectorXf theta_conv(VectorXf GR, float Theta_to_pos){
    VectorXf motor_pos = pull();
    VectorXf theta_1 = GR *(Theta_to_pos*motor_pos.segment(1,3));
    VectorXf theta_j(theta_1.size() + motor_pos[4]);
    return theta_j;
}


VectorXf main(){
    return(pull());
}

MatrixXf trig_func(VectorXf GR, float Theta_to_pos){
    float theta1, theta2, theta3, time, s1, s2, s3, c1, c2, c3 ;
    MatrixXf {theta1, theta2, theta3, time} = theta_conv(GR, Theta_to_pos);
    s1 = sin(theta1);
    s2 = sin(theta2);
    s3 = sin(theta3);
    c1 = cos(theta1);
    c2 = cos(theta2);
    c3 = cos(theta3);
    MatrixXf trig_matrix {s1, s2, s3, c1,c2,c3, time};
    return trig_matrix;
}

VectorXf ee_pos(MatrixXf trig_matrix, float a){
    float x,y,z,t;
    
     //   s1 = trig_matrix2[0,1];
      //  s2 = trig_matrix2[0,1];
       // s3 = trig_matrix2[0,2];
       // c1 = trig_matrix2[1,0];
        //c2 = trig_matrix2[1,1];
        //c3 = trig_matrix2[1,2];
    
    t = trig_matrix[3];
    x = a* trig_matrix[1,2]*(trig_matrix[1,0]- trig_matrix[1,1]); 
    y = a*(trig_matrix[0,0] - trig_matrix[0,1]);
    z = a*trig_matrix[0,2]*(trig_matrix[0,0]- trig_matrix[0,1]);
    VectorXf pos {x,y,z,t};
    return pos;
}

VectorXf p_vec(VectorXf p1, VectorXf p2){
    VectorXf p_vector {p1[0] -= p2[0], p1[1] - p2[1], p1[2] - p2[2]};
    return p_vector; 
}


VectorXf normal_dist(VectorXf e_e){
    float px,py,pz,nx,ny,nz;
    VectorXf {px,py,pz,nx,ny,nz} = wall_vec ();
    
    
    VectorXf p {px,py,pz};
    VectorXf p_v = p_vec(p, e_e);
    VectorXf norm_dist = p_v * VectorXf{nx,ny,nz};
    return norm_dist;

}

VectorXf vel_ee(VectorXf e_e1, VectorXf e_e2){
    float dt = e_e2[3]- e_e1[3];
    VectorXf velocity {(e_e2[0]-e_e1[0])/dt,(e_e2[1]-e_e1[1])/dt, (e_e2[2]-e_e1[2])/dt, dt};
    return VectorXf {velocity, dt};
}

VectorXf accel_ee(VectorXf e_e1, VectorXf e_e2, VectorXf e_e3){
    VectorXf v1, v2; 
    float dt1, dt2;
    VectorXf {v1, dt1}= vel_ee(e_e1, e_e2);
    VectorXf {v2, dt2}= vel_ee(e_e2, e_e3);
    float dt_accel = dt2-dt1;
    VectorXf accel {(v2[0]-v1[0])/dt_accel,(v2[1]-v1[1])/dt_accel, (v2[2]-v1[2])/dt_accel};
    return VectorXf {accel, dt_accel};
}

VectorXf jacobian_torque(MatrixXf trig_matrix, float a, VectorXf F,VectorXf GR){
    
    float J11,J12,J13,J21,J22,J23,J31,J32,J33;
    J11 = -a*trig_matrix[0,0]* trig_matrix[1,2];
    J12 = a*trig_matrix[0,1]* trig_matrix[1,2];
    J13 = -a*trig_matrix[1,0]* trig_matrix[0,2] + a*trig_matrix[1,1]* trig_matrix[0,2];
    J21 = a*trig_matrix[1,0];
    J22 = -a*trig_matrix[1,1];
    J23 = 0;
    J31 = -a*trig_matrix[0,0]* trig_matrix[0,2];
    J32 = a*trig_matrix[0,1]* trig_matrix[0,2];
    J33 = a*trig_matrix[1,0]* trig_matrix[1,2] -a*trig_matrix[1,1]* trig_matrix[1,2];

    MatrixXf J {{J11,J21,J31},{J12,J22,J32},{J13,J23,J33}};
    MatrixXf inv = J.inverse();
    VectorXf Tau = inv*F;
    VectorXf T_motor = Tau*GR;
    return (T_motor);
}
