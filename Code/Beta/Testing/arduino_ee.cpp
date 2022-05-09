#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>



using Eigen::MatrixXf;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::VectorXf;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::RowVector3f;
using Eigen:: Matrix;
using Eigen:: Vector;


Vector4f pull(){
    //arduino function to pull current encoder values w/ some timestamp
    //random values for testing
    //row
    Vector4f test {{1,2,-1,60}};
    return test;

}




//frame correction: 
Vector<float, 6> wall_vec(){
    // hard code wall - otherwise, given from UNITY
   Matrix4f Tpb {{1.,2.,1.,1.},{1.,1.,2.,2.},{1.,1.,2.,1.},{1.,1.,9.,1.}};
   Matrix4f Tsw {{2.,4.,1.,1.},{1.,2.,1.,2.},{1.,1.,1.,1.},{1.,8,2.,1.}};
   Matrix4f Tsp {{1.,0.,1.,1.},{1.,1.,1.,2.},{3.,1.,8.,1.},{2.,1.,1.,1.}};

   Matrix4f Tsb = Tsp*Tpb;
   Matrix4f Tbw = Tsw * Tsb.inverse();
   Vector<float, 6> wall {{Tbw(0,1), Tbw(1,1), Tbw(2,1), Tbw(0,3), Tbw(1,3),Tbw(2,3)}};
   return wall;
}



Vector4f theta_conv(Vector3f GR, float Theta_to_pos){
    Vector4f motor_pos = pull();
    Vector3f theta_1 =GR.cwiseProduct(Theta_to_pos*motor_pos.head(3));
    Vector4f theta_j(theta_1(0), theta_1(1), theta_1(2), motor_pos(3));
    return theta_j;
}


Vector<float,7> trig_func(Vector3f GR, float Theta_to_pos){
    float theta1, theta2, theta3, time, s1, s2, s3, c1, c2, c3 ;
    Vector4f temp = theta_conv(GR, Theta_to_pos);
    theta1 = temp(0);
    theta2 = temp(1);
    theta3 = temp(2);
    time = temp(3);

    s1 = sin(theta1);
    s2 = sin(theta2);
    s3 = sin(theta3);
    c1 = cos(theta1);
    c2 = cos(theta2);
    c3 = cos(theta3);
    Vector<float,7> trig_matrix {{s1, s2, s3, c1,c2,c3, time}};
    return trig_matrix;
}

Vector4f ee_pos(VectorXf trig_mat, float a){
    float x,y,z,t;
    
     //   s1 = trig_matrix2[0,1];
      //  s2 = trig_matrix2[0,1];
       // s3 = trig_matrix2[0,2];
       // c1 = trig_matrix2[1,0];
        //c2 = trig_matrix2[1,1];
        //c3 = trig_matrix2[1,2];
    
    t = trig_mat(6);
    
   
    x = (trig_mat(3)- trig_mat(4));  
    y = a*(trig_mat(0) - trig_mat(1));
    z = a*trig_mat(2)*(trig_mat(0)- trig_mat(1));
    Vector4f pos {{x,y,z,t}};

    return pos;
}



Vector3f normal_dist(Vector4f e_e){
    float px,py,pz,nx,ny,nz;
    VectorXf temp = wall_vec ();
   
    Vector3f norm {temp(0),temp(1),temp(2)};
    Vector3f p {temp(3),temp(4),temp(5)};
    Vector3f p_v = p-e_e.head(3);//p_vec(p, e_e);
    Vector3f norm_dist = p_v.cwiseProduct(norm);
    return norm_dist;

}

Vector4f vel_ee(Vector4f e_e1, Vector4f e_e2){
    float dt = e_e2(3)- e_e1(3);
    Vector4f temp  = (e_e2 - e_e1)/dt;
    Vector4f velocity {{temp(0), temp(1), temp(2), dt}};
    return velocity;
}

Vector3f accel_ee(Vector4f e_e1, Vector4f e_e2, Vector4f e_e3){ 
    
    Vector4f v1= vel_ee(e_e1, e_e2);
    Vector4f v2= vel_ee(e_e2, e_e3);
    float dt1 = v1(3);
    float dt2 = v2(3);
    float dt_accel = dt2-dt1;
    Vector4f temp = (v2-v1)/dt_accel; 
    Vector3f accel {{temp(0), temp(1), temp(2)}};
    return accel;
}



Vector3f jacobian_torque(VectorXf trig_matrix, float a, Vector3f F,Vector3f GR){
    // Vector<float,7> trig_matrix {{s1, s2, s3, c1,c2,c3, time}};
    float J11,J12,J13,J21,J22,J23,J31,J32,J33;
    J11 = -a*trig_matrix(0)* trig_matrix(5);
    J12 = a*trig_matrix(1)* trig_matrix(5);
    J13 = -a*trig_matrix(3)* trig_matrix(2) + a*trig_matrix(4)* trig_matrix(3);
    J21 = a*trig_matrix(3);
    J22 = -a*trig_matrix(4);
    J23 = 0;
    J31 = -a*trig_matrix(0)* trig_matrix(2);
    J32 = a*trig_matrix(1)* trig_matrix(5);
    J33 = a*trig_matrix(3)* trig_matrix(5) -a*trig_matrix(4)* trig_matrix(5);

    Matrix3f J {{J11,J21,J31},{J12,J22,J32},{J13,J23,J33}};
    Matrix3f inv = J.inverse();
    Vector3f Tau = inv *(F);
    Vector3f T_motor = GR.cwiseProduct(Tau);
    return (T_motor);
}


int main(){
    float a = 0.45;
    float Theta_to_pos = 2*3.14159;
    Vector3f GR {{10,0.22,10}};
    std::cout << "The e-e position for the given encoder values are:\n "<<  ee_pos(trig_func(GR, Theta_to_pos), a)<< std::endl;
    return(1);
}
