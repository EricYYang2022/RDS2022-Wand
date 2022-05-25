// includes
#include <rds_helper_1.h>


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
Vector<float, 6> wall_vec(){
    // hard code wall - otherwise, given from UNITY

   float ds = 0.4;
   float distance= 0.3; //change back to 0.5 m
   
   float dsw= ds+distance;
   Matrix4f Tsp {{0.,0.,1.,0.},{1.,0.,0.,ds},{0.,1.,0.,0.},{0.,0,0.,1.}};
   Matrix4f Tpb {{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};
   Matrix4f Tsw {{1.,0.,0.,0.},{0.,-1.,0.,dsw},{0.,0.,1.,0.},{0.,0,0.,1.}};
   Matrix4f Tsb = Tsp*Tpb;
   Matrix4f Tbw = Tsb.inverse()*Tsw;
   Vector<float, 6> wall {{Tbw(0,1), Tbw(1,1), Tbw(2,1), Tbw(0,3), Tbw(1,3),Tbw(2,3)}};
   return wall;
}


Vector<float, 4> theta_conv(Vector<float, 4> motor_pos, Vector<float, 3> GR, float Theta_to_pos = 2*3.141592653){
    //Vector<float, 4> motor_pos {{p_0,p_1,p_2,60}};
   
    //correction for homing + math
    // homing adjustment:
    float mp1 = 1*motor_pos(0) + 1.25;
    float mp2 = 1*motor_pos(1) + 2.5;
    float mp3 = motor_pos(2) - 0.783;
    
    Vector<float, 3> mp_vec {{mp1,mp2,mp3}};

    Vector<float, 3> theta_1 =GR.cwiseProduct(Theta_to_pos*mp_vec);
    
    Vector<float, 4> theta_j(theta_1(0), theta_1(1), theta_1(2), motor_pos(3));
    return theta_j;
}


Vector<float,7> trig_func(Vector<float, 4> motor_pos, Vector<float, 3> GR){
    float theta1, theta2, theta3, time, s1, s2, s3, c1, c2, c3 ;
    Vector<float, 4> temp = theta_conv(motor_pos, GR);
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

Vector<float, 4> ee_pos(VectorXf trig_mat, float a){
    float x,y,z,t;
    
    // s1 = trig_matrix2[0,1];
    // s2 = trig_matrix2[0,1];
    // s3 = trig_matrix2[0,2];
    // c1 = trig_matrix2[1,0];
    // c2 = trig_matrix2[1,1];
    // c3 = trig_matrix2[1,2];
    
    t = trig_mat(6);
    
    x = a*trig_mat(5)*(trig_mat(3)- trig_mat(4));  
    y = a*(trig_mat(0) - trig_mat(1));
    z = a*trig_mat(2)*(trig_mat(3)- trig_mat(4));
    Vector<float, 4> pos {{x,y,z,t}};

    return pos;
}

 
 Vector<float,1> normal_dist(Vector<float, 4>  e_e, Vector<float, 6> temp){

    Vector<float, 3> norm {temp(0),temp(1),temp(2)};
    Vector<float, 3> point = {temp(3),temp(4),temp(5)};
    Vector<float, 3> point_vec = point-e_e.head(3);
    Vector<float,1> norm_dist {{ point_vec.dot(norm)}};
    return norm_dist;

}

Vector<float, 4> vel_ee(Vector<float, 4> e_e1,  Vector<float, 4> e_e2){
    float dt = e_e2(3)- e_e1(3);
    Vector<float, 4> temp  = (e_e2 - e_e1)/dt;
    Vector<float, 4> velocity {{temp(0), temp(1), temp(2), dt}};
    return velocity;
}

Vector<float, 3> accel_ee(Vector<float, 4> e_e1, Vector<float, 4> e_e2, Vector<float, 4> e_e3){ 
    
    Vector<float, 4> v1= vel_ee(e_e3, e_e2);
    Vector<float, 4> v2= vel_ee(e_e2, e_e1);
    float dt1 = v1(3);
    float dt2 = v2(3);
    float dt_accel = dt2-dt1;
    Vector<float, 4> temp = (v2-v1)/dt_accel; 
    Vector<float, 3> accel {{temp(0), temp(1), temp(2)}};
    return accel;
}


Vector<float, 3> jacobian_torque(VectorXf trig_matrix, Vector<float, 3> F, Vector<float, 3> GR, float a){
    // Vector<float,7> trig_matrix {{s1, s2, s3, c1,c2,c3, time}};
    float J11,J12,J13,J21,J22,J23,J31,J32,J33;
    J11 = -a*trig_matrix(0)* trig_matrix(5);
    J12 = a*trig_matrix(1)* trig_matrix(5);
    J13 = -a*trig_matrix(3)* trig_matrix(2) + a*trig_matrix(4)* trig_matrix(2);
    J21 = a*trig_matrix(3);
    J22 = -a*trig_matrix(4);
    J23 = 0;
    J31 = -a*trig_matrix(0)* trig_matrix(2);
    J32 = a*trig_matrix(1)* trig_matrix(2);
    J33 = a*trig_matrix(3)* trig_matrix(5) -a*trig_matrix(4)* trig_matrix(5);

    Matrix3f J {{J11,J21,J31},{J12,J22,J32},{J13,J23,J33}};
    Matrix3f inv = J.inverse();
    Vector<float, 3> Tau = inv *(F);
    Vector<float, 3> T_motor = GR.cwiseProduct(Tau);
    return (T_motor);
}


Vector<float, 3> whiteboard(Vector<float, 4> motor_pos, Vector<float, 3> GR, Ref<Vector<float, 4>> ee2, float k = 20, float c = 0, float a = 0.45){
    
    Vector<float, 6> wall = wall_vec();

    Vector<float,7> trig_mat1 = trig_func(motor_pos, GR);
    // Vector<float,7> trig_mat2 = trig_func(motor_pos, GR);

    Vector<float, 4> ee1 = ee_pos(trig_mat1, a);
    // Vector<float, 4> ee2 = ee_pos(trig_mat2, a);
    Vector<float, 1> norm = normal_dist(ee2, wall);
    float dist = norm(0);
    
    Vector<float, 4> nee1 = (ee1.head(3).dot(wall.head(3)))*wall.head(4);
    Vector<float, 4> nee2 = (ee2.head(3).dot(wall.head(3)))*wall.head(4);
    
    nee1(3) = ee1(3);
    nee2(3) = ee2(3);
    
    Vector<float, 4> nvelocity = vel_ee(nee1, nee2);

    //Vector<float, 4> velocity = vel_ee(ee1, ee2);
    
    Vector<float, 3> Forces = k*(dist * wall.head(3)) -(c* nvelocity.head(3));
    Vector<float, 3> motor_torque =  jacobian_torque(trig_mat1, Forces, GR, a);
    Vector<float, 3> Zero_T {{0,0,0}};

    ee2 = ee_pos(trig_mat1, a);
    
    if( dist >0.0005){
      return(motor_torque);
    }
    else{
      return(Zero_T);
      
    }
    
}


Vector<float, 3> interia(Vector<float, 4> motor_pos, Vector<float, 3> GR, Ref<Vector<float, 4>> ee2, Ref<Vector<float, 4>> ee3, float m, int button, float a = 0.45){
    
    Vector<float,7> trig_mat1 = trig_func(motor_pos, GR);
    // Vector<float,7> trig_mat2 = trig_func(motor_pos, GR);

    Vector<float, 4> ee1 = ee_pos(trig_mat1, a);
    // Vector<float, 4> ee2 = ee_pos(trig_mat2, a);
    

    Vector<float, 3> accel= accel_ee(ee1,ee2,ee3);


    //This is based off the direction of gravity being in the -y direction in our base frame axis
    Vector<float, 3> F_g {{0,-9.81,0}};
    
    Vector<float, 3> Forces = -1*(m*accel.head(3) - F_g.head(3));
    Vector<float, 3> motor_torque =  jacobian_torque(trig_mat1, Forces, GR, a);
    Vector<float, 3> Zero_T {{0,0,0}};

    ee3 = ee2;
    ee2 = ee_pos(trig_mat1, a);
    
    
    if( button > .5){
      return(motor_torque);
    }
    else{
      return(Zero_T);
      
    }
    
}

/*
Vector<float, 3> inertia(Vector<float, 3> GR, float m, float a, float theta_to_pos, Vector<float, 4> motor_pos){
    Vector<float,7> trig_mat1 = trig_func(GR,theta_to_pos, motor_pos);
    Vector<float,7> trig_mat2 = trig_func(GR,theta_to_pos, motor_pos);
    Vector<float,7> trig_mat3 = trig_func(GR,theta_to_pos, motor_pos);

    Vector<float, 4> ee1 = ee_pos(trig_mat1, a);
    Vector<float, 4> ee2 = ee_pos(trig_mat2, a);
    Vector<float, 4> ee3 = ee_pos(trig_mat3, a);

    Vector<float, 3> accel = accel_ee(ee1, ee2, ee3);
    float g = -9.81;
    Vector<float, 3> F_gobj {{0,0,m*g}};
    Vector<float, 3> F = -(m*accel.head(3) - F_gobj);

    Vector<float, 3> motor_torque =  jacobian_torque(trig_mat3, a, F, GR);
    return(motor_torque);
}
*/
