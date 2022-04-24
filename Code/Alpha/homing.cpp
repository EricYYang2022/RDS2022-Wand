// (column, row)

//g++ 2R_2D_Kinematics.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include "matplotlibcpp.h"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include<vector>
#include "homing.hpp"


using Eigen::MatrixXd;
using Eigen::MatrixXf;

namespace plt = matplotlibcpp;



MatrixXf FWD_Kinemeatic (float theta1, float theta2, float theta3){

    float c1 = std::cos(theta1);
    float c2 = std::cos(theta2);
    float c3 = std::cos(theta3);
    float s1 = std::sin(theta1);
    float s2 = std::sin(theta2);
    float s3 = std::sin(theta3);

    
    float a = 1; //set to our a


    MatrixXf FWD(3,1);

    FWD(0,0) = a*c1*c3 -a*c2*c3;
    FWD(1,0) = a*s1-a*s2;
    FWD(2,0) = a*s1*s3 -a*s2*s3;




    return FWD;
}




MatrixXf computePseudoInverse(float theta1, float theta2, float theta3)
{

    MatrixXf J(3,3);
    
    float c1 = std::cos(theta1);
    float c2 = std::cos(theta2);
    float c3 = std::cos(theta3);
    float s1 = std::sin(theta1);
    float s2 = std::sin(theta2);
    float s3 = std::sin(theta3);

    
    float a = 1; //set to our a

    J(0,0) = -a*s1*c3; J(0,1) = a*s2*c3 ; J(0,2)=-a*c1*s3 + a*c2*s3;
    J(1,0) = a*c1; J(1,1) = -a*c2 ; J(1,2)=0;
    J(2,0) = -a*s1*s3; J(2,1) = a*s2*s3; J(2,2)= a*c1*c3 - a*c2*c3;

    Eigen::MatrixXf pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();


    return pinvJ;
}


/* IS THERE A INVERSE FUNCTION?

MatrixXf invJacobian( float theta1, float theta2){

    MatrixXf invJ (2,2);

    float c1 = std::cos(theta1);
    float c2 = std::cos(theta2);
    float c3 = std::cos(theta3);
    float s1 = std::sin(theta1);
    float s2 = std::sin(theta2);
    float s3 = std::sin(theta3);

    
    float a = 1; //set to our a


    float factor = 1/(l1 * l2 * s2);

    invJ(0,0) = l2 * c12; 
    invJ(0,1) = l2 * s12;
    invJ(1,0) = -l1 * c1 - l2 * c12;
    invJ(1,1) = -l1 * s1 - l2 * s12;



    return factor*invJ;


} */

MatrixXf applyAlgorithm(float Xd_x,float Xd_y, float Xd_z){



   
    MatrixXf e(3,1);
    MatrixXf Xd (3,1);
    MatrixXf xiFWD (3,1);
    MatrixXf FWD (3,3);

    MatrixXf i_1_theta (3,1);
    MatrixXf i_theta (3,1);

//change for test
    i_theta(0,0) =  M_PI/3;
    i_theta(1,0) = M_PI/4; 
    i_theta(3,0) = M_PI/4; 


 
   
    FWD = FWD_Kinemeatic(i_theta(0,0), i_theta(1,0), i_theta(3,0));

//change for test

    Xd(0,0) = Xd_x; 
    Xd(1,0) = Xd_y;
    Xd(2,0) = Xd_z; 
    
    e = Xd - FWD;





    while((std::abs(e(0,0)) > 0.000001) || (std::abs(e(1,0)) > 0.000001))
    {

        MatrixXf invJ = computePseudoInverse(i_theta(0,0), i_theta(1,0),i_theta(2,0) );

 
        
        i_1_theta = i_theta + invJ * e;

        FWD = FWD_Kinemeatic(i_1_theta(0,0), i_1_theta(1,0), i_theta(2,0));
       

        e = Xd - FWD;
 

        i_theta = i_1_theta;
        
        std::cout << i_theta* (180/M_PI) << std::endl;

        

    }

    return i_theta;
}


//this part Junior wrote


float homing(){
    
    // call for or input wand and puck position 

    //set motor position count to zero

    MatrixXf T_sp(4,4);
    MatrixXf T_pb(4,4);
    MatrixXf T_ce(4,4);
    MatrixXf T_be(4,4);
    MatrixXf thetas(3,1);

    //Matrix math: T_be= (Tsp*Tpb)^-1 *Tsc *T_ce


    thetas = applyAlgorithm(T_be(0,3),T_be(1,3), T_be(2,3));

    float off_thetas[3]={thetas(0,0),thetas(1,0), thetas(2,0)};
    
    return(off_thetas);

}