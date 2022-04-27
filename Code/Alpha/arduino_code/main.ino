// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ArduinoEigen.h>
#include <math.h>

//#define PI 3.141592653

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////
HardwareSerial& odrive_serial = Serial1;

ODriveArduino odrive(odrive_serial);

// setup motor axis constant
int m_0 = 0;
int m_1 = 1;

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

/*
Vector<float, 4> pull(){
    //arduino function to pull current encoder values w/ some timestamp
    //random values for testing
    //row
    Vector<float, 4> encoders {{p_0,p_1,p_2,60}};
    return encoders;
}
*/


//frame correction: 
Vector<float, 6> wall_vec(){
    // hard code wall - otherwise, given from UNITY
   Matrix4f Tsp {{0.,0.,1.,0.},{1.,0.,0.,.4},{0.,1.,0.,0.},{0.,0,0.,1.}};
   Matrix4f Tpb {{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};
   Matrix4f Tsw {{1.,0.,0.,0.},{0.,-1.,0.,1.1},{0.,0.,1.,0.},{0.,0,0.,1.}};
   Matrix4f Tsb = Tsp*Tpb;
   Matrix4f Tbw = Tsb.inverse()*Tsw;
   Vector<float, 6> wall {{Tbw(0,1), Tbw(1,1), Tbw(2,1), Tbw(0,3), Tbw(1,3),Tbw(2,3)}};
   return wall;
}




Vector<float, 4> theta_conv(Vector3f GR, float Theta_to_pos, Vector<float, 4> motor_pos){
    //Vector<float, 4> motor_pos {{p_0,p_1,p_2,60}};
   
    //correction for homing + math
    // homing adjustment:
    float mp1 = 1*motor_pos(0) +1.45;
    float mp2 = 1*motor_pos(1) +2.5;
    float mp3 = motor_pos(2) +0;
    
    Vector<float, 3> mp_vec {{mp1,mp2,mp3}};

    
    Vector<float, 3> theta_1 =GR.cwiseProduct(Theta_to_pos*mp_vec);
    
    Vector<float, 4> theta_j(theta_1(0), theta_1(1), theta_1(2), motor_pos(3));
    return theta_j;
}


Vector<float,7> trig_func(Vector3f GR, float Theta_to_pos, Vector<float, 4> motor_pos){
    float theta1, theta2, theta3, time, s1, s2, s3, c1, c2, c3 ;
    Vector<float, 4> temp = theta_conv(GR, Theta_to_pos, motor_pos);
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
    
     //   s1 = trig_matrix2[0,1];
      //  s2 = trig_matrix2[0,1];
       // s3 = trig_matrix2[0,2];
       // c1 = trig_matrix2[1,0];
        //c2 = trig_matrix2[1,1];
        //c3 = trig_matrix2[1,2];
    
    t = trig_mat(6);
    
   
    x = a*trig_mat(5)*(trig_mat(3)- trig_mat(4));  
    y = a*(trig_mat(0) - trig_mat(1));
    z = a*trig_mat(2)*(trig_mat(0)- trig_mat(1));
    Vector<float, 4> pos {{x,y,z,t}};

    return pos;
}

 
 Vector<float,1> normal_dist( Vector<float, 4>  e_e){
    float px,py,pz,nx,ny,nz;
    VectorXf temp = wall_vec ();
   
    Vector<float, 3> norm {temp(0),temp(1),temp(2)};
    Vector<float, 3> point = {temp(3),temp(4),temp(5)};
    Vector<float, 3> point_vec = point-e_e.head(3);
    Vector<float,1> norm_dist {{ point_vec.dot(norm)}};
    return norm_dist;

}

Vector<float, 4> vel_ee( Vector<float, 4> e_e1,  Vector<float, 4> e_e2){
    float dt = e_e2(3)- e_e1(3);
    Vector<float, 4> temp  = (e_e2 - e_e1)/dt;
    Vector<float, 4> velocity {{temp(0), temp(1), temp(2), dt}};
    return velocity;
}

Vector<float, 3> accel_ee( Vector<float, 4> e_e1,  Vector<float, 4> e_e2,  Vector<float, 4> e_e3){ 
    
    Vector<float, 4> v1= vel_ee(e_e1, e_e2);
    Vector<float, 4> v2= vel_ee(e_e2, e_e3);
    float dt1 = v1(3);
    float dt2 = v2(3);
    float dt_accel = dt2-dt1;
    Vector<float, 4> temp = (v2-v1)/dt_accel; 
    Vector<float, 3> accel {{temp(0), temp(1), temp(2)}};
    return accel;
}


Vector<float, 3> jacobian_torque(VectorXf trig_matrix, float a, Vector<float, 3> F,Vector<float, 3> GR){
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
    Vector<float, 3> Tau = inv *(F);
    Vector<float, 3> T_motor = GR.cwiseProduct(Tau);
    return (T_motor);
}


Vector<float, 3> whiteboard(Vector<float, 3> GR, float k, float c, float a, float theta_to_pos, Vector<float, 6> wall, Vector<float, 4> motor_pos){
    Vector<float,7> trig_mat1 = trig_func(GR,theta_to_pos, motor_pos);
    Vector<float,7> trig_mat2 = trig_func(GR,theta_to_pos,motor_pos);
    
    trig_mat2(6) = 70;// only here for testing, trig-mat should be different

    Vector<float, 4> ee1 = ee_pos(trig_mat1, a);
    Vector<float, 4> ee2 = ee_pos(trig_mat2, a);
    Vector<float, 1> norm = normal_dist(ee2);
    float dist = norm(0);
    Vector<float, 4> velocity = vel_ee(ee1, ee2);
    Vector<float, 3> Forces = k*(dist * wall.head(3)) -(c* (velocity.head(3)).dot(wall.head(3))*wall.head(3));
    Vector<float, 3> motor_torque =  jacobian_torque(trig_mat2, a, Forces, GR);
    Vector<float, 3> Zero_T {{0,0,0}};
    if( dist >0){
      return(motor_torque);
    }
    else{
      return(Zero_T);
    }
}


//global variables:
float p_0;
float v_0;
float p_1;
float v_1;
float p_2;
float v_2;

Vector<float, 4> ee_1;
Vector<float, 4> ee_2;

float a = 0.4767;
float k = 1;
float c = 0.5;
 
float Theta_to_pos = 2*3.141592653;
//Vector3f GR {{.10,0.22,10}};
Vector3f GR {{0.1,0.1,0.1}};
 
void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");

  // Set Torque to 0
  float t_0 = 0.0;
  odrive.SetCurrent(m_0, t_0);
  odrive.SetCurrent(m_1, t_0);
  
  // Set Torque Control
  int control_mode = CONTROL_MODE_TORQUE_CONTROL;
  odrive_serial << "w axis" << m_0 << ".controller.config.control_mode " << control_mode << '\n';
  odrive_serial << "w axis" << m_1 << ".controller.config.control_mode " << control_mode << '\n';

  // Set up closed loop control
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive_serial << "w axis" << m_0 << ".requested_state " << requested_state << '\n';
  odrive_serial << "w axis" << m_1 << ".requested_state " << requested_state << '\n';

  
  float time = millis();
    
  Vector<float, 4> motor_pos {{0,0,0,time/1000}};
  Vector<float, 4> ee_1 = ee_pos(trig_func(GR, Theta_to_pos,motor_pos), a);
  Vector<float, 4> ee_2 = ee_pos(trig_func(GR, Theta_to_pos,motor_pos), a);
    
}

void loop() {
  while (true) {
    
    p_0 = odrive.GetPosition(m_0);
    // v_0 = odrive.GetVelocity(m_0);
    p_1 = odrive.GetPosition(m_1);
    // v_1 = odrive.GetVelocity(m_0);
    // p_2 = odrive.GetPosition(m_0);
    // v_2 = odrive.GetVelocity(m_0);
    float time = millis();
    
    Vector<float, 4> motor_pos {{p_0,p_1,0,time/1000}};
  
    VectorXf trig_matrix = trig_func(GR, Theta_to_pos,motor_pos);
 
    Vector<float, 4> ee_3 = ee_pos(trig_matrix, a);

    Vector<float, 6> wall = wall_vec();

    Vector<float,1> norm = normal_dist(ee_3);

    float dist = norm(0);

    Vector<float, 4> velocity = vel_ee( ee_1, ee_2);
    Vector<float, 3> acceleration = accel_ee( ee_1,  ee_2, ee_3);

    Vector<float, 3> Forces = k*(dist * wall.head(3)) -(c* (velocity.head(3)).dot(wall.head(3))*wall.head(3));

    
    Vector<float, 3> Tau = whiteboard(GR,  k,  c,  a,  Theta_to_pos, wall, motor_pos);//jacobian_torque( trig_matrix,  a, Forces, GR);

    
    

    Serial.println("Pos: ");
    Serial << p_0 << '\n';
    Serial << p_1 << '\n';
    
    /*
    Serial.println("e-e position: ");
    Serial << ee_3(0) << '\n';
    Serial << ee_3(1) << '\n';
    Serial << ee_3(2) << '\n';
    */
    

    Serial.println("e-e position (inches): ");
    Serial << 39.37*ee_3(0) << '\n';
    Serial << 39.37*ee_3(1) << '\n';
    Serial << 39.37*ee_3(2) << '\n';

    /*

     Serial.println("wall: ");
    Serial << wall(0)<< '\n';
    Serial << wall(1)<< '\n';
    Serial << wall(2)<< '\n';
    Serial << wall(3)<< '\n';
    Serial << wall(4)<< '\n';
    Serial << wall(5)<< '\n';

    */
   
    Serial.println("Norm: ");
    Serial << dist << '\n';

    /*
    Serial.println("velocity: ");
    Serial << velocity(0)<< '\n';
    Serial << velocity(1)<< '\n';
    Serial << velocity(2)<< '\n';

    


    
    Serial.println("acceleration: ");
    Serial << acceleration(0)<< '\n';
    Serial << acceleration(1)<< '\n';
    Serial << acceleration(2)<< '\n';

    */
    
    Serial.println("Torques: ");
    Serial << Tau(0)<< '\n';
    Serial << Tau(1)<< '\n';
    Serial << Tau(2)<< '\n';
    
    

    Serial.println("Force: ");
    //Serial << typeid(Forces(0)).name()<< '\n';
    Serial << Forces(0)<< '\n';
    Serial << Forces(1)<< '\n';
    Serial << Forces(2)<< '\n';
    
    
  

    

    


    
    
    ee_1= ee_2;
    ee_2= ee_3;

    
    
    
  }
  
}
