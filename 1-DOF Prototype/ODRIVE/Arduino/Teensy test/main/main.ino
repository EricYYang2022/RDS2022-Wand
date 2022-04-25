// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ArduinoEigen.h>
#include <math.h>


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
   Matrix4f Tpb {{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};
   Matrix4f Tsw {{2.,4.,1.,1.},{1.,2.,1.,2.},{1.,1.,1.,1.},{0.,0.,0.,1.}};
   Matrix4f Tsp {{1.,0.,1.,1.},{1.,1.,1.,2.},{3.,1.,8.,1.},{0.,0.,0.,1.}};

   Matrix4f Tsb = Tsp*Tpb;
   Matrix4f Tbw = Tsw * Tsb.inverse();
   Vector<float, 6> wall {{Tbw(0,1), Tbw(1,1), Tbw(2,1), Tbw(0,3), Tbw(1,3),Tbw(2,3)}};
   return wall;
}



Vector<float, 4> theta_conv(Vector3f GR, float Theta_to_pos, Vector<float, 4> motor_pos){
    //Vector<float, 4> motor_pos {{p_0,p_1,p_2,60}};
    Vector<float, 3> theta_1 =GR.cwiseProduct(Theta_to_pos*motor_pos.head(3));
    Vector<float, 4> theta_j(theta_1(0), theta_1(1), theta_1(2), motor_pos(3));

    // Serial << "theta_1_0:  " << theta_1(0) << "theta_1_1:  " << theta_1(1) << "theta_1_2:  " << theta_1(2) << '\n';
    
    return theta_j;
}


Vector<float,7> trig_func(Vector3f GR, float Theta_to_pos, Vector<float, 4> motor_pos){
    float theta1, theta2, theta3, time, s1, s2, s3, c1, c2, c3 ;
    Vector<float, 4> temp = theta_conv(GR, Theta_to_pos,motor_pos);
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

//global variables:
float p_0;
float v_0;
float p_1;
float v_1;
float p_2;
float v_2;

float a = 0.45;
float Theta_to_pos = 2*3.14159;
Vector3f GR {{10,0.22,10}};
 

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
    
}

void loop() {
  while (true) {
    p_0 = odrive.GetPosition(m_0);
    p_1 = odrive.GetPosition(m_1);
    p_2 = 0.0;
    float time = millis();
    
    Vector<float, 4> motor_pos {{p_0,p_1,p_2,time/1000}};
    Vector<float, 4> ee = ee_pos(trig_func(GR, Theta_to_pos,motor_pos), a);
    
    //Serial.println("Pos: ");
    //Serial << p_0 << '\n';
    //Serial << p_1 << '\n';

    //Serial.println("e-e position: ");
    Serial << ee(0) << ',' << ee(1) << ',' << ee(2) << '\n';
  
  }
  
}
