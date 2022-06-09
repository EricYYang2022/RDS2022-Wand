// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ArduinoEigen.h>
#include <math.h>
#include <string.h>
#define HWSERIAL Serial5


//VR below
String incoming_string = "";
String incoming_string1 = "";
String incoming_string2 = "";

String wall_transformation ="";
String puck_transformation = "";
String controller_transformation = "";
int do_once =0;

double wall_transformationmatrix[4][4];
double puck_transformationmatrix[4][4];
double controller_transformationmatrix[4][4];

void convertStrtoArr4x4(char* str, char identifier)
{

  // Iterator to access array
  int i = 0, j = 0;
  char s[2] = ",";
  char *token;

  // get the first token
  token = strtok(str, s);

  // Iterates through all tokens
  while ( token != NULL ) {
    if(identifier=='a'){
        wall_transformationmatrix[j][i] = atof(token);
    }
    else if(identifier=='b'){
        puck_transformationmatrix[j][i] = atof(token);
    }
    
    else if(identifier=='c'){
        controller_transformationmatrix[j][i] = atof(token);
    }
    // iterates through the array
    i += 1;
    if (i == 4) {
      i = 0;
      j += 1;
    }

    // Goes to next token
    token = strtok(NULL, s);
  }
}


//#define PI 3.141592653

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////
HardwareSerial& odrive_serial = Serial1;

ODriveArduino odrive(odrive_serial);

HardwareSerial& odrive_serial1 = Serial2;
ODriveArduino odrive1(odrive_serial1);

// setup motor axis constant
int m_0 = 0;
int m_1 = 1;
int m_2 = 0;

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






Vector<float, 6> wall_vec(){
    // hard code wall - otherwise, given from UNITY

    Matrix4f Tsp {{puck_transformationmatrix[0][0],puck_transformationmatrix[1][0],puck_transformationmatrix[2][0],puck_transformationmatrix[3][0]},
   {puck_transformationmatrix[0][1],puck_transformationmatrix[1][1],puck_transformationmatrix[2][1],puck_transformationmatrix[3][1]},
   {puck_transformationmatrix[0][2],puck_transformationmatrix[1][2],puck_transformationmatrix[2][2],puck_transformationmatrix[3][2]},
   {puck_transformationmatrix[0][3],puck_transformationmatrix[1][3],puck_transformationmatrix[2][3],puck_transformationmatrix[3][3]}};

   // Matrix4f Tpb {{0.,-1.,0.,-0.05},{-0.8,0.,-0.5,-.3},{0.5,0.,-0.8,0.19},{0.,0.,0.,1.}};
   Matrix4f Tpb {{-1.,0.,0.,-0.15},{.0,0.,-1.,-.22},{0.,1.,0.,0.12},{0.,0.,0.,1.}};
  
   Matrix4f Tsw {{wall_transformationmatrix[0][0],wall_transformationmatrix[1][0],wall_transformationmatrix[2][0],wall_transformationmatrix[3][0]},
   {wall_transformationmatrix[0][1],wall_transformationmatrix[1][1],wall_transformationmatrix[2][1],wall_transformationmatrix[3][1]},
   {wall_transformationmatrix[0][2],wall_transformationmatrix[1][2],wall_transformationmatrix[2][2],wall_transformationmatrix[3][2]},
   {wall_transformationmatrix[0][3],wall_transformationmatrix[1][3],wall_transformationmatrix[2][3],wall_transformationmatrix[3][3]}};

   Matrix4f Tsb = Tsp*Tpb;
   Matrix4f Tbw = Tsb.inverse()*Tsw;
  // Vector<float, 6> wall {{Tbw(0,1), Tbw(1,1), Tbw(2,1), Tbw(0,3), Tbw(1,3),Tbw(2,3)}};
   Vector<float, 6> wall {{Tbw(0,2), Tbw(1,2), Tbw(2,2), Tbw(0,3), Tbw(1,3),Tbw(2,3)}};

   return wall;
}

Vector<float, 4> theta_conv(Vector3f GR, float Theta_to_pos, Vector<float, 4> motor_pos){
    //Vector<float, 4> motor_pos {{p_0,p_1,p_2,60}};
   
    //correction for homing + math
    // homing adjustment:
     float mp1 = -1*motor_pos(0) ;//+1.25;

    float mp2 = motor_pos(1) +(0.25/0.167);

    float mp3 = motor_pos(2);//-0.783;
    
    Vector<float, 3> mp_vec {{mp1,mp2,mp3}};

    
    Vector<float, 3> theta_1 =Theta_to_pos*(GR.cwiseProduct(mp_vec));
    
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
    
       //s1 = trig_matrix2[0,1];
       //s2 = trig_matrix2[0,1];
       //s3 = trig_matrix2[0,2];
       //c1 = trig_matrix2[1,0];
       //c2 = trig_matrix2[1,1];
       //c3 = trig_matrix2[1,2];
    
    t = trig_mat(6);
    
   
    x = a*trig_mat(5)*(trig_mat(3)- trig_mat(4));  
    y = a*(trig_mat(0) - trig_mat(1));
    z = a*trig_mat(2)*(trig_mat(3)- trig_mat(4));
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
    J13 = -a*trig_matrix(3)* trig_matrix(2) + a*trig_matrix(4)* trig_matrix(2);
    J21 = a*trig_matrix(3);
    J22 = -a*trig_matrix(4);
    J23 = 0;
    J31 = -a*trig_matrix(0)* trig_matrix(2);
    J32 = a*trig_matrix(1)* trig_matrix(2);
    J33 = a*trig_matrix(3)* trig_matrix(5) -a*trig_matrix(4)* trig_matrix(5);

    //Jacobian to capstan center
    Matrix3f J {{J11,J21,J31},{J12,J22,J32},{J13,J23,J33}};
    
    // F is the force at the end effector.
    Matrix3f transpose = J.transpose(); //TRANSPOSE NOT INVERSE
    Vector<float, 3> Tau = J.transpose() *(F);
    Vector<float, 3> T_motor = GR.cwiseProduct(Tau);

    //rotation corrections:
   //T_motor(1) = -1* T_motor(1);

    //RETURNS MOTOR TORQUES
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
    
    Vector<float, 4> nee1 = (ee1.head(3).dot(wall.head(3)))*wall.head(4);
    Vector<float, 4> nee2 = (ee2.head(3).dot(wall.head(3)))*wall.head(4);
    
    nee1(3) = ee1(3);
    nee2(3) = ee2(3);
    
    Vector<float, 4> nvelocity = vel_ee(nee1, nee2);

    //Vector<float, 4> velocity = vel_ee(ee1, ee2);
    
    Vector<float, 3> Forces = k*(dist * wall.head(3)) -(c* nvelocity.head(3));
    Vector<float, 3> motor_torque =  jacobian_torque(trig_mat2, a, Forces, GR);
    Vector<float, 3> Zero_T {{0,0,0}};
    if( dist >0.00005){
      return(motor_torque);
      // return(Zero_T);
      
    }
    else{
      return(Zero_T);
      
    }
}








//global variables:
float p_0;
float initial_p_0;
float p_1;
float initial_p_1;
float p_2;
float initial_p_2;


Vector<float, 4> ee_1;
Vector<float, 4> ee_2;

float a = 0.48;
float c = 0.;
float k = 100.; 
int i = 0;

//SO k =10 = 1N @0.1m into wall. SO K UNITS ARE N/M!!!

int calibrated0 = 0;
int calibrated1 = 0;
 
float Theta_to_pos = 2*3.141592653;

//Vector3f GR {{0.115,0.115,0.115}};

// 0.75 4.5
Vector3f GR {{0.167,0.167,0.167}};
float time = millis();
String input;
 
void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(912600);
  odrive_serial1.begin(912600);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Ready!");
  Serial.println("Send '0' to begin encoder index search GOOD");


  
  
  //set motor positions to zero
  Vector<float, 4> motor_pos {{0,0,0,time/1000}};

    //wait for use input to begin calibration
   while( !Serial.available()) {
    //loop until given a response of 0;
   }


    // Run calibration sequence
   
     //Motor 0 calibration (odrive 0)
      int requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
      Serial << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, true);

      delay(2500);

      requested_state = CONTROL_MODE_TORQUE_CONTROL;
      Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, false);
      
      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, false); // don't wait

     
      
       //Motor 1 calibration (odrive 0)
      requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
      Serial << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      odrive.run_state(1, requested_state, true);

      delay(2500);

      requested_state = CONTROL_MODE_TORQUE_CONTROL;
      Serial << "Axis" << '1'<< ": Requesting state " << requested_state << '\n';
      odrive.run_state(1, requested_state, false);

      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      odrive.run_state(1, requested_state, false); // don't wait

      //Motor 2 calibration (odrive 1, motor 0)
      requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
      Serial << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      odrive1.run_state(0, requested_state, true);

      delay(2500);

      requested_state = CONTROL_MODE_TORQUE_CONTROL;
      Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
      odrive1.run_state(0, requested_state, true);
      

      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
      odrive1.run_state(0, requested_state, false); // don't wait
      

      Serial.println("Calibration Complete. Place at home, then press any key");

      //input = Serial.readString();

      //Send zero torques
      odrive.SetCurrent(0,0);
      odrive.SetCurrent(1,0);
      odrive1.SetCurrent(0,0);

      
     //clear previous input to be able to recieve another
     while (Serial.available()) Serial.read();

    //wait until second user input
     while( !Serial.available()) {
    //loop until given a response of 0;
      }

     
    //collect initial encoder offsets to make this the home position.
    //intials store encoder offsets values globally. DO NOT CHANGE WHILE RUNNING.
      initial_p_0 = odrive.GetPosition(m_0);
      initial_p_1 = odrive.GetPosition(m_1);
      initial_p_2 = odrive1.GetPosition(m_2);

     Serial.println("IT WORKS! Encoders set to zero!\n");


     Serial.println("Hit any key to begin sending torques!\n");


      //clear previous input to be able to recieve another
     while (Serial.available()) Serial.read();

    //wait until third user input
     while( !Serial.available()) {
    //loop until given a response of 0;
      }
   

 
  HWSERIAL.begin(921600);
      Serial.print("wall initialization ->");
          for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            Serial.print(wall_transformationmatrix[j][i]);
            Serial.print(",");
          }
        }
        Serial.print("\n");
    
      Serial.print("puck initialization ->");
          for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            Serial.print(puck_transformationmatrix[j][i]);
            Serial.print(",");
          }
        }
        Serial.print("\n");
    
      Serial.print("controller initialization ->");
          for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            Serial.print(controller_transformationmatrix[j][i]);
            Serial.print(",");
          }
        }
        Serial.print("\n");  
 
}


void loop() {

  if(Serial5.available()>0){
    Serial5.println("a new line - -");

  
   //retrieve current motor positions via encoder:
    p_0 = odrive.GetPosition(m_0) - initial_p_0;
    p_1 = odrive.GetPosition(m_1) -initial_p_1;
    p_2 = odrive1.GetPosition(m_2)- initial_p_2;

    
    //create vector of motor positions & time    
    Vector<float, 4> motor_pos {{p_0,p_1,p_2,time/1000}};

    //calculate sines and cosines of capstan radians (converts motor rads to capstan too)
    VectorXf trig_matrix = trig_func(GR, Theta_to_pos,motor_pos);

    //calculates the motor radians
    Vector<float, 4> angles =theta_conv( GR, Theta_to_pos, motor_pos);
    angles = 180*(angles/(3.14159));// rad to degree conversion

    //calculates end-effector position:
    Vector<float, 4> ee_1 = ee_pos(trig_func(GR, Theta_to_pos,motor_pos), a);

    //calculate distance to the wall
    Vector<float,1> norm = normal_dist(ee_1);
    float dist = norm(0);
    
    //set forces for Jacobian tests
    //THIS FORCE IS ON THE EE ( pos torque is in pos axes direction)
    //Vector<float, 3> Forces {0.0f,0.0f,1.0f};
    //calculate jacobian torques from given force - motor torque
    //Vector<float, 3> Tau =  jacobian_torque(trig_matrix, a, Forces, GR);

    //print the wall vec
    Vector<float, 6> wall = wall_vec();

    incoming_string = HWSERIAL.readStringUntil('\n');
    Serial.println(incoming_string);
    
    if(incoming_string.substring(0,1)=='a'){
        wall_transformation = incoming_string.substring(2);
        char wall_temp[wall_transformation.length()];
        wall_transformation.toCharArray(wall_temp, wall_transformation.length());
        convertStrtoArr4x4(wall_temp,'a');
    }
    else if(incoming_string.substring(0,1)=='b'){
        //Serial.print(incoming_string);
        puck_transformation = incoming_string.substring(2);
        char puck_temp[puck_transformation.length()];
        puck_transformation.toCharArray(puck_temp, puck_transformation.length());
        convertStrtoArr4x4(puck_temp,'b');
    }   
    else if(incoming_string.substring(0,1)=='c'){
         controller_transformation = incoming_string.substring(2);
         char controller_temp[controller_transformation.length()];
         controller_transformation.toCharArray(controller_temp, controller_transformation.length());
         convertStrtoArr4x4(controller_temp,'c');
    }

    incoming_string1 = HWSERIAL.readStringUntil('\n');
    Serial.println(incoming_string1);
    
    if(incoming_string1.substring(0,1)=='a'){
        wall_transformation = incoming_string1.substring(2);
        char wall_temp[wall_transformation.length()];
        wall_transformation.toCharArray(wall_temp, wall_transformation.length());
        convertStrtoArr4x4(wall_temp,'a');
    }
    else if(incoming_string1.substring(0,1)=='b'){
        //Serial.print(incoming_string);
        puck_transformation = incoming_string1.substring(2);
        char puck_temp[puck_transformation.length()];
        puck_transformation.toCharArray(puck_temp, puck_transformation.length());
        convertStrtoArr4x4(puck_temp,'b');
    }   
    else if(incoming_string1.substring(0,1)=='c'){
         controller_transformation = incoming_string1.substring(2);
         char controller_temp[controller_transformation.length()];
         controller_transformation.toCharArray(controller_temp, controller_transformation.length());
         convertStrtoArr4x4(controller_temp,'c');
    }

    incoming_string2 = HWSERIAL.readStringUntil('\n');
    Serial.println(incoming_string2);
    
    if(incoming_string2.substring(0,1)=='a'){
        wall_transformation = incoming_string2.substring(2);
        char wall_temp[wall_transformation.length()];
        wall_transformation.toCharArray(wall_temp, wall_transformation.length());
        convertStrtoArr4x4(wall_temp,'a');
    }
    else if(incoming_string2.substring(0,1)=='b'){
        //Serial.print(incoming_string);
        puck_transformation = incoming_string2.substring(2);
        char puck_temp[puck_transformation.length()];
        puck_transformation.toCharArray(puck_temp, puck_transformation.length());
        convertStrtoArr4x4(puck_temp,'b');
    }   
    else if(incoming_string2.substring(0,1)=='c'){
         controller_transformation = incoming_string2.substring(2);
         char controller_temp[controller_transformation.length()];
         controller_transformation.toCharArray(controller_temp, controller_transformation.length());
         convertStrtoArr4x4(controller_temp,'c');
    }

    
    Serial.print("puck: "); 
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        Serial.print(puck_transformationmatrix[j][i]);
        Serial.print(",");
      }
    }
    Serial.print("\n");
    

    Serial.print("wall: "); 
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        Serial.print(wall_transformationmatrix[j][i]);
        Serial.print(",");
      }
    }

    
    Serial.print("\n");
    
    
    //run whiteboard
    Vector<float, 3> Tau = whiteboard(GR,  k,  c,  a,  Theta_to_pos, wall, motor_pos);

    
/*
    
    //Send zero torques
    odrive.SetCurrent(0, 0);
    odrive.SetCurrent(1, 0);
    odrive1.SetCurrent(0,0);
    */

    
    
    odrive.SetCurrent(0, Tau(0));
    odrive.SetCurrent(1, Tau(1));
    odrive1.SetCurrent(0,Tau(2));
    
/*
    
    
  
    //Serial.println("error:");
    //Serial<< odrive_serial.readString()<< '\n';

   Serial.println("INITIAL Pos: ");
    Serial << initial_p_0 << '\n';
    Serial << initial_p_1 << '\n';
    Serial << initial_p_2 << '\n';
    
    
    Serial.println("Encoder Pos: ");
    Serial << p_0 << '\n';
    Serial << p_1 << '\n';
    Serial << p_2 << '\n';
    float mp1 = p_0 +0;
    float mp2 = p_1 +2.5;
    float mp3 = p_2;
    
    Serial.println("Offset POS: ");
    Serial << mp1 << '\n';
    Serial << mp2 << '\n';
    Serial << mp3 << '\n';
    
    
    

    
    
    Serial.println("angles: ");
    Serial << angles(0) << '\n';
    Serial << angles(1) << '\n';
    Serial << angles(2) << '\n';

    */
    
    
    Serial.println("e-e position (inches): ");
    Serial << 39.37*ee_1(0) << '\n';
    Serial << 39.37*ee_1(1) << '\n';
    Serial << 39.37*ee_1(2) << '\n';

    /*
    Serial.println("Motor Torques: ");
    Serial << Tau(0)<< '\n';
    Serial << Tau(1)<< '\n';
    Serial << Tau(2)<< '\n';
    
    
    Serial.println("Capstan Torques: ");
    Serial << Tau(0)/GR(0)<< '\n';
    Serial << Tau(1)/GR(1)<< '\n';
    Serial << Tau(2)/GR(2)<< '\n';
    */
    
    Serial.println("Norm: ");
    Serial << dist << '\n';
    
  
    
  }

  
 }
