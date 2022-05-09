// includes
#include "rds_helper.h"

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////
HardwareSerial& odrive_serial = Serial1;
HardwareSerial& odrive_serial1 = Serial2;

ODriveArduino odrive(odrive_serial);
ODriveArduino odrive1(odrive_serial1);

// setup motor axis constant
int m_0 = 0;
int m_1 = 1;
int m_2 = 0;


//global variables:
float p_0;
float p_1;
float p_2;
char c_motor;

Vector<float, 4> ee_1;
Vector<float, 4> ee_2;

float a = 0.45;
float c = 0;
float k = 20;

int calibrated0 = 0;
int calibrated1 = 0;
 
float Theta_to_pos = 2*3.141592653;
//Vector3f GR {{.10,0.22,10}};
Vector3f GR {{0.1,0.1,0.1}};

unsigned long start_time;

 
void setup() {
    // ODrive uses 115200 baud
    odrive_serial.begin(115200);
    odrive_serial1.begin(115200);

    // Serial to PC
    Serial.begin(115200);
    while (!Serial) ; // wait for Arduino Serial Monitor to open

    Serial.println("ODriveArduino");
    Serial.println("Setting parameters...");

    Serial.println("Ready!");
    /*
    // Set Torque to 0 
    float t_0 = 0.0;
    odrive.SetCurrent(m_0, t_0);
    odrive.SetCurrent(m_1, t_0);
    
    // Set Torque Control
    int control_mode = CONTROL_MODE_TORQUE_CONTROL;
    odrive_serial << "w axis" << m_0 << ".controller.config.control_mode " << control_mode << '\n';
    odrive_serial << "w axis" << m_1 << ".controller.config.control_mode " << control_mode << '\n';
  
    odrive_serial << "w axis" << m_0 << ".encoder.config.control_mode " << 0.0 << '\n';
    odrive_serial << "w axis" << m_1 << ".encoder.config.control_mode " << 0.0 << '\n';
  
    // Set up closed loop control
    int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    odrive_serial << "w axis" << m_0 << ".requested_state " << requested_state << '\n';
    odrive_serial << "w axis" << m_1 << ".requested_state " << requested_state << '\n';
    */

    float time = millis();
    
    Vector<float, 4> motor_pos {{0,0,0,time/1000}};
    // Vector<float, 4> ee_1 = ee_pos(trig_func(GR, Theta_to_pos,motor_pos), a);
    // Vector<float, 4> ee_2 = ee_pos(trig_func(GR, Theta_to_pos,motor_pos), a);
    
    // Run calibration sequence

    //set encoders to 0
    odrive_serial << "w axis" << '0' << ".encoder.set_linear_count " << 0.f << '\n';
    odrive_serial << "w axis" << '1' << ".encoder.set_linear_count " << 0.f << '\n';
    odrive_serial1 << "w axis" << '0' << ".encoder.set_linear_count " << 0.f << '\n';


    // First motor:
    // I dont know what this does?:::
    Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
    c_motor = Serial.read();
    
    int requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
    Serial << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
    odrive.run_state(0, requested_state, true);
    delay(2500);

    // Second motor:
    // Still dont know what this is:

    requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
    Serial << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
    odrive.run_state(1, requested_state, true);
    delay(2500);

    // Third motor:
    // Dont know what this is:
    
    requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
    Serial << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
    odrive1.run_state(0, requested_state, true);
    delay(2500);

    // Config the control mode and state
    requested_state = CONTROL_MODE_TORQUE_CONTROL;
    Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
    odrive.run_state(0, requested_state, false);

    requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
    odrive.run_state(0, requested_state, false); // don't wait

    requested_state = CONTROL_MODE_TORQUE_CONTROL;
    Serial << "Axis" << '1'<< ": Requesting state " << requested_state << '\n';
    odrive.run_state(1, requested_state, false);

    requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    Serial << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
    odrive.run_state(1, requested_state, false); // don't wait

    requested_state = CONTROL_MODE_TORQUE_CONTROL;
    Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
    odrive1.run_state(0, requested_state, false);

    requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    Serial << "Axis" << '0'<< ": Requesting state " << requested_state << '\n';
    odrive1.run_state(0, requested_state, false); // don't wait
}


void loop() {
    start_time = millis();
    // Getting position data 
    p_0 = odrive.GetPosition(m_0);
    p_1 = odrive.GetPosition(m_1);
    p_2 = odrive1.GetPosition(m_2);
    float time = millis();
    
    Vector<float, 4> motor_pos {{p_0,p_1,p_2,time/1000}};
    VectorXf trig_matrix = trig_func(GR, Theta_to_pos, motor_pos);
    Vector<float, 4> ee_3 = ee_pos(trig_matrix, a);
    Vector<float, 6> wall = wall_vec();
    
    // Vector<float,1> norm = normal_dist(ee_3);
    // float dist = norm(0);
    // Vector<float, 4> velocity = vel_ee(ee_1, ee_2);
    
    Vector<float, 4> nee1 = (ee_1.head(3).dot(wall.head(3)))*wall.head(4);
    Vector<float, 4> nee2 = (ee_2.head(3).dot(wall.head(3)))*wall.head(4);
    nee1(3) = ee_1(3);
    nee2(3) = ee_2(3);
    
    // Vector<float, 4> nvelocity = vel_ee(nee1, nee2);
    // Vector<float, 3> Forces = k*(dist * wall.head(3)) -(c* nvelocity.head(3));
    
    Vector<float, 3> Tau = whiteboard(GR,  k,  c,  a,  Theta_to_pos, wall, motor_pos); //jacobian_torque( trig_matrix,  a, Forces, GR);
    
    // Send Torque commands to motors
    odrive.SetCurrent(0, Tau(0));
    odrive.SetCurrent(1, Tau(1));
    odrive1.SetCurrent(0, Tau(2));

    /*
    Serial.println("e-e position (inches): ");
    Serial << 39.37*ee_3(0) << '\n';
    Serial << 39.37*ee_3(1) << '\n';
    Serial << 39.37*ee_3(2) << '\n';
    
    Serial.println("Norm: ");
    Serial << dist << '\n';
    
    Serial.println("Force: ");
    //Serial << typeid(Forces(0)).name()<< '\n';
    Serial << Forces(0)<< '\n';
    Serial << Forces(1)<< '\n';
    Serial << Forces(2)<< '\n';
    */
    
    // I dont know what this is?
    ee_1 = ee_2;
    ee_2 = ee_3;

    Serial << "Time: " << (millis() - start_time);
 }
  
