#include <rds_helper_1.h>

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


void callibrate_encoder(ODriveArduino odrv_srl, int axis) {
    Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
    int c_motor = Serial.read();
    
    if (c_motor == 1) {
      int requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
      Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
      odrv_srl.run_state(axis, requested_state, true);
      delay(2500);
  
      requested_state = CONTROL_MODE_TORQUE_CONTROL;
      Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
      odrv_srl.run_state(0, requested_state, false);
  
      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
      odrv_srl.run_state(0, requested_state, false); // don't wait
    }
}



// setup motor axis constant
int m_0 = 0;
int m_1 = 1;
int m_2 = 0;


//global variables:
float p_0;
float p_1;
float p_2;

Vector<float, 4> ee_1;
Vector<float, 4> ee_2;

float a = 0.45;
float c = 0;
float k = 20;

int calibrated0 = 0;
int calibrated1 = 0;
 
//Vector3f GR {{.10,0.22,10}};
Vector<float, 3> GR {{0.1,0.1,0.1}};

Vector<float, 4>* ee2;

float time;
 
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

    float time = millis();
    
    Vector<float, 4> motor_pos {{0,0,0,time/1000}};
    
    // Run calibration sequence

    //set encoders to 0
    odrive_serial << "w axis" << '0' << ".encoder.set_linear_count " << 0.f << '\n';
    odrive_serial << "w axis" << '1' << ".encoder.set_linear_count " << 0.f << '\n';
    odrive_serial1 << "w axis" << '0' << ".encoder.set_linear_count " << 0.f << '\n';

    callibrate_encoder(odrive, 0);
    callibrate_encoder(odrive, 1);
    callibrate_encoder(odrive1, 0);

    *ee2 = ee_pos(trig_func(motor_pos, GR), a);
}


void loop() {
    // Getting position data 
    p_0 = odrive.GetPosition(m_0);
    p_1 = odrive.GetPosition(m_1);
    p_2 = odrive1.GetPosition(m_2);
    
    Vector<float, 4> motor_pos {{p_0,p_1,p_2,time/1000}};
    Vector<float, 3> Tau = whiteboard(motor_pos, GR, *ee2, k, c, a); 
    
    // Send Torque commands to motors
    odrive.SetCurrent(0, Tau(0));
    odrive.SetCurrent(1, Tau(1));
    odrive1.SetCurrent(0, Tau(2));

 }
  
