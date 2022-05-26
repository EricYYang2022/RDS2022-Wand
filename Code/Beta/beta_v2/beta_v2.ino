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
    Serial.println("Callibrating encoders");
    
    int requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH;
    odrv_srl.run_state(axis, requested_state, true);
    delay(3000);
    
    requested_state = CONTROL_MODE_TORQUE_CONTROL;
    odrv_srl.run_state(axis, requested_state, false);
    
    delay(100);
    
    requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    odrv_srl.run_state(axis, requested_state, false); // don't wait

    delay(100);

    return;
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
Vector<float, 4> ee_3;

float a = 0.45;
float c = 0;
float k = 20;
//Can increase this of course
float m=0;

//Change this to turn on interial mass demo
int button=0;

int calibrated0 = 0;
int calibrated1 = 0;
 
//Vector3f GR {{.10,0.22,10}};
Vector<float, 3> GR {{0.1,0.1,0.1}};

Vector<float, 4> ee2;
Vector<float, 4> ee3;

float pin_state = 1;

float time = 0;

int start = 1;
 
void setup() {
    pinMode(33, OUTPUT);
    
    // ODrive uses 115200 baud
    odrive_serial.begin(115200);
    odrive_serial1.begin(115200);

    // Serial to PC
    Serial.begin(115200);
    while (!Serial) ; // wait for Arduino Serial Monitor to open

    Serial.println("ODriveArduino");
    Serial.println("Ready!");

    time = millis();

    Vector<float, 4> motor_pos {{0,0,0,time/1000}};
    
    // Run calibration sequence

    //set encoders to 0
    Serial.println("Reading Home!");
    odrive_serial << "w axis" << '0' << ".encoder.set_linear_count " << 0.f << '\n';
    delay(50);
    odrive_serial << "w axis" << '1' << ".encoder.set_linear_count " << 0.f << '\n';
    delay(50);
    odrive_serial1 << "w axis" << '0' << ".encoder.set_linear_count " << 0.f << '\n';
    delay(50);

    delay(2500);

    callibrate_encoder(odrive, 0);
    callibrate_encoder(odrive, 1);
    callibrate_encoder(odrive1, 0);

    ee3 = ee_pos(trig_func(motor_pos, GR), a);
    ee2 = ee_pos(trig_func(motor_pos, GR), a);

    Serial.println("Start Loop!");
    odrive_serial << "c " << 0 << " " << 0.0 << "\n";
    delayMicroseconds(50);
    odrive_serial << "c " << 1 << " " << 0.0 << "\n";
    delayMicroseconds(50);
    odrive_serial1 << "c " << 0 << " " << 0.0 << "\n";

}


void loop() {
    digitalWrite(33, pin_state);
    pin_state  =! pin_state;
    
    // Getting position data 
    p_0 = odrive.GetPosition(m_0);
    p_2 = odrive1.GetPosition(m_2);
    p_1 = odrive.GetPosition(m_1);

    /*
    time = 0.01;
    Vector<float, 4> motor_pos {{p_0,p_1,p_2,time/1000}};
    //Vector<float, 3> Tau = whiteboard(motor_pos, GR, ee2, k, c, a); 
    
    Serial << "Tau 0: " << Tau(0) << "\n";
    delayMicroseconds(50);
    Serial << "Tau 1: " << Tau(1) << "\n";
    delayMicroseconds(50);
    Serial << "Tau 2: " << Tau(2) << "\n";
    */
    
    // Send Torque commands to motors, 1ms
    odrive_serial << "c " << 0 << " " << 0.0 << "\n";
    odrive_serial1 << "c " << 0 << " " << 0.0 << "\n";
    odrive_serial << "c " << 1 << " " << 0.0 << "\n";
 }
  
