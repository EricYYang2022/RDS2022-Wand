// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////
HardwareSerial& odrive_serial = Serial1;

ODriveArduino odrive(odrive_serial);

// setup motor axis constant
int m = 1;

// Set constants for spring mass damper system
float wall = 0.5;
float kp = 7.5;
float kv = 0.06;

// Impulse force and impulse start is turned off
bool impulse_wall = false;
bool impulse_enabled = false;

// Set up initial constants for the impulse wall force
float t = 0;
float impulse_time = 0.01;
float dt = 0.005;
float mass = 0.00005;
float const_v = 0;
float torque = 0;

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");

  // Set Torque to 0
  float t_0 = 0.0;
  odrive.SetCurrent(m, t_0);
  
  // Set Torque Control
  int control_mode = CONTROL_MODE_TORQUE_CONTROL;
  odrive_serial << "w axis" << m << ".controller.config.control_mode " << control_mode << '\n';

  // Set up closed loop control
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive_serial << "w axis" << m << ".requested_state " << requested_state << '\n';
}

void loop() {
  while (true) {
    float p = odrive.GetPosition(m);
    float v = odrive.GetVelocity(m);
    
    if (p >= wall){
      // If within wall send the spring mass damper, send spring mass torque
      torque = -1.0 * kp * (p - wall) + -1.0 * kv * v;

      if (impulse_wall != true){
        // Turn on Impulse force if needed and set vel constant
        impulse_wall = true;
        impulse_enabled = true;
        const_v = v + 0.0;
      }

      if (impulse_enabled == true) {
        // Send impulse force if impulse enabled is positive
        if (t < impulse_time) {
          torque += -1.0 * (mass * const_v / dt);
        }
        else {
          impulse_enabled = false;
        }
      }
      t += dt;
    }
    else {
      // Resets impulse start when outside the wall
      impulse_enabled = false;
      t = 0;
      torque = 0.0;
      if (p < (wall - 0.08)) {
        impulse_wall = false;
      }
    }
    odrive.SetCurrent(m, torque);
  }
  
}
