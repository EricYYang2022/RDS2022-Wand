#define HWSERIAL Serial2
// This file is to be run on the secondary Arduino Teensy to facilitate the transfer of the VR data to the robot's arduino. Ensure the two arduinos are wired together before use.
String incoming_string="";
String incoming_string1="";
String incoming_string2="";

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(921600);
}

void loop() {
  if (Serial.available() > 0) {
      incoming_string = Serial.readStringUntil('\n');
      delay(5);
      incoming_string1 = Serial.readStringUntil('\n');
      delay(5);
      incoming_string2 = Serial.readStringUntil('\n');
      delay(5);
      //Serial.print(incoming_string +'\n');
      //Serial.print(incoming_string1 + '\n');
      //Serial.print(incoming_string2 + '\n');
      HWSERIAL.println(incoming_string);
      delay(5);
      HWSERIAL.println(incoming_string1);
      delay(5);
      HWSERIAL.println(incoming_string2);
      delay(5);
  }
}
