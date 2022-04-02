char inByte[100];
int do_once = 0;
void setup() {
  // start serial port at 9600 bps:
  Serial.begin(9600);
}
void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
//    inByte = Serial.readStringUntil('\r');
    inByte = Serial.readString();
    char* cString = (char*) malloc(sizeof(char)*(inByte.length() + 1));
    char s[2] = ",";
    char *token;
   
     /* get the first token */
     token = strtok(inByte, s);
     
     /* walk through other tokens */
     while( token != NULL ) {
        Serial.println(token);
        Serial.println("\n");
      
        token = strtok(NULL, s);
//    inByte.toCharArray(cString, inByte.length() + 1);
//    Serial.write(cString);
    
  }
    
   if (do_once == 0) {
//   char inByte[20] = "Hi, Please, Work";
   
   }
   do_once = 1;
   }

}
