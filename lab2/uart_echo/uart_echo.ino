/*
* uart_echo.ino
* 
* Echo program to see UART serial communication between Arduino and computer.
* 
* David Elliott, Ray Sun, Adi Telik, Evan Yeh
* EE 154a Winter 2019
* California Institute of Technology
*/

void setup() {
  // start serial port at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  while(Serial.available() > 0) {
    Serial.write(Serial.read());
  }
}
