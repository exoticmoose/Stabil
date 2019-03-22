//MASTER BOARD CODE
#include <Wire.h>

int NO_SPEED = 0;
int MED_SPEED = 128;
int MAX_SPEED = 255;


char slave_id[4] = {0x08, 0x10, 0x20, 0x40}; //slave IDs
char assign = 0x00;
int input = 0;
int input_2 = 0;

void messageOne(){ //Wheel selection menu
    Serial.println("Select wheel with the following options: ");
    Serial.println(" ");
    Serial.println("Command 1 for Front Left Wheel");
    Serial.println("Command 2 for Front Right Wheel");
    Serial.println("Command 3 for Back Left Wheel");
    Serial.println("Command 4 for Back Right Wheel");
    Serial.println(" ");
    Serial.print("Input: ");
}

void messageTwo(){ //Speed menu selection
  Serial.println("Set speed num with the following options: ");
  Serial.println(" ");
  Serial.println("Command 5 for NO SPEED");
  Serial.println("Command 6 for MEDIUM SPEED");
  Serial.println("Command 7 for MAX_SPEED");
} 

void setup()
{
  Wire.begin(); // Start I2C Bus as Master
  messageOne();
  Serial.begin(38400);
}

void assignWheel(int input_1){
  if(input_1 == 1){ //Front left wheel
    assign = slave_id[0];
   // assignSpeed(assign);
    if(Serial.available()){
        messageTwo();
        Serial.print("Input 2: ");
        input_2 = Serial.parseInt();
        Serial.println(input_2);
 
      if (input_2 == 5){
        Wire.beginTransmission(assign);
        Wire.write(NO_SPEED);
        Serial.println("Wheel set to no speed");
        Wire.endTransmission();
       // Serial.println("Wheel set to no speed");
      }
      else if (input_2 == 6){
        Wire.beginTransmission(assign);
        Wire.write(MED_SPEED);
        Serial.println("Wheel set to medium speed");
        Wire.endTransmission();
        //Serial.println("Wheel set to medium speed");
      }
      else if (input_2 == 7){
        Wire.beginTransmission(assign);
        Wire.write(MAX_SPEED);
        Serial.println("Wheel set to maximum speed");
        Wire.endTransmission();
       // Serial.println("Wheel set to maximum speed");
      }
    }
  }
  
  else if(input_1 == 2){ //Front right wheel
    assign = slave_id[1];
    //assignSpeed(assign);
    if(Serial.available()){
        messageTwo();
        Serial.print("Input 2: ");
        input_2 = Serial.parseInt();
        Serial.println(input_2);
 
      if (input_2 == 5){
        Wire.beginTransmission(assign);
        Wire.write(NO_SPEED);
        Serial.println("Wheel set to no speed");
        Wire.endTransmission();
      }
      else if (input_2 == 6){
        Wire.beginTransmission(assign);
        Wire.write(MED_SPEED);
        Serial.println("Wheel set to medium speed");
        Wire.endTransmission();
      }
      else if (input_2 == 7){
        Wire.beginTransmission(assign);
        Wire.write(MAX_SPEED);
        Serial.println("Wheel set to maximum speed");
        Wire.endTransmission();
      }
    }
    
  }
  else if(input_1 == 3){ // Back left wheel
    assign = slave_id[2];
    //assignSpeed(assign);
    if(Serial.available()){
        messageTwo();
        Serial.print("Input 2: ");
        input_2 = Serial.parseInt();
        Serial.println(input_2);
 
      if (input_2 == 5){
        Wire.beginTransmission(assign);
        Wire.write(NO_SPEED);
        Serial.println("Wheel set to no speed");
        Wire.endTransmission();
      }
      else if (input_2 == 6){
        Wire.beginTransmission(assign);
        Wire.write(MED_SPEED);
        Serial.println("Wheel set to medium speed");
        Wire.endTransmission();
      }
      else if (input_2 == 7){
        Wire.beginTransmission(assign);
        Wire.write(MAX_SPEED);
        Serial.println("Wheel set to maximum speed");
        Wire.endTransmission();
      }
    }
  }
  else if(input_1 == 4){ // Back right wheel
    assign = slave_id[3];
    //assignSpeed(assign);
    if(Serial.available()){
        messageTwo();
        Serial.print("Input 2: ");
        input_2 = Serial.parseInt();
        Serial.println(input_2);
 
      if (input_2 == 5){
        Wire.beginTransmission(assign);
        Wire.write(NO_SPEED);
        Serial.println("Wheel set to no speed");
        Wire.endTransmission();
      }
      else if (input_2 == 6){
        Wire.beginTransmission(assign);
        Wire.write(MED_SPEED);
        Serial.println("Wheel set to medium speed");
        Wire.endTransmission();
      }
      else if (input_2 == 7){
        Wire.beginTransmission(assign);
        Wire.write(MAX_SPEED);
        Serial.println("Wheel set to maximum speed");
        Wire.endTransmission();
      }
    }
    
  }
  
} 

void loop()
{
  if(Serial.available()){
    messageOne();
    input = Serial.parseInt();
    Serial.println(input);
    assignWheel(input);
  }

  delay(500);
}


/*
 // --------------------------------------
// i2c_scanner for checking purposes
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>


void setup()
{
  Wire.begin();

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}*/
