//THIRD MOTOR

#include <Wire.h>

int enA = 9; //Motor A
int in1 = 8;
int in2 = 7;

unsigned int wheelSpeed = 0;
const char slaveID = 0x20;

void setup() {
  Wire.begin(slaveID);                // join i2c bus with address 
  Serial.begin(38400);           // start serial for output
  Wire.onReceive(receiveEvent);
  pinMode(enA, OUTPUT); //Motor Control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
}

void loop() {
  delay(100);
  
}

void receiveEvent(int numBytes) {
 // unsigned char slave_id = Wire.read();
 wheelSpeed = Wire.read();
  //Serial.print("Slave id: ");
 // Serial.println(slave_id,HEX);
  
  Serial.println(" ");
  
  Serial.print("wheelSpeed: ");
  Serial.println(wheelSpeed);

  if(wheelSpeed == 0){ //Sets new speed and locks in
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    Serial.print("ZERO - ");
    Serial.println(wheelSpeed);
    analogWrite(enA, wheelSpeed);
  }
  else if(wheelSpeed == 128){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    Serial.print("MED - ");
    Serial.println(wheelSpeed);
    analogWrite(enA, wheelSpeed);
  }
  else if(wheelSpeed == 255) {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    Serial.print("MAX - ");
    Serial.println(wheelSpeed);
    analogWrite(enA, wheelSpeed);
  }
  
}
