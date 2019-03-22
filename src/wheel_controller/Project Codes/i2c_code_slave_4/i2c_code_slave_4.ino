//FOURTH MOTOR
#include <Wire.h>

int enB = 3; //Motor B
int in3 = 5;
int in4 = 4;

unsigned int wheelSpeed = 0;
const char slaveID = 0x40;

void setup() {
  Wire.begin(slaveID);                // join i2c bus with address 
  Serial.begin(38400);           // start serial for output
  Wire.onReceive(receiveEvent);
  pinMode(enB, OUTPUT); //Motor Control pins to outputs
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
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
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    Serial.print("ZERO - ");
    Serial.println(wheelSpeed);
    analogWrite(enB, wheelSpeed);
  }
  else if(wheelSpeed == 128){
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    Serial.print("MED - ");
    Serial.println(wheelSpeed);
    analogWrite(enB, wheelSpeed);
  }
  else if(wheelSpeed == 255) {
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    Serial.print("MAX - ");
    Serial.println(wheelSpeed);
    analogWrite(enB, wheelSpeed);
  }
  
}
