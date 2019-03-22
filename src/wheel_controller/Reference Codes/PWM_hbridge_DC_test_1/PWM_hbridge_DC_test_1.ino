//Motor A
int enA = 9;
int in1 = 8;
int in2 = 7;

//Motor B
/*int enB = 3;
int in3 = 5;
int in4 = 4;
*/

void setup() {

 // put your setup code here, to run once:

//Motor Control pins to outputs
  pinMode(enA, OUTPUT);
  //pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //pinMode(in3, OUTPUT);
  //pinMode(in4, OUTPUT);
}

void test_One() { // Runs both motors in the same direction at a fixed speed 

  //Turn on Motor A
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);

  //Sets speed from a range of 0-255
  analogWrite(enA, 200); 
  
  //Turn on Motor B
  //digitalWrite(in3, HIGH); 
  //digitalWrite(in4, LOW);

  //Sets speed from a range of 0-255
  //analogWrite(enB, 200); 

  //delay(2000);

  //Changing motor directions 
  /*digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  delay(2000);

  // Motors Off
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
 */
}

/*void test_Two() { // Run motors across the range of possible speeds
                 // Max speed is determined by the motor itself and operating voltage
  // Turn on Motors
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  //Accelerating from zero to maximum speed
  for(int i = 0; i < 256; i++) //Accelerating from zero to maximum speed
  {
    analogWrite(enA, i);
    analogWrite(enB, i);

    delay(20);
  }
  
  //Decelerating from zero to maximum speed
  for(int i = 255; i >= 0; --i) 
  {
    analogWrite(enA, i);
    analogWrite(enB, i);

    delay(20);
  }

   // Turn off Motors
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}*/

void loop() {
  // put your main code here, to run repeatedly:
test_One();
//delay(1000);

//test_Two();
//delay(1000);
}
