//Motor A
int enA = 9;
int in1 = 8;
int in2 = 7;

//Motor B
/*int enB = 3;
int in3 = 5;
int in4 = 4;
*/
//Potentiometer Controls 
int potA = A0;
int potB = A1;

//Motor Speed due to the effects from the pots
int speed1 = 0;
int speed2 = 0;

void setup() {

 // put your setup code here, to run once:

//Motor Control pins to outputs
  pinMode(enA, OUTPUT);
 // pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //pinMode(in3, OUTPUT);
 // pinMode(in4, OUTPUT);
}

void loop() {

  // Motor A Forward
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);

  // Motor B Forward
//  digitalWrite(in3, HIGH); 
 // digitalWrite(in4, LOW);

  //Read Values from the POTS
  speed1 = analogRead(potA); 
  speed2 = analogRead(potB);

  //Convert from 0-255 range 
  speed1 = map(speed1, 0 , 1023, 0, 255); 
  speed2 = map(speed2, 0 , 1023, 0, 255);

  // Adjust to prevent "buzzing" at very low speed
  if(speed1 < 8)speed1 = 0; 
  if(speed2 < 8)speed2 = 0;

  // Set the motor speeds
  analogWrite(enA, speed1); 
 // analogWrite(enB, speed2);
  
}
