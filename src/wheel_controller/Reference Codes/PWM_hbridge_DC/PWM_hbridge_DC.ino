
//Motor A
int enA = 9;
int in1 = 8;
int in2 = 7;

//Motor B
int enB = 3;
int in3 = 5;
int in4 = 4;

//Joystick input 
int joyVert = A0; //Vertical
int joyHoriz = A1; //Horizontal 

//Motor Speed due to the effects from the joystick 
int speed1 = 0;
int speed2 = 0;

//Joystick values
int joyposVert = 512; //512 is the middle position
int joyposHoriz = 512;

void setup() {
  // put your setup code here, to run once:

  //Motor Control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Motors disabled and direction forward, joystick use ONLY
  digitalWrite(enA, LOW); // Motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(enB, LOW); // Motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void loop() {

  //Read joystick x and y positions
  joyposVert = analogRead(joyVert); 
  joyposHoriz = analogRead(joyHoriz);

  // Determine forward or reverse based of the joystick input 
  
  if(joyposVert < 460) 
  {
    //Backward
    
    digitalWrite(in1, LOW); //Motor A
    digitalWrite(in2, HIGH);

    digitalWrite(in3, LOW); //Motor B
    digitalWrite(in4, HIGH);

    //Motor Speeds
    joyposVert = joyposVert - 460;  // negative number
    joyposVert = joyposVert * -1; // turns result to a positive

    speed1 = map(joyposVert, 0, 460, 0, 255);
    speed2 = map(joyposVert, 0, 460, 0, 255);
    
  } 
  else if (joyposVert > 564) 
  {
    //Forward
    
    digitalWrite(in1, HIGH); //Motor A
    digitalWrite(in2, LOW);

    digitalWrite(in3, HIGH); //Motor B
    digitalWrite(in4, LOW);

    speed1 = map(joyposVert, 564, 1023, 0, 255);
    speed2 = map(joyposVert, 564, 1023, 0, 255);
    
  }
  else 
  {
    // motor stops
    speed1 = 0;
    speed2 = 0;
  }

  //Steering portion
  
  if(joyposHoriz < 460) 
  {
    //left 
    joyposHoriz = joyposHoriz - 460;  // negative number
    joyposHoriz = joyposHoriz * -1; // turns result to a positive

    joyposHoriz = map(joyposHoriz, 0, 460, 0, 255); // 255 Maximum value

    speed1 = speed1 - joyposHoriz;
    speed2 = speed2 + joyposHoriz;

    if(speed1 < 0)speed1 = 0; //Used to not exceed range of 0-255
    if(speed2 > 255)speed2 = 255;
    
    
  }
  else if (joyposHoriz > 564) 
  {
    //right
   joyposHoriz = map(joyposHoriz, 564, 1023, 0, 255); // 255 Maximum value

    speed1 = speed1 + joyposHoriz;
    speed2 = speed2 - joyposHoriz;

    if(speed1 > 255)speed1 = 255; //Used to not exceed range of 0-255
    if(speed2 < 0)speed2 = 0;
    
    
  }

  // Adjust to prevent "buzzing" at very low speed
  /*if(speed1 < 8)speed1 = 0; 
  if(speed2 < 8)speed2 = 0;*/
  if (speed1 < 70) {
    speed1 = 0;
  }
  if (speed2 < 70) {
    speed2 = 0;
  }
  
  analogWrite(enA, speed1); // Set the motor speeds 
  analogWrite(enB, speed2);
  
}
