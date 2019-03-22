#include <PID_v1.h>


double kp = 0.05, ki = 1, kd = 0.01;
double input = 0, output = 0, setpoint = 0; //test values
PID myPID(&input, &output, &setpoint, kp, ki, kd, REVERSE);
//PID_ATune aTune(&input, &output);
volatile int32_t counter = 0;  //This variable will increase on the rotation of encoder
volatile int32_t currentPosition = 0;
int cur = 0; //current position
int prev = 0; //previous position
float newRPM = 0.0, setRPM = 0.0; //updated RPM
float rpm = 0.0; //current rpm
unsigned long lastTime = 0;
const long interval = 100; //ms

int enA = 9; //Motor A
int in1 = 8;
int in2 = 7;


void ai0() { //CW direction counter
  // ai0 is activated if pin 2 is going from LOW to HIGH
  if(digitalRead(3)==LOW) { //CW
    counter++;
  }
}

void ai1() { //CCW direction counter
  if(digitalRead(2)==LOW) { //CCW
    counter--;
  }
}



void setup() {
  Serial.begin(9600);
 // put your setup code here, to run once:

  pinMode(enA, OUTPUT); //Motor Control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(2, INPUT_PULLUP); //encoder inputs
  pinMode(3, INPUT_PULLUP);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);

 
  attachInterrupt(0, ai0, RISING); //encoder interrupts
  attachInterrupt(1, ai1, RISING);
  TCCR1B = TCCR1B & 0b11111000 | 1; //reduce motor noise
  myPID.SetMode(AUTOMATIC); //automatic or manual, turns on PID Loop
  myPID.SetSampleTime(1); // in paranthesis (1/freq) in ms, 
  myPID.SetOutputLimits(-255, 255); //vary output within a given range

  Serial.println("Enter desired RPM: "); //serially inputting RPM
}

int currentPos(int x){ //returns current position
  return x;
}

bool timeInterval() { //Updates the program every 100ms
    unsigned long currentTime = millis();
    if(currentTime - lastTime >= interval) {
      lastTime = lastTime + interval;

      return true;
    } else if(currentTime < lastTime) {
      lastTime = 0;
    } else {
      return false;
    }
    return false;
}

void sendData() { // RPM Calculation
    prev = cur;
    cur = currentPos(counter);   
    rpm = ((cur - prev)*10*60) / 600; 
  //  Serial.print("PREV: ");
 //   Serial.println(prev);
   // Serial.print("CURR: ");
   // Serial.println(cur);  
    Serial.print("rpm: ");
    Serial.println(rpm); 
   
}


void updateRPM(float out) { //Set new RPM
  newRPM = abs(out) * 255;
  Serial.print("new rpm:");
  Serial.println(newRPM);

  
  if((newRPM > 0) && (newRPM < 255)) {
  digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, newRPM);
    Serial.println("in range");
    
  }
  
     else if(newRPM > 255) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 255);
      Serial.println("too high");
    }
    else if(newRPM <= 0){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);
      Serial.println("too low");
    }

}


void loop() {
  if(timeInterval){
      sendData();
      
 
  }
  input = rpm;
  Serial.print("INPUT: ");
  Serial.println(input);
    
  if(Serial.available()){
    setpoint = Serial.parseInt(); 
    Serial.print("Setpoint = ");
    Serial.println(setpoint);

    myPID.Compute();
    Serial.print("OUTPUT: ");
    Serial.println(output);
  }
 
    Serial.println(" ");
    updateRPM(output);
    Serial.print("Kp, Ki, Kd: ");
    Serial.print(kp);
    Serial.print(",");
    Serial.print(ki);
    Serial.print(",");
    Serial.println(kd);
    
 }
