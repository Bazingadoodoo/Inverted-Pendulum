#include<PID_v1.h>

// Rotary Encoder Inputs
#define CLK 12  // yellow
#define DT 13   // green

// Motor A connections
int enA = 9;    // purple
int in1 = 8;    // blue
int in2 = 7;    // green

// Motor B connections
int enB = 3;    // gray
int in3 = 4;    // white 
int in4 = 5;    // black

int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";

int motorSpeed = 0;
double Input = 0;
double Output = 0;
double Setpoint = 0;
double Kp = 0.01, Ki = 30, Kd = 0.03;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{  
  // Setup Serial Monitor
  Serial.begin(9600);
  
  // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop()
{
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter ++;
      currentDir ="CW";
    } 
    else {
      // Encoder is rotating CW so increment
      counter --;
      currentDir ="CCW";
    }
  
    Serial.print("Counter:");
    Serial.println(counter);
    Serial.print("Speed:");
    Serial.println(motorSpeed); 
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;
  
  Input = counter;
  
  if (counter > 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (counter < 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  /*
  myPID.Compute();
  motorSpeed = Output;
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  */

  delay(1);
}
