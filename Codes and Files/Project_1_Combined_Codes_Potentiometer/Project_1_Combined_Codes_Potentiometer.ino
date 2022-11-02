#include <PID_v1.h>

// Motor A connections
int enA = 9;    // purple
int in1 = 8;    // blue
int in2 = 7;    // green
// Motor B connections
int enB = 3;    // gray
int in3 = 4;    // white 
int in4 = 5;    // black

int motorSpeed = 0;
double Input = 0.0;
double Output = 127.0;
double Setpoint = 487; //static range[478,494]
double Kp = 4.2, Ki = 0.05, Kd = 0.03;                      
//(3,1.5,0) //(0.002,5.0,0.04) //(0.002,5.0,0.05) //(0.002,5.2,0.2) //(0.002,5.2,0.05) //(0,5.2,0.05)   //484,488
//(0.06,3.9,0.02) //(0.06,3.7,0.02) //(0.09,3.5,0.02) //(1,5,0.02) //482,490
//(2,5,0.02) //(2.4,4,0.02) //(3,5,0.02) //(3.5,0,0.2) //(3.6,0,0.02) //(4.1,0,0.02) //(4.1,0.05,0.02) //(4.1,0.07,0.02) 
//(4.2,0.05,0.01)//482,490
//16,100,14//60,270,2.2
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  int sensorValue = analogRead(A0);

  Input = sensorValue;
  myPID.Compute();
  motorSpeed = Output;

  if ((sensorValue > 835)|(sensorValue < 115)){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
    
  if (sensorValue > 490){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    motorSpeed = map(Output, 127, 0, 0, 255);
  }
  else if (sensorValue < 482){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    motorSpeed = map(Output, 128, 255, 0, 255);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    motorSpeed = 0;
  }

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  
  Serial.print(Input);
  Serial.print(" ");
  Serial.println(motorSpeed); 

  Serial.println(Setpoint);
  
  delay(1);
}
