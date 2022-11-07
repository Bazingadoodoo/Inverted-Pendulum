#include <PID_v1.h>

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
double Input = 0.0;
double Output = 255.0;
double Setpoint = 0.0;
double Kp = 1.0, Ki = 1.0, Kd = 1.0;

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
  currentStateCLK = digitalRead(CLK);

  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    if (digitalRead(DT) != currentStateCLK) {
      counter ++;
      currentDir ="CW";
    } 
    else {
      counter --;
      currentDir ="CCW";
    }
    
    Input = counter;

    /*
    Serial.print("Input: "); 
    Serial.println(Input);
    Serial.print("Output: ");
    Serial.println(Output); 
    Serial.print("motorSpeed: ");
    Serial.println(motorSpeed); 
    */
  }

  lastStateCLK = currentStateCLK;
  myPID.Compute();
  
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

  motorSpeed = abs(Output - 255);
   
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  Serial.print(Input);
  Serial.print(" ");
  Serial.println(Output); 
  //Serial.println(motorSpeed); 

/*
  Serial.print("counter: "); 
  Serial.println(counter);
  Serial.print("Output: ");
  Serial.println(Output); 
*/  
  delay(1);
}
