//This code implements a controller for Toyota's ETCS electronic throttle system for the 1G-FE Engine found in the Lexus IS 200.
//Created by Daniel Bereczky 


//pin assignment

//sensor inputs (to be made redundant)
int pedalPin = A0; // pin for throttle pedal position sensor 1 (rising voltage)
int pedalPin2 = A1;

int throttlePin = A2; // pin for throttle valve position sensor 1 (rising voltage)
int throttlePin2 = A3;


//tc% input

int TCPin = A4;

//outputs
int throttleMotorPin = 3; // PWM Signal for controlling the throttle body's DC Motor
int motorControl1 = 6; //IN1 signal of the L298N DC motor driver, used for controlling speed and direction
int motorControl2 = 7; //IN2 signal of the L298N DC motor driver, used for controlling speed and direction

//variables
long pedalVal = 0;
int pedalValPercent = 0;
int pedalValZeroOffset = 430;

long throttleVal = 0;
int throttleValPercent = 0;
int throttleValZeroOffset = 507;

int TCVal = 0;
int TCCut = 0; // value of applied Traction Control Cut, in percent
int TCZeroOffset = 0;

//modifiers
#define ALS_REQ 10.0f //ALS request value, only done when als request is active, and driver is off-throttle.
#define IDLE_REQ  5.0f //Idle request value, this is the targeted throttle opening when there is an idle request active, and user is off-throttle.

int idleActive = 0; // is idle requested?
int ALSActive = 0; // is ALS requested?


//misc
float returnPWMMultiplier = 0.6f; // to compensate for throttle return spring

//comms
float compensation = 0.0f; //this is the value received from a ROS master, a correction of throttle position

float kP = 19.0f;
float kI = 16.0f;
float kD = 0.5f;

float lastError = 0;
float integral = 0;

unsigned long lastTime = 0;


#include <ros.h>
#include <std_msgs/Float32.h>  // Use Float32 for correction value

ros::NodeHandle nh;

// Callback function: updates compensation value whenever a new message arrives
void correctionCallback(const std_msgs::Float32& msg){
  compensation = msg.data;
}

// Subscriber subscribing to topic "throttle_correction"
ros::Subscriber<std_msgs::Float32> correction_sub("throttle_correction", &correctionCallback);

void setup(){
  nh.initNode();
  nh.subscribe(correction_sub);

  //setting up pins for motor control
  pinMode(throttleMotorPin,OUTPUT);
  pinMode(motorControl1,OUTPUT);
  pinMode(motorControl2,OUTPUT);

  //Initializing inputs
  digitalWrite(throttleMotorPin,LOW);
  digitalWrite(motorControl1,LOW); 
  digitalWrite(motorControl2,LOW);

  //serial port
  Serial.begin(115200);
}

void readVPA(){
  //read the throttle pedal 64 times and average the values, so noise does not mess up the readings.
  pedalVal = 0;
  for(unsigned int i = 0; i < 64;i++){
    pedalVal += analogRead(pedalPin);
  }
  pedalVal /=  64;
  pedalValPercent = constrain(map(pedalVal,pedalValZeroOffset,1023,0,100),0,100);
}

void readVTA(){
  //read the throttle pedal 64 times and average the values, so noise does not mess up the readings.
  throttleVal = 0;
  for(unsigned int i = 0; i < 64;i++){
    throttleVal += analogRead(throttlePin);
  }
  throttleVal /=  64;
  throttleValPercent = constrain(map(throttleVal,throttleValZeroOffset,1023,0,100),0,100);
}

void readTCCut(){
  TCVal = 0;
  for(unsigned int i = 0; i < 8;i++){
    TCVal += analogRead(TCPin);
  }
  TCVal /=  8;
  TCCut = constrain(map(TCVal,TCZeroOffset,1023,0,100),0,100);
}

void driveThrottleMotorPWM(float pwmValue,int direction){

  pwmValue = constrain(pwmValue,0,255.0); // constrain PWM value to 8 bits
  
  //if direction is 1 , throttle is driven towards open position, otherwise the polarity is reversed to close it.
  if(direction){
    digitalWrite(motorControl1, LOW);
    digitalWrite(motorControl2, HIGH);
    analogWrite(throttleMotorPin,(int)pwmValue); 
  }
  else{
    digitalWrite(motorControl1, HIGH);
    digitalWrite(motorControl2, LOW);
    analogWrite(throttleMotorPin,(int)pwmValue); // because there exists a spring in the throttle body, which would normally return it to zero, we don't need to commands as much PWM to move towards zero. 
  }
}

float clampVal(float f){
  return max(0.0f,min(f,100.0f));
}

float calculateSetPoint(){

  //ALS
  if(pedalValPercent < 1 && ALSActive == 1){ //only call ALS if drive is off-throttle, and a request is active
    return ALS_REQ;
  }
  //idle
  if(pedalValPercent < 1 && idleActive == 1 && compensation == 0){ //only use idle when ADAS requests no correction, driver is off-throttle, and an idle request is active.
    return IDLE_REQ;
  }

  float corrected_pedal = (float)pedalValPercent + compensation;
  
  //apply TC Cut
  corrected_pedal = corrected_pedal * (100-TCCut);

  return clampVal(corrected_pedal);
}

void closedLoopControl(){

  float setpoint = calculateSetPoint(); // call function to calculate PID setpoint based on TPS and requests.

  //PID Control algorithm

  unsigned long now = micros();

  float dt = (now - lastTime) / 1000000.0f;

  lastTime = now;

  if(dt <= 0) dt = 0.001f;

  float error = setpoint - throttleValPercent; // P

  integral += error * dt; // I

  float derivative = (error - lastError) / dt; //D

  lastError = error; 
  
  float controlVal = kP * error + kI* integral + kD * derivative;


  //allow for reverse motor movement, change direction if there is an overshoot
  if (controlVal > 0) {
    driveThrottleMotorPWM(controlVal,1);
  }else {
    controlVal *= -1; // to reverse direction
    driveThrottleMotorPWM(controlVal,0);
  }
}

void loop() {


  // basic loop: read pedal input, read throttle position, calculate target, write value using a PID algorithm, and drive the motor with a PWM signal

  //read pedal input
  readVPA();

  //read throttle input
  readVTA();

  //read TC Pot
  readTCCut();

  //drive the throttle body with PID.
  closedLoopControl();

  nh.spinOnce(); // update ROS

//plotting for tuning PID

  Serial.print(pedalValPercent);
  Serial.print('\t');
  Serial.println(throttleValPercent);

}


