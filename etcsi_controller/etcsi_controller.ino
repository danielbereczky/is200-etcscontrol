//This code implements a controller for Toyota's ETCS electronic throttle system for the 1G-FE Engine found in the Lexus IS 200.
//Created by Daniel Bereczky 


//pin assignment

//sensor inputs (to be made redundant)
int pedalPin = A0; // pin for throttle pedal position sensor 1 
int pedalPin2 = A6;

int throttlePin = A3; // pin for throttle valve position sensor 1 (rising voltage)
int throttlePin2 = A7;

//tc% input

int TCPin = A4;

//outputs
int throttleMotorPin = 3; // PWM Signal for controlling the throttle body's DC Motor
int motorControl1 = 6; //IN1 signal of the L298N DC motor driver, used for controlling speed and direction
int motorControl2 = 7; //IN2 signal of the L298N DC motor driver, used for controlling speed and direction

//variables

#define VTA_MAX 1023
#define VTA2_MAX 583
#define VPA_MAX 1023
#define VPA2_MAX 606
#define VTA_OFFSET 507
#define VTA2_OFFSET 160
#define VPA_OFFSET 428
#define VPA2_OFFSET 114

long pedalVal = 0;
int pedalValPercent = 0;

long pedalVal2 = 0;
int pedalVal2Percent = 0;

long throttleVal = 0;
int throttleValPercent = 0;

long throttleVal2 = 0;
int throttleVal2Percent = 0;

int TCVal = 0;
int TCCut = 0; // value of applied Traction Control Cut, in percent
int TCZeroOffset = 0;


int invalid = 0;

//modifiers
#define SAMPLES 8
#define ERR_MAX 5 // maximum percentage error difference between the two throttle plate sensors
#define INVALID_MAX 7// maximum number of invalid sensor reads before a shutdown is triggered
#define ALS_REQ 10.0f //ALS request value, only done when als request is active, and driver is off-throttle.
#define IDLE_REQ 5.0f //Idle request value, this is the targeted throttle opening when there is an idle request active, and user is off-throttle.

int idleActive = 0; // is idle requested?
int ALSActive = 0; // is ALS requested?

int invalidSamples = 0;


//misc
float returnPWMMultiplier = 1.0f; // to compensate for throttle return spring

//comms
float compensation = 0.0f; //this is the value received from a ROS master, a correction of throttle position

float kP = 14.0f;  //19 16 0.5
float kI = 10.7f;
float kD = 0.2f;

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
  pinMode(LED_BUILTIN, OUTPUT);
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
  for(unsigned int i = 0; i < SAMPLES;i++){
    pedalVal += analogRead(pedalPin);
  }
  pedalVal /=  SAMPLES;
  pedalValPercent = constrain(map(pedalVal,VPA_OFFSET,VPA_MAX,0,100),0,100);
}

void readVPA2(){
  //read the throttle pedal 64 times and average the values, so noise does not mess up the readings.
  pedalVal2 = 0;
  for(unsigned int i = 0; i < SAMPLES;i++){
    pedalVal2 += analogRead(pedalPin2);
  }
  pedalVal2 /=  SAMPLES;
  pedalVal2Percent = constrain(map(pedalVal2,VPA2_OFFSET,VPA2_MAX,0,100),0,100);
}

void readVTA(){
  //read the throttle pedal 64 times and average the values, so noise does not mess up the readings.
  throttleVal = 0;
  for(unsigned int i = 0; i < SAMPLES;i++){
    throttleVal += analogRead(throttlePin);
  }
  throttleVal /=  SAMPLES;
  throttleValPercent = constrain(map(throttleVal,VTA_OFFSET,VTA_MAX,0,100),0,100);
}

void readVTA2(){
  //read the throttle pedal 64 times and average the values, so noise does not mess up the readings.
  throttleVal2 = 0;
  for(unsigned int i = 0; i < SAMPLES;i++){
    throttleVal2 += analogRead(throttlePin2);
  }
  throttleVal2 /=  SAMPLES;
  throttleVal2Percent = constrain(map(throttleVal2,VTA2_OFFSET,VTA2_MAX,0,100),0,100);
}

void validate(){ //returns whether two sensor inputs are valid
  if(abs(throttleValPercent - throttleVal2Percent) < ERR_MAX  && abs(pedalValPercent - pedalVal2Percent) < ERR_MAX) { // validate between sensors
    invalidSamples = 0;
    invalid = 0;
  }
  else{
    invalidSamples++;
    if(invalidSamples >= INVALID_MAX){
      invalid = 1;
    }
  }
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
  //corrected_pedal = (100.0f - TCCut) / 100.0f;

  return clampVal(corrected_pedal);
}

void closedLoopControl(){
  //validate sensor inputs, if invalid freeze controller, return throttle to closed

  if(invalid != 0){
    digitalWrite(LED_BUILTIN,HIGH);
    driveThrottleMotorPWM(20,0);
    integral = integral;
    lastError = lastError;
    return;
  }

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
  if (controlVal > 0 ) {
    driveThrottleMotorPWM(controlVal,1);
  }else {
    controlVal *= -1; // to reverse direction
    driveThrottleMotorPWM(controlVal,0);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {


  // basic loop: read pedal input, read throttle position, calculate target, write value using a PID algorithm, and drive the motor with a PWM signal

  //read pedal input
  readVPA();
  readVPA2();

  //read throttle input
  readVTA();
  readVTA2();

  //validate sensors
  validate();

  //read TC Pot
  readTCCut();

  //drive the throttle body with PID.
  closedLoopControl();

  nh.spinOnce(); // update ROS

  //plotting for tuning PID
  Serial.print("RequestedPosition:");
  Serial.print(pedalValPercent);
  Serial.print(",");
  Serial.print("ActuatorPosition:");
  Serial.println(throttleValPercent);

}


