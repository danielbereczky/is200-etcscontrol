//This code implements a controller for Toyota's ETCS electronic throttle system for the 1G-FE Engine found in the Lexus IS 200.
//Created by Daniel Bereczky 


//pin assignment

//sensor inputs (redundant)
int pedalPin = A0; // pin for throttle pedal position sensor 1 (rising voltage)
int pedalPin2 = A1;

int throttlePin = A2; // pin for throttle valve position sensor 1 (rising voltage)
int throttlePin2 = A3;

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
int throttleValZeroOffset = 506;

//misc
float returnPWMMultiplier = 0.1f; // to compensate for throttle return spring
float lowAreaMultiplier = 0.3f; // to compensate for the initial throttle area where the motor doesnt react as well as in the higher areas;

//comms
float compensation = 0.0f; //this is the value received from a ROS master, a correction of throttle position (max. 50% authority)

  //PID variables
  /* -- kindof works
  float kP = 0.96f;
  float kI = 1.12f;
  float kD = 1.8f;
  */

float kP = 2.0f;
float kI = 1.4f;
float kD = 1.7f;

int lastError = 0;
float integral = 0;

//debug
int commandedPWM = 0;

#include <ros.h>
#include <std_msgs/Float32.h>  // Use Float32 for correction value

ros::NodeHandle nh;

// Callback function: updates compensation value whenever a new message arrives
void correctionCallback(const std_msgs::Float32& msg){
  compensation = msg.data;
}

// Subscriber subscribing to topic "throttle_correction"
ros::Subscriber<std_msgs::Float32> correction_sub("throttle_correction", &correctionCallback);

void setup() {
  
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

  // setting up breather LED
  pinMode(LED_BUILTIN, OUTPUT);
  //serial port
  Serial.begin(115200);
}

void readVPA(){
  //read the throttle pedal 256 times and average the values, so noise does not mess up the readings.
  pedalVal = 0;
  for(unsigned int i = 0; i < 256;i++){
    pedalVal += analogRead(pedalPin);
  }
  pedalVal /=  256;
  pedalValPercent = map(pedalVal,pedalValZeroOffset,1023,0,100);
}

void readVTA(){
    //read the throttle pedal 256 times and average the values, so noise does not mess up the readings.
  throttleVal = 0;
  for(unsigned int i = 0; i < 256;i++){
    throttleVal += analogRead(throttlePin);
  }
  throttleVal /=  256;
  throttleValPercent = map(throttleVal,throttleValZeroOffset,1023,0,100);
}

void driveThrottleMotorPWM(int pwmValue,int direction){

  pwmValue = constrain(pwmValue,0,255); // constrain PWM value to 8 bits

if (throttleValPercent < pedalValPercent && pedalValPercent < 40) {
    float normalized = pedalValPercent / 40.0f; // 0.0 to 1.0
    float scale = 1.0f + pow(1.0f - normalized, 2.0f) * 2.0f; // from 3.0 to 1.0
    pwmValue *= scale;
}
  //if direction is 1 , throttle is driven towards open position, otherwise the polarity is reversed to close it.
  if(direction){
    digitalWrite(motorControl1, LOW);
    digitalWrite(motorControl2, HIGH);
    analogWrite(throttleMotorPin,pwmValue); 
    commandedPWM = pwmValue;
  }
  else{
    digitalWrite(motorControl1, HIGH);
    digitalWrite(motorControl2, LOW);
    analogWrite(throttleMotorPin,pwmValue * returnPWMMultiplier); // because there exists a spring in the throttle body, which would normally return it to zero, we don't need to commands as much PWM to move towards zero. 
    commandedPWM = pwmValue * returnPWMMultiplier;
  }
}

void throttleBodyDemo(int timeSteps){

  // Sweep PWM up (opening throttle)
  for (int pwm = 0; pwm <= 255; pwm += 5) {
    driveThrottleMotorPWM(pwm,1);
    delay(timeSteps); // adjust for speed
  }

  delay(20); // hold open

  // Sweep PWM down (closing throttle)
  for (int pwm = 255; pwm >= 0; pwm -= 5) {
    driveThrottleMotorPWM(pwm,0);
    delay(timeSteps); // adjust for speed
  }
  delay(20);
}

void closedLoopControl(){

  //TO avoid integral windup, reset integral when throttle body is closed
  if(throttleValPercent == 0){
    integral = 0;
  }

  float corrected_pedal = pedalValPercent + compensation;
  
  // Clamp corrected pedal to [0, 100]
  if(corrected_pedal > 100) corrected_pedal = 100;
  if(corrected_pedal < 0) corrected_pedal = 0;

  //PID Control algorithm

  int error = corrected_pedal - throttleValPercent;
  //int error = pedalValPercent - throttleValPercent; // error calculation

  integral += error; // I

  int derivative = error - lastError; //D
  
  float controlVal = kP * error + kI* integral + kD * derivative;


  // if the error is small, the motor should not be moved
  if(abs(error <= 1)){
    digitalWrite(motorControl1,LOW);
    digitalWrite(motorControl2,LOW);
    analogWrite(throttleMotorPin,0); 
  }

  //allow for reverse motor movement, change direction if there is an overshoot
  if (controlVal > 0) {
    driveThrottleMotorPWM(controlVal,1);
  } else {
    controlVal *= -1; // to reverse direction
    driveThrottleMotorPWM(controlVal,0);
  }

  lastError = error;

}

void loop() {


  // basic loop: read pedal input, read throttle position, calculate target, write value using a PID algorithm, and drive the motor with a PWM signal

  //read pedal input
  readVPA();

  //read throttle input
  readVTA();

  //drive throttle body with a PWM signall
  //throttleBodyDemo(60); // demo

  closedLoopControl();

   nh.spinOnce(); // update ROS
  /*
  //ONLY USED FOR DEBUG
  Serial.print("Throttle Pedal percentage:  ");
  Serial.print(pedalValPercent);
  Serial.print("  Throttle Valve percentage:  ");
  Serial.print(throttleValPercent);
  Serial.print("  Commanded PWM:");
  Serial.print(commandedPWM);
  Serial.print('\n');
  */

  //breather
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

//plotting for tuning PID

Serial.print(pedalValPercent);
Serial.print('\t');
Serial.println(throttleValPercent);


}


