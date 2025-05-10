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

  //PID variables
  float kP = 3.5f;
  float kI = 0.03f;
  float kD = 0.0f;

  int lastError = 0;
  float integral = 0;

  //debug
  int commandedPWM = 0;

void setup() {

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

  //if direction is 1 , throttle is driven towards open position, otherwise the polarity is reversed to close it.
  if(direction){
    digitalWrite(motorControl1, LOW);
    digitalWrite(motorControl2, HIGH);
  }
  else{
    digitalWrite(motorControl1, HIGH);
    digitalWrite(motorControl2, LOW);
  }
  analogWrite(throttleMotorPin,pwmValue); 
  commandedPWM = pwmValue;
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
  //PID Control algorithm

  int error = pedalValPercent - throttleValPercent; // error calculation

  integral += error; // I

  int derivative = error - lastError; //D
  
  float controlVal = kP * error + kI* integral + kD * derivative;


  // if the error is small, the motor should not be moved
  if(abs(error < 3)){
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

  //ONLY USED FOR DEBUG
  Serial.print("Throttle Pedal percentage:  ");
  Serial.print(pedalValPercent);
  Serial.print("  Throttle Valve percentage:  ");
  Serial.print(throttleValPercent);
  Serial.print("  Commanded PWM:");
  Serial.print(commandedPWM);
  Serial.print('\n');


  //breather
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);



}


