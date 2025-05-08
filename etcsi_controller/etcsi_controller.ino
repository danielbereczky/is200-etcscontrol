//This code implements a controller for Toyota's ETCS electronic throttle system for the 1G-FE Engine found in the Lexus IS 200.
//Created by Daniel Bereczky 

//pin assignment

  //sensor inputs (redundant)
  int pedalPin = A0; // pin for throttle pedal position sensor 1 (rising voltage)
  int pedalPin2 = A1;

  int throttlePin = A2; // pin for throttle valve position sensor 1 (rising voltage)
  int throttlePin2 = A3;

  //outputs
  int throttleClutchPin = 13; // motor clutch, should be engaged all the time 
  int throttleMotorPin = 2; // PWM Signal for controlling the throttle body's DC Motor
  int motorControl1 = 6; //IN1 signal of the L298N DC motor driver, used for controlling speed and direction
  int motorControl2 = 7; //IN2 signal of the L298N DC motor driver, used for controlling speed and direction

  //variables
  long pedalVal = 0;
  int pedalValNormalized = 0;
  int pedalValZeroOffset = 430;

  long throttleVal = 0;
  int throttleValNormalized = 0;
  int throttleValZeroOffset = 506;

void setup() {

  //setting up pins for motor control
  pinMode(throttleClutchPin,OUTPUT);
  pinMode(throttleMotorPin,OUTPUT);
  pinMode(motorControl1,OUTPUT);
  pinMode(motorControl2,OUTPUT);

  //Initializing inputs
  digitalWrite(throttleClutchPin,LOW);
  digitalWrite(throttleMotorPin,LOW);
  digitalWrite(motorControl1,HIGH); //combined with the motorControl2, this setting makes the throttle motor go forward- we have a spring returning it to zero, so no need to drive it backwards ever (maybe as a failsafe?)
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
  pedalValNormalized = map(pedalVal,pedalValZeroOffset,1023,0,100);
}

void readVTA(){
    //read the throttle pedal 256 times and average the values, so noise does not mess up the readings.
  throttleVal = 0;
  for(unsigned int i = 0; i < 256;i++){
    throttleVal += analogRead(throttlePin);
  }
  throttleVal /=  256;
  throttleValNormalized = map(throttleVal,throttleValZeroOffset,1023,0,100);
}

void driveThrottleMotorPWM(int pwmValue){
  analogWrite(throttleMotorPin,pwmValue); 
}

void loop() {
  // put your main code here, to run repeatedly:
  // basic loop: read pedal input, read throttle position, calculate target, write value using a PID algorithm, and drive the motor with a PWM signal

  //read pedal input
  readVPA();

  //read throttle input
  readVTA();

  //drive throttle body with a PWM signal
  driveThrottleMotorPWM(100); // used for testing whether the control works at all

  //ONLY USED FOR DEBUG
  Serial.print("Throttle Pedal percentage:  ");
  Serial.print(pedalValNormalized);
  Serial.print("  Throttle Valve percentage:  ");
  Serial.print(throttleValNormalized);
  Serial.print('\n');


  //breather
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);



}


