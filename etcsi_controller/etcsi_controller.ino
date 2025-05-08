//This code implements a controller for Toyota's ETCS electronic throttle system for the 1G-FE Engine found in the Lexus IS 200.
//Created by Daniel Bereczky 

//pin assignment

  //sensor inputs (redundant)
  int pedalPin = A0; // pin for throttle pedal position sensor 1 (rising voltage)
  int pedalPin2 = A1;

  int throttlePin = A2; // pin for throttle valve position sensor 1 (rising voltage)
  int throttlePin2 = A3;
  //outputs
  //int throttleClutchPin = 0; // motor clutch, should be engaged all the time pretty much
  //int throttleMotorPin = 0; // the throttle body motor itself

  //variables
  long pedalVal = 0;
  int pedalValNormalized = 0;
  int pedalValZeroOffset = 430;

  long throttleVal = 0;
  int throttleValNormalized = 0;
  int throttleValZeroOffset = 506;

void setup() {
  // put your setup code here, to run once:
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

void calculateMotorPWM();

void loop() {
  // put your main code here, to run repeatedly:
  // basic loop: read pedal input, read throttle position, calculate target, write value using a PID algorithm, and drive the motor with a PWM signal
  //read pedal input
  readVPA();
  //read throttle input
  readVTA();

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


