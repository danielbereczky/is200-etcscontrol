//This code implements a controller for Toyota's ETCS electronic throttle system for the 1G-FE Engine found in the Lexus IS 200.
//Created by Daniel Bereczky 

//pin assignment

  //sensor inputs (redundant)
  int pedalPin = A0;
  int pedalPin2 = A1;

  int throttlePin = A2;
  int throttlePin2 = A3;
  //outputs
  int throttleClutchPin = 0; // motor clutch, should be engaged all the time pretty much
  int throttleMotorPin = 0; // the throttle body motor itself


  //variables
  long pedalVal = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  //serial port
  Serial.begin(115200);
}

void readVPA(){
  //read the throttle pedal 256 times and average the values, so noise does not mess up the readings.
  for(unsigned int i = 0; i < 256;i++){
    pedalVal += analogRead(pedalPin);
  }
  pedalVal = pedalVal / 256;
}

void readVTA();

void calculateMotorPWM();

void loop() {
  // put your main code here, to run repeatedly:
  // basic loop: read sensor target, calculate target write value using a PID algorithm, and then drive the motor with a PWM signal
  readVPA();
  Serial.write(pedalVal);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

}


