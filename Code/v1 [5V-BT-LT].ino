/* == Loli == */
/* 
 *  LINE TRACING CAR - STEM 03/26/2021
 *  
 */
#include <Arduino.h>
#include <kmotor.h>

Kmotor _motor(1);
/// SENSORS
uint8_t sens[] = {A5, A7, A6, A4};
float avg[] = {770, 670, 670, 770}; // calibrate line sensor, larger is black and otherwise

/// CONFIGURATIONS
const int voltage=5; //current voltage
const bool debug=1; //0=prevent && 1=allow
int delayTime=30; //30ms
int whiteCountLimit=25; //1s

/// GENERAL VARIABLES
float P, I, D, PID_value, previous_error=0, Kp, Kd, Ki, reduce, initSpeed, error=0;
bool state[5];
int whiteCount=0;

void setup(){
    _motor.init(); Serial.begin(9600);
    for (int i = 0; i < 4; i++) pinMode(sens[i], INPUT);
    delay(70);
}

void calculate_error() {
  bool fl=0;
       if(state[0]==0 && state[1]==0 && state[2]==0 && state[3]==1) error=-3;
  else if(state[0]==0 && state[1]==0 && state[2]==1 && state[3]==1) error=-2;
  else if(state[0]==0 && state[1]==1 && state[2]==1 && state[3]==1) error=-1;
  else if(state[0]==1 && state[1]==1 && state[2]==1 && state[3]==1) error=0;
  else if(state[0]==0 && state[1]==1 && state[2]==1 && state[3]==0) error=0;
  else if(state[0]==1 && state[1]==1 && state[2]==1 && state[3]==0) error=1;
  else if(state[0]==1 && state[1]==1 && state[2]==0 && state[3]==0) error=2;
  else if(state[0]==1 && state[1]==0 && state[2]==0 && state[3]==0) error=3;
  else if(state[0]==0 && state[1]==0 && state[2]==0 && state[3]==0) {
    whiteCount++;
    fl=1;
  }
  if(!fl) whiteCount=0;
}

void stopMotor() {
  while( 1==1 ) {
    _motor.stop();
    Serial.print("Stopped "); Serial.print(PID_value); Serial.println();
  }
}
void calculate_pid() {
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (Kp*P) + (Ki*I) + (Kd*D);
  Serial.print(Kp); Serial.print(' ');Serial.print(P); Serial.print(' ');
  Serial.print(Ki); Serial.print(' ');Serial.print(I); Serial.print(' ');
  Serial.print(Kd); Serial.print(' ');Serial.print(D);Serial.print(' ');
  previous_error=error;
}

void control() {
  float motor[3];
  motor[0]=initSpeed-PID_value;
  motor[1]=initSpeed+PID_value;
  motor[0]=constrain(motor[0],0,255);
  motor[1]=constrain(motor[1],0,255);
  // CONSTANTS CONFIGURATION
  if(voltage==9) {
    // cfg 9v (unstable)
    initSpeed=160;
    if(abs(error)>=1) {
      Kp=15; Kd=120; Ki=5;
      reduce=20.0;
    }
    else {
      Kp=15; Kd=100; Ki=4;
      reduce=10.0;
    }
  }
  else if(voltage==5) {
    // cfg 5v (stable)
    initSpeed=256;
    Kp=31; Kd=9; Ki=1.8;
    reduce=0.0;
  }
  if(whiteCount>=whiteCountLimit) stopMotor();
  _motor.engine(0,motor[0]-reduce); _motor.engine(1, motor[1]-reduce);  
}

void bluetoothMode() {
  bluetooth = Serial.read();
  if(bluetooth > -1) Serial.println((char)(bluetooth));
  if (bluetooth == 'F') {
    _kmotor.engine(0, 250);
    _kmotor.engine(1, 250);
  }
  if (bluetooth == 'B') {
    _kmotor.engine(0, -250);
    _kmotor.engine(1, -250);
  }
  if (bluetooth == 'L') {
    _kmotor.engine(0, -200);
    _kmotor.engine(1, 200);
  }
  if (bluetooth == 'R') {
    _kmotor.engine(0, 200);
    _kmotor.engine(1, -200);
  }
  if (bluetooth == 'S') {
    _kmotor.engine(0, 0);
    _kmotor.engine(1, 0);
  }
  if (bluetooth == 'M') {
    _kmotor.engine(0, 250);
    _kmotor.engine(1, 200);
  }
  if (bluetooth == 'C') {
    _kmotor.engine(0, 200);
    _kmotor.engine(1, 250);
  }
  if (bluetooth == 'P') {
    _kmotor.engine(0, -200);
    _kmotor.engine(1, -150);
  }
  if (bluetooth == 'E') {
    _kmotor.engine(0, -150);
    _kmotor.engine(1, -200);
  }
  if (bluetooth == 'o') {
    servo_1.write(75);
    delay(500);
    servo_0.write(90);
    delay(500);
  }
  if (bluetooth == 'O') {
    servo_0.write(0);
    delay(500);
    servo_1.write(0);
    delay(500);
  }
}
void loop() {
    for (int i = 0; i < 4; i++){
        float val = analogRead(sens[i]);
        //Serial.print(val); Serial.print(' ');
        state[i]=(val>avg[i]);
        Serial.print(state[i]); Serial.print(' ');
    }
    calculate_error();
    calculate_pid();
    Serial.print(PID_value); Serial.println();
    if(debug) control();
    delay(delayTime);
    Serial.println();
}