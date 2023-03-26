/*  == DESCRIPTION:
    SUPER HYBRID CAR [Line-tracing and Bluetooth Controllable Car]
    From Loli - TTK31 with love <3
    Latest Revision: 08/03/2023

    == SPECIFICATION:
  - Base: KCBot Ino 2022 version.
  - Claw: General Claw from the kit. It has 2 servos, with the Analog pin connected at A0 and A1, or can be put at Digital pin D10, D11 respectively.
  - Line-reader: Pololu QTR-8RC Reflectance Sensor Array, with 6 Analog pin connected at A0-A5 respectively.
                 OR a General Sensors with 4 Analog pin from left to right: A5-A7-A6-A4, respectively.
  - Motor: 2 general Motors connected to their reserved port (and those ports used D3, D6, D7, D8).
  - Bluetooth: General Bluetooth module connected to its reserved port (and that port used D0, D1).
  - Power: A 3.7V battery used to power the circuit board, and a 9V battery used to power the motors.
  - Ambient Sensor: General Sensor to detect the brightness of the surrounding area, uses 5V, Analog port A2 and use AO in the module (or can be put 
                    at Digital pin D9 and DO in the module).
  - Lights: One or two LEDs in the bottom of the car to help brighten up the area, only be turned on when it is dark (data from the ambient sensor).
            Use Digital pin D12.

    == USAGE:
  - Everything can be controlled with Bluetooth connectivity using "Robo Control" app. Switch to the 2nd mode.
  - In the Bluetooth mode:
    + Using the steering wheel to control the car's direction.
    + Using the <- -> button to grab an item in the claw, and by using it again reverse the process,
      which puts the item down.
    + Using the ^v mode to change between mode.
  - In the Line-tracing mode:
    + The steering wheel and other button will be disabled, except the ^v button to change working mode.
    + It will use the PID Algorithm, tweaked to the best settings possible to run on the line by itself.
*/

/// LIBRARIES ///
#include <Arduino.h>
#include <Servo.h>
#include <kmotor.h>      // KMotor Library. You must download and install it to control the motors.
#include <QTRSensors.h>  // Pololu QTR Sensor Library. First you must download and install QTRSensors library.

/// CONFIGURATIONS ///
const bool verboseStatus = 0; // 0 means no Sensors value will be printed (to save some CPU cycle) and vice versa.

// Lights
const bool Lights = 1;          // Enable lights
const bool LightsOnStartup = 1; // Turn on lights when calibrating
const int AmbientThreshold = 700; // Threshold to control the lights. Lights will be turn on when the sensor value is greater than this Threshold.
#define LightDPin 12            // Lights' Digital pin
#define AmbientSensorAPin A2    // Ambient Sensor Analog pin

// Line Sensors
const bool generalOrQTR = 0; // 0 means we are using General Sensors, 1 is QTR.
const bool Cali = 1;    // Setting it to 0 will prevent the sensors from calibrating and vice versa.

// Line Sensors (General)
uint8_t sens[] = { A5, A7, A6, A4 };
int generalMaxCali[6], generalMinCali[6], avgg[6];

// Line Sensors (QTR-8RC)
#define MIDDLE_SENSOR 4  // Number of middle sensor used.
#define NUM_SENSORS 6    // Number of sensors used.
#define TIMEOUT 2500     // Waits for 2500 us for sensor outputs to go low.
#define EMITTER_PIN 2    // emitterPin is the Arduino digital pin that controls the IR LEDs. Default is D2.
// Sensors 2 through 7 are connected to analog inputs 0 through 5, respectively
// QTRSensorsRC qtrrc((unsigned char[]){ A0, A1, A2, A3, A4, A5 }, NUM_SENSORS); // Enable the QTR sensors.
QTRSensorsRC qtrrc((unsigned char[]){  }, NUM_SENSORS); // Disable the QTR sensors.

// Motors and Servos
const bool Motor = 1;           // Setting it to 0 will prevent the motors from running and vice versa.
const float LeftOffset = 1.0;  // Left motor offset. Less value == less energy given to the motor.
const float RightOffset = 0.985; // Right motor offset. Less value == less energy given to the motor.

// 9v = 150, 120, 90
// 5v = 255, 210, 180
const float btMaxSp = 130; // Maximum speed, mainly for just forwarding/backwarding.
const float btMedSp = 80; // Medium speed, mainly for 45 degrees turn.
const float btLowSp = 60; // Low speed, mainly for turning L/R.
Servo servo_0, servo_1;
int ServoMode = 0; // Current servo mode.
Kmotor _motor(1);

/// GENERAL VARIABLES
float initSpeed = 200;
float Kp = 0, Kd = 0, Ki = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int bluetooth, reli = 0, prebt = -1;
float pre_pos = 0;
int waitInterval = 0, waitSetServo = 0, waitSetAngle = 0;

/// MANUAL SENSORS CALIBRATION
// Calibrate for 10 seconds by sliding the sensors across the line.
void manual_calibration(int times = 100) {
  Serial.print("Manual calibrating for 2 seconds..");
  if(generalOrQTR) {
    for (int i = 0; i < times; ++i) {
      qtrrc.calibrate(QTR_EMITTERS_ON);
      delay(20);
    }
    
    // Print calibrated results
    Serial.print("Calibration Results:\nMinimum: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.print("\nMaximum: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
  }
  else {
    for (int j = 0; j < 4; j++) {
      generalMaxCali[j] = 0;
      generalMinCali[j] = 1000;
    }
    for (int i = 0; i < 100; ++i) {
      for (int j = 0; j < 4; j++) {
        int val = analogRead(sens[j]);
        generalMaxCali[j] = max(generalMaxCali[j], val-30);
        generalMinCali[j] = min(generalMinCali[j], val+30);
      }
      delay(20);
    }

    // Calculate the in-between value.
    for (int i = 0; i < 4; i++) {
      avgg[i] = (generalMaxCali[i]-generalMinCali[i])/3;
    }

    // Print calibrated results
    Serial.print("Calibration Results:\nMinimum: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(generalMinCali[i]);
      Serial.print(' ');
    }
    Serial.print("\nMaximum: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(generalMaxCali[i]);
      Serial.print(' ');
    }
  }
  Serial.println();
}

// PID Calculation
void calculate_pid() {
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void setup() {
  Serial.begin(9600);             // Set the serial baud rate.
  servo_1.attach(A0+1);             // Attach the servo to the designated pin.
  servo_0.attach(A0);
  servo_0.write(100);
  servo_1.write(90);
  _motor.init();                  // Motor initialization.
  if(Lights) {
    pinMode(LightDPin, OUTPUT);
    if(LightsOnStartup) digitalWrite(LightDPin, HIGH);
  }
  if(Cali) manual_calibration();  // Sensor calibration.
  else manual_calibration(1);
  if(Lights && LightsOnStartup) digitalWrite(LightDPin, LOW);
  while (bluetooth != -1) bluetooth = Serial.read();
}

void bluetoothMode() {
  // Steering wheel related config.
  if (bluetooth == 'F') {
    _motor.engine(0, LeftOffset*btMaxSp);
    _motor.engine(1, RightOffset*btMaxSp);
  }
  if (bluetooth == 'B') {
    _motor.engine(0, -0.99*btMaxSp);
    _motor.engine(1, -btMaxSp);
  }
  if (bluetooth == 'L') {
    _motor.engine(0, -btLowSp);
    _motor.engine(1, btLowSp);
  }
  if (bluetooth == 'R') {
    _motor.engine(0, btLowSp);
    _motor.engine(1, -btLowSp);
  }
  if (bluetooth == 'S') {
    _motor.engine(0, 0);
    _motor.engine(1, 0);
  }
  if (bluetooth == 'M') {
    _motor.engine(0, btMaxSp);
    _motor.engine(1, btMedSp);
  }
  if (bluetooth == 'C') {
    _motor.engine(0, btMedSp);
    _motor.engine(1, btMaxSp);
  }
  if (bluetooth == 'P') {
    _motor.engine(0, -btMaxSp);
    _motor.engine(1, -btMedSp);
  }
  if (bluetooth == 'E') {
    _motor.engine(0, -btMedSp);
    _motor.engine(1, -btMaxSp);
  }
}

/// LINE-TRACING USING GENERAL SENSORS
void linetracingMode2() {
  // Get calibrated readings along with the line position.
  float eachError[6], pos = 0.0;
  if(verboseStatus) {
    Serial.print("Sens: ");
    Serial.print(' ');
  }
  for (int i = 0; i < 4; i++) {
    int val = analogRead(sens[i]);
    eachError[i] = constrain((1.0*(val-generalMinCali[i]-avgg[i]*2))/(1.0*avgg[i]), 0.0, 1.0);
    if(verboseStatus) {
      Serial.print(val);
      Serial.print(' ');
    }

    // Get current position of the car with the line. If the sensor i is directly underneath the line, the value should be 1000*i.
    pos += i*1000.0*eachError[i];
  }

  // This can only happen if it's out of the line. When that happens, use the previous valid position value.
  float sumError = eachError[0] + eachError[1] + eachError[2] + eachError[3];
  if(sumError < 0.01) pos = pre_pos;
  else {
    pos /= sumError;
    pre_pos = pos;
  }

  // Calculate error value based on the position to the line. (-) means it is to the right, and vice versa.
  error=(pos-1500.0)/500.0;

  // Calculate PID and the speed needed between motors.
  if(pos < 1700 && pos > 1300) ++reli; // In-between the lines (currently stable).
  else reli = 0;
  // if(reli < 20) {
  //   initSpeed = 195;
  //   Kp = 55;
  //   Ki = 1;
  //   Kd = 40;
  // }
  // else {
  //   initSpeed = 225;
  //   Kp = 35;
  //   Ki = 0.5;
  //   Kd = 40;
  // }
  // calculate_pid();
  // int left_motor_speed = initSpeed + PID_value;
  // int right_motor_speed = initSpeed - PID_value;
  // left_motor_speed = constrain(left_motor_speed, 0, 255);
  // right_motor_speed = constrain(right_motor_speed, 0, 255);
  if(reli < 20) {
    initSpeed = 130;
    Kp = 20;
    Ki = 1;
    Kd = 10;
  }
  else {
    initSpeed = 150;
    Kp = 15;
    Ki = 0.5;
    Kd = 20;
  }
  calculate_pid();
  int left_motor_speed = initSpeed + PID_value;
  int right_motor_speed = initSpeed - PID_value;
  left_motor_speed = constrain(left_motor_speed, 0, 200);
  right_motor_speed = constrain(right_motor_speed, 0, 200);
  // Set the motors' speed.
  if(Motor) {
    _motor.engine(0, LeftOffset*left_motor_speed);
    _motor.engine(1, 0.98*right_motor_speed);
  }

  // Print the debug if needed.
  if(verboseStatus) {
    Serial.print(" | ");
    for(int i = 0; i<4; ++i) {
      Serial.print(eachError[i]);
      Serial.print(' ');  
    }
    Serial.print("| Pos: ");
    Serial.print(pos);
    Serial.print(" | ");
    Serial.print(error);
    Serial.print(" | LR: ");
    Serial.print(left_motor_speed);
    Serial.print(' ');
    Serial.print(right_motor_speed);
    Serial.print("\n");
  }

  // Delay for the next change.
  delay(20 - verboseStatus*15);
}

/// LINE-TRACING USING QTR-8RC
void linetracingMode() {
  unsigned int sensors[NUM_SENSORS];
  // Get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int position = qtrrc.readLine(sensors); 

  // Calculate error value based on the position to the line.
  error=(position-2500)/500.0;

  // Calculate PID and the speed needed between motors.
  if(position < 2700 && position > 2300) ++reli; // In-between the lines (currently stable).
  else reli = 0;
  if(reli < 20) {
    initSpeed = 190;
    Kp = 40;
    Ki = 2;
    Kd = 20;
  }
  else {
    initSpeed = 210;
    Kp = 35;
    Ki = 1;
    Kd = 40;
  }
  calculate_pid();
  int left_motor_speed = initSpeed + PID_value;
  int right_motor_speed = initSpeed - PID_value;
  left_motor_speed = constrain(left_motor_speed, 0, 200);
  right_motor_speed = constrain(right_motor_speed, 0, 200);

  // Set the motors' speed.
  if(Motor) {
    _motor.engine(0, LeftOffset*left_motor_speed);
    _motor.engine(1, RightOffset*right_motor_speed);
  }

  // Print the debug if needed.
  if(verboseStatus) {
    Serial.print("Sens: ");
    for(int i=0; i<NUM_SENSORS; i++) {
      Serial.print(sensors[i]);
      Serial.print(' ');
    }
    Serial.print("| Pos: ");
    Serial.print(position);
    Serial.print(" | ");
    Serial.print(error);
    Serial.print(" | LR: ");
    Serial.print(left_motor_speed);
    Serial.print(' ');
    Serial.print(right_motor_speed);
    Serial.print("\n");
  }

  // Delay for the next change.
  delay(30 - verboseStatus*15);
}

int LightsCooldown = 0;
int modee = 0;
void loop() {
  // Check if any waiting is happening. Act if the waiting time is over.
  if(waitInterval == 1) {
    waitInterval = 0;
    if(waitSetServo) servo_1.write(waitSetAngle);
    else servo_0.write(waitSetAngle);
  }
  else if(waitInterval > 1) {
    --waitInterval;
    if(verboseStatus) {
      Serial.print("W");
      Serial.print(waitInterval);
      Serial.print('\n');
    }
  }
  // Read the bluetooth signal.
  bluetooth = Serial.read();
  if (bluetooth > -1) {
    if(verboseStatus) {
      Serial.print((char)(bluetooth));
      Serial.print('\n');
    }
    prebt = bluetooth;
  }
  // If current mode is 1, since we don't need any value other than the val to change mode, clear the input queue.
  if (modee == 1)
    while (bluetooth != -1 && !(bluetooth == 'Y' || bluetooth == 'T')) bluetooth = Serial.read();
  
  // If the change mode command appear, stop the motors, clear value and change mode.
  if (bluetooth == 'Y') {
    if(modee) _motor.stop();
    else {
      if(Motor) {
        _motor.engine(0,255);
        _motor.engine(1,255);
      }
      waitInterval = waitInterval*3/(15 + verboseStatus*20);
    }
    initSpeed = 255;
    Kp = 0, Kd = 0, Ki = 0;
    error = 0, P = 0, I = 0, D = 0, PID_value = 0;
    previous_error = 0, previous_I = 0;
    pre_pos = 0;
    modee ^= 1;
  }

  // Claw related config.
  // Only wait and turn if no operation are currently running.
  if (bluetooth == 'T' && waitInterval == 0) {
    ServoMode ^= 1;
    if(!ServoMode) {
      servo_0.write(100);
      waitInterval = (modee == 0 ? 100 : 300/(15 + verboseStatus*20));
      waitSetServo = 1;
      waitSetAngle = 90;
    }
    else {
      servo_1.write(0);
      waitInterval = (modee == 0 ? 100 : 300/(15 + verboseStatus*20));
      waitSetServo = 0;
      waitSetAngle = 30;
    }
  }
  
  if(Lights && LightsCooldown == 0) {
    if(analogRead(AmbientSensorAPin) >= AmbientThreshold) {
      digitalWrite(LightDPin, HIGH);
    }
    else digitalWrite(LightDPin, LOW);
  }
  if(LightsCooldown == 0) LightsCooldown = 50;
  LightsCooldown--;

  // Execute the magic!
  if (modee) {
    if(generalOrQTR) linetracingMode();
    else linetracingMode2();
  }
  else bluetoothMode();
  if(modee == 0 && !verboseStatus) delay(3);
}