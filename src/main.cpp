#include <Arduino.h>
#include <PID_v1.h>
#include "Oversampling.h"

/* -------------------------------------------------------------------------- */
/*                                   Config                                   */
/* -------------------------------------------------------------------------- */

// Coil spacing in mm
#define COIL_SPACING 100
// Below the min signal, that coil is ignored. If both below, motors disabled.
#define MIN_SIGNAL 10
// Place the robot at this displacement (pos to the right) during calibration (align with notch on sensor)
#define CALIBRATION_DIST 30 // must be less than COIL_SPACING/2, musn't be close to the coils!
// Number of extra bits of resolution to aquire when sampling
#define OVERSAMPLING 2
// Smoothing factor (using simple EWMA to reduce processing time and memory)
#define SMOOTHING 100
// Offset from 2*mid value before signal is inverted
#define INVERSION_OFFSET 0

/* -------------------------------------------------------------------------- */
/*                               Pin definitions                              */
/* -------------------------------------------------------------------------- */

/* ------------------------------ Coil signals ------------------------------ */
// Pins for peak detection signal
#define L_SIG_PIN A0 
#define R_SIG_PIN A1
/* ---------------------------    Potentionmeters --------------------------- */
// pot which sets max speed
#define MAX_SPEED_PIN A4 
// pots for tuning pid
#define KP_PIN A5
#define KI_PIN A6
#define KD_PIN A7
/* --------------------------------- Buttons -------------------------------- */
#define MOTOR_EN_BUTTON 9 // motor enable pin
#define ZERO_BUTTON 10 // press when robot aligned with zero displacement and angle
#define CALIBRATE_BUTTON 11 // press when robot at calibration distance
/* --------------------------------- Outputs -------------------------------- */
// PWM 
#define L_MOTOR_PIN 3 // pwm for left motor
#define R_MOTOR_PIN 4 // pwm for right motor
// LEDs
#define CLIPPING_LED 12 // on if signal above range
#define LOW_SIGNAL_LED 13 // on if signal below range

/* -------------------------------------------------------------------------- */
/*                                  Variables                                 */
/* -------------------------------------------------------------------------- */

/* -------------------------------- Settings -------------------------------- */

// Calibration values
int rawDistanceZero[2]={-1,-1}; // 1/E at zero distance
int rawDistanceCalibration[2]={-1,-1}; // 1/E at calibration distance
// Values from potentiometers
uint16_t maxSpeed=255; // max speed to prevent flying off
int KP,KI,KD; // PID loop settings

/* ------------------------------ Measurements ------------------------------ */
// raw E pk-pk signal
int16_t rawL,rawR; 
// linearised 1/E signal
int16_t linL,linR; 
// calibration values
int16_t zeroL,zeroR; // at displacement=0 (midpoint)
int16_t calL,calR; // at calibration distance (on the right hand side)

int16_t linL_mm,linR_mm; // scaled to mm

int16_t dispL_mm,dispR_mm; // inflection points removed (negative when wire goes past coil)

double displacement=0; // linear displacement mapped based on calibration values at 0mm and calibration distance
double angle=0; // angle between vehicle and tangent of AC line

/* --------------------------------- Outputs -------------------------------- */

double steeringRatio=0; // differential steering applies an offset to avgspeed fr each side
double avgSpeed=0; // avg speed of wheels, range 0-255
bool motorsEnabled=false;

/* --------------------------------- Objects -------------------------------- */

Oversampling lPeak(10,10+OVERSAMPLING,1);
Oversampling rPeak(10,10+OVERSAMPLING,1);

// PID steeringControl(&displacement,&steeringRatio,);

/* -------------------------------------------------------------------------- */
/*                                   Program                                  */
/* -------------------------------------------------------------------------- */

/* -------------------------------- Functions ------------------------------- */
// read values from EEPROM to avoid recalibrating each time
void loadEEPROM() {

}
// read settings
void readPotentiometers() {
  KP=analogRead(KP_PIN);
  KI=analogRead(KI_PIN);
  KD=analogRead(KD_PIN);
  maxSpeed=analogRead(MAX_SPEED_PIN);
}
// read peak values from inductors, convert to linear values
void readInductors() {
  // Read coils
  rawL=lPeak.read(L_SIG_PIN);
  rawR=rPeak.read(L_SIG_PIN);
  // Linearise
  linL=(2<<(10+OVERSAMPLING))/rawL;
  linR=(2<<(10+OVERSAMPLING))/rawR;
}
// Check if calibration button pressed, set zero points
void calButton() {
  if (digitalRead(CALIBRATE_BUTTON)==LOW) {
    Serial.println("Setting Calibration Point");
    calR=linR;
    calL=linL;
    delay(500); // prevent multiple activations
  }
}
// Check if zero button pressed, set calibration points
void zeroButton() {
  if (digitalRead(ZERO_BUTTON)==LOW) {
    Serial.println("Setting Zero Point");
    zeroR=linR;
    zeroL=linL;
    delay(500); // prevent multiple activations
  }
}
// Map measured values to real units
void scaleToMM() {
  linL_mm=map(linL,zeroL,calL,COIL_SPACING/2,COIL_SPACING/2+CALIBRATION_DIST);
  linR_mm=map(linR,zeroR,calR,COIL_SPACING/2,COIL_SPACING/2-CALIBRATION_DIST);
}
// As soon as the difference is greater than COIL_SPACING, a coil has passed inflection
// So we invert the output such that crossing the wire gives a negative output
void removeInflection() {
  if (linL_mm-linR_mm>COIL_SPACING+INVERSION_OFFSET) {
    dispR_mm=-linR_mm;
  } else {
    dispR_mm=linR_mm;
  }
  if (linR_mm-linL_mm>COIL_SPACING+INVERSION_OFFSET) {
    dispL_mm=-linL_mm;
  } else {
    dispL_mm=linL_mm;
  }
}
void calcDisplacement() {
  displacement=dispL_mm-dispR_mm;
}
void calcAngle() {
  int16_t rawAngle=dispL_mm+dispR_mm;
  angle=map(rawAngle,0,COIL_SPACING,0,90);
}
// read slope sensor, boost if detected
void detectSlope() {

}
// detect if save button pressed. If it is, save calibration and tuning values to EEPROM
void saveButton() {
  // print values to be saved as well
}

/* ------------------------------- Interrupts ------------------------------- */

// set flag when button has been pressed
bool motorEnablePressed = false;
void motorEnableButton() {
  motorEnablePressed = true;
}

/* ---------------------------------- Setup --------------------------------- */

void setup() {
  // init PWM pins
  pinMode(L_MOTOR_PIN,OUTPUT);
  pinMode(R_MOTOR_PIN,OUTPUT);
  // init LED pins
  pinMode(CLIPPING_LED,OUTPUT);
  pinMode(LOW_SIGNAL_LED,OUTPUT);
  // init button pins with pullups, thus read LOW when pressed 
  pinMode(MOTOR_EN_BUTTON,INPUT_PULLUP);
  pinMode(ZERO_BUTTON,INPUT_PULLUP);
  pinMode(CALIBRATE_BUTTON,INPUT_PULLUP);  

  // readPotentiometers(); 
  loadEEPROM();

  // rising edge interrupt on enable button
  attachInterrupt(digitalPinToInterrupt(MOTOR_EN_BUTTON),motorEnableButton,RISING);
}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {
  // Analog read
  //readPotentiometers();
  readInductors();
  // Read buttons
  zeroButton();
  calButton();
  saveButton();
  // Calculate vehicle position
  scaleToMM();
  removeInflection();
  calcDisplacement();
  calcAngle();

  // High quality control scheme here...
  maxSpeed=map(90-angle,0,90,50,255);

  int speedL=constrain(map(displacement,0,100,0,maxSpeed),0,255);
  int speedR=constrain(map(-displacement,0,100,0,maxSpeed),0,255);

  // Apply settings
  // maxSpeed=analogRead(MAX_SPEED_PIN)>>2; // max output speed between 0 and 255
  // minSignal=analogRead(MIN_SIG_PIN); // read potentiometer

  // Calculate motor speeds
  // Vey simple open loop, just increases turning rate as distance increases via skid steering, slowing one side down from the max speed
  // int L_SPEED = constrain(direction ? maxSpeed : (maxSpeed-map(displacement,0,100,0,maxSpeed)),0,255);
  // int R_SPEED = constrain(direction ? (maxSpeed-map(displacement,0,100,0,maxSpeed)) : maxSpeed,0,255);



  // display if either amplitude is clipping high
  if (rawL>=1023 || rawR>=1023){
    digitalWrite(CLIPPING_LED,HIGH);
  } else {
    digitalWrite(CLIPPING_LED,LOW);
    // display if either amplitude is too low. If it is, disable motors
    if (rawL<MIN_SIGNAL || rawR<MIN_SIGNAL) {
      motorsEnabled=false;
      digitalWrite(LOW_SIGNAL_LED,HIGH);
    } else{
      digitalWrite(LOW_SIGNAL_LED,LOW);
    }
  }

  if (motorEnablePressed) {
    motorEnablePressed=false;
    if (motorsEnabled) {
      motorsEnabled=false;
      analogWrite(L_MOTOR_PIN,0);
      analogWrite(R_MOTOR_PIN,0);
    } else {
      motorsEnabled=true;
    }
    delay(10); // dodgey debounce
  }

  // Activate motors only if enable pressed, signal within range, and calibration data loaded
  if (motorsEnabled) {
    analogWrite(L_MOTOR_PIN,speedL);
    analogWrite(R_MOTOR_PIN,speedR);
  } 

  Serial.print(">disp:");Serial.println(displacement);
  Serial.print(">angle:");Serial.println(angle);
  // Serial.print(">3D|position:S:cube:R:0:"); Serial.print(angle); Serial.print(":0:P:");Serial.print(displacement);Serial.println(":0:0");


}
