#include <Arduino.h>
#include <PID_v1.h>
#include "Oversampling.h"

/* -------------------------------------------------------------------------- */
/*                                   Config                                   */
/* -------------------------------------------------------------------------- */

// Coil spacing in mm
#define COIL_SPACING 100
// Below the min signal, that coil is ignored. If both below, motors disabled.
#define MIN_SIGNAL 0.5
// Place the robot at this displacement (pos to the right) during calibration (align with notch on sensor)
#define CALIBRATION_DIST 30 // must be less than COIL_SPACING/2, musn't be close to the coils!
// Number of extra bits of resolution to aquire when sampling
#define OVERSAMPLING 0
// Smoothing factor (using simple EWMA to reduce processing time and memory)
#define SMOOTHING 100
// Offset from 2*mid value before signal is inverted
#define INVERSION_OFFSET 0

#define ADC_OFFSET 0.5

#define A_REF 4.5

#define ADC_RANGE 1024.0

/* -------------------------------------------------------------------------- */
/*                               Pin definitions                              */
/* -------------------------------------------------------------------------- */

/* ------------------------------ Coil signals ------------------------------ */
// Pins for peak detection signal
#define L_SIG_PIN A7
#define R_SIG_PIN A6
/* ---------------------------    Potentionmeters --------------------------- */
// pot which sets max speed
// #define MAX_SPEED_PIN A4 
// pots for tuning pid
#define KP_PIN A0
#define KI_PIN A2
#define KD_PIN A3
/* --------------------------------- Buttons -------------------------------- */
#define MOTOR_EN_BUTTON 2 // motor enable pin
#define ZERO_BUTTON 4 // press when robot aligned with zero displacement and angle
#define CALIBRATE_BUTTON 3 // press when robot at calibration distance
/* --------------------------------- Outputs -------------------------------- */
// PWM 
#define L_MOTOR_PIN 10 // pwm for left motor
#define R_MOTOR_PIN 11 // pwm for right motor
// LEDs
#define CLIPPING_LED 6 // on if signal above range
#define LOW_SIGNAL_LED 7 // on if signal below range

/* -------------------------------------------------------------------------- */
/*                                  Variables                                 */
/* -------------------------------------------------------------------------- */

/* -------------------------------- Settings -------------------------------- */

// Calibration values
// int rawDistanceZero[2]={-1,-1}; // 1/E at zero distance
// int rawDistanceCalibration[2]={-1,-1}; // 1/E at calibration distance
// Values from potentiometers
uint16_t maxSpeed=255; // max speed to prevent flying off
int KP,KI,KD; // PID loop settings

/* ------------------------------ Measurements ------------------------------ */
// raw E pk-pk signal
float rawL,rawR; 
float rawMax=A_REF; // max voltage
// linearised 1/E signal
float linL,linR; 
// calibration values
float zeroL,zeroR; // at displacement=0 (midpoint)
float calL,calR; // at calibration distance (on the right hand side)

float linL_mm,linR_mm; // scaled to mm

float dispL_mm,dispR_mm; // inflection points removed (negative when wire goes past coil)

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
  // maxSpeed=analogRead(MAX_SPEED_PIN);
}
// read peak values from inductors, convert to linear values
void readInductors() {
  // Read coils
  // rawL=lPeak.read(L_SIG_PIN);
  // rawR=rPeak.read(R_SIG_PIN);

  rawL=(analogRead(L_SIG_PIN)+ADC_OFFSET)*(A_REF/ADC_RANGE); // calc input voltages
  rawR=(analogRead(R_SIG_PIN)+ADC_OFFSET)*(A_REF/ADC_RANGE);

  // rawL=lPeak.read(KP_PIN);
  // rawR=rPeak.read(KD_PIN);
  
  // Linearise
  if (rawL==0)
    linL=0;
  else
    linL=((1.0))/rawL;
  if (rawR==0)
    linR=0;
  else
    linR=((1.0))/rawR;

  
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
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Map measured values to real units
void scaleToMM() {
  if (linL==NAN)
    linL_mm=0;
  else
    linL_mm=floatMap(linL,zeroL,calL,COIL_SPACING/2,COIL_SPACING/2+CALIBRATION_DIST);
  if (linR==NAN)
    linR_mm=0;
  else
  linR_mm=floatMap(linR,zeroR,calR,COIL_SPACING/2,COIL_SPACING/2-CALIBRATION_DIST);
  
}
// As soon as the difference is greater than COIL_SPACING, a coil has passed inflection
// So we invert the output such that crossing the wire gives a negative output
void removeInflection() {
  // if (linL_mm-linR_mm>(float)(COIL_SPACING+INVERSION_OFFSET)) {
  //   dispR_mm=-linR_mm;
  // } else {
  //   dispR_mm=linR_mm;
  // }
  // if (linR_mm-linL_mm>(float)(COIL_SPACING+INVERSION_OFFSET)) {
  //   dispL_mm=-linL_mm;
  // } else {
  //   dispL_mm=linL_mm;
  // }
  dispL_mm=linL_mm;
  dispR_mm=linR_mm;
}
void calcDisplacement() {
  displacement=dispL_mm-dispR_mm;
}
void calcAngle() {
  float rawAngle=dispL_mm+dispR_mm;
  angle=-floatMap(rawAngle,0,COIL_SPACING,180,0);
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
  // Serial.println("x");
}

/* ---------------------------------- Setup --------------------------------- */

void setup() {
  Serial.begin(9600);
  // init PWM pins
  pinMode(L_MOTOR_PIN,OUTPUT);
  pinMode(R_MOTOR_PIN,OUTPUT);
  pinMode(8,INPUT_PULLUP);
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
  // attachInterrupt(digitalPinToInterrupt(MOTOR_EN_BUTTON),motorEnableButton,FALLING);
}

/* ---------------------------------- Loop ---------------------------------- */

bool lowSignal=false;

void loop() {
  // Analog read
  //readPotentiometers();
  readInductors();
  // Read buttons
  zeroButton();
  calButton();
  // saveButton();
  // Calculate vehicle position
  scaleToMM();
  removeInflection();
  calcDisplacement();
  calcAngle();

  if (!digitalRead(MOTOR_EN_BUTTON))
    motorEnableButton();

  // High quality control scheme here...
  // maxSpeed=map(90-angle,0,90,50,255);

  // int speedL=constrain(map(displacement,0,100,0,maxSpeed),0,255);
  // int speedR=constrain(map(-displacement,0,100,0,maxSpeed),0,255);

  bool tilt=digitalRead(8);
  if (tilt) {
    maxSpeed=200;
  } else {
    maxSpeed=100;
  }
  int speedL=constrain((int)(floatMap(rawL,1.5,4.5,10,(float)maxSpeed))-(int)angle,0,255);
  int speedR=constrain((int)(floatMap(rawR,1.5,4.5,10,(float)maxSpeed))-(int)angle,0,255);

  // Apply settings
  // maxSpeed=analogRead(MAX_SPEED_PIN)>>2; // max output speed between 0 and 255
  // minSignal=analogRead(MIN_SIG_PIN); // read potentiometer

  // Calculate motor speeds
  // Vey simple open loop, just increases turning rate as distance increases via skid steering, slowing one side down from the max speed
  // int L_SPEED = constrain(direction ? maxSpeed : (maxSpeed-map(displacement,0,100,0,maxSpeed)),0,255);
  // int R_SPEED = constrain(direction ? (maxSpeed-map(displacement,0,100,0,maxSpeed)) : maxSpeed,0,255);


  
  // display if either amplitude is clipping high
  if (rawL>=rawMax || rawR>=rawMax){
    digitalWrite(CLIPPING_LED,HIGH);
  } else {
    digitalWrite(CLIPPING_LED,LOW);
    // display if either amplitude is too low. If it is, disable motors
    if (rawL<MIN_SIGNAL || rawR<MIN_SIGNAL) {
      if (!lowSignal){
        // motorsEnabled=false;
        digitalWrite(LOW_SIGNAL_LED,HIGH);
        lowSignal=true;
        Serial.println("Low Signal");
      }
    } else {
      if (lowSignal){
        digitalWrite(LOW_SIGNAL_LED,LOW);
        lowSignal=false;
        Serial.println("Signal restored");
      }
    }
  }

  if (motorEnablePressed) {
    motorEnablePressed=false;
    if (motorsEnabled) {
      motorsEnabled=false;
      analogWrite(L_MOTOR_PIN,0);
      analogWrite(R_MOTOR_PIN,0);
      Serial.println("PWM output disabled");
    } else {
      motorsEnabled=true;
      Serial.println("PWM output enabled");
    }
    noInterrupts();
    delay(100); // dodgey debounce
    interrupts();
  }

  // Activate motors only if enable pressed, signal within range, and calibration data loaded
  if (motorsEnabled) {
    analogWrite(L_MOTOR_PIN,speedL);
    analogWrite(R_MOTOR_PIN,speedR);
  } else {
    analogWrite(L_MOTOR_PIN,0);
    analogWrite(R_MOTOR_PIN,0);
  }

  Serial.print(">tilt:");Serial.println(tilt);
  Serial.print(">rawL:");Serial.println(rawL);
  Serial.print(">rawR:");Serial.println(rawR);
  Serial.print(">speedL:");Serial.println(speedL);
  Serial.print(">speedR:");Serial.println(speedR);

  // Serial.print(">linL:");Serial.println(linL);
  // Serial.print(">linR:");Serial.println(linR);
  // Serial.print(">linL_mm:");Serial.println(linL_mm);
  // Serial.print(">linR_mm:");Serial.println(linR_mm);
  Serial.print(">dispL_mm:");Serial.println(dispL_mm);
  Serial.print(">dispR_mm:");Serial.println(dispR_mm);
  Serial.print(">disp:");Serial.println(displacement);
  Serial.print(">angle:");Serial.println(angle);
  // Serial.print(">3D|position:S:cube:R:0:"); Serial.print(angle); Serial.print(":0:P:");Serial.print(displacement);Serial.println(":0:0");
  


}
