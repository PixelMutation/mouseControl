#include <Arduino.h>
#include <PID_v1.h>
#include "Oversampling.h"
#include <EEPROM.h>

/* -------------------------------------------------------------------------- */
/*                                   Config                                   */
/* -------------------------------------------------------------------------- */

// Coil spacing in mm
#define COIL_SPACING 90
// Below the min signal, that coil is ignored. If both below, motors disabled.
#define MIN_SIGNAL 0
// Place the robot at this displacement (pos to the right) during calibration (align with notch on sensor)
#define CALIBRATION_DIST 25 // must be less than COIL_SPACING/2, musn't be close to the coils!
// Number of extra bits of resolution to aquire when sampling
#define OVERSAMPLING 0
// Smoothing factor (using simple EWMA to reduce processing time and memory)
#define SMOOTHING 100
// Offset from 2*mid value before signal is inverted
#define INVERSION_OFFSET -10

#define ADC_OFFSET 0.5

#define A_REF 4.5

#define ADC_RANGE 1024.0

#define BAUD 74880

/* ------------------------------- PID config ------------------------------- */

#define PID_TARGET 0
#define PID_SAMPLE_TIME 20

#define PID_OUT_MIN -127
#define PID_OUT_MAX 127

#define KP_MIN 0
#define KP_MAX 3

#define KI_MIN 0
#define KI_MAX 2

#define KD_MIN 0
#define KD_MAX 3

/* -------------------------------------------------------------------------- */
/*                               Pin definitions                              */
/* -------------------------------------------------------------------------- */

/* ------------------------------ Coil signals ------------------------------ */
// Pins for peak detection signal
#define L_SIG_PIN A6
#define R_SIG_PIN A7
/* ---------------------------    Potentionmeters --------------------------- */
// pot which sets max speed
// #define MAX_SPEED_PIN A4 
// pots for tuning pid
#define KP_PIN A0
#define KI_PIN A2
#define KD_PIN A3
/* --------------------------------- Sensors -------------------------------- */
#define SLOPE_SENSOR 8
/* --------------------------------- Buttons -------------------------------- */
#define MOTOR_EN_BUTTON 2 // motor enable pin
#define CAL_LEFT_BUTTON 4 // press when robot at calibration distance on left side
#define CAL_RIGHT_BUTTON 3 // press when robot at calibration distance on right side
/* --------------------------------- Outputs -------------------------------- */
// PWM 
#define L_MOTOR_PIN 10 // pwm for left motor
#define R_MOTOR_PIN 11 // pwm for right motor
// LEDs
#define OUTSIDE_WIRE_LED 6 // on if vehicle outside the wires
#define LOW_SIGNAL_LED 7 // on if signal below range

/* -------------------------------------------------------------------------- */
/*                                  Variables                                 */
/* -------------------------------------------------------------------------- */

/* -------------------------------- Settings -------------------------------- */

// Calibration values
// int rawDistanceZero[2]={-1,-1}; // 1/E at zero distance
// int rawDistanceCalibration[2]={-1,-1}; // 1/E at calibration distance
// Values from potentiometers
uint16_t maxSpeed=100; // max speed to prevent flying off
uint16_t boostSpeed=255;
float KP,KI,KD; // PID loop settings

/* ------------------------------ Measurements ------------------------------ */
// raw E pk-pk signal
float rawL,rawR; 
float rawMax=A_REF; // max voltage
// linearised 1/E signal
float linL,linR; 
// calibration values

struct settings{
  float calL1,calR1; // Calibration distance away from sensor
  float calL2,calR2; // Calibration distance close to sensor
};
settings Settings;

float linL_mm,linR_mm; // scaled to mm

float dispL_mm,dispR_mm; // inflection points removed (negative when wire goes past coil)

double displacement=0; // linear displacement mapped based on calibration values at 0mm and calibration distance
double angle=0; // angle between vehicle and tangent of AC line
double target=0;
bool calibrated=false;

/* --------------------------------- Outputs -------------------------------- */

double steeringRatio=0; // differential steering applies an offset to avgspeed fr each side
double avgSpeed=0; // avg speed of wheels, range 0-255
bool motorsEnabled=false;

/* --------------------------------- Objects -------------------------------- */

Oversampling lPeak(10,10+OVERSAMPLING,1);
Oversampling rPeak(10,10+OVERSAMPLING,1);
// Setpoint is target displacement 0
PID pidSteering(&displacement,&steeringRatio,&target,KP,KI,KD,DIRECT);


/* -------------------------------------------------------------------------- */
/*                                   Program                                  */
/* -------------------------------------------------------------------------- */

/* -------------------------------- Functions ------------------------------- */
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// read values from EEPROM to avoid recalibrating each time
void loadEEPROM() {
  Settings=EEPROM.get(0,Settings);
  calibrated=true;
}
void saveEEPROM() {
  EEPROM.put(0,Settings);
}
// read settings
void updatePID() {
  KP=floatMap(analogRead(KP_PIN),0,1024,KP_MIN,KP_MAX);
  // KI=floatMap(analogRead(KI_PIN),0,1024,KI_MIN,KI_MAX);
  KD=floatMap(analogRead(KD_PIN),0,1024,KD_MIN,KD_MAX);
  maxSpeed=analogRead(KI_PIN)>>2;
  // maxSpeed=analogRead(MAX_SPEED_PIN);
  pidSteering.SetTunings(KP,KI,KD);

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
void calRight() {
  if (digitalRead(CAL_RIGHT_BUTTON)==LOW) {
    Serial.println("Setting Right Calibration Point");
    Settings.calR1=linR;
    Settings.calL2=linL;
    delay(500); // prevent multiple activations
    calibrated=true;
    saveEEPROM();
  }
}
// Check if zero button pressed, set calibration points
void calLeft() {
  if (digitalRead(CAL_LEFT_BUTTON)==LOW) {
    Serial.println("Setting Left Calibration Point");
    Settings.calR2=linR;
    Settings.calL1=linL;
    delay(500); // prevent multiple activations
    saveEEPROM();
  }
}

// Map measured values to real units
void scaleToMM() {
  if (linL==NAN)
    linL_mm=0;
  else
    linL_mm=floatMap(linL,Settings.calL2,Settings.calL1,COIL_SPACING/2-CALIBRATION_DIST,COIL_SPACING/2+CALIBRATION_DIST);
  if (linR==NAN)
    linR_mm=0;
  else
    linR_mm=floatMap(linR,Settings.calR2,Settings.calR1,COIL_SPACING/2-CALIBRATION_DIST,COIL_SPACING/2+CALIBRATION_DIST);
  
}
// As soon as the difference is greater than COIL_SPACING, a coil has passed inflection
// So we invert the output such that crossing the wire gives a negative output
bool outsideWires=false;
void removeInflection() {
  if (linL_mm-linR_mm>(float)(COIL_SPACING+INVERSION_OFFSET)) {
    // Serial.println("Outside wires");
    dispR_mm=-linR_mm;
    outsideWires=true;
  } else {
    dispR_mm=linR_mm;
    outsideWires=false;
  }
  if (linR_mm-linL_mm>(float)(COIL_SPACING+INVERSION_OFFSET)) {
    // Serial.println("Outside wires");
    dispL_mm=-linL_mm;
    outsideWires=true;
  } else {
    dispL_mm=linL_mm;
    outsideWires=false;
  }
  // dispL_mm=linL_mm;
  // dispR_mm=linR_mm;
}
void calcDisplacement() {
  displacement=dispL_mm-dispR_mm;
  // if (outsideWires)
  //   displacement=-displacement;
  displacement/=2;
}
void calcAngle() {
  float rawAngle=dispL_mm+dispR_mm;
  angle=-floatMap(rawAngle,0,COIL_SPACING,360,0);
}
bool boost=false;
bool tilt=false;
long boostStop;
// read slope sensor, boost if detected
void detectSlope() {
  tilt=digitalRead(SLOPE_SENSOR);
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
  Serial.begin(BAUD);
  EEPROM.begin();
  // init PWM pins
  pinMode(L_MOTOR_PIN,OUTPUT);
  pinMode(R_MOTOR_PIN,OUTPUT);
  pinMode(SLOPE_SENSOR,INPUT_PULLUP);
  // init LED pins
  pinMode(OUTSIDE_WIRE_LED,OUTPUT);
  pinMode(LOW_SIGNAL_LED,OUTPUT);
  // init button pins with pullups, thus read LOW when pressed 
  pinMode(MOTOR_EN_BUTTON,INPUT_PULLUP);
  pinMode(CAL_LEFT_BUTTON,INPUT_PULLUP);
  pinMode(CAL_RIGHT_BUTTON,INPUT_PULLUP);  

  // readPotentiometers(); 
  loadEEPROM();

  // rising edge interrupt on enable button
  // attachInterrupt(digitalPinToInterrupt(MOTOR_EN_BUTTON),motorEnableButton,FALLING);
  pidSteering.SetOutputLimits(PID_OUT_MIN,PID_OUT_MAX);
  pidSteering.SetSampleTime(PID_SAMPLE_TIME);
  pidSteering.SetMode(AUTOMATIC);
  
}

/* ---------------------------------- Loop ---------------------------------- */

bool lowSignal=false;
long prevupdatePID=0;

void loop() {
  
  if (millis()-prevupdatePID>500){
    prevupdatePID=millis();
    updatePID();
  }
  
  // Analog read
  readInductors();
  // Read buttons
  calLeft();
  calRight();
  // saveButton();
  // Calculate vehicle position
  scaleToMM();
  removeInflection();
  calcDisplacement();
  calcAngle();

  detectSlope();

  if (!digitalRead(MOTOR_EN_BUTTON)) {
    motorEnableButton();
    delay(500);
  }

  digitalWrite(OUTSIDE_WIRE_LED,outsideWires);

  // Apply PID
  int speedR=int(steeringRatio)+255;
  int speedL=255-int(steeringRatio);
  // Apply max speed
  speedL=map(speedL,0,255,0,maxSpeed);
  speedR=map(speedR,0,255,0,maxSpeed);

  
  // display if either amplitude is clipping high
  if (rawL>=rawMax || rawR>=rawMax){
  } else {
    // display if either amplitude is too low. If it is, disable motors
    if (rawL<MIN_SIGNAL || rawR<MIN_SIGNAL) {
      if (!lowSignal){
        motorsEnabled=false;
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
  Serial.print(">linL_mm:");Serial.println(linL_mm);
  Serial.print(">linR_mm:");Serial.println(linR_mm);
  Serial.print(">dispL_mm:");Serial.println(dispL_mm);
  Serial.print(">dispR_mm:");Serial.println(dispR_mm);
  Serial.print(">disp:");Serial.println(displacement);
  Serial.print(">angle:");Serial.println(angle);

  Serial.print(">KP:");Serial.println(KP);
  Serial.print(">KI:");Serial.println(KI);
  Serial.print(">KD:");Serial.println(KD);

  Serial.print(">ratio:");Serial.println(steeringRatio);
  // Serial.print(">3D|position:S:cube:R:0:"); Serial.print(angle); Serial.print(":0:P:");Serial.print(displacement);Serial.println(":0:0");
  
  if (!isnan(displacement)&&calibrated)
    pidSteering.Compute();
}