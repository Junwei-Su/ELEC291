
#include <LiquidCrystal.h>
#include <IRremote.h>
#include <Wire.h>

// pin assignments
const int ULTRASONIC_PIN1 = 4;
const int ULTRASONIC_PIN2 = 3;
const int LINESENSL_PIN = A0;
const int LINESENSR_PIN = A1;
const int LINESENSM_PIN = A2;
const int IRRECV_PIN = 11;
const int HALLL_PIN = 2;
const int HALLR_PIN = 3;
const int MODEBTN_PIN = 12;
const int MODEBTNUP_PIN = 10;
const int MODEBTNDOWN_PIN = 9;

int ultrasonicDistance;
unsigned long prevUltrasonicMilli = 0;
const long ULTRASONIC_INTERVAL = 250;

int hallLCount = 0;
int hallRCount = 0;

IRrecv irrecv(IRRECV_PIN);
decode_results IRVal;

int mode = 1;
int modeBtnState;
int prevModeBtnState = LOW;
unsigned long lastMBDebounceTime = 0;
int modeBtnUpState;
int prevModeBtnUpState = LOW;
unsigned long lastMBUDebounceTime = 0;
int modeBtnDownState;
int prevModeBtnDownState = LOW;
unsigned long lastMBDDebounceTime = 0;
long debounceDelay = 50;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();

  attachInterrupt(digitalPinToInterrupt(HALLL_PIN), updateHallL, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLR_PIN), updateHallR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLL_PIN), updateHallL, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALLR_PIN), updateHallR, FALLING);
  
  Wire.begin();
}

void loop() {
  selectMode(); // choose mode

  switch ( mode ) {
    case 1:
      while ( debounceModeButton() == HIGH ) {
        //lineFollow();
      }
      break;

    case 2:
      while ( debounceModeButton() == HIGH ) {
        int distance = getUltrasonic();
        /*motorSpeed = getMotorSpeed(distance);
          //driveStraight(motorSpeed, L_sensor, R_sensor);
          /if (distance < distanceThreshold) {
          int dir = scanSurroundings();
          turn(dir, 500);
          }*/
      }
      break;

    case 3:
      while ( debounceModeButton() == HIGH) {
        // clarence loop
      }
      break;

    case 4:
      while ( debounceModeButton() == HIGH) {
        // kwong loop
      }
      break;

    default:
      selectMode();
      break;
  }
}

void selectMode() {
  sendToSlave(0);
  byte dispCode = 0;
  while( modeBtnState == LOW ) {
    debounceAllButtons();
    if ( modeBtnDownState = HIGH ) {
      if( dispCode >= 4 )
        dispCode = 1;
       else
        dispCode++;
    }
    else if( modeBtnUpState == HIGH ) {
      if ( dispCode <= 1 )
        dispCode = 4;
      else
        dispCode--;
    }
    sendToSlave(dispCode);
    delay(250);
  }
  mode = dispCode;
}

void sendToSlave(byte x) {
  Wire.beginTransmission(8);
  Wire.write(x);
  Wire.endTransmission();
}

void debounceAllButtons() {
  debounceModeButton();
  debounceModeDownButton();
  debounceModeDownButton();
}

int debounceModeButton() {
  int reading = digitalRead(MODEBTN_PIN);
  if ( reading != prevModeBtnState ) {
    lastMBDebounceTime = millis();
  }
  if ( (millis() - lastMBDebounceTime) >= debounceDelay ) {
  if ( reading != modeBtnState )
      modeBtnState = reading;
  }
  prevModeBtnState = reading;
  return modeBtnState;
}

int debounceModeUpButton() {
  int reading = digitalRead(MODEBTNUP_PIN);
  if ( reading != prevModeBtnUpState ) {
    lastMBUDebounceTime = millis();
  }
  if ( (millis() - lastMBUDebounceTime) >= debounceDelay ) {
  if ( reading != modeBtnState )
      modeBtnUpState = reading;
  }
  prevModeBtnUpState = reading;
  return modeBtnUpState;
}

int debounceModeDownButton() {
  int reading = digitalRead(MODEBTNDOWN_PIN);
  if ( reading != prevModeBtnDownState ) {
    lastMBDDebounceTime = millis();
  }
  if ( (millis() - lastMBDDebounceTime) >= debounceDelay ) {
  if ( reading != modeBtnDownState )
      modeBtnDownState = reading;
  }
  prevModeBtnDownState = reading;
  return modeBtnDownState;
}

int getUltrasonic() {
  unsigned long currMilli = millis();
  if ( currMilli - prevUltrasonicMilli >= ULTRASONIC_INTERVAL ) {
    prevUltrasonicMilli = currMilli;
    //ultrasonicDistance = getUltrasonic(ULTRASONIC_PIN1, ULTRASONIC_PIN2);
  }
  return ultrasonicDistance;
}

void updateHallL() {
  hallLCount++;
}

void updateHallR() {
  hallRCount++;
}

