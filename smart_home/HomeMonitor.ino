
#include <Wire.h>

// sensor pins
#define DOOR_HALL_PIN 3   // ON ISR
#define PIR_PIN 2         // ON ISR
#define PCELL0_PIN A0
#define PCELL1_PIN A1
#define PCELL2_PIN A2
#define LASER0_PIN 9
#define LASER1_PIN 10
#define LASER2_PIN 11
// light pins
#define LIGHT0_PIN 4   // washroom
#define LIGHT1_PIN 5   // bedroom
#define LIGHT2_PIN 6   // living room
#define LIGHT3_PIN 7   // kitchen
#define LIGHT4_PIN 8   // master bedroom

const long motionDetectTime = 500;   // max time that motion is detected before alarm triggers
const int pCellAlarmThreshold = 125;  // max pcell value before alarm triggers

// arm/disarm alarms
int doorArmed = 0;
int pirArmed = 0;
int pCellArmed = 0;

// light status
int light0Status = 0;
int light1Status = 0;
int light2Status = 0;
int light3Status = 0;
int light4Status = 0;
//light timers
unsigned long light0Timer = 0;
unsigned long light1Timer = 0;
unsigned long light2Timer = 0;
unsigned long light3Timer = 0;
unsigned long light4Timer = 0;
// max time that a light will remain on (default ~60s)
unsigned long light0OnTime = 60000;
unsigned long light1OnTime = 60000;
unsigned long light2OnTime = 60000;
unsigned long light3OnTime = 60000;
unsigned long light4OnTime = 60000;

// alarms and flags
int doorAlarm = 0;
int pirAlarm = 0;
int pirStatus = 0;
int pCellAlarm = 0;
int triggered = 0;
int pwTrigger = 0;
int manualAlarm = 0;

// status strings
String prevStatusStr = "";

void setup() {
  Serial.begin(38400);    // default rpi baud rate
  Wire.begin(8);
  Wire.onReceive(recieveEvent);   // listening to lcd+keypad arduino to arm/disarm system
  prevStatusStr.reserve(16);      // reserve 16 bytes (arbitary) to store the previous status string

  // set up interrupt
  attachInterrupt(digitalPinToInterrupt(DOOR_HALL_PIN), doorHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionSensISR, CHANGE);

  // pinmodes
  pinMode(LIGHT0_PIN, OUTPUT);
  pinMode(LIGHT1_PIN, OUTPUT);
  pinMode(LIGHT2_PIN, OUTPUT);
  pinMode(LIGHT3_PIN, OUTPUT);
  pinMode(LIGHT4_PIN, OUTPUT);

  pinMode(LASER0_PIN, OUTPUT);
  pinMode(LASER1_PIN, OUTPUT);
  pinMode(LASER2_PIN, OUTPUT);

  // initialize laser and light states
  digitalWrite(LIGHT0_PIN, HIGH);
  digitalWrite(LIGHT1_PIN, HIGH);
  digitalWrite(LIGHT2_PIN, HIGH);
  digitalWrite(LIGHT3_PIN, HIGH);
  digitalWrite(LIGHT4_PIN, HIGH);

  int lasers = pCellArmed ? HIGH : LOW;
  controlLasers(lasers);
}

void loop() {
  checkAndTriggerAlarms();

  // send new status string if it changed from old one
  String statusStr = createStatusStr();
  if ( !statusStr.equals(prevStatusStr) ) {
    Serial.print(statusStr);
    prevStatusStr = statusStr;
  }
  
  powerSaveLights();
}


//////////////////////////////////////////////////////
//              SYSTEM CONTROL CODE                 //
//////////////////////////////////////////////////////

/*
 * Create a binary status string containing all information needed for RPi to process the state of the house.
 */
String createStatusStr() {
  String s;                 // byte #
  s.concat(triggered);      // 0
  s.concat(pwTrigger);      // 1
  s.concat(doorAlarm);      // 2
  s.concat(pirAlarm);       // 3
  s.concat(pCellAlarm);     // 4
  s.concat(doorArmed);      // 5
  s.concat(pirArmed);       // 6
  s.concat(pCellArmed);     // 7
  s.concat(manualAlarm);    // 8
  s.concat(light0Status);   // 9
  s.concat(light1Status);   // 10
  s.concat(light2Status);   // 11
  s.concat(light3Status);   // 12
  s.concat(light4Status);   // 13
  s.concat(":");            // 14 - delimiter
  return s;
}

/*
 * Process commands from the RPi to adjust the state of the system.
 */
void serialEvent() {
  delay(10);    // wait for all data to be written
  if ( Serial.available() == 1 ) {    // security commands
    int x = Serial.read() - '0';
    switch (x) {
      case 0:   // alarm off
        turnOffAndResetAlarms();
        break;
      case 1:   // alarm on
        manualAlarmOn();
        break;
      case 2:   // arm entire system
        controlLasers(HIGH);
        delay(10);
        doorArmed = pirArmed = pCellArmed = 1;
        break;
      case 3:   // disarm entire system
        doorArmed = pirArmed = pCellArmed = 0;
        delay(10);
        controlLasers(LOW);
        break;
      case 4:   // arm door hall sensor
        doorArmed = 1;
        break;
      case 5:   // arm motion sensor
        pirArmed = 1;
        break;
      case 6:   // arm lasers
        controlLasers(HIGH);
        delay(10);
        pCellArmed = 1;
        break;
      case 7:   // disarm door hall sensor
        doorArmed = 0;
        break;
      case 8:   // disarm motion sensor
        pirArmed = 0;;
        break;
      case 9:   // disarm lasers
        pCellArmed = 0;
        delay(10);
        controlLasers(LOW);
        break;
      default: break;
    }
  }
  else if ( Serial.available() == 2 ) {   // light commands
    int light = Serial.read() - '0';  // convert from ascii to byte value
    if ( light >= 0 && light <= 5 ) {
      int set = Serial.read() - '0';  // convert from ascii to byte value
      int state = set ? LOW : HIGH;
      // turn on and off lights
      switch (light) {
        case 0:
          digitalWrite(LIGHT0_PIN, state);
          light0Status = set;
          light0Timer = millis();
          break;
        case 1:
          digitalWrite(LIGHT1_PIN, state);
          light1Status = set;
          light1Timer = millis();
          break;
        case 2:
          digitalWrite(LIGHT2_PIN, state);
          light2Status = set;
          light2Timer = millis();
          break;
        case 3:
          digitalWrite(LIGHT3_PIN, state);
          light3Status = set;
          light3Timer = millis();
          break;
        case 4:
          digitalWrite(LIGHT4_PIN, state);
          light4Status = set;
          light4Timer = millis();
          break;
        case 5:
          digitalWrite(LIGHT0_PIN, state);
          digitalWrite(LIGHT1_PIN, state);
          digitalWrite(LIGHT2_PIN, state);
          digitalWrite(LIGHT3_PIN, state);
          digitalWrite(LIGHT4_PIN, state);
          light0Timer = light1Timer = light2Timer = light3Timer = light4Timer = millis();
          light0Status = light1Status = light2Status = light3Status = light4Status = set;
          break;
        default: break;
      }
    }
    // set timers for lights
    else if ( light >= 6 && light <= 10 ) {
      unsigned long setTime = (byte)Serial.read() * 1000.0;
      switch (light) {
        case 6:
          light0OnTime = setTime;
          break;
        case 7:
          light1OnTime = setTime;
          break;
        case 8:
          light2OnTime = setTime;
          break;
        case 9:
          light3OnTime = setTime;
          break;
        case 10:
          light4OnTime = setTime;
          break;
        default: break;
      }
    }
  }
  Serial.print("r:");   // tell RPi that Arduino is ready for next command
}

/*
 * Process commands from the keypad+lcd Arduino.
 */
void recieveEvent(int howMany) {
  if (Wire.available()) {
    int x = Wire.read();
    switch (x) {
      case 0:   // pw correct, disarm systems
        doorArmed = pirArmed = pCellArmed = 0;
        delay(10);
        controlLasers(LOW);
        break;
      case 1:   // owner leaving, arm systems
        controlLasers(HIGH);
        delay(10);
        doorArmed = pirArmed = pCellArmed = 1;
        break;
      case 2:   // pw correct, disable alarm, disarm systems
        doorArmed = pirArmed = pCellArmed = 0;
        delay(10);
        controlLasers(LOW);
        pwTrigger = manualAlarm = 0;
        break;
      case 3:   // pw incorrect, sound alarm
        pwTrigger = manualAlarm = 1;
        break;
    }
  }
}

/*
 * Reset the system to a disarmed state without alarms.
 */
void turnOffAndResetAlarms() {
  doorAlarm = pirAlarm = pCellAlarm = doorArmed = pirArmed = pCellArmed = triggered = pwTrigger = manualAlarm = pirStatus = 0;
  delay(10);
}

/*
 * Turn on the alarm.
 */
void manualAlarmOn() {
  manualAlarm = 1;
}

/*
 * Turn on/off the lasers.
 * Note: Disarm the laser system before turning lasers off.
 */
void controlLasers(int state) {
  digitalWrite(LASER0_PIN, state);
  digitalWrite(LASER1_PIN, state);
  digitalWrite(LASER2_PIN, state);
}


//////////////////////////////////////////////////////
//            MONITORING AND ALARM CODE             //
//////////////////////////////////////////////////////

/*
 * Check if lights have been on for longer than their respective lightOnTime.
 * Turn off that light if it is the case.
 */
void powerSaveLights() {
  unsigned long currTime = millis();
  if ( light0Status && currTime - light0Timer >= light0OnTime ) {
    digitalWrite(LIGHT0_PIN, HIGH);
    light0Status = 0;
  }
  if ( light1Status && currTime - light1Timer >= light1OnTime ) {
    digitalWrite(LIGHT1_PIN, HIGH);
    light1Status = 0;
  }
  if ( light2Status && currTime - light2Timer >= light2OnTime ) {
    digitalWrite(LIGHT2_PIN, HIGH);
    light2Status = 0;
  }
  if ( light3Status && currTime - light3Timer >= light3OnTime ) {
    digitalWrite(LIGHT3_PIN, HIGH);
    light3Status = 0;
  }
  if ( light4Status && currTime - light4Timer >= light4OnTime ) {
    digitalWrite(LIGHT4_PIN, HIGH);
    light4Status = 0;
  }
}

/*
 * Update all alarm flags and trigger alarm if neccessary.
 */
void checkAndTriggerAlarms() {
  checkAlarms();
  triggerAlarm();
}

/*
 * Trigger the alarm if a system is armed and flagged.
 */
void triggerAlarm() {
  if ( (doorArmed & doorAlarm) |  (pirArmed & pirAlarm) | (pCellArmed & pCellAlarm) ) {
    if ( !triggered ) {
      triggered = manualAlarm = 1;
    }
  }
}

/*
 * Update the flags of each alarm.
 */
void checkAlarms() {
  checkPIR();
  if (pCellArmed)
    checkPCells();
}

/*
 * Update the pirAlarm flag.
 */
void checkPIR() {
  unsigned long currTime = millis();
  // if armed, use latching logic
  if (pirArmed) {
    while ( pirStatus & !pirAlarm) {
      if ( millis() - currTime >= motionDetectTime )
        pirAlarm = 1;
    }
  }
  // otherwise, allow flag to freely change
  else {
    if (!pirAlarm) {
      while ( pirStatus & !pirAlarm ) {
        if ( millis() - currTime >= motionDetectTime )
          pirAlarm = 1;
      }
    }
    else {
      while(!pirStatus & pirAlarm) {
        if ( millis() - currTime >= motionDetectTime )
          pirAlarm = 0;
      }
    }
  }
}

/*
 * Update the pCellAlarm flag. Triggered if any photoresistor reading drops below the threshold level.
 */
void checkPCells() {
  if ( analogRead(PCELL0_PIN) < pCellAlarmThreshold || analogRead(PCELL1_PIN) < pCellAlarmThreshold || analogRead(PCELL2_PIN) < pCellAlarmThreshold)
    pCellAlarm = 1;
}

/*
 * ISR
 * Update doorAlarm flag.
 */
void doorHallISR() {
  doorAlarm = digitalRead(DOOR_HALL_PIN);
}

/*
 * ISR
 * Update PIR sensor reading.
 * Note: Does NOT update pirAlarm flag.
 */
void motionSensISR() {
  pirStatus = digitalRead(PIR_PIN);
}
