
#include <Wire.h>
#include <Servo.h>

//////////////////////////////////////////////////////
//                    PIN LAYOUT                    // 
//////////////////////////////////////////////////////
#define LEFT_DRIVE_SPEED 5
#define LEFT_DRIVE 4
#define RIGHT_DRIVE_SPEED 6
#define RIGHT_DRIVE 7

#define LINE_LEFT A0
#define LINE_MIDDLE A1
#define LINE_RIGHT A2

#define SERVO_PIN 10  // Servo requires PWM pin
Servo servo;

#define TRIG_PIN 9
#define ECHO_PIN 8
#define TEMP_PIN A3

#define HALL_L_PIN 2
#define HALL_R_PIN 3

#define MODEBTN_PIN 12
//////////////////////////////////////////////////////
//                    CONSTANTS                     // 
//////////////////////////////////////////////////////
#define HALL_LEFT 0
#define HALL_RIGHT 1

#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3

#define scanLeft 180
#define scanRight 0
#define centerServo 90

/* Conversion constant from analogRead() output to temperature in C*
   Derived from LM35 temperature sensor datasheet
   Voltage vs. Temperature 0-1500mV <--> 0-150C */
#define CONVERT_TO_TEMP 1/1024.0*500

// Define motor speed control and distance constants
#define DIST_THRESH 5.0
#define MAP_CONST 40
#define MIN_MOVEMENT_POWER 100
#define MAX_SPEED 255

#define ticksPerCM 1.1765

#define LINE_THRESHOLD 100

//////////////////////////////////////////////////////
//                    VARIABLES                     // 
//////////////////////////////////////////////////////

// determines which functionality to run
int mode = 2;   

// debouncing variables
int modeBtnState;
int prevModeBtnState = LOW;
unsigned long lastMBDebounceTime = 0;
long debounceDelay = 50;

// drive variables
volatile int hallLCount = 0;
volatile int hallRCount = 0;
double drive_integral = 0;

void setup()
{
  // enable motor shield for motors
  pinMode(LEFT_DRIVE, OUTPUT);
  pinMode(RIGHT_DRIVE, OUTPUT);
  pinMode(LEFT_DRIVE_SPEED, OUTPUT);
  pinMode(RIGHT_DRIVE_SPEED, OUTPUT);

  // enable ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // enable servos
  servo.attach(SERVO_PIN);

  // enable interrupt service routine for hall effect sensors
  attachInterrupt(digitalPinToInterrupt(HALL_L_PIN), updateHallL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_R_PIN), updateHallR, CHANGE);

  // begin coms with slave Arduino
  Wire.begin();   
  
  Serial.begin(9600);

  //delay(100);
}

void loop() 
{
  // choose mode
  selectMode();   
  switch ( mode ) {
    // drive and scan mode
    case 1:
      // enable interrupts for hall effect sensors
      hallLCount = hallRCount = 0;
      attachInterrupt(digitalPinToInterrupt(HALL_L_PIN), updateHallL, CHANGE);
      attachInterrupt(digitalPinToInterrupt(HALL_R_PIN), updateHallR, CHANGE);
      while ( debounceModeButton() == HIGH ) {
        driveAndScan();
      }
      stopMotors();
      break;
      
    // line following mode
    case 2:
      // disable interrupts when line following
      detachInterrupt(digitalPinToInterrupt(HALL_L_PIN));
      detachInterrupt(digitalPinToInterrupt(HALL_R_PIN));
      while ( debounceModeButton() == HIGH ) {
        lineFollow(255);
      }
      stopMotors();
      break;
      
    default:
      stopMotors();
      selectMode();
      break;
  }
}

//////////////////////////////////////////////////////
//                  MODE SELECT FUNCTIONS           // 
//////////////////////////////////////////////////////
/*
 * Prompts the user to select the functionality to run on the robot
 */
void selectMode() {
  sendToSlave(0);   // prompt user to select mode on LCD on slave Arduino
  byte dispCode = 0;
  int button = -1;
  while( debounceModeButton() == LOW ) {    // mode is confirmed when user toggles mode button
    Wire.requestFrom(8,1);  // reads button presses on slave Arduino's LCD shield
    while(Wire.available())
      button = (int)Wire.read();
    // increment or decrement the display code(looping between 1-2)depending on button presses
    if ( button == 1 ) {
      if( dispCode >= 2 )
        dispCode = 1;
       else
        dispCode++;
    }
    else if( button == 0 ) {
      if ( dispCode <= 1 )
        dispCode = 2;
      else
        dispCode--;
    }
    sendToSlave(dispCode);  // shows user what mode will start if mode button is toggled
    delay(250);
    Serial.print(dispCode);
  }
  mode = (dispCode == 0) ? mode : dispCode;   // resumes previous functionality if user toggles mode button without selecting a mode
  sendToSlave(mode);
}

/*
 * Writes a single bytle to the slave Arduino over I2C
 */
void sendToSlave(byte x) {
  Wire.beginTransmission(8);
  Wire.write(x);
  Wire.endTransmission();
}

/*
 * Tells the slave Arduino to display the given speed and direction on the LCD
 */
void sendSpeedToSlave(byte speed, byte direction) {
  Wire.beginTransmission(8);
  Wire.write(speed);
  Wire.write(direction);
  Wire.endTransmission();
}

/*
 * Debounces the mode button (toggle switch)
 */
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

//////////////////////////////////////////////////////
//                  DRIVE FUNCTIONS                 // 
//////////////////////////////////////////////////////

// Uses P to lower/increase motor speed to match the rotation speed to drive straight
// Needs to be called in a while loop, does not stop the motors
void drive_P_Straight(int direction, int speed)
{ 
  digitalWrite(LEFT_DRIVE, (direction == 2) ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, (direction == 3) ? HIGH : LOW);

  // PI constants and error calculation
  double Kp = 70;
  double Ki = 0.1;
  double dt = 0.01;
  double error = (hallRCount - hallLCount) * Kp;
  drive_integral = drive_integral + error*dt;

  double leftSpeed = 0;
  double rightSpeed = 0;

  // Calculate left motor speed based on error
  if(speed+error+Ki*drive_integral>255)
    leftSpeed = 255;
  else if (speed+error+Ki*drive_integral< MIN_MOVEMENT_POWER)
    leftSpeed = MIN_MOVEMENT_POWER;
  else
    leftSpeed = speed + (error) - Ki*drive_integral;

  // Calculate right motor speed based on error
  if(speed-error-Ki*drive_integral>255)
    rightSpeed = 255;
  else if (speed-error-Ki*drive_integral < MIN_MOVEMENT_POWER)
    rightSpeed = MIN_MOVEMENT_POWER;
  else
    rightSpeed = speed - (error) - Ki*drive_integral;

/*Set motor speed
  Serial.print(leftSpeed);
  Serial.print(" ");
  Serial.print(rightSpeed);
  Serial.print(" ");
  Serial.print(hallLCount);
  Serial.print(" ");
  Serial.print(hallRCount);
  Serial.print(" ");
  Serial.println(error*Ki*drive_integral);
 // */

  // Set the motor speed
  analogWrite(LEFT_DRIVE_SPEED,leftSpeed);
  analogWrite(RIGHT_DRIVE_SPEED,rightSpeed);
}

void turn_Sensored(int direction, int speed, int distance)
{
  // Reset hall effect sensor counts
  hallLCount = hallRCount = 0;
  
  // Set direction
  digitalWrite(LEFT_DRIVE, (direction == LEFT) ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, (direction == RIGHT) ? HIGH : LOW);

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, speed);
  analogWrite(RIGHT_DRIVE_SPEED, speed);
  sendSpeedToSlave(speed, direction);

  // Run motors until hall effect count reaches distance
  while(hallLCount < distance){}
  
  // Stop motors
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

// Drive without acceleration or decleration, Speed 0-255, timed
// Sets the motors, waits and then stops the motors
void drive_Timed(int direction, int speed, int time)
{
  // Set direction
  digitalWrite(LEFT_DRIVE, (direction == 2) ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, (direction == 3) ? HIGH : LOW);

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, speed);
  analogWrite(RIGHT_DRIVE_SPEED, speed);

  delay(time);

  // Stop motors
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

// Stop motors
void stopMotors() 
{
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

/*
 * Update hall effect sensor count every time
 * the Arduino receives an interrupt from either
 * hall effect sensor.
 */
void updateHallL()
{
  hallLCount++;
}
void updateHallR()
{
  hallRCount++;
}


//////////////////////////////////////////////////////
//               PRINCIPLE FUNCTION 1               // 
//////////////////////////////////////////////////////
/*
 * Drive the robot up to an object up to a predetermined threshold. 
 * Once the robot reaches the distance threshold, 
 * the robot will scan its left and right side
 * and select a direction to turn to.
 */
void driveAndScan()
{
  // Get ultrasonic distance
  double initDistance = 0.0;
  
  // Set baseline speed
  double motorSpeed = 210;
  sendSpeedToSlave(motorSpeed, FORWARD);

  // Get the distance 
  do {
    initDistance = getUltrasonic();
    drive_P_Straight(FORWARD, motorSpeed);
  } while (initDistance >= 500 || initDistance == 0.0);
  //Serial.println(distance);

  // Calculate number for ticks to reach the required distance
  double requiredTicks = (initDistance - DIST_THRESH)*ticksPerCM;
  
  drive_integral = 0;
  hallLCount = hallRCount = 0;
  unsigned long prevMillis = 0;
  // Run loop until robot has reached the required distance
  while(hallLCount <  requiredTicks)
  {
    // Call getMotorSpeed to calculate speed based on distance traveled
    motorSpeed = getMotorSpeed(initDistance - (hallLCount  / ticksPerCM));
    
    //Serial.print(initDistance*ticksPerCM - ((hallLCount + hallRCount) / 2.0));
    //Serial.print(" ");
    //Serial.println(motorSpeed);
    //Serial.print(initDistance - ((hallLCount + hallRCount) / 2.0 / ticksPerCM));
    //Serial.print(" ");
    //Serial.println(motorSpeed);
    
    drive_P_Straight(FORWARD, motorSpeed);

    // update the speed displayed on LCD every 500ms
    unsigned long currMillis = millis();        
    if( currMillis - prevMillis >= 500 ) {
      prevMillis = currMillis;
      sendSpeedToSlave(motorSpeed, FORWARD);
    }
  }

  // Stop motors
  stopMotors();
  sendSpeedToSlave(0, FORWARD);
  
  // Find a direction to turn to, if scanSurroundings returns -1, scan again
  int turnDir;
  do
  {
    turnDir = scanSurroundings(); 
  } while(turnDir == -1);

  delay(500);

  // Turn left or right based on clearance of each side
  turn_Sensored(turnDir, 150, 11);
  sendSpeedToSlave(0, FORWARD);
  delay(500);  
}

/*
 * Calculate the motor speed based on 
 * the measured ultrasonic sensor value and
 * the distance an object is away from the robot.
 * Param: distance - measured distance from ultrasonic
 * Return: motorVal - updated motor speed
 */
int getMotorSpeed(double distance)
{
  // Set motor value to MAX_SPEED
  int motorVal = MAX_SPEED;

  // Check if robot has not approached object
  if(distance > DIST_THRESH)
  {
    // Calculate new motor value based on distance of object using map() function
    motorVal = map(distance, DIST_THRESH, MAP_CONST, MIN_MOVEMENT_POWER, 210);
  }
  else
  {
    motorVal = 0;
  }
  return motorVal;
}

/*
 * Check the left and right side of the robot
 * using the ultrasonic sensor on a servo mount.
 * Determine which direction to turn based on
 * the further of the two distances.
 * Returns: 0 - to turn the robot left
 *          1 - to turn the robot right
 *          2 - scan again
 */
int scanSurroundings()
{
  // Set the servo to scan left side
  servo.write(scanLeft);
  delay(1500);
  
  // Get distance of object on the left side
  double distanceLeft = getUltrasonic();
  delay(250);

  // Set the servo to scan right side
  servo.write(scanRight);
  delay(1500);

  // Get distance of object on the right side
  double distanceRight = getUltrasonic();
  delay(250);

  // Reset servo to face forward
  servo.write(centerServo);

  // If left and right distances are garbage, tell main loop to scan again
  if(distanceLeft > 500 && distanceRight > 500)
    return -1;
  // Determine which direction to turn in based on surroundings
  if (distanceLeft > distanceRight)
    return LEFT;
  else
    return RIGHT;
}

// Return distance in centimetres
double getUltrasonic()
{
  //double temp = analogRead(tempPin) * CONVERT_TO_TEMP;
  double temp = 21; // Standard temp in celsius
  double speedOfSound = (331.5 + (0.6 * temp)) * 0.0001; // cm/microsecond

  // Ping the ultrasonic
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Return pulse from echo
  return (pulseIn(ECHO_PIN, HIGH) / 2) * speedOfSound;
}

void servoTest()
{
  servo.write(scanLeft);
  delay(2000);
  servo.write(centerServo);
  delay(2000);
  servo.write(scanRight);
  delay(2000);
}

//////////////////////////////////////////////////////
//               PRINCIPLE FUNCTION 2               // 
//////////////////////////////////////////////////////
/*
 * Send the robot to trace a line. Each optical sensor's value
 * is read and the speed of each wheel is determined based on
 * which optical sensors detect the line.
 */
void lineFollow(int motorValue)
{
  // Set speed
  int speed = motorValue;

  // Obtain sensor data from optical sensors
  int L_LINE = analogRead(LINE_LEFT);
  int M_LINE = analogRead(LINE_MIDDLE);
  int R_LINE = analogRead(LINE_RIGHT);

  // Set direction to forwards
  digitalWrite(LEFT_DRIVE, HIGH);
  digitalWrite(RIGHT_DRIVE, LOW);

  // If only right sensor is on, hard turn right
  if (R_LINE > LINE_THRESHOLD)
  {
    // Swerve right
    analogWrite(LEFT_DRIVE_SPEED,  speed);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
  }
  // If only left sensor is on, hard turn left
  else if (L_LINE > LINE_THRESHOLD)
  {
    // Swerve left
    analogWrite(LEFT_DRIVE_SPEED, 0);
    analogWrite(RIGHT_DRIVE_SPEED, speed);
  }
  // If both right and middle sensors are on, minor turn right
  else if (R_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Slow turn right
    analogWrite(LEFT_DRIVE_SPEED,  speed - 75);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
  }
  // If both left and middle sensors are on, minor turn left
  else if (L_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Slow turn left
    analogWrite(LEFT_DRIVE_SPEED,  0);
    analogWrite(RIGHT_DRIVE_SPEED, speed - 75);
  }
  // If middle sensor is on, drive straight
  else if(M_LINE > LINE_THRESHOLD)
  {
    // Drive forward
    analogWrite(LEFT_DRIVE_SPEED, speed);
    analogWrite(RIGHT_DRIVE_SPEED, speed);
  }
}

void lineTest()
{
  int leftVal = analogRead(LINE_LEFT);
  int midVal = analogRead(LINE_MIDDLE);

  Serial.print(leftVal);
  Serial.print(" ");
  int rightVal = analogRead(LINE_RIGHT);

  //    LEFT < LINE_THRESHOLD? 1 : 0;
  //    MIDDLE = MIDDLE < LINE_THRESHOLD? 1 : 0;
  //    RIGHT = RIGHT < LINE_THRESHOLD? 1 : 0;
  Serial.print(midVal);
  Serial.print(" ");
  Serial.println(rightVal);
}

