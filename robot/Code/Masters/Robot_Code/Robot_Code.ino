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

//////////////////////////////////////////////////////
//                    VARIABLES                     // 
//////////////////////////////////////////////////////
#define HALL_LEFT 0
#define HALL_RIGHT 1

#define FORWARD 0
#define BACKWARD 1
#define RIGHT 0
#define LEFT 1

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

volatile int hallLCount = 0;
volatile int hallRCount = 0;

void setup()
{
  pinMode(LEFT_DRIVE, OUTPUT);
  pinMode(RIGHT_DRIVE, OUTPUT);
  pinMode(LEFT_DRIVE_SPEED, OUTPUT);
  pinMode(RIGHT_DRIVE_SPEED, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo.attach(SERVO_PIN);

  attachInterrupt(digitalPinToInterrupt(HALL_L_PIN), updateHallL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_R_PIN), updateHallR, CHANGE);
  
  Serial.begin(9600);
  delay(100);
}

void loop() 
{
  driveAndScan();
  //servoTest();
  //drive_P_Straight(FORWARD,200);
 // lineFollow(255);
  //turn_Sensored(RIGHT, 150, 11);
  //delay(2500);
}

//////////////////////////////////////////////////////
//                  DRIVE FUNCTIONS                 // 
//////////////////////////////////////////////////////
int drive_integral = 0;

// Uses P to lower/increase motor speed to match the rotation speed to drive straight
// Needs to be called in a while loop, does not stop the motors
// Reset encoders before first call
void drive_P_Straight(int direction, int speed)
{
  digitalWrite(LEFT_DRIVE, direction ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, direction ? HIGH : LOW);

  // Calculate error
  double Kp = 80;
  double Ki = 0.1;
  double dt = 0.005;
  double error = (hallRCount - hallLCount) * Kp;
  drive_integral = drive_integral + error*dt;

  double leftSpeed = 0;
  double rightSpeed = 0;

  if(speed+error+Ki*drive_integral>255)
    leftSpeed = 255;
  else if (speed+error+Ki*drive_integral< MIN_MOVEMENT_POWER)
    leftSpeed = MIN_MOVEMENT_POWER;
  else
    leftSpeed = speed + error + Ki*drive_integral;

  if(speed-error-Ki*drive_integral>255)
    rightSpeed = 255;
  else if (speed-error-Ki*drive_integral < MIN_MOVEMENT_POWER)
    rightSpeed = MIN_MOVEMENT_POWER;
  else
    rightSpeed = speed - error -  Ki*drive_integral;

  /* Set motor speed
  Serial.print(leftSpeed);
  Serial.print(" ");
  Serial.print(rightSpeed);
  Serial.print(" ");
  Serial.print(hallLCount);
  Serial.print(" ");
  Serial.print(hallRCount);
  Serial.print(" ");
  Serial.println(error);
  */
  
  analogWrite(LEFT_DRIVE_SPEED,leftSpeed);
  analogWrite(RIGHT_DRIVE_SPEED,rightSpeed);
}

void turn_Sensored(int direction, int speed, int distance)
{
  hallLCount = hallRCount = 0;
  // Set direction
  digitalWrite(LEFT_DRIVE, direction ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, direction ? LOW : HIGH);

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, speed);
  analogWrite(RIGHT_DRIVE_SPEED, speed);

  while(hallLCount < distance)
  {
  }
  // Stop motors
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
  
}

// Drive without acceleration or decleration, Speed 0-255, timed
// Sets the motors, waits and then stops the motors
void drive_Timed(int direction, int speed, int time)
{
  // Set direction
  digitalWrite(LEFT_DRIVE, direction ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, direction ? HIGH : LOW);

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, speed);
  analogWrite(RIGHT_DRIVE_SPEED, speed);

  delay(time);

  // Stop motors
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

void turn_Timed(int direction, int speed, int time)
{
  // Set direction
  digitalWrite(LEFT_DRIVE, ~direction);
  digitalWrite(RIGHT_DRIVE, ~direction);

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, speed);
  analogWrite(RIGHT_DRIVE_SPEED, speed);

  delay(time);

  // Stop motors
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

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
  double initDistance = 0;
  double motorSpeed = MAX_SPEED;
  do
  {
    initDistance = getUltrasonic();
    drive_P_Straight(FORWARD, motorSpeed);
  } while (initDistance >= 500 || initDistance == 0.0);
  //Serial.println(distance);

  // Set baseline speed
//  double motorSpeed = MAX_SPEED;
  double requiredTicks = (initDistance - DIST_THRESH)*ticksPerCM;
  drive_integral = 0;
  hallLCount = hallRCount = 0;
  while(hallLCount <  requiredTicks)
  {
    //motorSpeed = getMotorSpeed(initDistance*ticksPerCM - ((hallLCount + hallRCount) / 2.0));
    
    motorSpeed = getMotorSpeed(initDistance - (hallLCount  / ticksPerCM));
    //Serial.print(initDistance*ticksPerCM - ((hallLCount + hallRCount) / 2.0));
    //Serial.print(" ");
    //Serial.println(motorSpeed);
    // Serial.print(initDistance - ((hallLCount + hallRCount) / 2.0 / ticksPerCM));
    //Serial.print(" ");
    //  Serial.println(motorSpeed);
    
    drive_P_Straight(FORWARD, motorSpeed);
  }
 
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
  
  // Find a direction to turn to, if scanSurroundings returns 2, scan again
  int turnDir;
  do
  {
    turnDir = scanSurroundings(); 
  } while(turnDir == 2);

  delay(500);

  // Turn left or right based on clearance of each side
  turn_Sensored(turnDir, 150, 12);

  hallLCount = hallRCount = 0;

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
    motorVal = map(distance, DIST_THRESH, MAP_CONST, MIN_MOVEMENT_POWER, MAX_SPEED);
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
  if((distanceLeft > 500 || distanceLeft==0) && (distanceRight > 500 || distanceRight==0))
    return 2;
  // Determine which direction to turn in, 0 = right, 1 = left
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

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
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
  int speed = motorValue;
  int L_LINE = analogRead(LINE_LEFT);
  int M_LINE = analogRead(LINE_MIDDLE);
  int R_LINE = analogRead(LINE_RIGHT);

  if (R_LINE > LINE_THRESHOLD)
  {
    // Set direction
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED,  speed);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
  }
  else if (L_LINE > LINE_THRESHOLD)
  {
    // Set direction
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED, 0);
    analogWrite(RIGHT_DRIVE_SPEED, speed);
  }
  else if (R_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Set direction
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED,  speed - 75);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
  }
  else if (L_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Set direction
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED,  0);
    analogWrite(RIGHT_DRIVE_SPEED, speed - 75);
  }
  else if(M_LINE > LINE_THRESHOLD)
  {
    // Set direction
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed
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

