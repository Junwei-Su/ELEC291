#include <Servo.h>
#include <IRremote.h>

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

#define SERVO_PIN 11  // Servo requires PWM pin
Servo servo;

#define RECV_PIN 10
IRrecv irrecv(RECV_PIN);
String in;
decode_results results;

#define TRIG_PIN 8
#define ECHO_PIN A3
#define TEMP_PIN A3

//////////////////////////////////////////////////////
//                    CONSTANTS                     // 
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
#define DIST_THRESH 7.0
#define MAP_CONST 60
#define MIN_MOVEMENT_POWER 70
#define MAX_SPEED 255

#define LINE_THRESHOLD 100

void setup()
{
  pinMode(LEFT_DRIVE, OUTPUT);
  pinMode(RIGHT_DRIVE, OUTPUT);
  pinMode(LEFT_DRIVE_SPEED, OUTPUT);
  pinMode(RIGHT_DRIVE_SPEED, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo.attach(SERVO_PIN);
  irrecv.enableIRIn();
  Serial.begin(9600);
  delay(100);
}

void loop() 
{
  //driveAndScan();
  //lineFollow();
  //lineTest();
  detectIR();
}

//////////////////////////////////////////////////////
//                  DRIVE FUNCTIONS                 // 
//////////////////////////////////////////////////////
void loopDriveStraight()
{
  updateEncoders();
  drive_P_Straight(FORWARD, 200);
}

// Call to update tick value, must be called 
// as soon as possible in order to not miss ticks
int L_Encoder = 0;
bool left_lastState = 0;
int R_Encoder = 0;
bool right_lastState = 0;

void updateEncoders()
{
  if (digitalRead(HALL_LEFT) != left_lastState)
  {
    L_Encoder++;
    left_lastState = !left_lastState;
  }

  if (digitalRead(HALL_RIGHT) != right_lastState)
  {
    R_Encoder++;
    right_lastState = !right_lastState;
  }
}

// Uses P to lower/increase motor speed to match the rotation speed to drive straight
// Needs to be called in a while loop, does not stop the motors
void drive_P_Straight(int direction, int speed)
{
  digitalWrite(LEFT_DRIVE, direction ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, direction ? HIGH : LOW);

  // Calulate error
  int Kp = 1.5;
  int error = R_Encoder - L_Encoder * Kp;

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, speed + error);
  analogWrite(RIGHT_DRIVE_SPEED, speed - error);
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
  double distance = getUltrasonic();

  // Set baseline speed
  double motorSpeed = MAX_SPEED;

  // Loop until robot reaches distance threshold
  while(distance > DIST_THRESH)
  {
    // Update distance 
    distance = getUltrasonic();

    // Update motor value only if ultrasonic returns valid value
    if(distance <= 500)
      motorSpeed = getMotorSpeed(distance);

    // Set to drive forward
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed
    analogWrite(LEFT_DRIVE_SPEED, motorSpeed);
    analogWrite(RIGHT_DRIVE_SPEED, motorSpeed);
  }

  // Find a direction to turn to, if scanSurroundings returns 2, scan again
  int turnDir;
  do
  {
    turnDir = scanSurroundings(); 
  } while(turnDir == 2);

  delay(1000);

  // Turn left or right based on clearance of each side
  turn_Timed(turnDir ? LEFT : RIGHT, 150, 600);

  delay(2000);  
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
  if(distanceLeft > 500 && distanceRight > 500)
    return 2;
  // Determine which direction to turn in, 0 = right, 1 = left
  return distanceLeft > distanceRight;
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

//////////////////////////////////////////////////////
//               PRINCIPLE FUNCTION 2               // 
//////////////////////////////////////////////////////
/*
 * Send the robot to trace a line. Each optical sensor's value
 * is read and the speed of each wheel is determined based on
 * which optical sensors detect the line.
 */
void lineFollow()
{
  int  speed = 255;
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

void detectIR(){
  if (irrecv.decode(&results)) {
    in = String(int(results.value), HEX);
    irrecv.resume(); // Receive the next value

    if(in == "a05f"){
      Serial.println("2");
      loopDriveStraight();
      }
    if(in == "906f"){
      Serial.println("5");
      }
    if(in == "10ef"){
      Serial.println("4");
      //turn_Timed(turnDir ? LEFT : RIGHT, 150, 600);
      }
    if(in == "50af"){
      Serial.println("6");
      //turn_Timed(turnDir ? LEFT : RIGHT, 150, 600);
      }
  }
 // make it break out of loop if not sensing data
  delay(100);
}
