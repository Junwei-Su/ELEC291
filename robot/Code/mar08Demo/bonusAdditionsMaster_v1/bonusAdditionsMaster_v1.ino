
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
#define IRPIN 11

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

// Bonus
boolean data_ready = false;
const int rowNum = 11;
const int colNum = 11;
const int gridLength = 15;
String serialString = "";
float global_maps[rowNum][colNum];
//a 3 dimensional matrices that represent the surrounding
//0: clear
//1: obstacle
//(0,1) the chance that there is obstacle in that box
//-1: unexplore
//represent a point on the map
typedef struct {
  float x;
  float y;
} point;

#define pi 3.14159265358979323846
#define maxRange 200 //unit cm
point robot_location;

//buffer for data
const int data_number = 91;

//////////////////////////////////////////////////////
//                    VARIABLES                     //
//////////////////////////////////////////////////////

// determines which functionality to run
int mode = 0;

// slave variables
byte prevDirToSlave = -1;

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

  delay(100);
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
    case 3:
      hallLCount = hallRCount = 0;
      attachInterrupt(digitalPinToInterrupt(HALL_L_PIN), updateHallL, CHANGE);
      attachInterrupt(digitalPinToInterrupt(HALL_R_PIN), updateHallR, CHANGE);
      while ( debounceModeButton() == HIGH ) {
        IR();
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
   Prompts the user to select the functionality to run on the robot
*/
void selectMode() {
  sendToSlave(0);   // prompt user to select mode on LCD on slave Arduino
  byte dispCode = 0;
  int button = -1;
  while ( debounceModeButton() == LOW ) {   // mode is confirmed when user toggles mode button
    Wire.requestFrom(8, 1); // reads button presses on slave Arduino's LCD shield
    while (Wire.available())
      button = (int)Wire.read();
    // increment or decrement the display code(looping between 1-2)depending on button presses
    if ( button == 1 ) {
      if ( dispCode >= 3 )
        dispCode = 1;
      else
        dispCode++;
    }
    else if ( button == 0 ) {
      if ( dispCode <= 1 )
        dispCode = 3;
      else
        dispCode--;
    }
    sendToSlave(dispCode);  // shows user what mode will start if mode button is toggled
    delay(250);
    //Serial.print(dispCode);
  }
  mode = (dispCode == 0) ? mode : dispCode;   // resumes previous functionality if user toggles mode button without selecting a mode
  sendToSlave(mode);
}

/*
   Writes a single bytle to the slave Arduino over I2C
*/
void sendToSlave(byte x) {
  Wire.beginTransmission(8);
  Wire.write(x);
  Wire.endTransmission();
}

/*
   Tells the slave Arduino to display the given speed and direction on the LCD
*/
void sendSpeedToSlave(byte speed, byte direction) {
  Wire.beginTransmission(8);
  Wire.write(speed);
  Wire.write(direction);
  Wire.endTransmission();
}

void sendLineFollowToSlave(byte direction) {
  Wire.beginTransmission(8);
  Wire.write(direction);
  Wire.write(0);
  Wire.write(0);
  Wire.endTransmission();
}

/*
   Debounces the mode button (toggle switch)
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
  drive_integral = drive_integral + error * dt;

  double leftSpeed = 0;
  double rightSpeed = 0;

  // Calculate left motor speed based on error
  if (speed + error + Ki * drive_integral > 255)
    leftSpeed = 255;
  else if (speed + error + Ki * drive_integral < MIN_MOVEMENT_POWER)
    leftSpeed = MIN_MOVEMENT_POWER;
  else
    leftSpeed = speed + (error) - Ki * drive_integral;

  // Calculate right motor speed based on error
  if (speed - error - Ki * drive_integral > 255)
    rightSpeed = 255;
  else if (speed - error - Ki * drive_integral < MIN_MOVEMENT_POWER)
    rightSpeed = MIN_MOVEMENT_POWER;
  else
    rightSpeed = speed - (error) - Ki * drive_integral;

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
  analogWrite(LEFT_DRIVE_SPEED, leftSpeed);
  analogWrite(RIGHT_DRIVE_SPEED, rightSpeed);
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
  while (hallLCount < distance) {}

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
   Update hall effect sensor count every time
   the Arduino receives an interrupt from either
   hall effect sensor.
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
   Drive the robot up to an object up to a predetermined threshold.
   Once the robot reaches the distance threshold,
   the robot will scan its left and right side
   and select a direction to turn to.
*/
void driveAndScan()
{
  // Get ultrasonic distance
  double initDistance = 0.0;

  // Set baseline speed
  double motorSpeed = 150;
  sendSpeedToSlave(motorSpeed, FORWARD);

  // Get the distance
  do {
    initDistance = getUltrasonic();
    drive_P_Straight(FORWARD, motorSpeed);
  } while (initDistance >= 500 || initDistance == 0.0);
  //Serial.println(distance);

  // Calculate number for ticks to reach the required distance
  double requiredTicks = (initDistance - DIST_THRESH) * ticksPerCM;

  drive_integral = 0;
  hallLCount = hallRCount = 0;
  unsigned long prevMillis = 0;
  // Run loop until robot has reached the required distance
  while (hallLCount <  requiredTicks)
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
    if ( currMillis - prevMillis >= 500 ) {
      prevMillis = currMillis;
      sendSpeedToSlave(motorSpeed, FORWARD);
    }
  }

  // Stop motors
  stopMotors();
  sendSpeedToSlave(0, 4);

  // Find a direction to turn to, if scanSurroundings returns -1, scan again
  int turnDir;
  do
  {
    turnDir = scanSurroundings();
  } while (turnDir == -1);

  delay(500);

  // Turn left or right based on clearance of each side
  turn_Sensored(turnDir, 125, 11);
  sendSpeedToSlave(0, FORWARD);
  delay(500);
}

/*
   Calculate the motor speed based on
   the measured ultrasonic sensor value and
   the distance an object is away from the robot.
   Param: distance - measured distance from ultrasonic
   Return: motorVal - updated motor speed
*/
int getMotorSpeed(double distance)
{
  // Set motor value to MAX_SPEED
  int motorVal = MAX_SPEED;

  // Check if robot has not approached object
  if (distance > DIST_THRESH)
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
   Check the left and right side of the robot
   using the ultrasonic sensor on a servo mount.
   Determine which direction to turn based on
   the further of the two distances.
   Returns: 0 - to turn the robot left
            1 - to turn the robot right
            2 - scan again
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
  if (distanceLeft > 500 && distanceRight > 500)
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
  return (pulseIn(ECHO_PIN, HIGH) / 2.0) * speedOfSound;
  //  return (ultraSonReading/2.0)*speedOfSound;
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
   Send the robot to trace a line. Each optical sensor's value
   is read and the speed of each wheel is determined based on
   which optical sensors detect the line.
*/
void lineFollow(int motorValue)
{
  int speed = motorValue;
  int L_LINE = analogRead(LINE_LEFT);
  int M_LINE = analogRead(LINE_MIDDLE);
  int R_LINE = analogRead(LINE_RIGHT);
  byte dirToSlave;
  
  if (R_LINE > LINE_THRESHOLD)
  {
    // Swerve right
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED,  speed);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
    dirToSlave = 0;
  }
  else if (L_LINE > LINE_THRESHOLD)
  {
    // Swerve left
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED, 0);
    analogWrite(RIGHT_DRIVE_SPEED, speed);
    dirToSlave = 2;
  }
  else if (R_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Slow turn right
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED,  speed - 75);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
    dirToSlave = 3;
  }
  else if (L_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Slow turn left
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed (Right)
    analogWrite(LEFT_DRIVE_SPEED,  0);
    analogWrite(RIGHT_DRIVE_SPEED, speed - 75);
    dirToSlave = 4;
  }
  else if (M_LINE > LINE_THRESHOLD)
  {
    // Drive foward
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);

    // Set motor speed
    analogWrite(LEFT_DRIVE_SPEED, speed);
    analogWrite(RIGHT_DRIVE_SPEED, speed);
    dirToSlave = 1;
  }
  // Update the LCD data if it differs from previously
  if( dirToSlave != prevDirToSlave ) {
      sendLineFollowToSlave(dirToSlave);
      prevDirToSlave = dirToSlave; 
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

//////////////////////////////////////////////////////
//               ADDITIONAL FUNCTIONALITY            //
//////////////////////////////////////////////////////
/*
   IR Remote code to decode signals from remote control
*/
void IR()
{
  //variables for IR
  boolean binary[32];   //array for Samsung 32 bits
  int data = 0;         //data to determine the button pressed

  //While pulseIn to go from LOW to HIGH
  while (pulseIn(IRPIN, LOW) < 4000);
  for (int i = 0; i < 32; i++)
  {
    //According to Samsung Protocol, the 1-bit will stay as HIGH for about 1000 microseconds
    if (pulseIn(IRPIN, HIGH) > 1000)
    //If arund 1000 microseconds, we know it is a 1-bit so add to binary array.
      binary[i] = 1;
    else
      binary[i] = 0;
  }
  //Decode data from binary into data
  for (int i = 0; i < 8; i++)
  {
    if (binary[i + 16] == 1)
      data += 1 << i;
  }

  //if detect button to drive forward
  // 96 is a key on the remote
  if (data == 96)
  {
    //Drive forward
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);
    analogWrite(LEFT_DRIVE_SPEED, 180);
    analogWrite(RIGHT_DRIVE_SPEED, 200);
  }
  //if detect button to turn left
  // 101 is a key on the remote
  if (data == 101)
  {
    //Turn left
    turn_Timed(LEFT, 255, 300);
  }
  //If detect button to turn right
  // 98 is a key on the remote
  if (data == 98)
  {
    //turn right
    turn_Timed(RIGHT, 255, 300);
  }
  //If detect button to drive reverse
  // 97 is a key on the remote
  if (data == 97)
  {
    //Drive reverse
    digitalWrite(LEFT_DRIVE, LOW);
    digitalWrite(RIGHT_DRIVE, HIGH);
    analogWrite(LEFT_DRIVE_SPEED, 200);
    analogWrite(RIGHT_DRIVE_SPEED, 200);
  }
  //If detect button to stop robot
  //104 is a key on the remote
  if (data == 104) {
    //Stop the motors
    analogWrite(LEFT_DRIVE_SPEED, 0);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
  }
  // Press button to start navigation algorithm
  // 71 is a key on the remote
  if (data == 71) {
    completed_mapping_algorithm();
    data_ready = true;
    sendToSlave(4);
  }
  // Prints data onto Serial Monitor
  // 73 is a key on the remote
  if (data == 73 && data_ready) {
    printData();
    data_ready = false;
  }

  delay(50);

}

//Function to turn robot left or right
void turn_Timed(int direction, int speed, int time)
{
  int turnDirection;
  // Set direction
  if (direction == LEFT) {
    //direction is Left
    turnDirection = 0;
  }
  if (direction == RIGHT) {
    //direction is Right
    turnDirection = 1;
  }
  //Set the wheels according to the direction
  digitalWrite(LEFT_DRIVE, (direction == LEFT) ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, (direction == RIGHT) ? HIGH : LOW);

  // Set motor speed to turn
  analogWrite(LEFT_DRIVE_SPEED, speed);
  analogWrite(RIGHT_DRIVE_SPEED, speed);

  delay(time);

  // Stop motors
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

//////////////////////////////////////////////////////
//                    NAVIGATION ALGORITHM          //
//////////////////////////////////////////////////////
//the comments for the following code is not detailed due to 
//the complexity of the algorithm. For detail comments on the algorithm
//refer to the algorithm appendix in the report(Asked Professor Farshid)


void completed_mapping_algorithm() {
  init_map();//initialize the array and the robot location
  float sonarData[data_number] = {0};
  float angleData[data_number] = {0};

  //  receive_data(sonarData, angleData);
  scan_in_front(sonarData, angleData);
  probability_distribution_model(sonarData, angleData);
}

//initialize the map
void init_map() {
  int row, col;
  robot_location = {0.0, 5.0};
  for (row = 0; row < rowNum; row++) {
    for (col = 0; col < colNum; col++) {
      global_maps[row][col] = -1; //initialized the array to -1 to indice unexplored
    }
  }
}

//for printing data to Serial monitor
void printData() {
  int row, col;
  for (row = 0; row < rowNum; row++) {
    for (col = 0; col < colNum; col++) {
      float toPrint = global_maps[row][col];

      //Serial.println(String(toPrint));
      if (col == 0) {
        serialString = String(toPrint);
      }
      else {
        serialString = serialString + "," + String(global_maps[row][col]);
      }
    }
    Serial.print(serialString + ",0");
    Serial.print("\n");
    delay(1000);
  }
}

//test
void check_radian_degree() {
  float result = sin(pi / 2);
  if (abs(result - 1) < 0.1) {
    Serial.print("the sin is in radia \n");
  }
  else {
    Serial.print("the sin is in degree \n");
  }
}
//given a distance and angle to compute its probability distribution effect to the map
void probability_distribution_model(float sonarData[], float angleData[]) {
  int i;
  float effectRange;
  point left_terminal;
  point right_terminal;
  point center;
  for (i = 0; i < data_number; i++) {
    effectRange = compute_effective_range(sonarData[i]);
    center = compute_centeral_point(sonarData[i], angleData[i]);
    left_terminal = compute_terminal_lpoint(center, angleData[i], effectRange);
    right_terminal = compute_terminal_rpoint(center, angleData[i], effectRange);
    float effective_x_range = compute_effective_range_x(effectRange, angleData[i]);
    float effective_x_increment = effective_x_range / gridLength;
    float effective_y_range = compute_effective_range_y(effectRange, angleData[i]);
    float effective_y_increment = effective_y_range / gridLength;
    distribution_helper_fn(center, left_terminal, right_terminal, angleData[i], effective_x_increment, effective_y_increment);
  }
}

//compute effective range
float compute_effective_range(float distance) {
  return 2 * distance * tan(degree_to_radian(15));
}

//compute effective range x component
float compute_effective_range_x(float range, float angle_a) {
  if (angle_a<90 && angle_a>0) {
    return range * cos(degree_to_radian(90 - angle_a));
  }
  else if (angle_a > 90 && angle_a < 180) {
    return range * cos(degree_to_radian(angle_a - 90));
  }
  else if (angle_a == 90) {
    return range;
  }
  else if (angle_a == 180) {
    return 0;
  }
  else {
    return 0;
  }
}


//compute effective range y component
float compute_effective_range_y(float range, float angle_a) {
  if (angle_a<90 & angle_a>0) {
    return range * sin(degree_to_radian(90 - angle_a));
  }
  else if (angle_a > 90 && angle_a < 180) {
    return range * sin(degree_to_radian(angle_a - 90));
  }
  else if (angle_a == 90) {
    return 0;
  }
  else if (angle_a == 180) {
    return range / 2.0;
  }
  else {
    return range / 2.0;
  }
}

//act as a helper function for probability_distribution_model()
void distribution_helper_fn(point center, point left_t, point right_t, float angle, float x_incre, float y_incre) {
  float x = left_t.x;
  float y = left_t.y;
  matrix_marked_explored(center);
  matrix_marked_explored(left_t);
  matrix_marked_explored(right_t);
  if (angle > 0 && angle < 90) {
    for (; x < floor(right_t.x);) {
      global_maps[(int)floor(y)][(int)floor(x)] += 1 / x_incre;
      x = cap(x + x_incre);
      y = cap(y + y_incre);
      //Serial.print("I am stucking at helper");
    }
  }
  else if (angle >= 90 && angle < 180) {
    for (; x < floor(right_t.x);) {
      global_maps[(int)floor(y)][(int)floor(x)] += 1 / x_incre;
      x = cap(x + x_incre);
      y = cap(y - y_incre);
    }
  }
  else if (angle == 0) {
    for (; y < floor(right_t.y);) {
      global_maps[(int)floor(y)][(int)floor(x)] += 1 / y_incre;
      x = cap(x);
      y = cap(y + y_incre);
    }
  }
  else {
    for (; y > floor(right_t.y);) {
      global_maps[(int)floor(y)][(int)floor(x)] += 1 / y_incre;
      x = cap(x);
      y = cap(y - y_incre);
    }
  }
}

//initilize sub matrix
void matrix_marked_explored(point p1) {
  int lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y;
  lower_bound_y = robot_location.y;
  upper_bound_y = p1.y;
  //determine the lower and upper bound
  if (p1.x >= robot_location.x) {
    lower_bound_x = robot_location.x;
    upper_bound_x = p1.x;
  }
  else {
    lower_bound_x = p1.x;
    upper_bound_x = robot_location.x;
  }

  int x, y;
  for (x = lower_bound_x; x <= upper_bound_x; x++) {
    for (y = lower_bound_y; y <= upper_bound_y; y++) {
      if (global_maps[y][x] == -1) global_maps[y][x] = 0; // marked as explored
    }
  }
}


//receive data from another arduino
void receive_data(float sonarData[], float angleData[]) {
  int i;
  for (i = 0; i < data_number; i++ ) {
    sonarData[i] = random_distance();
    angleData[i] = random_angle();
  }
}

//for testing: random float generator
float random_distance() {
  float range = 200;
  return ((float)rand() / (float)(RAND_MAX)) * range + 50;
}

float random_angle() {
  float range = 180;
  return ((float)rand() / (float)(RAND_MAX)) * range;
}

//compute the left terminal point
point compute_terminal_lpoint(point center, float angle_a, float range) {
  point terminal_point = {0.0, 0.0};
  if (angle_a<90 & angle_a>0) {
    terminal_point.x = cap(center.x - range * cos(degree_to_radian(90 - angle_a)) / gridLength);
    terminal_point.y = cap(center.y - range * sin(degree_to_radian(90 - angle_a)) / gridLength);
  }
  else if (angle_a > 90 && angle_a < 180) {
    terminal_point.x = cap(center.x - range * cos(degree_to_radian(angle_a - 90)) / gridLength);
    terminal_point.y = cap(center.y + range * sin(degree_to_radian(angle_a - 90)) / gridLength);
  }
  else if (angle_a == 90) {
    terminal_point.x = cap(center.x - range / (2 * gridLength));
    terminal_point.y = center.y;
  }
  else if (angle_a == 0) {
    terminal_point.x = center.x;
    terminal_point.y = center.y;
  }
  else {
    terminal_point.x = center.x;
    terminal_point.y = cap(center.y + range / (2 * gridLength));
  }
  return terminal_point;
}

//compute the right terminal point
point compute_terminal_rpoint(point center, float angle_a, float range) {
  point terminal_point = {0.0, 0.0};
  if (angle_a < 90 && angle_a > 0) {
    terminal_point.x = cap(center.x + range * cos(degree_to_radian(90 - angle_a)) / gridLength);
    terminal_point.y = cap(center.y + range * sin(degree_to_radian(90 - angle_a)) / gridLength);
  }
  else if (angle_a > 90 && angle_a < 180) {
    terminal_point.x = cap(center.x + range * cos(degree_to_radian(angle_a - 90)) / gridLength);
    terminal_point.y = cap(center.y - range * sin(degree_to_radian(angle_a - 90)) / gridLength);
  }
  else if (angle_a == 90) {
    terminal_point.x = cap(center.x + range / (2 * gridLength));
    terminal_point.y = center.y;
  }
  else if (angle_a == 180) {
    terminal_point.x = center.x;
    terminal_point.y = center.y;
  }
  else {
    terminal_point.x = center.x;
    terminal_point.y = cap(center.y + range / (2 * gridLength));
  }
  return terminal_point;
}

//tested
//compute the central point
point compute_centeral_point(float angle, float distance) {
  point center = {0.0, 0.0};
  if (angle > 90 && angle < 180) {
    center.x = cap(robot_location.x - distance * cos(degree_to_radian(180 - angle)) / gridLength);
    center.y = cap(robot_location.y + distance * sin(degree_to_radian(180 - angle)) / gridLength);
  }
  else if (angle<90 & angle>0) {
    center.x = cap(robot_location.x + distance * cos(degree_to_radian(angle)) / gridLength);
    center.y = cap(robot_location.y + distance * sin(degree_to_radian(angle)) / gridLength);
  }
  else if (angle == 90) {
    center.x = cap(robot_location.x);
    center.y = cap(robot_location.y + distance / gridLength);
  }
  else if (angle == 180) {
    center.x = cap(robot_location.x + distance / gridLength);
    center.y = cap(robot_location.y);
  }
  else {
    center.x = cap(robot_location.x - distance / gridLength);
    center.y = cap(robot_location.y);
  }
  return center;
}


//cap the point value inside the map
float cap(float x) {
  if (x < 0) return 0;
  if (x > 11) return 11;
  return x;
}

//convert degree to radian
float degree_to_radian(float degree) {
  return degree / 180.0 * pi;
}

//scan the 180 degree in fron of the robot
void scan_in_front(float sonarData[], float angleData[])
{
  // start servo in left pos
  servo.write(scanRight);
  delay(200);
  float previous_data = getUltrasonic_bonus();
  delay(200);
  while (previous_data > maxRange) {
    previous_data = getUltrasonic_bonus();
    delay(200);
  }
  // sweep ultrasonic 180 degrees on servo
  int i;
  float currentReading;
  for (i = 0; i < data_number; i++)
  {
    // get distance
    currentReading = getUltrasonic_bonus();
    delay(200);
    if (currentReading < maxRange && currentReading != 0) {
      sonarData[i] = currentReading;
      //      Serial.println(sonarData[i], 4);
      // set servo to new angle
      angleData[i] = i * 2.0;
      servo.write(i * 2);
      delay(200);
      previous_data = currentReading;
    }
    else {
      sonarData[i] = previous_data;
      //      Serial.println(sonarData[i], 4);
      // set servo to new angle
      angleData[i] = i * 2.0;
      delay(200);
      servo.write(i * 2);
    }
  }
  servo.write(centerServo);
}

// Return distance in centimetres
float getUltrasonic_bonus()
{
  //double temp = analogRead(tempPin) * CONVERT_TO_TEMP;
  float temp = 21; // Standard temp in celsius
  float speedOfSound = (331.5 + (0.6 * temp)) * 0.0001; // cm/microsecond

  // Ping the ultrasonic

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Return pulse from echo
  return (pulseIn(ECHO_PIN, HIGH) / 2.0) * speedOfSound;
}
