
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
boolean data_ready =false;
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
#define maxRange 300 //unit cm
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
byte prevSpeedToSlave = -1;

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
      if( dispCode >= 3 )
        dispCode = 1;
       else
        dispCode++;
    }
    else if( button == 0 ) {
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

void sendLineFollowToSlave(byte speed, byte direction) {
  Wire.beginTransmission(8);
  Wire.write(speed);
  Wire.write(direction);
  Wire.write(0);
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
  sendSpeedToSlave(0, 4);
  
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
  int leftSpeed = 0;
  int rightSpeed = 0;
  int speed = motorValue;

  // Obtain sensor data from optical sensors
  int L_LINE = analogRead(LINE_LEFT);
  int M_LINE = analogRead(LINE_MIDDLE);
  int R_LINE = analogRead(LINE_RIGHT);

  // Set direction to forwards
  digitalWrite(LEFT_DRIVE, HIGH);
  digitalWrite(RIGHT_DRIVE, LOW);
  
  byte dirToSlave;
  // If only right sensor is on, hard turn right
  if (R_LINE > LINE_THRESHOLD)
  {
    // Swerve right
    leftSpeed = motorValue;
    rightSpeed = 0;
    dirToSlave = 0;
  }
  // If only left sensor is on, hard turn left
  else if (L_LINE > LINE_THRESHOLD)
  {
    // Swerve left
    leftSpeed = 0;
    rightSpeed = motorValue;
    dirToSlave = 2;
  }
  // If both right and middle sensors are on, minor turn right
  else if (R_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Slow turn right
    leftSpeed = motorValue - 75;
    rightSpeed = 0;
    dirToSlave = 3;
  }
  // If both left and middle sensors are on, minor turn left
  else if (L_LINE > LINE_THRESHOLD && M_LINE > LINE_THRESHOLD)
  {
    // Slow turn left
    leftSpeed = 0;
    rightSpeed = motorValue - 75;
    dirToSlave = 4;
  }
  // If middle sensor is on, drive straight
  else if(M_LINE > LINE_THRESHOLD)
  {
    // Drive forward
    leftSpeed = motorValue;
    rightSpeed = motorValue;
    dirToSlave = 1;
  }
    analogWrite(LEFT_DRIVE_SPEED,  leftSpeed);
    analogWrite(RIGHT_DRIVE_SPEED, rightSpeed);

    byte speedToSlave = (leftSpeed == 0) ? rightSpeed : leftSpeed;

    if ( speedToSlave != prevSpeedToSlave || dirToSlave != prevDirToSlave ) {
      sendLineFollowToSlave(speedToSlave, dirToSlave);
      prevSpeedToSlave = speedToSlave;
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
  int addr = 0;
  boolean bin[32];                    //boolean consumes less space
  int data = 0;

  while (pulseIn(IRPIN, LOW) < 4000);
  for (int i = 0; i < 32; i++)
  {
    if (pulseIn(IRPIN, HIGH) > 1000) //store binary values
      bin[i] = 1;
    else
      bin[i] = 0;
  }

  for (int i = 0; i < 8; i++)     //extract data bits
  {
    if (bin[i + 16] == 1)
      data += 1 << i;
  }

  //Up
  if (data == 96)
  {
    //Serial.println("Up");
    //drive_P_Straight(0, 150);
    digitalWrite(LEFT_DRIVE, HIGH);
    digitalWrite(RIGHT_DRIVE, LOW);
    analogWrite(LEFT_DRIVE_SPEED, 180);
    analogWrite(RIGHT_DRIVE_SPEED, 200);
  }
  //Left
  if (data == 101)
  {
    //Serial.println("Left");
    //turn_Sensored(LEFT, 150, 11);
    turn_Timed(LEFT, 255,300);
  }
  if (data == 98)
  {
   //Serial.println("Right");
   turn_Timed(RIGHT, 255, 300);
  }
  if (data == 97)
  {
    //Serial.println("Down");
    //drive_P_Straight(1, 150);
    digitalWrite(LEFT_DRIVE, LOW);
    digitalWrite(RIGHT_DRIVE, HIGH);
    analogWrite(LEFT_DRIVE_SPEED, 200);
    analogWrite(RIGHT_DRIVE_SPEED, 200);
  }
  if (data == 104) {
    //Serial.println("Stop");
    analogWrite(LEFT_DRIVE_SPEED, 0);
    analogWrite(RIGHT_DRIVE_SPEED, 0);
  }
  if (data == 71) {
    completed_mapping_algorithm();
    data_ready=true;
  }
  if(data ==73&&data_ready){
    print_test();
  }

  delay(50);

}

void IRTurn(int direction, int speed){
  digitalWrite(LEFT_DRIVE, (direction == LEFT) ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, (direction == RIGHT) ? HIGH : LOW);

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, speed);

  delay(500);
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

void turn_Timed(int direction, int speed, int time)
{ 
  int turnDirection;
  // Set direction
  if(direction == LEFT){
    //Left
    turnDirection = 0;
  }
  if(direction == RIGHT){
    //Right
    turnDirection = 1;
  }
  digitalWrite(LEFT_DRIVE, (direction == LEFT) ? LOW : HIGH);
  digitalWrite(RIGHT_DRIVE, (direction == RIGHT) ? HIGH : LOW);

  // Set motor speed
  analogWrite(LEFT_DRIVE_SPEED, speed);
  analogWrite(RIGHT_DRIVE_SPEED, speed);

  delay(time);

  // Stop motors
  analogWrite(LEFT_DRIVE_SPEED, 0);
  analogWrite(RIGHT_DRIVE_SPEED, 0);
}

void completed_mapping_algorithm() {
  //Serial.print("it is printing");                                                            
  init_map();
  float sonarData[data_number] = {0};
  float angleData[data_number] = {0};

//  receive_data(sonarData, angleData);
  scan_in_front(sonarData, angleData);
  probability_distribution_model(sonarData, angleData);
  //Serial.print("random thing");
  //print_test();
}

//initialize the map
void init_map() {
  int row, col;
  robot_location = {5.0, 0.0};
  for (row = 0; row < rowNum; row++) {
    for (col = 0; col < colNum; col++) {
      global_maps[row][col] = -1;
    }
  }
}

//for test
void print_test() {
  int row, col;
  for (row = 0; row < rowNum; row++) {
    for (col = 0; col < colNum; col++) {
      float toPrint = global_maps[row][col];
      /*printf("the float row: %d, col: %d is %f ", row, col, toPrint);
        printf(" end");
        printf("\n");*/

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

//not needed
float calibrateDistance(float travelTime) {
  float soundSpeed = 331 + 0.6 * 25.0; //sound speed in the particular temperature
  float timeInSec = travelTime / 1000000.0;  //travelTime in second
  return timeInSec * soundSpeed / 2.0 * 100; //distance = time * speed divide the value by 2 to find the one trip distance
}

int compute_x_component(float distance, float angle) {
  return distance * cos(degree_to_radian(angle));
}

int compute_y_component(float distance, float angle) {
  return distance * sin(degree_to_radian(angle));
}

//test
void check_radian_degree() {
  float result = sin(pi / 2);
  if (abs(result - 1) < 0.1) {
    printf("the sin is in radia \n");
  }
  else {
    printf("the sin is in degree \n");
  }
}

//given a distance and angle to compute its probability distribution effect to the map
void probability_distribution_model(float sonarData[], float angleData[]) {
  int i;
  float effectRange;
  for (i = 0; i < data_number; i++) {
    effectRange = compute_effective_range(sonarData[i]);
    float effective_x_range = compute_effective_range_x(effectRange, angleData[i]);
    float effective_x_increment = effective_x_range / gridLength;
    float effective_y_range = compute_effective_range_y(effectRange, angleData[i]);
    float effective_y_increment = effective_y_range / effective_x_increment;
    point center = compute_centeral_point(sonarData[i], angleData[i]);
    point left_terminal = compute_terminal_lpoint(center, angleData[i], effectRange);
    point right_terminal = compute_terminal_rpoint(center, angleData[i], effectRange);
    distribution_helper_fn(center, left_terminal, right_terminal, angleData[i], effective_x_increment, effective_y_increment);
  }
}

//act as a helper function for probability_distribution_model()
void distribution_helper_fn(point center, point left_t, point right_t, float angle, float x_incre, float y_incre) {
  float x = left_t.x;
  float y = left_t.y;
  matrix_marked_explored(center, robot_location);
  if (angle > 0 && angle < 90) {
    for (; x < floor(right_t.x);) {
      global_maps[(int)floor(x)][(int)floor(y)] += 1 / x_incre;
      x = cap(x + x_incre);
      y = cap(y + y_incre);
      //Serial.print("I am stucking at helper");
    }
  }
  else {
    for (; x < floor(right_t.x);) {
      global_maps[(int)floor(x)][(int)floor(y)] += 1 / x_incre;
      x = cap(x + x_incre);
      y = cap(y - y_incre);
      //Serial.print("I am stucking at helper");
    }
  }
}

//initilize sub matrix
void matrix_marked_explored(point p1, point p2) {
  int lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y;

  //determine the lower and upper bound
  if (p1.x >= p2.x) {
    lower_bound_x = p2.x;
    upper_bound_x = p1.x;
  }
  else {
    lower_bound_x = p1.x;
    upper_bound_x = p2.x;
  }

  if (p1.y >= p2.y) {
    lower_bound_y = p2.y;
    upper_bound_y = p1.y;
  }
  else {
    lower_bound_y = p1.y;
    upper_bound_y = p2.y;
  }
  int x, y;
  for (x = lower_bound_x; x <= upper_bound_x; x++) {
    for (y = lower_bound_y; y <= upper_bound_y; y++) {
      if (global_maps[x][y] == -1) global_maps[x][y] = 0; // marked as explored
    }
  }
}


//send data to another arduino
void send_data() {
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
  if (angle<90 & angle>0) {
    center.x = cap(robot_location.x - distance * cos(degree_to_radian(90 - angle)) / gridLength);
    center.y = cap(robot_location.y + distance * sin(degree_to_radian(90 - angle)) / gridLength);
  }
  else if (angle > 90 && angle < 180) {
    center.x = cap(robot_location.x + distance * cos(degree_to_radian(180 - angle)) / gridLength);
    center.y = cap(robot_location.y + distance * sin(degree_to_radian(180 - angle)) / gridLength);
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

//compute effective range
float compute_effective_range(float distance) {
  return distance * tan(degree_to_radian(30));
}

//compute effective range x component
float compute_effective_range_x(float range, float angle_a) {
  if (angle_a<90 & angle_a>0) {
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

//cap the point value inside the map
float cap(float x) {
  if (x < 0) return 0;
  if (x > 14) return 14;
  return x;
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
  // sweep ultrasonic 180 degrees on servo
  int i;
  for (i = 0; i < data_number; i++)
  {
    // get distance
    sonarData[i] = (float) getUltrasonic();
    // set servo to new angle
    angleData[i] = i*2.0;
    servo.write(i*2);
    delay(200);
  }
  servo.write(centerServo);
}
