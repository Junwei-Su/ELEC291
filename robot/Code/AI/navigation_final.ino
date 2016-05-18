#include <Servo.h>

const int rowNum = 11;
const int colNum = 11;
const int gridLength = 15;
float global_maps[rowNum][colNum];
Servo servo;
const int servo_pin = 10;
String serialString = "";
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


//for testing
#define pi 3.14159265358979323846
#define maxRange 300 //unit cm
point robot_location;

//buffer for data
const int data_number = 91;

void setup() {

  Serial.begin(9600);
  servo.attach(servo_pin);
  completed_mapping_algorithm();

}

void loop() {
  //Serial.print("nothing wrong with serial");
  //completed_mapping_algorithm();

  //Serial.print("test");
}

void completed_mapping_algorithm() {
  //Serial.print("it is printing");                                                            
  init_map();
  float sonarData[data_number] = {0};
  float angleData[data_number] = {0};

  receive_data(sonarData, angleData);
  probability_distribution_model(sonarData, angleData);
  //Serial.print("random thing");
  print_test();
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
