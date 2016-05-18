float global_maps[3][15][15] = {-1}; 
int mapNum=0;
//a 3 dimensional matrices that represent the surrounding
//0: clear
//1: obstacle
//(0,1) the chance that there is obstacle in that box
//-1: unexplore

//represent a point on the map
typedef struct{
  float x;
  float y;
  }point;

//for testing
#define ultraTri 9
#define ultraEcho 10
#define maxRange 300 //unit cm
const int LM35 = A1;
const int temp = 25;
const float effect_sonar_angle=30.0/180*3.14;
point robot_location;


//buffer for data
const int data_number = 500;
float sonarData[data_number]={0};
float angleData[data_number]={0};

void setup() {
  
//for testing
  Serial.begin(9600);
  pinMode(LM35, INPUT);
  pinMode(ultraTri,OUTPUT);

  
}

void loop() {
  robot_location={6.0,0.0};
  // for testing
  //reading the sonar
  int angle;
  digitalWrite(ultraTri, LOW);
  delayMicroseconds(5);
  digitalWrite(ultraTri, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraTri, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ultraEcho, INPUT);
  float ultraSonReading = pulseIn(ultraEcho, HIGH);
  float LM35reading =  analogRead(LM35);
  float distance = calibrateDistance(ultraSonReading); //in cm

  //angle

 
  //need a probability distribution model

  

}

float calibrateDistance(float travelTime){
    float soundSpeed = 331 + 0.6* temp;  //sound speed in the particular temperature
    float timeInSec = travelTime / 1000000.0;  //travelTime in second
    return timeInSec * soundSpeed / 2.0 * 100; //distance = time * speed divide the value by 2 to find the one trip distance
  }

int compute_x_component(float distance, float angle){
  return distance*cos(angle);
  }

int compute_y_component(float distance, float angle){
  return distance*sin(angle);
  }
//doing
//given a distance and angle to compute its probability distribution effect to the map
void probability_distribution_model(float angle, float distance){
  int i;
  int effectRange;
  for(i=0; i<data_number; i++){
    effectRange = compute_effective_range(sonarData[data_number]);
    float effective_x_range = compute_effective_range_x(effectRange,angleData[data_number]);
    float effective_x_increment = effective_x_range/15;
    float effective_y_range = compute_effective_range_y(effectRange,angleData[data_number]);
    float effective_y_increment = effective_y_range/effective_x_increment;
    point center = compute_centeral_point(sonarData[data_number],angleData[data_number]);
    point left_terminal = computer_terminal_lpoint(center, angleData[data_number], effectRange);
    point right_terminal = computer_terminal_rpoint(center, angleData[data_number], effectRange);
    }
  }

void distribution_helper_fn(point center, point left_t, point right_t, float angle, float x_incre, float y_incre){
        float x = left_t.x;
        float y = left_t.y;
        if(angle>0 && angle<90){
          while(x<right_t.x){
            if((global_maps[mapNum][(int)floor(x)][(int)floor(y)]+1)<0.1){
              global_maps[mapNum][(int)floor(x)][(int)floor(y)]=1/x_incre;
            }
            else{
              global_maps[mapNum][(int)floor(x)][(int)floor(y)]+=1/x_incre;
              }
              x+=x_incre;
              y+=y_incre;
            }
            }
         else{
          while(x<right_t.x){
            if((global_maps[mapNum][(int)floor(x)][(int)floor(y)]+1)<0.1){
              global_maps[mapNum][(int)floor(x)][(int)floor(y)]=1/x_incre;
            }
            else{
              global_maps[mapNum][(int)floor(x)][(int)floor(y)]+=1/x_incre;
              }
              x+=x_incre;
              y-=y_incre;
          } 
  }
}

//given a map, find the optimal way to navigate
void navigate(){
  //todo
  }

//send data to another arduino
void send_data(){
  }

//receive data from another arduino
void receive_data(){
  int i;
  for(i=0; i<data_number; i++ ){
    sonarData[i] = random_distance();
    angleData[i] = random_angle();
    }
  }

//for testing: random float generator
float random_distance(){
  int range = 200;
  return ((float)rand()/(float)(RAND_MAX)) * range;
  }

float random_angle(){
  int range = 2*3.14;
  return ((float)rand()/(float)(RAND_MAX)) * range;
  }

//compute the left terminal point
point computer_terminal_lpoint(point center,float angle_a, float range){
    point terminal_point={0.0,0.0};
    if(angle_a<90&angle_a>0){
        terminal_point.x = center.x - range*cos(90-angle_a)/15.0;
        terminal_point.y = center.y - range*sin(90-angle_a)/15.0;
            }
    else if(angle_a>90&&angle_a<180){
        terminal_point.x = center.x - range*cos(angle_a-90)/15.0;
        terminal_point.y = center.y + range*sin(angle_a-90)/15.0;
    }
    else if(angle_a==90){
        terminal_point.x = center.x - range/30.0;
        terminal_point.y = center.y;
    }
    else if(angle_a==0){
        terminal_point.x = center.x;
        terminal_point.y = center.y;
    }
    else{
        terminal_point.x = center.x;
        terminal_point.y = center.y+range/30.0;
    }
    return terminal_point;
}

//compute the right terminal point
point computer_terminal_rpoint(point center,float angle_a, float range){
    point terminal_point={0.0,0.0};
    if(angle_a<90&&angle_a>0){
        terminal_point.x = center.x + range*cos(90-angle_a)/15.0;
        terminal_point.y = center.y + range*sin(90-angle_a)/15.0;
    }
    else if(angle_a>90&&angle_a<180){
        terminal_point.x = center.x + range*cos(angle_a-90)/15.0;
        terminal_point.y = center.y - range*sin(angle_a-90)/15.0;
    }
    else if(angle_a==90){
        terminal_point.x = center.x + range/30.0;
        terminal_point.y = center.y;
    }
    else if(angle_a==180){
        terminal_point.x = center.x;
        terminal_point.y = center.y;
    }
    else{
        terminal_point.x = center.x;
        terminal_point.y = center.y+range/30.0;
    }
    return terminal_point;
}

//compute the central point
point compute_centeral_point(float angle, float distance){
  point center={0.0,0.0};
  if(angle<90&angle>0){
        center.x = robot_location.x - distance * cos(90-angle)/15.0;
        center.y = robot_location.y + distance * sin(90-angle)/15.0;
    }
    else if(angle>90&&angle<180){
        center.x = robot_location.x + distance * cos(angle-90)/15.0;
        center.y = robot_location.y + distance * sin(angle-90)/15.0;
    }
    else if(angle==90){
        center.x = robot_location.x;
        center.y = robot_location.y + distance/15.0;
    }
    else if(angle==180){
        center.x = robot_location.x + distance/15.0;
        center.y = robot_location.y;
    }
    else{
        center.x = robot_location.x - distance/15.0;
        center.y = robot_location.y;
    }
    return center;
  }
  
//compute effective range
float compute_effective_range(float distance){
  //effective angle of the sonar is 30
  return distance*tan(30);
  }

//compute effective range x component
float compute_effective_range_x(float range, float angle_a){
  if(angle_a<90&angle_a>0){
        return range * cos(90-angle_a);
    }
    else if(angle_a>90&&angle_a<180){
        return range * cos(angle_a-90);
    }
    else if(angle_a==90){
        return range;
    }
    else if(angle_a==180){
        return 0;
    }
    else{
        return 0;
    }
  }

 //compute effective range y component
float compute_effective_range_y(float range, float angle_a){
  if(angle_a<90&angle_a>0){
        return range * sin(90-angle_a);
    }
    else if(angle_a>90&&angle_a<180){
        return range * sin(angle_a-90);
    }
    else if(angle_a==90){
        return 0;
    }
    else if(angle_a==180){
        return range/2.0;
    }
    else{
        return range/2.0;
    }
  }
