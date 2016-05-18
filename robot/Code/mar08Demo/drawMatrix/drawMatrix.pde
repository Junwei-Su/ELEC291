import processing.serial.*;
import java.util.*;

//variable and data for drawing map
float background_color = 0;
float white =255;
float green=150;
float red = 255;
final int data_num= 11;
float [][] map = new float[data_num][data_num];
int index = 0;
Serial myPort;        // The serial port

//flag for drawing
boolean data_read=false;

//varialbe for robot
PImage robot;
int robot_x=250;
int robot_y=100;

PFont font;

void setup()
{
  size(900, 670);
  myPort = new Serial(this, Serial.list()[1], 9600);  
  myPort.bufferUntil('\n');
  smooth();
  font = loadFont("TimesNewRomanPSMT-48.vlw");
  robot = loadImage("robot.png");
  robot.resize(50, 50);
  //testing
  for (int index = 0; index < data_num; index++) {
    for (int index2 = 0; index2 < data_num; index2++) {
      map[index][index2]=-1;
      //println(map[index][index2]);
    }
  }
}

void draw()
{
  //Set background to black
  background(background_color);
  draw_map();
  draw_robot();
  draw_header();
  draw_menu();
  draw_axis();
  //fill(white, white, white);
  //rect(0, 600, 50, 50);
}

//draw the robot in corresponding location
void draw_robot() {
  image(robot, robot_x, robot_y);
}

//draw the obstacle map with the data from the map array
void draw_map() {
  for (int color_y=0; color_y<data_num; color_y++ ) {
    for (int color_x=0; color_x<data_num; color_x++) {
      //case 1: no obstacle or low chance there is obstacle
      if (map[color_y][color_x]<=0.3 && map[color_y][color_x]>=0) {
        noStroke();
        float color_code = white - 100*map[color_y][color_x];
        fill(color_code, color_code, color_code); 
        rect(color_x*50, 600-color_y*50, 50, 50);
      }
      //case 2: not explored
      else if (map[color_y][color_x]==-1) {
        noStroke();
        //orange color code == unexplored
        fill(204, 102, 0); 
        rect(color_x*50, 600-color_y*50, 50, 50);
      }
      //case 3: high chance there is obstacle
      else {
        if(map[color_y][color_x]<0){
          map[color_y][color_x]=1+map[color_y][color_x];
        }
        noStroke();
        float color_code = red *(map[color_y][color_x]+0.5);
        fill(color_code, 0, 0); 
        rect(color_x*50, 600-color_y*50, 50, 50);
      }
    }
  }
}

//draw the header information 
void draw_header(){
  textFont(font);
  fill(#f7ec57);
  text("Obstacle Density Map (cm)", 10, 60);
}

//draw the menu bar
void draw_menu(){
  int x_offset = 580; //<>//
  textFont(font,28);
  fill(#f7ec57);
  text("Color Code:", x_offset+10, 120);
  
  //red color code explain
  fill(red, 0, 0); 
  rect(x_offset+30, 130, 50, 50);
  textFont(font,20);
  fill(white);
  text("Red Square:", x_offset+90, 150);
  text("High Chance Of Obstacle", x_offset+90, 170);
  
  //white color code explain
  fill(white, white, white); 
  rect(x_offset+30, 200, 50, 50);
  textFont(font,20);
  fill(white);
  text("White Square:", x_offset+90,220 );
  text("Low Chance Of Obstacle", x_offset+90, 240);
  
  //orange code explain
  fill(204, 102, 0); 
  rect(x_offset+30, 270, 50, 50);
  textFont(font,20);
  fill(white);
  text("Orange Square:", x_offset+90,290 );
  text("Unknown Area", x_offset+90, 310);
  
  //robot
  image(robot, x_offset+30, 340);
  textFont(font,20);
  fill(white);
  text("Robot Location", x_offset+90,360 );
  
}

void draw_axis(){
    int y_offset=100;
    int x_offset=550;
    
  //x-axis
  fill(#109856);
  textFont(font, 16);
  text("0", 0, y_offset);
  text("15", 50, y_offset);
  text("30",2*50, y_offset);
  text("45",3*50, y_offset);
  text("60", 4*50, y_offset);
  text("75", 5*50, y_offset);
  text("90", 6*50, y_offset);
  text("105", 7*50, y_offset);
  text("120", 8*50, y_offset);
  text("135", 9*50, y_offset);
  text("150", 10*50, y_offset);
  text("165", 11*50, y_offset);
  
  //y-axis
  text("165", x_offset, 650);
  text("150", x_offset, 650-50);
  text("135",x_offset, 650-2*50);
  text("120",x_offset, 650-3*50);
  text("115", x_offset, 650-4*50);
  text("90", x_offset, 650-5*50);
  text("75", x_offset, 650-6*50);
  text("60", x_offset, 650-7*50);
  text("45", x_offset, 650-8*50);
  text("30", x_offset, 650-9*50);
  text("15", x_offset, 650-10*50);
  text("0", x_offset, 650-11*50+10);
}

void serialEvent (Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  String[] sensorData;
  
  if (inString != null) {
    sensorData = split(inString, ',');
      for(int i = 0 ; i < data_num; i++){
        map[index][i] = float(sensorData[i]);
        println(map[index][i]);
      }
      if(index < data_num)
        index++;
      else{
        index = 0;
      }
  }
}