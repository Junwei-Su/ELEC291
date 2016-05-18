import processing.serial.*;
import java.util.*;

//varialbe for color
float background_color = 0;
float white =255;
float green=150;
float red = 255;

//font
PFont font;

//image
PImage onButton;
PImage upButton;
PImage downButton;
PImage leftButton;
PImage rightButton;

//button variables
boolean onButtonOver = false;
boolean upButtonOver = false;
boolean downButtonOver = false;
boolean cursorOn = false;
int buttonWidth = 50, buttonHeight = 50;
int onButtonX= 650, upButtonX=730, downButtonX=810;
int buttonY=550;
int cursorNum=0;
int cursorsY[]=new int[2];

//buffer for voltage and time data
float voltage_buffer[] = new float[400];
long  time_buffer[] = new long[400];

Serial myPort;

//scale or inrecement for x and y axis
float scale_factor = 1;
float x_increment=0.2;

//variable used to compute period
float pre_voltage = 0;
int pre_time = 0;
float current_voltage = 0;
int current_time = 0;
int current_x=0;
int current_y=0;
int pre_x=0;
int pre_y=0;
long period=0;


//debug
int index=0;
float data_inteval=35;


void setup() {
  size(900, 670);
  background(background_color);
  myPort = new Serial(this, Serial.list()[1], 250000);  
  myPort.bufferUntil('\n');
  font = loadFont("BanglaMN-48.vlw");
  onButton = loadImage("onButton.png");
  onButton.resize(50, 50);
  upButton = loadImage("upArrow.png");
  upButton.resize(50, 50);
  downButton = loadImage("downArrow.png");
  downButton.resize(50, 50);
}

void draw() {
  draw_transparent_rect();  //update the area whose information is changing
  draw_display_windows();   //draw the windonws
  draw_addtional_feature(); //draw other things
  updateButtonStatus();      //check if button is pressed
  drawCursor();              //draw the cursor
  draw_data();              //plot the data
}

//draw the transparent rectangle
void draw_transparent_rect() {
  int middle_y =(600-50)/2 +55;
  noStroke();
  fill(0, 50);
  rect(0, 0, 50, 670); 
  rect(600, 0, 300, 670);
}

void draw_display_windows() {
  int top = 50;
  int bot = 600;
  int mid = (600-50)/2 +50;

  //frame
  stroke(255);
  strokeWeight(1);
  line(top, top, bot, top);
  line(top, bot, bot, bot);
  line(top, top, top, bot);
  line(bot, top, bot, bot);
  strokeWeight(3);
  line(top+2, mid, bot-2, mid);

  //grid line
  stroke(255);
  strokeWeight(0);
  //horizontal grid line
  for (int i=1; i<11; i++) {
    line(top, top+i*55, bot, top+i*55);
  }
  //vertical grid line
  for (int i=0; i<11; i++) {
    line(top+i*55, top, top+i*55, bot);
  }

  //label for axis
  //y-axis
  fill(255, 204, 0);
  textFont(font, 16);
  text("Voltage(v)", 0, top-20);
  textFont(font, 12);
  float init_label_y=5;
  for (int i=0; i<11; i++ ) {
    float label_value  = init_label_y*scale_factor-i*scale_factor;
    String label = String.format("%.1f", label_value);
    text(label, top-24, top+i*56);
  }
  //x-axis label
  fill(255, 204, 0);
  textFont(font, 16);
  text("Time", bot+5, mid);
}

//additional function
void draw_addtional_feature() {
  image(onButton, onButtonX, buttonY);
  image(upButton, upButtonX, buttonY);
  image(downButton, downButtonX, buttonY);
}

//check if the the mouse is on the up button
boolean overUpButton() {
  if (mouseX >= upButtonX && mouseX <= upButtonX+buttonWidth && 
    mouseY >= buttonY && mouseY <= buttonY+buttonHeight) {
    return true;
  } else {
    return false;
  }
}

//check if the the mouse is on the down button
boolean overDownButton() {
  if (mouseX >= downButtonX && mouseX <= downButtonX+buttonWidth && 
    mouseY >= buttonY && mouseY <= buttonY+buttonHeight) {
    return true;
  } else {
    return false;
  }
}

//check if the the mouse is on the cursor Button
boolean overOnButton() {
  if (mouseX >= onButtonX && mouseX <= onButtonX+buttonWidth && 
    mouseY >= buttonY && mouseY <= buttonY+buttonHeight) {
    return true;
  } else {
    return false;
  }
}


//check if one of the button is selected, if yes, resize the button to indicate the selection
//effect and set that button status to true
void updateButtonStatus() {
  if (overOnButton()) {
    onButtonOver = true;
    downButtonOver = false;
    upButtonOver = false;
    onButton.resize(60, 60);
    upButton.resize(50, 50);
    downButton.resize(50, 50);
  } else if (overUpButton()) {
    onButtonOver = false;
    downButtonOver = false;
    upButtonOver = true;
    onButton.resize(50, 50);
    upButton.resize(60, 60);
    downButton.resize(50, 50);
  } else if (overDownButton()) {
    onButtonOver = false;
    downButtonOver = true;
    upButtonOver = false;
    onButton.resize(50, 50);
    upButton.resize(50, 50);
    downButton.resize(60, 60);
  } else {
    onButtonOver = upButtonOver = downButtonOver = false;
    onButton.resize(50, 50);
    upButton.resize(50, 50);
    downButton.resize(50, 50);
  }
}

//mouse click event and check wichi button is clicked
void mousePressed() {
  if (onButtonOver) {
    cursorOn();
    fill(0, 250);
    rect(50, 50, 550, 600);
  } else if (upButtonOver) {
    //scale up the y-axis
    if (scale_factor==0.5 ) 
    {
      scale_factor=1;
    } else if (scale_factor==0.1 ) {
      scale_factor=0.5;
    }
  } else if (downButtonOver) {
    //scale down the y-axis
    if (scale_factor==1 ) 
    {
      scale_factor=0.5;
    } else if (scale_factor==0.5 ) {
      scale_factor=0.1;
    }
  } else if (cursorOn && cursorNum<2) {
    //draw cursor line with the mouse coordinate
    cursorsY[cursorNum]=mouseY;
    cursorNum++;
  }
}

//turn on cursor
void cursorOn() {
  cursorOn = true;
  cursorNum = 0;
}

//draw cursor
void drawCursor() {
  if (cursorOn) {
    int i=0;
    for (i=0; i<cursorNum; i++) {
      //draw the cursor line
      strokeWeight(3);
      stroke(#FFFF00);
      line(52, cursorsY[i], 598, cursorsY[i]);
    }
    if (cursorNum==2) {
      //compute the cursor value and the period of the data
      //display the result on the window
      float cursorValue = abs(cursorsY[1]-cursorsY[0])/55.0*scale_factor;
      String stringValue = String.format("%.2f", cursorValue);
      String toPrint = "Cursor Value: " + stringValue;
      fill(white, white, white);
      text(toPrint, 650, 100);
      compute_period();
      String frequencyValue = String.format("%d", period);
      String  stringFreq = "Period: "+ frequencyValue +" ms";
      text(stringFreq, 650, 150);
    }
  }
}

//read data
void serialEvent (Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  String[] data;

  if (inString != null) {
    //parse the data
    data = split(inString, ',');
    current_voltage = float(data[0]);
    current_time = int(data[1]);
    
    //store the data into a buffer for later computation
    if (index<400) {
      voltage_buffer[index]= current_voltage;
      time_buffer[index]= current_time;
    }
    index++;
    //for debug
    println("the index is " + index);
    //index++;
    //println(data[0]);
    //println(data[1]);
  }
}

//draw data on the window
void draw_data() {
  int middle_y =(600-50)/2+50;

  //debug
  //println("current voltage is "+current_voltage);
  
  //draw the plot by connecting the previous data point and the current data point
  current_y = int((middle_y - current_voltage*55/scale_factor));
  pre_y = int((middle_y - pre_voltage*55/scale_factor));
  strokeWeight(2);
  stroke(#b0b031);
  line(50+index*x_increment, pre_y, 50+(index+25)*x_increment, current_y);
  if (50+(index+1)*x_increment>=595) {
    index=0;
    fill(0, 200);
    noStroke();
    rect(50, 0, 550, 670);
  }
  strokeWeight(0);

  pre_voltage = current_voltage;
}

void compute_period() {
  boolean find_init=false;
  boolean find_end=false;
  long start_time = 0;
  long end_time=0;
  if (index>=400) {
    for (int i=0; i<399; i++) {
      //find the first zero in the period
      if (voltage_buffer[i]==0.0 &&
        voltage_buffer[i+1]!=0.0) {
        find_init = true;
        start_time = time_buffer[i];
        if (i<350) i+=10;
      }
      //find the second zero in the period after the first zero is found
      if (find_init && voltage_buffer[i]==0.0) {
        find_end=true;
        end_time = time_buffer[i];
      }

      if (find_init && find_end ) {
        //after finding the first and second zero
        //compute the period
        period = (end_time-start_time)*2/1000;
        return;
      }
    }
  }
}

//keyEvent
void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      //increase the x-scale if up key is pressed
      x_increment+=0.1;
    } else if (keyCode == DOWN) {
      if (x_increment>0.2) {
       //decrease the x-scale if down key is pressed
        x_increment-=0.1;
      }
    }
  }
}