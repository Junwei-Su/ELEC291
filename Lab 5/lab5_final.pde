import processing.serial.*;
import twitter4j.conf.*;
import twitter4j.*;
import twitter4j.auth.*;
import twitter4j.api.*;
import java.util.*;

// button images
PImage leftButton;
PImage rightButton;
PImage twitterButton;

Twitter twitter;

// button positions
int leftButtonX=950, leftButtonY=800; // Position of left button
int rightButtonX=1100, rightButtonY=800;
int twitterButtonX=1250, twitterButtonY=800;

// button variables for indicating if the button selected
boolean twitterButtonOver = false;
boolean leftButtonOver = false;
boolean rightButtonOver = false;
int buttonWidth = 80, buttonHeight = 80;

// radar variables
final float scanSpeed = PI/70;  // speed of scanning, lower => faster

Serial myPort;
PFont font;

float scanAngle = 0;   // current angle of the scanning line
int currentWedge = 3;  // abstraction for programmer to decide which angle ultrasonic is facing
float wedge5Mem = 0;   // 2 o'clock edge case

// real time data
float realTimeItemDistance = 0;
float realTimeTemperature = 0;

// displayed data
float itemDistance = 0;  // used to calculate position of red dot
float dispDist;  // used to display the position of object as red dot is shown

void setup() {
  size(1400, 1000);
  smooth();
  font = loadFont("TimesNewRomanPSMT-12.vlw");
  //Declaring Serial port to run
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.bufferUntil('\n');
  
  background(0);
  //Load images to buttons
  leftButton = loadImage("leftArrow.png");
  leftButton.resize(100, 80);
  rightButton = loadImage("rightArrow.png");
  rightButton.resize(100, 80);
  twitterButton = loadImage("twitter.png");
  twitterButton.resize(100, 80);
  size(1400, 1000);
  
  //Input Tokens for Twitter API
  ConfigurationBuilder cb = new ConfigurationBuilder();
  cb.setOAuthConsumerKey("/*ADD CONSUMER KEY*/");
  cb.setOAuthConsumerSecret("/*ADD CONSUMER SECRET*/");
  cb.setOAuthAccessToken("/*ADD ACCESS TOKEN*/");
  cb.setOAuthAccessTokenSecret("/*ADD ACCESS TOKEN SECRET*/");
  
  // Create a twitter object to use to send tweet
  TwitterFactory tf = new TwitterFactory(cb.build());
  twitter = tf.getInstance();
}

void draw() {
  noStroke();
  fill(0, 30);
  rect(0, 0, width, height);    // semi-transparent rectangle drawn over window to fade anything that isn't redrawn
  
  drawRadar();
  drawTextInfo();
  drawScanLineAndDot();
  
  updateButtonStatus();
  image(leftButton,leftButtonX, leftButtonY);
  image(twitterButton,twitterButtonX, twitterButtonY);
  image(rightButton,rightButtonX, rightButtonY);
}

// draws the radar bounded by 1000x1000 area
void drawRadar() {
  
  // all the rings
  strokeWeight(1);
  noFill();
  stroke(#109856);
  ellipse(500, 500, 1000, 1000);
  ellipse(500, 500, 750, 750);
  ellipse(500, 500, 500, 500);
  ellipse(500, 500, 250, 250);
  
  // all the lines center at (500, 500) to the edge of the radar
  strokeWeight(2);
  line(500, 500, 500+500*cos(0), 500+500*sin(0));
  line(500, 500, 500+500*cos(PI/6), 500+500*sin(PI/6));
  line(500, 500, 500+500*cos(PI/3), 500+500*sin(PI/3));
  line(500, 500, 500+500*cos(PI/2), 500+500*sin(PI/2));
  line(500, 500, 500+500*cos(2*PI/3), 500+500*sin(2*PI/3));
  line(500, 500, 500+500*cos(5*PI/6), 500+500*sin(5*PI/6));
  line(500, 500, 500+500*cos(PI/3), 500+500*sin(PI/3));
  
  line(500, 500, 500+500*cos(PI), 500+500*sin(PI));
  line(500, 500, 500+500*cos(7*PI/6), 500+500*sin(7*PI/6));
  line(500, 500, 500+500*cos(4*PI/3), 500+500*sin(4*PI/3));
  line(500, 500, 500+500*cos(3*PI/2), 500+500*sin(3*PI/2));
  line(500, 500, 500+500*cos(5*PI/3), 500+500*sin(5*PI/3));
  line(500, 500, 500+500*cos(11*PI/6), 500+500*sin(11*PI/6));
  
  // thicken the line for the currently selected angle
  strokeWeight(8);
  switch( currentWedge ) {
    case 0: line(500, 500, 500+500*cos(PI), 500+500*sin(PI)); break;
    case 1: line(500, 500, 500+500*cos(7*PI/6), 500+500*sin(7*PI/6)); break;
    case 2: line(500, 500, 500+500*cos(4*PI/3), 500+500*sin(4*PI/3)); break;
    case 3: line(500, 500, 500+500*cos(3*PI/2), 500+500*sin(3*PI/2)); break;
    case 4: line(500, 500, 500+500*cos(5*PI/3), 500+500*sin(5*PI/3)); break;
    case 5: line(500, 500, 500+500*cos(11*PI/6), 500+500*sin(11*PI/6)); break;
    case 6: line(500, 500, 500+500*cos(0), 500+500*sin(0)); break;
  }
  
  // display axis in terms of cm
  textFont(font);
  fill(#109856);
  text("50", 505, 390);
  text("100", 505, 265);
  text("150", 505, 140);
  text("200", 505, 15);
  
  text("50", 505, 620);
  text("100", 505, 745);
  text("150", 505, 870);
  text("200", 505, 995);
  
  text("50", 380, 515);
  text("100", 255, 515);
  text("150", 130, 515);
  text("200", 5, 515);
  
  text("50", 610, 515);
  text("100", 730, 515);
  text("150", 855, 515);
  text("200", 980, 515);
  noFill();
}

// draws the scanning line and displays a red dot(if sensor data is valid) at the correct angle and distance on the radar
void drawScanLineAndDot() {
  strokeWeight(10);
  stroke(#109856);
  line(500, 500, 500+500*cos(scanAngle), 500+500*sin(scanAngle));    // draw the new position of the scanning line
  
  //float dist = random(0, 200);  // RNG used for testing
  
  if( itemDistance != 0 ) {    // ignore data distances that are 0, which means out of range according to Arduino code
    
    // determine the currently selected angle and draw a red dot at the distance calculated
    //   until the scanning line moves PI/3 radians past the selected angle,
    //   at which point the dot will start to fade
    // also update the displayed distance for drawTextInfo()
    
    if( currentWedge == 0 && scanAngle > PI && scanAngle < 4*PI/3 ) {
      stroke(#ff0000);
      ellipse(500+(itemDistance*2.5)*cos(PI), 500+(itemDistance*2.5)*sin(PI), 12, 12);
      dispDist = itemDistance;
    }
    
    else if( currentWedge == 1 && scanAngle > 7*PI/6 && scanAngle < 3*PI/2 ) {
      stroke(#ff0000);
      ellipse(500+(itemDistance*2.5)*cos(7*PI/6), 500+(itemDistance*2.5)*sin(7*PI/6), 12, 12);
      dispDist = itemDistance;
    }
    
    else if( currentWedge == 2 && scanAngle > 4*PI/3 && scanAngle < 5*PI/3 ) {
      stroke(#ff0000);
      ellipse(500+(itemDistance*2.5)*cos(4*PI/3), 500+(itemDistance*2.5)*sin(4*PI/3), 12, 12);
      dispDist = itemDistance;
    }
    
    else if( currentWedge == 3 && scanAngle > 3*PI/2 && scanAngle < 11*PI/6 ) {
      stroke(#ff0000);
      ellipse(500-(itemDistance*2.5)*cos(3*PI/2), 500+(itemDistance*2.5)*sin(3*PI/2), 12, 12);
      dispDist = itemDistance;
    }
    
    else if( currentWedge == 4 && scanAngle > 5*PI/3 && scanAngle < 2*PI ) {
      stroke(#ff0000);
      ellipse(500+(itemDistance*2.5)*cos(5*PI/3), 500+(itemDistance*2.5)*sin(5*PI/3), 12, 12);
      dispDist = itemDistance;
    }
    
    else if( currentWedge == 5 && scanAngle > 11*PI/6 && scanAngle < 2*PI ) {
      wedge5Mem = itemDistance;
      stroke(#ff0000);
      ellipse(500+(itemDistance*2.5)*cos(11*PI/6), 500+(itemDistance*2.5)*sin(11*PI/6), 12, 12);
      dispDist = itemDistance;
    }
    
    else if( wedge5Mem != 0 && currentWedge == 5 && scanAngle > 0 && scanAngle < PI/6 ) {
      stroke(#ff0000);
      ellipse(500+(wedge5Mem*2.5)*cos(11*PI/6), 500+(wedge5Mem*2.5)*sin(11*PI/6), 12, 12);
    }
    
    else if( currentWedge == 6 && scanAngle > 0 && scanAngle < PI/3 ) {
      stroke(#ff0000);
      ellipse(500+(itemDistance*2.5)*cos(0), 500+(itemDistance*2.5)*sin(0), 12, 12);
      dispDist = itemDistance;
    }
}
  
  // update the red dot distance every time the scan line moves 2*PI radians
  // also reset the scanAngle back to 0 rads
  if ( scanAngle > 2*PI ) {
    scanAngle = 0;
    itemDistance = realTimeItemDistance;
    //itemDistance = dist;    // for RNG testing
  }
  else {      // otherwise just increment the scanAngle for the next scanning line to be drawn
    scanAngle += scanSpeed; 
  }
}

// draws all the text that is displayed in the area x > 1000 and y < 500
// text includes name of project, range, temperature, and direction data
void drawTextInfo() {
  // title
  fill(#109856);
  textFont(font, 48);
  text("LAB 5 RADAR", 1010, 100);
  
  // labels & units
  textFont(font, 28);
  text("dist: ", 1010, 150);
  text("dir: ", 1010, 200);
  text("temp: ", 1010, 250);
  text(realTimeTemperature, 1100, 250);
  text("°C", 1275, 250);
  text(getDirection(), 1100, 200);
  
  if( dispDist == 0 ) {    // data of 0 means out of range as decided by Arduino code
    text("OUT OF RANGE!", 1100, 150);
  }
  else {
    text(str(dispDist), 1100, 150 );
    text("cm", 1275, 150);
  } 
  noFill();
}

// returns a String to indicate the current direction of scanning
String getDirection() {
  switch( currentWedge ) {
    case 0: return "900 o'clock";
    case 1: return "1000 o'clock";
    case 2: return "1100 o'clock";
    case 3: return "1200 o'clock";
    case 4: return "100 o'clock";
    case 5: return "200 o'clock";
    case 6: return "300 o'clock";
    default: return null;
  }
}

// parse and update the distance and temperature data from the sensors
void serialEvent(Serial myPort) {
  String inString = myPort.readString();
  String[] sensorData;
  
  if ( inString != null ) {
    sensorData = split(inString, ',');
    
    realTimeItemDistance = Float.parseFloat(sensorData[0]);
    realTimeTemperature = Float.parseFloat(sensorData[1]);
  } 
}

// sends a tweet displaying the current direction and distance away from an object
//    if not object, tweets that there is nothing in range
//    throws TwitterException if a problem is encountered while tweeting
void tweet()
{
    try
    {
      //Declare a Status object in order to update Twitter status
      Status status;
      if (dispDist == 0) {
        //Status on Twitter will be "No Object in range!" if not in our defined range
        status = twitter.updateStatus("No object in range!");
      }
      else {
        //Otherwise display Object's current location
        status = twitter.updateStatus("Object " +dispDist + " cm away at " +getDirection() + "!");
      }
        System.out.println("Status updated to [" + status.getText() + "].");  
    }
    catch (TwitterException te)
    {
        System.out.println("Error: "+ te.getMessage());
    }
}


//check if one of the button is selected, if yes, resize the button to indicate the selection
//effect and set that button status to true
void updateButtonStatus() {
  if (overTwitterButton()) {
    twitterButtonOver = true;
    leftButtonOver = false;
    rightButtonOver = false;
    twitterButton.resize(110, 90);
    leftButton.resize(100, 80);
    rightButton.resize(100, 80);
  }else if(overLeftButton()){
    twitterButtonOver = false;
    leftButtonOver = true;
    rightButtonOver = false;
    leftButton.resize(110, 90);
    rightButton.resize(100, 80);
    twitterButton.resize(100, 80);
  }else if(overRightButton()){
    twitterButtonOver = false;
    leftButtonOver = false;
    rightButtonOver = true;
    rightButton.resize(110, 90);
    leftButton.resize(100, 80);
    twitterButton.resize(100, 80);
  }
  else {
    leftButtonOver = rightButtonOver = twitterButtonOver = false;
    leftButton.resize(100, 80);
    twitterButton.resize(100, 80);
    rightButton.resize(100, 80);
  }
}

//mouse click event and check wichi button is clicked
void mousePressed() {
  if (twitterButtonOver) {
    tweet();
  }
  else if(leftButtonOver){
    if(currentWedge-1>=0){
      --currentWedge; //move the scan edge to left
    }
  }
  else if(rightButtonOver){
    if(currentWedge+1<=6){
      ++currentWedge; //move the scan edge to right
    }
  }
}

//check if the the mouse is on the left button
boolean overLeftButton()  {
  if (mouseX >= leftButtonX && mouseX <= leftButtonX+buttonWidth && 
      mouseY >= leftButtonY && mouseY <= leftButtonY+buttonHeight) {
    return true;
  } else {
    return false;
  }
}

//check if the the mouse is on the right button
boolean overRightButton()  {
  if (mouseX >= rightButtonX && mouseX <= rightButtonX+buttonWidth && 
      mouseY >= rightButtonY && mouseY <= rightButtonY+buttonHeight) {
    return true;
  } else {
    return false;
  }
}

//check if the the mouse is on the twitter button
boolean overTwitterButton()  {
  if (mouseX >= twitterButtonX && mouseX <= twitterButtonX+buttonWidth && 
      mouseY >= twitterButtonY && mouseY <= twitterButtonY+buttonHeight) {
    return true;
  } else {
    return false;
  }
}