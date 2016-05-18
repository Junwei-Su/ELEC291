import processing.serial.*;


Serial myPort;        // The serial port

int inByte;         // Variable for converting serial port string to float for DHT Humidity 
int tempDHTByte;    // Variable for DHT Temperature
int tempLM35Byte;   // Variable for LM35 Temperature
int photocellByte;  // Variable for Photocell Light Data
//Last height and x position for for DHT Humidity.
int xPos = 1;  // horizontal position of the graph 
int lastxPos=1;
int lastheight=0;
//Last height and x position for tempDHTByte
int xPos2 = 1;
int lastxPos2=1;
int lastheight2=0;
//Last height and x position for tempLM35Byte
int xPos3 = 1;
int lastxPos3=1;
int lastheight3=0;
//Last height and x position for photocellByte
int xPos4 = 1;
int lastxPos4=1;
int lastheight4=0;
PFont font;

void setup () {
  // set the window size:
  size(600,250);
  smooth();
  
  //Loading font for text
  font = loadFont("AppleSymbols-24.vlw");
  textFont(font);
  // List all the available serial ports
  printArray(Serial.list());

  // Select port for computer
  myPort = new Serial(this, Serial.list()[1], 9600);  

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}
void draw () {
    

    // Writing text for Y-axis
    textFont(font,20); 
    float x = 20;
    float y = 150;
    textAlign(CENTER,BOTTOM);

    pushMatrix();
    translate(x,y);
    rotate(-HALF_PI);
    text("DATA",50,5);
    popMatrix();
    
    // Y-Axis Units
    textFont(font,12); // change font to 12
    text("100", 15, height - 100);
    text("50", 15, height - 48);
    text("0",  13, height -4);
    
    //Change font back to size 20 
    textFont(font,24);
    //Writing text for X-axis 
    text("Time (s)", 200, height-4);
    text("Sensor Data vs Time", 300, 50); //Text for title
    fill(127,34,255); //Fill in color for legend
    text("DHT Humidity in RH" , 300, 70);
    fill(255,50,245); //Fill in color for legend
    text("DHT Temperature in Degrees Celsius" , 300, 90);
    fill(127,250,55); //Fill in color for legend
    text("LM35 Temperature in Degrees Celsius" , 300, 110);
    fill(107,20,45); //Fill in color for legend
    text("Photocell in dalx" , 300, 130);
    fill(100,123,32); //Fill in color for legend
    strokeWeight(4);        //stroke wider 
    stroke(200,34,255);     // stroke color
    line(2, 0, 2, height); // Draws vertical line for graph
    line(0, height-2, width, height-2); // Draws horizontal line for graph
    
    // DHT Humidity graph
    stroke(127,34,255);      //stroke color
    line(lastxPos, lastheight, xPos, height - inByte); 
    lastxPos= xPos; // Determines last x position for last line
    lastheight= int(height-inByte); // Determines last height of last line
    
    // DHT Temperature graph
    stroke(255,50,245);      //stroke color
    line(lastxPos2, lastheight2, xPos2, height - tempDHTByte); 
    lastxPos2= xPos2; // Determines last x position for last line
    lastheight2= int(height-tempDHTByte); // Determines last height of last line
    
    //  LM35 Temperature graph
    stroke(127,250,55);      //stroke color
    line(lastxPos3, lastheight3, xPos3, height - tempLM35Byte); 
    lastxPos3= xPos3; // Determines last x position for last line
    lastheight3= int(height-tempLM35Byte); // Determines last height of last line
    
    //Photocell graph
    stroke(107,20,45);      //stroke color
    line(lastxPos4, lastheight4, xPos4, height - photocellByte); 
    lastxPos4= xPos4; // Determines last x position for last line
    lastheight4= int(height-photocellByte); // Determines last height of last line
        
    // at the edge of the window, go back to the beginning:
    if (xPos  >= width || xPos2 >= width || xPos3 >= width || xPos4 >= width ) {
      xPos = 0;
      lastxPos= 0;
      xPos2 = 0;
      lastxPos2= 0;
      xPos3 = 0;
      lastxPos3= 0;
      xPos4 = 0;
      lastxPos4 = 0;
      background(0);  //Clear the screen.
    } 
    else {
      // increment the horizontal position:
      xPos++;
      xPos2++;
      xPos3++;
      xPos4++;
    }   
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  //String inString = myPort.readStringUntil('\n');
  String inString = myPort.readString();
  String[] sensorData;
  //inString = trim(inString);

  if (inString != null) {
    //inString = trim(inString);                // trim off whitespaces.
    sensorData = split(inString, ',');

    inByte = Integer.parseInt(sensorData[0]);             //DHT Humidity
    tempDHTByte = Integer.parseInt(sensorData[1]);        // Variable for DHT Temperature
    tempLM35Byte =  Integer.parseInt(sensorData[2]);   // Variable for LM35 Temperature
    photocellByte = Integer.parseInt(sensorData[3]);   // Variable for Photocell Light Data
    photocellByte = photocellByte / 10;
    println("Humidity: " + str(inByte));    
    println("DHT Temperature: " + str(tempDHTByte));
    println("LM Temperature: " + str(tempLM35Byte));
    println("Photocell: " + str(photocellByte));
  }
}