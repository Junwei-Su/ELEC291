
#include <LiquidCrystal.h>
#include <Wire.h>

#define MODEBTNUP_PIN 2
#define MODEBTNDOWN_PIN 3

int prevLCDCode = -1;
int prevLCDSpeed = 0;

// debouncing variables
int modeBtnUpState;
int prevModeBtnUpState = LOW;
unsigned long lastModeBtnUpDebounceTime = 0;
int modeBtnDownState;
int prevModeBtnDownState = LOW;
unsigned long lastModeBtnDownDebounceTime = 0;
long debounceDelay = 50;

byte upArrow[8] = {
  0b00100,
  0b01110,
  0b11111,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00000
};

byte downArrow[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

byte leftArrow[8] = {
  0b00000,
  0b00100,
  0b01100,
  0b11111,
  0b01100,
  0b00100,
  0b00000,
  0b00000
};

byte rightArrow[8] = {
  0b00000,
  0b00100,
  0b00110,
  0b11111,
  0b00110,
  0b00100,
  0b00000,
  0b00000
};

byte fullBox[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b00000
};

byte emptyBox[8] = {
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b00000
};

byte eye[8] = {
  0b00000,
  0b00000,
  0b01110,
  0b10101,
  0b01110,
  0b00000,
  0b00000,
  0b00000
};

LiquidCrystal lcd(A3, A2, 11, 10, 9, 8);

void setup() {
  lcd.begin(16, 2);

  lcd.clear();
  lcd.print("Setting up");

  Wire.begin(8);  // begin coms with master Arduino, this Arduino is on address 8

  Serial.begin(9600);
  lcd.createChar(0, upArrow);
  lcd.createChar(1, downArrow);
  lcd.createChar(2, leftArrow);
  lcd.createChar(3, rightArrow);
  lcd.createChar(4, fullBox);
  lcd.createChar(5, emptyBox);
  lcd.createChar(6, eye);
  
  lcd.clear();
  lcd.print("Setup complete");
  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() { 
  debounceAllButtons();  
}

void receiveEvent(int howMany) {
  Serial.println("recieve triggered");
  if ( Wire.available() == 1 ) {    // one byte means that user is currently selecting a mode
    int x = Wire.read();
    if ( x != prevLCDCode ) {       // reduces flickering text on LCD if user does not press a button
      prevLCDCode = x;
      lcd.clear();
      switch (x) {
        case 0:
          lcd.print("Select mode");
          lcd.setCursor(0, 1);
          lcd.print("using ");
          lcd.write(byte(2));
          lcd.print(" and ");
          lcd.write(byte(3));
          break;
        case 1:
          lcd.print("Principle 1:");
          lcd.setCursor(0, 1);
          lcd.print("Drive and Scan");
          break;
        case 2:
          lcd.print("Principle 2:");
          lcd.setCursor(0, 1);
          lcd.print("Line Following");
          break;
        case 3:
          lcd.print("BONUS: ");
          lcd.setCursor(0, 1);
          lcd.print("IR Navigation");
          break;
        default:
          lcd.print("Select mode");
          lcd.setCursor(0, 1);
          lcd.print("using ");
          lcd.write(byte(2));
          lcd.print(" and ");
          lcd.write(byte(3));
          break;
      }
    }
  }
  else if ( Wire.available() == 2 ) {     // two bytes means speed and direction of robot are to be displayed on LCD
    int speed = Wire.read();    // first byte is speed
    byte directionCode = Wire.read();    // second byte to be decoded into a direction

    byte increasing = (speed > prevLCDSpeed) ? 0 : 1;   // display an arrow for increasing or decreasing speed
    prevLCDSpeed = speed;

    String dir;     // decode the directionCode byte into a String representing the robot's current direction
    byte arrow;
    switch ( directionCode ) {
      case 0:
        dir = "Forward";
        arrow = 1;
        break;
      case 1:
        dir = "Backward";
        arrow = 0;
        break;
      case 2:
        dir = "Left";
        arrow = 3;
        break;
      case 3:
        dir = "Right";
        arrow = 2;
        break;
      case 4:
        dir = "Scanning...";
        arrow = 6;
    }
    // display data on LCD
    lcd.clear();
    lcd.print("Speed: ");
    lcd.print(speed);
    lcd.setCursor(15, 0);
    lcd.write(increasing);
    lcd.setCursor(0, 1);
    lcd.print(dir);
    lcd.setCursor(15, 1);
    lcd.write(arrow);
  }
  else if ( Wire.available() == 3 ) {
    int speed = Wire.read();
    byte boxCode = Wire.read();

    lcd.clear();
    lcd.print("Speed: ");
    lcd.print(speed);
    lcd.setCursor(0, 1);

    byte empty = 5;
    byte full = 4;
    switch(boxCode) {
      case 0:
        lcd.write(full);
        lcd.setCursor(7, 1);
        lcd.write(empty);
        lcd.setCursor(15, 1);
        lcd.write(empty);
        break;
      case 1:
        lcd.write(empty);
        lcd.setCursor(7, 1);
        lcd.write(full);
        lcd.setCursor(15, 1);
        lcd.write(empty);
        break;
    case 2:
        lcd.write(empty);
        lcd.setCursor(7, 1);
        lcd.write(empty);
        lcd.setCursor(15, 1);
        lcd.write(full);
        break;
      case 3:
        lcd.write(full);
        lcd.setCursor(7, 1);
        lcd.write(full);
        lcd.setCursor(15, 1);
        lcd.write(empty);
        break;
    case 4:
        lcd.write(empty);
        lcd.setCursor(7, 1);
        lcd.write(full);
        lcd.setCursor(15, 1);
        lcd.write(full);
        break;
    default:
        lcd.write(empty);
        lcd.setCursor(7, 1);
        lcd.write(empty);
        lcd.setCursor(15, 1);
        lcd.write(empty);
        break;
    }
  }
  
}
/*
   Currently only used for reading the LCD shield buttons
   Will be used later for bonus functionalities
*/
void requestEvent() {
  Serial.println("request triggered");
  if ( debounceModeUpButton() )
    Wire.write(0);
  else if ( debounceModeDownButton() )
    Wire.write(1);
  else
    Wire.write(-1);
}

void debounceAllButtons() {
  debounceModeUpButton();
  debounceModeDownButton();
}

int debounceModeUpButton() {
  int reading = digitalRead(MODEBTNUP_PIN);
  if ( reading != prevModeBtnUpState ) {
    lastModeBtnUpDebounceTime = millis();
  }
  if ( (millis() - lastModeBtnUpDebounceTime) >= debounceDelay ) {
    if ( reading != modeBtnUpState )
      modeBtnUpState = reading;
  }
  prevModeBtnUpState = reading;
  return modeBtnUpState;
}

int debounceModeDownButton() {
  int reading = digitalRead(MODEBTNDOWN_PIN);
  if ( reading != prevModeBtnDownState ) {
    lastModeBtnDownDebounceTime = millis();
  }
  if ( (millis() - lastModeBtnDownDebounceTime) >= debounceDelay ) {
    if ( reading != modeBtnDownState )
      modeBtnDownState = reading;
  }
  prevModeBtnDownState = reading;
  return modeBtnDownState;
}

