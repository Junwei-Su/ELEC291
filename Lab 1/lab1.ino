/* 
 * Code used in lab 1 of ELEC 291-20C 
 */

// CONSTANTS
const int ONBOARDLEDPIN = 13;
const int REDLEDPIN = 10;
const int GREENLEDPIN = 11;
const int BLUELEDPIN = 12;
const int SWITCH8 = 9;
const int SWITCH7 = 8;
const int PHOTOPIN = 0;
const int PHOTOLEDPIN = 6;
const int PHOTOTHRESHOLD = 800;
const long INTERVAL = 2000;

// VARIABLES
unsigned long prevMilli = 0;
int onBoardLEDState = LOW;
int sw8Status, sw7Status, swLEDState, photoCellStatus, photoCellState; // used for Serial.print()

void setup() {
  Serial.begin(9600); // begin serial comms

  pinMode(ONBOARDLEDPIN, OUTPUT);
  pinMode(REDLEDPIN, OUTPUT);
  pinMode(GREENLEDPIN, OUTPUT);
  pinMode(BLUELEDPIN, OUTPUT);
  pinMode(PHOTOLEDPIN, OUTPUT);
  // other pins implicitly treated as INPUT
}

void loop() {
  // read the two switches used on the DIP switch and the value of the photocell
  sw8Status = digitalRead(SWITCH8);
  sw7Status = digitalRead(SWITCH7);
  photoCellStatus = analogRead(PHOTOPIN);

  // Both switches OFF => RGB OFF
  if ( sw8Status == LOW && sw7Status == LOW ) {
    digitalWrite(REDLEDPIN, HIGH);
    digitalWrite(GREENLEDPIN, HIGH);
    digitalWrite(BLUELEDPIN, HIGH);
    swLEDState = LOW;
  }
  // sw8 ON, sw7 OFF => Red->Green->Blue pattern
  else if ( sw8Status == HIGH && sw7Status == LOW ) {
    digitalWrite(REDLEDPIN, LOW);
    delay(1000);
    digitalWrite(REDLEDPIN, HIGH);
    digitalWrite(GREENLEDPIN, LOW);
    delay(1000);
    digitalWrite(GREENLEDPIN, HIGH);
    digitalWrite(BLUELEDPIN, LOW);
    delay(1000);
    digitalWrite(BLUELEDPIN, HIGH);
    swLEDState =  HIGH;
  }
  // sw8 OFF, sw7 ON => Green
  else if ( sw8Status == LOW && sw7Status == HIGH ) {
    digitalWrite(REDLEDPIN, HIGH);
    digitalWrite(GREENLEDPIN, LOW);
    digitalWrite(BLUELEDPIN, HIGH);
    swLEDState = HIGH;
  }
  // both ON => Blue
  else {
    digitalWrite(REDLEDPIN, HIGH);
    digitalWrite(GREENLEDPIN, HIGH);
    digitalWrite(BLUELEDPIN, LOW);
    swLEDState = HIGH;
  }

  // single-colour LED turns on at low light
  if ( photoCellStatus >= PHOTOTHRESHOLD ) {
    digitalWrite(PHOTOLEDPIN, HIGH);
    photoCellState = LOW;
  }
  else {
    digitalWrite(PHOTOLEDPIN, LOW);
    photoCellState = HIGH;
  }

  // blinks the LED on the Arduino every 2s without using delay();
  // checks every loop() if INTERVAL ms has passed
  unsigned long currMilli = millis();
  if ( currMilli - prevMilli >= INTERVAL ) {
    prevMilli = currMilli;

    if ( onBoardLEDState == LOW ) {
      onBoardLEDState = HIGH;
    }
    else {
      onBoardLEDState = LOW;
    }
    digitalWrite(ONBOARDLEDPIN, onBoardLEDState);
  }

  // prints the status of multiple elements on a line
  // Switches: sw8 sw7
  Serial.print("Switches: ");
  Serial.print(sw8Status);
  Serial.print(" ");
  Serial.print(sw7Status);
  // LEDs: Arduino RGB Light-sens
  Serial.print("\tLEDs: ");
  Serial.print(onBoardLEDState);
  Serial.print(" ");
  Serial.print(swLEDState);
  Serial.print(" ");
  Serial.print(photoCellState);
  // value read from the photocell
  Serial.print("\tPhotocell value: ");
  Serial.print(photoCellStatus);
  Serial.print("\n");
  delay(200); // 200 ms delay between loops as requested
}
