
#include <LiquidCrystal.h>
#include <Wire.h>

const int LCDRS_PIN = 2;
const int LCDE_PIN = 3;
const int LCDD0_PIN = 4;
const int LCDD1_PIN = 5;
const int LCDD2_PIN = 6;
const int LCDD3_PIN = 7;
const int LCDD4_PIN = 8;
const int LCDD5_PIN = 9;
const int LCDD6_PIN = 10;
const int LCDD7_PIN = 11;

LiquidCrystal lcd(LCDRS_PIN, LCDE_PIN, LCDD0_PIN, LCDD1_PIN, LCDD2_PIN, LCDD3_PIN, LCDD4_PIN, LCDD5_PIN, LCDD6_PIN, LCDD7_PIN);

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

void setup() {
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
  lcd.createChar(0, upArrow);
  lcd.createChar(1, downArrow);
}

void loop() {
  
}

void receiveEvent(int howMany) {
  int x = Wire.read();
  lcd.clear();
  switch(x) {
    case 0:
      lcd.print("Select mode");
      lcd.setCursor(0, 1);
      lcd.print("using ");
      lcd.write(byte(0));
      lcd.print(" and ");
      lcd.write(byte(1));
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
      lcd.print("Additional 1:");
      lcd.setCursor(0, 1);
      lcd.print("Smarter Movement");
      break;
    case 4:
      lcd.print("Additional 2:");
      lcd.setCursor(0, 1);
      lcd.print("IR RC");
      break;
    default:
      lcd.print("Select mode");
      lcd.setCursor(0, 1);
      lcd.print("using ");
      lcd.write(byte(0));
      lcd.print(" and ");
      lcd.write(byte(1));
      break;
  }
}

