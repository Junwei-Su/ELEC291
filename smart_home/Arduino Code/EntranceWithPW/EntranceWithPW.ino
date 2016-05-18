
/////////////////////////////////////////////////
//                    KeyPad                   //
/////////////////////////////////////////////////

//keypad layout
char keyPad[4][3] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

//pin number
const int KeyRowP[] = {3, 4, 5, 6};
const int KeyColP[] = {7, 8, 9};

//other variables for keypad
int debounceTime = 1000;
char lastInput = ' ';

/////////////////////////////////////////////////
//                    LCD                      //
/////////////////////////////////////////////////

// LCD variables
int numArray[] = { LOW, HIGH };
int LCDDisplay = HIGH;
int LCDCursor = LOW;
int LCDCursorBlink = LOW;
String lcdOffSet="                        ";

// LCD analog pins
const int LCDrs = A0;
const int LCDenable = A1;
const int LCDdata[4] = {A5, A4, A3, A2};

//LCD character
char Binary[][9] =
{
  "01000001", //A 0
  "01000010", //B 1
  "01000011", //C 2
  "01000100", //D 3
  "01000101", //E 4
  "01000110", //F 5
  "01000111", //G 6
  "01001000", //H 7
  "01001001", //I 8
  "01001010", //J 9
  "01001011", //K 10
  "01001100", //L 11
  "01001101", //M 12
  "01001110", //N 13
  "01001111", //O 14
  "01010000", //P 15
  "01010001", //Q 16
  "01010010", //R 17
  "01010011", //S 18
  "01010100", //T 19
  "01010101", //U 20
  "01010110", //V 21
  "01010111", //W 22
  "01011000", //X 23
  "01011001", //Y 24
  "01011010", //Z 25
  "00110000", //0 26
  "00110001", //1 27
  "00110010", //2 28
  "00110011", //3 29
  "00110100", //4 30
  "00110101", //5 31
  "00110110", //6 32
  "00110111", //7 33
  "00111000", //8 34
  "00111001", //9 35
  "00100110", //& 36
  "00101010", //* 37
  "00100001", //! 38
  "00101101", //Hyphen 39
  "00100011", //# 40
  "00101110", //Period (.) 41
  "00100000", //Space  42
  "01100001", //a 43
  "01100010", //b 44
  "01100011", //c 45
  "01100100", //d 46
  "01100101", //e 47
  "01100110", //f 48
  "01100111", //g 49
  "01101000", //h 50
  "01101001", //i 51
  "01101010", //j 52
  "01101011", //k 53
  "01101100", //l 54
  "01101101", //m 55
  "01101110", //n 56
  "01101111", //o 57
  "01110000", //p 58
  "01110001", //q 59
  "01110010", //r 60
  "01110011", //s 61
  "01110100", //t 62
  "01110101", //u 63
  "01110110", //v 64
  "01110111", //w 65
  "01111000", //x 66
  "01111001", //y 67
  "01111010", //z  68
  "00111111" //? 69
};


//password
unsigned long password_hash_value = 0;
boolean password_exist = false;
const int PASSWORD_MAX_LENGTH = 8;
boolean verified_log_in = false;
const int time_between_input = 2000;
int verify_attempts = 5;

//todo:
//1.need to implement a time lock to prevent brute force break
//2.forgot password


/////////////////////////////////////////////////
//                    Execution                //
/////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  if(!password_exist) initialize_menu();
  while(!verified_log_in){
    input_password();
  }
  //execute other parts
  
}

/////////////////////////////////////////////////
//                  Interface                  //
/////////////////////////////////////////////////

void initialize_menu(){
  String welcomeStringp1="Initial set up ";
  //welcome message
  lcdClear();
  lcdNoAutoScroll();
  lcdPrint(welcomeStringp1);
  lcdPrint(lcdOffSet);
  delay(2000);
  }


/////////////////////////////////////////////////
//                    KeyPad                   //
/////////////////////////////////////////////////

//parameter: none
//return the key pressed by the user
char readKey() {
  int startTime = millis();
  int col, row;
  //read the pins of Keypad
  //implemented a super loop to keep keypad listening to the input
  while (true) {
    boolean noKey = true;
    //when there is no input keep the arduino scanning for the keys
    while (noKey) {
      for (int index = 0; index < 3; index++) {
        digitalWrite(KeyColP[index], HIGH);
      }
      for (int index = 0; index < 3; index++) {
        digitalWrite(KeyColP[index], LOW);
        for (int rowIndex = 0; rowIndex < 4; rowIndex++) {
          if (digitalRead(KeyRowP[rowIndex]) == LOW) {
            row = rowIndex;
            col = index;
            noKey = false;
            break;
          }
        }
        if (!noKey) break;
      }
      delay(25);
    }
    if (keyPad[row][col] != lastInput || (millis() - startTime >= debounceTime))break;
    //prevent the arduino from reading too many inputs. It only read new input when the new input does not equal to last input or exceeds the debounce time
  }
  //return the result
  lastInput = keyPad[row][col];
  return keyPad[row][col];
}





/////////////////////////////////////////////////
//                    LCD                      //
/////////////////////////////////////////////////

// Flashes the LCDenable pin to read instructions/data
void flashEnable() {
  digitalWrite(LCDenable, LOW);
  delay(1);
  digitalWrite(LCDenable, HIGH);
  delay(1);
  digitalWrite(LCDenable, LOW);
  delay(1);
}

// Sets the LCDdata pins
void setLCDPins(int rs, int d3, int d2, int d1, int d0) {
  digitalWrite(LCDrs, rs);
  digitalWrite(LCDdata[0], d3);
  digitalWrite(LCDdata[1], d2);
  digitalWrite(LCDdata[2], d1);
  digitalWrite(LCDdata[3], d0);
}

// Sends the data/instruction to the lcd and flashes enable
void sendCode(int rs, int d7, int d6, int d5, int d4, int d3, int d2, int d1, int d0) {
  setLCDPins(rs, d7, d6, d5, d4);
  flashEnable();
  setLCDPins(rs, d3, d2, d1, d0);
  flashEnable();
}

// Increments the cursor position to the right (moves to next line on the left if end of line)
void incrementCursor() {
  sendCode(LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW);
}

// Decrements the cursor position to the left (moves to previous line on the right if end of line)
void decrementCursor() {
  sendCode(LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, LOW, LOW);
}

// Shifts entire display to the right, cursor follows
void displayRight() {
  sendCode(LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW, LOW);
}

// Shifts entire screen to the left, cursor follows
void displayLeft() {
  sendCode(LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW);
}

// Prints a single char onto the lcd
void printChar(char c) {
  //Convert character to ASCII number
  int value = (int) c;
  //ASCII for 0-9
  if (value >= 48 && value <= 57) {
    value = value - 22;
    getCharacter(value);
  }
  //ASCII for A-Z
  else if (value >= 65 && value <= 90) {
    value = value - 65;
    getCharacter(value);
  }
  else if (value >= 97 && value <= 122) {
    value = value - 54;
    getCharacter(value);
  }
  //ASCII for period
  else if (value == 46) {
    getCharacter(41);
  }
  // ASCII For exclamation
  else if (value == 33) {
    getCharacter(38);
  }
  // ASCII For &
  else if (value == 38) {
    getCharacter(36);
  }
  //ASCII For *
  else if (value == 42) {
    getCharacter(37);
  }
  //ASCII for - (Hyphen)
  else if (value == 45) {
    getCharacter(39);
  }
  //ASCII for #
  else if (value == 35) {
    getCharacter(40);
  }
  //ASCII for Space
  else if (value == 32) {
    getCharacter(42);
  }
  //ASCII for ?
  else if (value == 63) {
    getCharacter(69);
  }
}

int convertToInt(char character) {
  int aNumber = character - '0';
  return aNumber;
}

void getCharacter(int value) {
  String inputNumber = Binary[value];
  sendCode(HIGH, numArray[convertToInt(inputNumber.charAt(0))],
           numArray[convertToInt(inputNumber.charAt(1))],
           numArray[convertToInt(inputNumber.charAt(2))],
           numArray[convertToInt(inputNumber.charAt(3))],
           numArray[convertToInt(inputNumber.charAt(4))],
           numArray[convertToInt(inputNumber.charAt(5))],
           numArray[convertToInt(inputNumber.charAt(6))],
           numArray[convertToInt(inputNumber.charAt(7))]
          );
}


// LCD FUNCTIONS - HIGH LEVEL
// Initializes the lcd to 4-bit mode at power on
void lcdInit() {
  delay(50);                          // wait for VDD
  for (int i = 0; i < 3; i++) {       // repeat x3
    setLCDPins(LOW, LOW, LOW, HIGH, HIGH);
    flashEnable();
    delay(40);
  }

  setLCDPins(LOW, LOW, LOW, HIGH, LOW);   // 4-bits mode
  flashEnable();
  setLCDPins(LOW, HIGH, LOW, LOW, LOW);   // display lines and font
  flashEnable();
  setLCDPins(LOW, HIGH, LOW, LOW, LOW);   // display off
  flashEnable();
  setLCDPins(LOW, LOW, LOW, LOW, HIGH);   // display clear
  flashEnable();
  setLCDPins(LOW, LOW, HIGH, HIGH, LOW);  // increment cursor, no display shift
  flashEnable();
}
// Clears the LCD screen and returns the cursor home
void lcdClear() {
  sendCode(LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH);
  delay(5);
}

// Returns the cursor home
void lcdCursorHome() {
  sendCode(LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW);
  delay(5);
}

// Turns ON the display
void lcdDisplay() {
  LCDDisplay = HIGH;
  sendCode(LOW, LOW, LOW, LOW, LOW, HIGH, LCDDisplay, LCDCursor, LCDCursorBlink);
}

// Turns OFF the display
void lcdNoDisplay() {
  LCDDisplay = LOW;
  sendCode(LOW, LOW, LOW, LOW, LOW, HIGH, LCDDisplay, LCDCursor, LCDCursorBlink);
}

// Displays the cursor
void lcdCursor() {
  LCDCursor = HIGH;
  sendCode(LOW, LOW, LOW, LOW, LOW, HIGH, LCDDisplay, LCDCursor, LCDCursorBlink);
}

// Hides the cursor
void lcdNoCursor() {
  LCDCursor = LOW;
  sendCode(LOW, LOW, LOW, LOW, LOW, HIGH, LCDDisplay, LCDCursor, LCDCursorBlink);
}

// Blinks the cursor
void lcdBlink() {
  LCDCursorBlink = HIGH;
  sendCode(LOW, LOW, LOW, LOW, LOW, HIGH, LCDDisplay, LCDCursor, LCDCursorBlink);
}

// Stops blinking the cursor
void lcdNoBlink() {
  LCDCursorBlink = LOW;
  sendCode(LOW, LOW, LOW, LOW, LOW, HIGH, LCDDisplay, LCDCursor, LCDCursorBlink);
}

// Turn ON scrolling when writing
void lcdAutoScroll() {
  sendCode(LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH);
}

// Turn OFF scrolling when writing
void lcdNoAutoScroll() {
  sendCode(LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW);
}

// Increments cursor position k times
void lcdMoveCursorRight(int k) {
  for (int i = 0; i < k; i++) {
    incrementCursor();
  }
}

// Decrements cursor position k times
void lcdMoveCursorLeft(int k) {
  for (int i = 0; i < k; i++) {
    decrementCursor();
  }
}

// Scrolls the display to the right k times
void lcdScrollRight(int k) {
  for (int i = 0; i < k; i++) {
    displayRight();
  }
}

// Scrolls the display to the left k times
void lcdScrollLeft(int k) {
  for (int i = 0; i < k; i++) {
    displayLeft();
  }
}

// Prints a string onto the lcd
void lcdPrint(String s) {
  for (int i = 0; i < s.length(); i++) {
    printChar(s.charAt(i));
  }
}


/////////////////////////////////////////////////
//                    password                 //
/////////////////////////////////////////////////

void initial_password() {
  //message to prompt the user for password
  int input[PASSWORD_MAX_LENGTH] = {0};
  boolean finished = false;
  //keep reading until the user finish input
  char currentReading;
  while (!finished) {
    for (int index = 0; index < PASSWORD_MAX_LENGTH; index++) {
      currentReading = readKey();
      if (currentReading == '#') {
        finished = true;
        break;
      }
      else {
        input[index] = convertToInt(currentReading);
      }
    }
  }
  password_hash_value = hash_function(input);
}

void change_password() {
  int input[PASSWORD_MAX_LENGTH] = {0};
  boolean finished = false;
  //keep reading until the user finish input
  char currentReading;
  while (!finished) {
    for (int index = 0; index < PASSWORD_MAX_LENGTH; index++) {
      currentReading = readKey();
      if (currentReading == '#') {
        finished = true;
        break;
      }
      else {
        input[index] = convertToInt(currentReading);
      }
    }
  }
  password_hash_value = hash_function(input);
}

boolean verify_password(int* input) {
  if (hash_function(input) == password_hash_value) return true;
  return false;
}

//read password from user
boolean input_password() {
  int input[PASSWORD_MAX_LENGTH] = {0};
  boolean finished = false;
  //keep reading until the user finish input
  char currentReading;
  while (!finished) {
    for (int index = 0; index < PASSWORD_MAX_LENGTH; index++) {
      currentReading = readKey();
      if (currentReading == '#') {
        finished = true;
        break;
      }
      else {
        input[index] = convertToInt(currentReading);
      }
    }
  }
  return verify_password(input);
}


long hash_function(int* input) {
  first_layer_enscription(input);
  int i;
  unsigned long hash_value;
  unsigned long intermediate;
  for (i = 0; i < PASSWORD_MAX_LENGTH; i++) {
    intermediate += input[i] * pow(10, i);
  }
  hash_value = intermediate + 3 * 7823 + 2 * 5783;
  return hash_value;
}

//first layer of encription: encript the int array
void first_layer_enscription(int* input) {
  swap(input, 0, 8);
  swap(input, 3, 6);
  swap(input, 4, 5);
}

//swap two elements in the array
void swap(int* input, int index1, int index2) {
  int intermediate = 0;
  intermediate = input[index1];
  input[index1] = input[index2];
  input[index2] = intermediate;
}



