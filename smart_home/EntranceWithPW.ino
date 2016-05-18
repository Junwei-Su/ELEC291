#include <Wire.h>

#include <Key.h>
#include <Keypad.h>


#include <LiquidCrystal.h>


/////////////////////////////////////////////////
//                    KeyPad                   //
/////////////////////////////////////////////////

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns

//keypad layout
char keyPad[4][3] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

//pin number
byte KeyRowP[ROWS] = {13, 12, 11, 10};
byte KeyColP[COLS] = {9, 8, 7};

//other variables for keypad
int debounceTime = 1000;
char lastInput = ' ';

Keypad keypad = Keypad( makeKeymap(keyPad), KeyRowP, KeyColP, ROWS, COLS );

/////////////////////////////////////////////////
//                    LCD                      //
/////////////////////////////////////////////////

//tested correect pins
LiquidCrystal lcd(A1, A0, 6, 5, 4, 3);


//password
long password_hash_value = 0;
boolean password_exist = false;
const int PASSWORD_MAX_LENGTH = 8;
boolean verified_log_in = false;
const int time_between_input = 2000;
int verify_attempts = 3;

//security constant
#define disarm 0
#define arm 1
#define turn_off_and_disarm 2
#define turn_on_alarm 3

/////////////////////////////////////////////////
//                    Execution                //
/////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin();
    lcd.begin(16, 2);
}

void loop() {
  //check if a password is set up
  if(!password_exist) initialize_menu();

  //prompt the user for logging in
  while(!verified_log_in&&verify_attempts!=0){
    lcd.clear();
    lcd.print("Enter password");
    lcd.setCursor(0,1);
    lcd.print("for entrance");
    if(input_password()){  
      lcd.clear();
      lcd.print("Login successful");
      //update the attempt number and send the verified log in status to be true
      verify_attempts=3;
      verified_log_in=true;
      //send the disarm message
      sendToSlave(disarm);
      delay(2000);
      }
    else{
      lcd.clear();
      lcd.print("Wrong password");
      lcd.setCursor(0,1);
      lcd.print("Try again");
      //decrement the attempt number
      verify_attempts--;
      delay(2000);
      }
  }

  //things to do when exhausted the attempts
  if(verify_attempts==0){
    sendToSlave(turn_on_alarm);
    lcd.clear();
    lcd.print("Enter password");
    lcd.setCursor(0,1);
    lcd.print("to turn off");
    if(input_password()){
      //turn off the alamr upon successful input
      sendToSlave(turn_off_and_disarm);
      //resent the attempt number and set the log in status to be true
      verify_attempts=3;
      verified_log_in=true;
      lcd.clear();
      lcd.print("Login successful");
    }
    }
  
  //check if the user want to change the password
  char current_reading = keypad.getKey();
  if(current_reading=='*'){
    change_password();
    }

  //check if the user want to exit and turn on the security system
  if(current_reading=='#'){
    lcd.clear();
    lcd.print("Enter password again");
    lcd.setCursor(0,1);
    lcd.print("to exit");
    if(input_password()){
      lcd.clear();
      lcd.print("Log out ");
      lcd.setCursor(0,1);
      lcd.print("successful");
      verified_log_in=false;
      verify_attempts=3;
      sendToSlave(arm);
      delay(2000);
      }
      else{
        lcd.clear();
        lcd.print("Wrong password");
        lcd.setCursor(0,1);
        lcd.print("try again");
        }
    }
}

//send to slave
void sendToSlave(byte x) {
  Wire.beginTransmission(8);
  Wire.write(x);
  Wire.endTransmission();
}

/////////////////////////////////////////////////
//                  Interface                  //
/////////////////////////////////////////////////
void initialize_menu(){
  //welcome message
  lcd.clear();
  lcd.noAutoscroll();
  lcd.setCursor(0, 0);
  lcd.print("Initial set up ");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Choose an eight-digit");
  lcd.setCursor(0, 1);
  lcd.print("password");
  delay(1000);
  initial_password();
  }


/////////////////////////////////////////////////
//                    password                 //
/////////////////////////////////////////////////
//method used to initialize password
void initial_password() {
  //message to prompt the user for password
  int input[PASSWORD_MAX_LENGTH] = {0};
  boolean finished = false;
  //keep reading until the user finish input
  char currentReading;
  int character_num = 0;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Choose a eight-");
  lcd.setCursor(0,1);
  lcd.print("digit password");
  while (!finished) {
      currentReading = keypad.waitForKey();
      //if key ="#", delete the previous input
      if (currentReading == '#') {
        character_num--;
        lcd_display_pw(character_num);
      }
      else if(currentReading == '*'){
        lcd.print("not legal input");
        lcd_display_pw(character_num);
        }
      else{
        input[character_num]=char_to_int(currentReading);
        character_num++;
        lcd_display_pw(character_num);
        }
     if(character_num==8) finished=true;
    }

  //for testing
//  Serial.print("\n");
//  Serial.print("content for the stored hash: ");
//  for(int i=0; i<PASSWORD_MAX_LENGTH; i++){
//    Serial.print(input[i]);
//    }
//  Serial.print("\n");
  
  password_hash_value = hash_function(input);
  password_exist=true;
  
  //for testing
//  Serial.print("\n");
//  Serial.print("hash value is: ");
//  Serial.print(password_hash_value);
//  Serial.print("\n");
}

void change_password() {
  int input[PASSWORD_MAX_LENGTH]={0};
  boolean finished = false;
  //keep reading until the user finish input
  char currentReading;
  int character_num=0;
  
  lcd.clear();
  lcd.print("Input the new");
  lcd.setCursor(0,1);
  lcd.print("password");
  
  while (!finished) {
      currentReading = keypad.waitForKey();
      if (currentReading == '#') {
        character_num--;
        lcd_display_pw(character_num);
      }
      else if(currentReading == '*'){
        lcd.print("not legal input");
        lcd_display_pw(character_num);
        }
      else{
        input[character_num]=char_to_int(currentReading);
        character_num++;
        lcd_display_pw(character_num);
        }
        
     if(character_num==8) finished=true;
    }
    
  password_hash_value = hash_function(input);
  
  lcd.clear();
  lcd.print("Password changed");
  lcd.setCursor(0,1);
  lcd.print("successfully");
  delay(2000);
}

boolean verify_password(int* input) {

  //for testing
//  Serial.print("\n");
//  Serial.print("stored hash value");
//  Serial.print(password_hash_value);
//  Serial.print("\n");
//
//  Serial.print("\n");
//  Serial.print("the input is: ");
//    for(int i=0; i<PASSWORD_MAX_LENGTH; i++){
//    Serial.print(input[i]);
//    }
//   Serial.print("\n");
//
//  Serial.print("the input hash is");
//  Serial.print(hash_function(input));
  
  if (hash_function(input)==(password_hash_value)) return true;
  return false;
}

//read password from user
boolean input_password() {
  int input[PASSWORD_MAX_LENGTH] = {0};
  boolean finished = false;
  //keep reading until the user finish input
  char currentReading;
  int character_num=0;
  while (!finished) {
      currentReading = keypad.waitForKey();
      if (currentReading == '#') {
        character_num--;
        lcd_display_pw(character_num);
      }
      else if(currentReading == '*'){
        lcd.print("not legal input");
        lcd_display_pw(character_num);
        }
      else{
        input[character_num]=char_to_int(currentReading);
        character_num++;
        lcd_display_pw(character_num);
        }
     if(character_num==8) finished=true;
    }

  //for testing
  Serial.print("\n");
  Serial.print("the input is ");
  Serial.print(verify_password(input));
  Serial.print("\n");
  
  return verify_password(input);
}

//display password on the lcd
void lcd_display_pw(int n){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Password:");
  lcd.setCursor(0, 1);
  //might need adjustment
  for(int i=0; i<n; i++){
    lcd.print("*");
    }
  }

//convert the password into a integer hash value
long hash_function(int input[]) {
  first_layer_encryption(input);
  int i;
  long hash_value = 0 ;
  long intermediate= 0 ;
  for (i = 0; i < PASSWORD_MAX_LENGTH; i++) {
    intermediate += input[i] * pow(10, i);
  }

    //for testing
//  Serial.print("\n");
//  Serial.print("the input intermediate is: ");
//  Serial.print(intermediate);
//  Serial.print("\n");
  
  
  hash_value = intermediate + 3 * 7823 + 2 * 5783;

  //for testing
//  lcd.clear();
//  lcd.print("hash is");
//  lcd.print(hash_value);

  return hash_value;
}

//first layer of encription: encript the int array
void first_layer_encryption(int* input) {
  swap(input, 0, 7);
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

//turn char into corresponding int value
int char_to_int(char a){
  switch (a){
    case '0': return 0; break;
    case '1': return 1; break;
    case '2': return 2; break;
    case '3': return 3; break;
    case '4': return 4; break;
    case '5': return 5; break;
    case '6': return 6; break;
    case '7': return 7; break;
    case '8': return 8; break;
    case '9': return 9; break;
  }
  }





