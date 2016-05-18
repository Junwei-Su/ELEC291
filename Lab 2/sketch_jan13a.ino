//pin assignments
const int LEDsegmentA = 2;
const int LEDsegmentB = 3;
const int LEDsegmentC = 4;
const int LEDsegmentD = 5;
const int LEDsegmentE = 6;
const int LEDsegmentF = 7;
const int LEDsegmentG = 8;
const int LEDsegmentDP = 9;
const int Buzzer = 10;
const int singleLED = 12;
const int RedLed = 11;
const int BlueLed = 13;


//user input variable & global flag
int WPM = 0;
String inputString;         //global flag for inputString
int choice;                 //global flag for choosing melody
boolean inputReady = false; //global flag for input 
const int VOLUME = 50;      //volume for the buzzor
int duration;               //length of dot
char inputCharArray [100];   //used to stored the input

void setup() {
  //set up the pins for the LED
  pinMode(LEDsegmentA, OUTPUT);
  pinMode(LEDsegmentB, OUTPUT);
  pinMode(LEDsegmentC, OUTPUT);
  pinMode(LEDsegmentD, OUTPUT);
  pinMode(LEDsegmentE, OUTPUT);
  pinMode(LEDsegmentF, OUTPUT);
  pinMode(LEDsegmentG, OUTPUT);
  pinMode(LEDsegmentDP, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(singleLED, OUTPUT);
  pinMode(RedLed, OUTPUT);
  pinMode(BlueLed, OUTPUT);
  
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  boolean input = false;  //no input for the set up yet
  Serial.flush();   //remove stuff in our buff
  //ask the user to input the speed
  Serial.println("input the speed: ");
  while(!input){
        if (Serial.available()) {
                // read the incoming byte:
                WPM = Serial.parseInt();
                input = true;
        }
  }
  Serial.println("the speed you input is: ");
  Serial.println(WPM);
  duration = 1200/(WPM);
}

void loop() {
  //turn off the LED first
  digitalWrite(BlueLed, HIGH);
  digitalWrite(RedLed, HIGH);
  digitalWrite(singleLED, HIGH);
  turnOffAllSegment();
  //if there is no input string, ask the user for input
  if(!inputReady){
       Serial.println("Please enter string input to generate Morse Code");
    }
  while(!inputReady){
    while(Serial.available()){
        Serial.println("Your input is: ");
        inputString = Serial.readString();
        Serial.println(inputString);
        inputReady=true;
        if(inputReady=true){
          break;
          }
    }
  }

    //parse the input string into char array
    if(inputReady){
      //Extra activity recogize the key word Melody and play the song that the user chooses
        if(inputString == "Melody"){
            inputReady = false;
            boolean  choiceMade = false;
            Serial.println("Input an interger to choose one of the following melody to play:");
            Serial.println("1. Star War Main Theme");
            Serial.println("2. Star War Imperial March");
            while(!choiceMade){
              if (Serial.available()) {
                      // read the incoming byte:
                      choice = Serial.parseInt();
                      choiceMade = true;
                  }
            }
              Serial.println("Your input is: ");
              Serial.println(choice);
              if(choice==1){
                playStarWarTheme();
              }
              else if(choice==2){
                playStarWarMarch();
                }
          }
          
        for(int index=0; index<inputString.length(); index++){
          inputCharArray[index] = inputString.charAt(index);
          }
      }

    //translate the the char into morse code and display on the LED as well
    if(inputReady){
        for(int index=0; index<inputString.length();index++){
//           LedTranslation(inputCharArray[index]);
           Serial.println(inputCharArray[index]);//for test
           morseCodeTranslation(inputCharArray[index]);
          }
          inputReady=false;
          delay(100);
      }
      
}

//LED encode for the characters
char LedSegmentChar[][9] = 
  {
    "11101110", // A - 0
    "00111110", // B - 1
    "10011100", // C - 2
    "01111010", // D - 3
    "10011110", // E - 4
    "10001110", // F - 5
    "11110110", // G - 6
    "01101110", // H - 7
    "00001100", // I - 8
    "01111000", // J - 9
    "01101110", // K - 10
    "00011100", // L - 11
    "10101000", // M - 12
    "00101010", // N - 13
    "11111100", // O - 14
    "11001110", // P - 15
    "11100110", // Q - 16
    "00001010", // R - 17
    "10110110", // S - 18
    "00011110", // T - 19
    "01111100", // U - 20
    "00111000", // V - 21
    "01010100", // W - 22
    "01101110", // X - 23
    "01110110", // Y - 24
    "11011010", // Z - 25
    "11111100", // 0 - 26
    "01100000", // 1 - 27
    "11011010", // 2 - 28
    "11110010", // 3 - 29
    "01100110", // 4 - 30
    "10110110", // 5 - 31
    "10111110", // 6 - 32
    "11100000", // 7 - 33
    "11111110", // 8 - 34
    "11110110", // 9 - 35
    "00000001", //period - 36
  };

//display the give char array into LED
void writeLED(char* a){
  int pinOffset = 2;
  for(int index=0; index<8; index++){
    if(a[index]=='1'){
      digitalWrite(index+pinOffset, LOW);
      }
    else{
      digitalWrite(index+pinOffset, HIGH);
      }
    }
  }


//turn off all the led Segment
void turnOffAllSegment(){
  digitalWrite(LEDsegmentA,HIGH);
  digitalWrite(LEDsegmentB,HIGH);
  digitalWrite(LEDsegmentC,HIGH);
  digitalWrite(LEDsegmentD,HIGH);
  digitalWrite(LEDsegmentE,HIGH);
  digitalWrite(LEDsegmentF,HIGH);
  digitalWrite(LEDsegmentG,HIGH);
  digitalWrite(LEDsegmentDP,HIGH);
  }

//morse code encode for the character
char morseCodeArray[][9] = 
{
  "13000000", //A
  "31110000", //B
  "31310000", //C
  "31100000", //D
  "10000000", //E
  "11310000", //F
  "33100000", //G
  "11110000", //H
  "11000000", //I
  "13330000", //J
  "31300000", //K
  "13110000", //L
  "33000000", //M
  "31000000", //N
  "33300000", //O
  "13310000", //P
  "33130000", //Q
  "13100000", //R
  "11100000", //S
  "30000000", //T
  "11300000", //U
  "11130000", //V
  "13300000", //W
  "31130000", //X
  "31330000", //Y
  "33110000", //Z
  "33333000", //0
  "13333000", //1
  "11333000", //2
  "11133000", //3
  "11113000", //4
  "11111000", //5
  "31111000", //6
  "33111000", //7
  "33311000", //8
  "33331000", //9
  "13131300", //Full Stop (.)
  "33113300", //Comma (,)
  "33311100", //Colon (:)
  "31111300", //Hyphen (-)
  "11111111" //Error
};

//translate the char into LED and morseCode
void morseCodeTranslation(char a){
    int value  = (int) a;
    if(value >= 65 && value <= 90){
      value = value -65;
      writeLED(LedSegmentChar[value]);
      writeBuzzer(morseCodeArray[value]);
      turnOffAllSegment();
      }
    // a to z
    else if(value >=97 && value <= 122){
      value = value -97;
      writeLED(LedSegmentChar[value]);
      writeBuzzer(morseCodeArray[value]);
      turnOffAllSegment();
      }
    // 0 to 9
    else if(value >= 48 && value <= 57){
      value = value -22;
      writeLED(LedSegmentChar[value]);
      writeBuzzer(morseCodeArray[value]);
      turnOffAllSegment();
      }
    else if(value == 46){
      //period
      writeLED(LedSegmentChar[36]);
      writeBuzzer(morseCodeArray[36]);
      turnOffAllSegment();
      }
    else if(value == 44){
      //comma
      turnOffAllSegment();
      writeBuzzer(morseCodeArray[37]);
      }
    else if(value == 58){
      turnOffAllSegment();
      writeBuzzer(morseCodeArray[38]);
      }
    else if(value == 45){
      turnOffAllSegment();
      writeBuzzer(morseCodeArray[39]);
      }
    else if(value == 32){
      delay(7*duration);
      }
    else {
      turnOffAllSegment();
      writeBuzzer(morseCodeArray[40]);
      }
    
  }

//make the buzzor sound according the character input
void writeBuzzer(char* a){
  //iterate through the led segments
  for(int index=0; index < 8; index++){
    delay(1*duration);
    buzz(a[index]);
    }
  delay(3*duration);//duration between characters
  }

//make the buzzor sound based on the dot or dash
void buzz(char a){
  //todo add a delay between characters
  if(a == '1'){
    analogWrite(Buzzer, VOLUME);
    digitalWrite(singleLED, LOW);
    Serial.println('1'); //for test purpose
    delay(duration);
    digitalWrite(singleLED, HIGH);
    analogWrite(Buzzer, 0);
    }
  else if(a == '3'){
    analogWrite(Buzzer, VOLUME);
    digitalWrite(singleLED, LOW);
    Serial.println('3'); //for test purpose
    delay(3*duration);
    digitalWrite(singleLED, HIGH);
    analogWrite(Buzzer, 0);
    }
  else if(a == '0'){
    Serial.println('0'); //for test purpose
    }
  }

/*
 * Extra activity section
 */
 //note frequency
int note_c5 = 523;
int note_c5s = 554;
int note_c4 = 262;
int note_c4s = 277;
int note_d4 = 294;
int note_d4s = 311;
int note_d5 = 587;
int note_d5s = 622;
int note_e4 = 330;
int note_e5 = 659;
int note_f4 = 349;
int note_f4s = 370;
int note_f5 = 698;
int note_f5s = 740;
int note_g4 = 392;
int note_g4s = 415;
int note_g5 = 784;
int note_a4 = 440;
int note_a4s = 466;
int note_b4 = 494;

//note of the song
int noteStarWar[]={note_d4, note_d4, note_d4, note_g4, note_d5, note_c5, note_b4, note_a4, note_g5, note_d5, note_c5, note_b4, note_a4,
                   note_g5, note_d5, note_c5, note_b4, note_c5, note_a4};
//duration of each note corresponding to the note array
int durationStarWar[] = {1, 1, 1, 6, 6, 1, 1, 1, 6, 3, 1, 1, 1, 6, 3, 1, 1, 1, 4};

//play the starWar melody
void playStarWarTheme(){
  int pace = 450;
  for(int index = 0; index<19; index++){
     if(durationStarWar[index]==1){
        digitalWrite(RedLed, LOW);
      }
     else if(durationStarWar[index]==3){
        digitalWrite(BlueLed, LOW);
      }
     else if(durationStarWar[index]==4){
        digitalWrite(singleLED, LOW);
      }
     else{
        digitalWrite(BlueLed, LOW);
        digitalWrite(RedLed, LOW);
      }
      //select what character to display on LED basing on the note
    if(noteStarWar[index]==note_c4||noteStarWar[index]==note_c4s||noteStarWar[index]==note_c5||noteStarWar[index]==note_c5s){
      writeLED(LedSegmentChar[2]);
      }
    else if(noteStarWar[index]==note_d4||noteStarWar[index]==note_d4s||noteStarWar[index]==note_d5s||noteStarWar[index]==note_d5){
      writeLED(LedSegmentChar[3]);
      }
    else if(noteStarWar[index]==note_e4||noteStarWar[index]==note_e5){
      writeLED(LedSegmentChar[4]);
      }
    else if(noteStarWar[index]==note_f4||noteStarWar[index]==note_f4s||noteStarWar[index]==note_f5s||noteStarWar[index]==note_f5){
      writeLED(LedSegmentChar[5]);
      }
    else if(noteStarWar[index]==note_g4||noteStarWar[index]==note_g4s||noteStarWar[index]==note_g5){
      writeLED(LedSegmentChar[6]);
      }
    else if(noteStarWar[index]==note_a4||noteStarWar[index]==note_a4s){
      writeLED(LedSegmentChar[0]);
      }
    else if(noteStarWar[index]==note_b4){
      writeLED(LedSegmentChar[1]);
      }
    tone(Buzzer, noteStarWar[index], durationStarWar[index]*pace);
    delay(400);
    digitalWrite(BlueLed, HIGH);
    digitalWrite(RedLed, HIGH);
    digitalWrite(singleLED, HIGH);
    turnOffAllSegment();
    }
  }

int sizeMarch = 38;
//note array for imperial march
int noteMarch[] = {note_g4, note_g4, note_g4, note_d4s, note_a4s, note_g4, note_d4s, note_a4s, note_g4, 
          note_d5, note_d5, note_d5, note_d5s, note_a4s, note_f4s, note_d4, note_a4s, note_g4, 
          note_g5, note_g4, note_g4, note_g5, note_f5s, note_f5, note_e5, note_g4s, note_c5s, note_b4,
          note_a4s, note_d4s, note_f4s, note_e4, note_f4, note_a4s, note_g4, note_b4, note_d5
        };
//duration of the note corresponding to the note array
int durationMarch[] = {4, 4, 4, 3, 1, 4, 3, 1, 8,
            4, 4, 4, 3, 1, 4, 3, 1, 8,
            4, 3, 1, 4, 3, 1, 4, 4, 4, 3, 1,
            4, 4, 4, 3, 1, 4, 3, 1, 6
          };
void playStarWarMarch(){
  int pace = 255;
  for(int index = 0; index<sizeMarch; index++){
     if(durationMarch[index]==1){
        digitalWrite(RedLed, LOW);
      }
     else if(durationMarch[index]==3){
        digitalWrite(BlueLed, LOW);
      }
     else if(durationMarch[index]==4){
        digitalWrite(singleLED, LOW);
      }
     else if(durationMarch[index]==6){
        digitalWrite(BlueLed, LOW);
        digitalWrite(RedLed, LOW);
      }
      else{
        digitalWrite(BlueLed, LOW);
        digitalWrite(RedLed, LOW);
        digitalWrite(singleLED, LOW);
        
        }

     //select what character to display on LED basing on the note
    if((noteStarWar[index]==note_c4)||(noteStarWar[index]==note_c4s)||(noteStarWar[index]==note_c5)||(noteStarWar[index]==note_c5s)){
      writeLED(LedSegmentChar[2]);
      }
    else if((noteStarWar[index]==note_d4)||(noteStarWar[index]==note_d4s)||(noteStarWar[index]==note_d5s)||(noteStarWar[index]==note_d5)){
      writeLED(LedSegmentChar[3]);
      }
    else if((noteStarWar[index]==note_e4)||(noteStarWar[index]==note_e5)){
      writeLED(LedSegmentChar[4]);
      }
    else if((noteStarWar[index]==note_f4)||(noteStarWar[index]==note_f4s)||(noteStarWar[index]==note_f5s)||(noteStarWar[index]==note_f5)){
      writeLED(LedSegmentChar[5]);
      }
    else if((noteStarWar[index]==note_g4)||(noteStarWar[index]==note_g4s)||(noteStarWar[index]==note_g5)){
      writeLED(LedSegmentChar[6]);
      }
    else if((noteStarWar[index]==note_a4)||(noteStarWar[index]==note_a4s)){
      writeLED(LedSegmentChar[0]);
      }
    else if(noteStarWar[index]==note_b4){
      writeLED(LedSegmentChar[1]);
      }
      
    tone(Buzzer, noteMarch[index], durationMarch[index]*pace);
    delay(450);
    digitalWrite(BlueLed, HIGH);
    digitalWrite(RedLed, HIGH);
    digitalWrite(singleLED, HIGH);
    turnOffAllSegment();
    }
  }
