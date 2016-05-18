#include <DHT.h>

// Pin assignments
const int DHTPIN = 5;
const int DHTTYPE = DHT11;
const int TEMPPIN = A0;
const int PHOTOPIN = A1;
const int BUTTONPIN = 7;
const int DEBOUNCEDELAY = 1000;

// Global variables
int ButtonState;
int prevButtonState = LOW;
unsigned long timeSinceDebounce=0;
int state;
int readingDataTime =1000;
const int carlibrateValue = 15; //calculated from the data 
DHT dht(DHTPIN,DHTTYPE);    // initialize the DHT 

void setup() {
  Serial.begin(9600);
  pinMode(BUTTONPIN, INPUT);
  dht.begin();
}

void loop() {
  // For debouncing
  int buttonIn = digitalRead(BUTTONPIN);
  if( buttonIn != prevButtonState ) {       // update the timeSinceDebounce if push button is fluctuating due to noise
    timeSinceDebounce = millis();
  }
  if( (millis() - timeSinceDebounce) > DEBOUNCEDELAY ) {    // if button has been in the same state for long enough
    if( buttonIn != ButtonState ) {                      // and button is in a different state from the previous actual button state
        ButtonState = buttonIn;                         //save the buttonIn to ButtonState
        delay(5);
        //switch the state, if state is 1, change it to 0, if 0 change it to 1
      if(state == 1){
          state=0;                                
          timeSinceDebounce = millis();       //update the debounce time
       } 
       else{
          state=1;
          timeSinceDebounce = millis();
       }
     }                         
  }
// // Read data from the sensors
  int dhth = dht.readHumidity();
  int dhtt = dht.readTemperature();
  int lm35 = analogRead(TEMPPIN);
  int light = analogRead(PHOTOPIN);
  int calibratedLm35 = lm35 - carlibrateValue;    //calibrate the data using a statistic difference 
// Task 1: Output the raw data for analysis by processing/matlab
  if( state == 0 ) {
    Serial.print(dhth);
    Serial.print(",");
    Serial.print(dhtt);
    Serial.print(",");
    Serial.print(calibratedLm35);
    Serial.print(",");
    Serial.print(light);
    Serial.print(",");
    Serial.print(0);
    Serial.print("\n");
    delay(readingDataTime);
  }
  else if(state ==1 ) {
  // Task 2: Output the data to the serial monitor with captions and units
    Serial.print("DHT Humidity: ");
    Serial.print(dhth);
    Serial.print(" RH");
    Serial.print("\t");
    //partial pressure of water vapor to the equilibrium vapor pressure of water at the same temperature
    Serial.print("DHT Temperature: ");
    Serial.print(dhtt);
    Serial.print(" degree");
    Serial.print("\t");
    Serial.print("LM35 Temperature: ");
    Serial.print(calibratedLm35);
    Serial.print(" degree");
    Serial.print("\t");
    Serial.print("Photocell: ");
    Serial.print(light);
    Serial.print(" lx");
    Serial.print("\n");
    delay(readingDataTime);
  }

    prevButtonState = buttonIn;   // store the previous button state for the next loop
}
