// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BUTTON_PIN 3
#define VOLTAGE_IN A5
#define BUFFER_SIZE 200
const long samplingDuration = 3000;
int nonZeroIndex = -1;
int ISRflag = 0;
int slopeFlag = -1;
int prevBufferEnd = 0;

void setup() {
  Serial.begin(250000);
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr, CHANGE);
}

void loop() {
  if ( ISRflag == 1 ) {
    int data[BUFFER_SIZE];
    unsigned long prevMilli = millis();

    int i = 0;
    // fill the buffer
    while ( millis() - prevMilli <= samplingDuration && i <= BUFFER_SIZE) {
      data[i] = (analogRead(VOLTAGE_IN));
      i++;

      // send data in buffer after it has been filled
      if ( i >= BUFFER_SIZE || millis() - prevMilli >= samplingDuration ) {
        int j = 0;
        // calculate which part of the start location of the buffer to append to the last set of data points
        switch ( slopeFlag ) {
          case -1:  // previous slope was negative
            while ( (data[j + 1] > data[j] || data[j] > prevBufferEnd) && j < i )
              j++;
            break;
          case 1:   // previous slope was positive
            while ( (data[j + 1] < data[j] || data[j] < prevBufferEnd) && j < i )
              j++;
            break;
          case 0:   // previous slope was 0
            while ( data[j] == 0 )
              j++;
            break;
          default:
            while (data[j] == 0)
              j++;
            break;
        };
        // send data points, ignores 0s, record the index of the last non-zero value
        while ( j < i ) {
          if (data[j] != 0) {
            Serial.print(data[j] * (5.0 / 1023.0));
            Serial.print(",0\n");
            nonZeroIndex = j;
          }
          j++;
        }
        Serial.print("BREAK\n");   // debugging info to tell us when a bufferred set of data ends
        i = 0;  // reset i to 0 so we can fill a new buffer later

        // calculates the slope of the end of the buffer that was just sent
        if (nonZeroIndex == -1)
          slopeFlag = 0;
        else if (data[nonZeroIndex] < data[nonZeroIndex - 1]) {
          slopeFlag = -1;
          prevBufferEnd = data[nonZeroIndex];
        }
        else if (data[nonZeroIndex] > data[nonZeroIndex - 1]) {
          slopeFlag = 1;
          prevBufferEnd = data[nonZeroIndex];
        }
        else
          slopeFlag = 0;

        nonZeroIndex = -1;  // resets the nonZeroIndex for next buffer sending
      }
      
    }
  }
  ISRflag = 0;    // stop running main loop code after samplingDuration milliseconds
}


void isr() {
  ISRflag = 1;
}

