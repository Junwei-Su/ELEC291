// defines setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BUTTON_PIN 3
#define VOLTAGE_IN A5
const long samplingDuration = 15000;
int ISRflag = 0;  // used to trigger main loop

void setup() {
  Serial.begin(250000);
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr, RISING);  // trigger ISR on rising edge of button
}

void loop() {
  if ( ISRflag == 1 ) {     // main loop runs after ISR detected
    unsigned long prevMilli = millis();   // record start time in millisecond
    unsigned long oriMicros = micros();   // record start time in microseconds

    while ( millis() - prevMilli <= samplingDuration ) {    // send data for samplingDuration milliseconds
      Serial.print(analogRead(VOLTAGE_IN) * (5.0 / 1023.0));  // convertion from analogRead to 0-5V
      Serial.print(",");
      Serial.print(micros() - oriMicros);   // time since we started sampling for this data point
      Serial.print(",0\n");
    }
    ISRflag = 0;  // reset ISRflag so we do not continue to loop
  }
}

// ISR that sets a flag to trigger the main loop
void isr() {
  ISRflag = 1;
}

