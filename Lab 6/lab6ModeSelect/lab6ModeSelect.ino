
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BUTTON_PIN 3
#define VOLTAGE_IN A5
#define MODEBTN_PIN 2
const long samplingDuration = 5000;
int ISRflag = 0;

// debouncing variables
int modeBtnState;
int prevModeBtnState = LOW;
unsigned long lastModeBtnDebounceTime = 0;
const long debounceDelay = 50;

void setup() {
  Serial.begin(250000);
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), isr, RISING);
}

void loop() {
  if ( ISRflag == 1 ) {
    if( debounceModeButton() == LOW ) {
      // MODE 1
    }
    else if( debounceModeButton() == HIGH ) {
      // MODE 2
    }
    ISRflag == 0;
  }
  debounceModeButton();
}

void isr() {
  ISRflag = 1;
}

int debounceModeButton() {
  int reading = digitalRead(MODEBTN_PIN);
  if ( reading != prevModeBtnState ) {
    lastModeBtnDebounceTime = millis();
  }
  if ( (millis() - lastModeBtnDebounceTime) >= debounceDelay ) {
    if ( reading != modeBtnState )
      modeBtnState = reading;
  }
  prevModeBtnState = reading;
  return modeBtnState;
}

