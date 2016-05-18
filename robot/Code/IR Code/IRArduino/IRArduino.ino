/*
 * IRremote: IRrecvDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>

int RECV_PIN = 11;

IRrecv irrecv(RECV_PIN);
String in;
decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  detectIR();
}

void detectIR(){
  if (irrecv.decode(&results)) {
    in = String(int(results.value), HEX);
    irrecv.resume(); // Receive the next value

    if(in == "a05f"){
      Serial.println("2");
      }
    if(in == "906f"){
      Serial.println("5");
      }
    if(in == "10ef"){
      Serial.println("4");
      }
    if(in == "50af"){
      Serial.println("6");
      }
  }

  delay(100);
}


