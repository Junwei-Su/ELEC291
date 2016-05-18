#define hall_pin 7

void setup()
{
  Serial.begin(9600);
  pinMode(hall_pin, INPUT); 
}

void loop()
{
  int hall = digitalRead(hall_pin);

  Serial.print("Sensor reading: ");
  Serial.println(!hall);

  delay(100);
}

