#define ultraTri 9
#define ultraEcho 10
#define maxRange 200 //unit cm
const int LM35 = A1;
const float soundSpeedinZero = 331; 

void setup() {
  Serial.begin(9600);
  pinMode(LM35, INPUT);
  pinMode(ultraTri,OUTPUT);
}

void loop() {
  delay(1000);
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(ultraTri, LOW);
  delayMicroseconds(5);
  digitalWrite(ultraTri, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraTri, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ultraEcho, INPUT);
  float ultraSonReading = pulseIn(ultraEcho, HIGH);
  float LM35reading =  analogRead(LM35);
  float temperature = calibrateLM35(LM35reading);
  float distance = calibrateDistance(ultraSonReading,temperature);
  if(distance <= 200.0){
    Serial.print(distance);
    Serial.print(",");
  }else {
    Serial.print(0.0);
    Serial.print(",");
    }
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(0.0);
  Serial.print(",");
  Serial.print("\n");
}

//calibrate the raw analog value from LM35
float calibrateLM35(float rawValue){
  return (5.0 * rawValue * 100.0) / 1024;
  }

//calibrate the distance with the temperature
float calibrateDistance(float travelTime, float temperature){
    float soundSpeed = 331 + 0.6* temperature;  //sound speed in the particular temperature
    float timeInSec = travelTime / 1000000.0;  //travelTime in second
    return timeInSec * soundSpeed / 2.0 * 100; //distance = time * speed divide the value by 2 to find the one trip distance
  }
