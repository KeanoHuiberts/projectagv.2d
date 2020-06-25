#include <Wire.h>
#include <ZumoShield.h>


#define sensorIR1 A1 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)

ZumoMotors motors;
// defines pins numbers
const int trigPin = 6;
const int echoPin3 = 13;
// defines variables
long durationUS3;
int distanceUS3;
int SPEED=100;
int actie=3;
int stoppen=0;

void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin3, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication 

}

void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
durationUS3 = pulseIn(echoPin3, HIGH);
// Calculating the distanceUS1
distanceUS3= durationUS3*0.034/2;
// Prints the distanceUS1 on the Serial Monitor
Serial.print("distance: ");
Serial.println(distanceUS3);
Serial.println(actie);


float volts = analogRead(sensorIR1)*0.0048828125;  // value from sensor * (5/1024)
  int distanceIR1 = 13*pow(volts, -1); // worked out from datasheet graph

      if(distanceUS3<5){
      actie=4;
    }else{
      if(distanceUS3<10){
        actie=0;
      }else{
            actie=1;
          }
        }
      
    
     Serial.println(distanceIR1);
     Serial.println(actie);
switch(actie){
  case 0: //stoppen
  motors.setSpeeds(0, 0);
  break;
  case 1: //vooruit
  motors.setSpeeds(SPEED, SPEED);
  break;
  case 2: //flauw naar rechts
  motors.setSpeeds(SPEED-25, SPEED);
  break;
  case 3: //flauw naar links
  motors.setSpeeds(SPEED, SPEED-25);
  break;
  case 4: //vooruit
  motors.setSpeeds(-SPEED, -SPEED);
  break;
  case 6: //stoppen
  
  motors.setSpeeds(0, 0);
  delay(2000);
  
    motors.setSpeeds(SPEED, SPEED);
    stoppen=0;
    delay(500);
  break;
}}
