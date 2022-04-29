//Libraries
#include <Wire.h>
#include <Servo.h>

//Initialize servos
Servo wheelOne;
Servo wheelTwo;
Servo wheelThree;
int thetaPin = 7;
int forwardPin = 8;
int sidePin = 9;

//Initialize kiwi drive variables
int ser1;
int ser2;
int ser3;

void setup() {
  //Initialize serial monitor
  Serial.begin(9600);

  //Initialize wheels
  wheelOne.attach(2);
  wheelTwo.attach(3);
  wheelThree.attach(4);
  calib();
  delay(3000);

  //initialize remote signals
  pinMode(forwardPin, INPUT); 
  pinMode(sidePin, INPUT);
  pinMode(thetaPin, INPUT);

  Serial.println("Done with setup");
}

void loop() {
  long thetaDuration = pulseIn(thetaPin, HIGH);  //Read PWM for theta input
  float thetaVal = map((float)thetaDuration, 1940.0, 950.0, -1.0, 1.0);
   
  long fwdDuration = pulseIn(forwardPin, HIGH);  //Read PWM for fwd input
  float fwdVal = map((float)fwdDuration, 1000.0, 1980.0, -1.0, 1.0);

  long sideDuration = pulseIn(sidePin, HIGH);  //Read PWM for side input
  float sideVal = map((float)sideDuration, 2030.0, 1080.0, -1.0, 1.0);

  kiwiFunct(fwdVal, sideVal, thetaVal);   //Calculate servo values with kiwi drive math


  wheelOne.writeMicroseconds(ser1);   //Write values to drive servos
  wheelTwo.writeMicroseconds(ser2);
  wheelThree.writeMicroseconds(ser3);
}

//Calculates servo values
void kiwiFunct(float fwdVal, float sideVal, float thetaVal){
  float m1 = .5 * sideVal -sqrt(3)/2 * fwdVal + thetaVal;
  float m2 = .5 * sideVal + sqrt(3)/2 * fwdVal + thetaVal;
  float m3 = -1 * sideVal + thetaVal;

  float avg1 = (m1)/3.0;
  float avg2 = (m2)/3.0;
  float avg3 = (m3)/3.0;
  
  ser1 = (int)(map(avg1, -1, 1, 1000, 2000));
  ser2 = (int)(map(avg2, -1, 1, 1000, 2000));
  ser3 = (int)(map(avg3, -1, 1, 1000, 2000));
}

//Rotates robot right
void rotateRight(){
  wheelOne.writeMicroseconds(1300);
  wheelTwo.writeMicroseconds(1300);
  wheelThree.writeMicroseconds(1300);
  delay(50);
  calib();
}

//Rotates robot left
void rotateLeft(){
  wheelOne.writeMicroseconds(1700);
  wheelTwo.writeMicroseconds(1700);
  wheelThree.writeMicroseconds(1700);
  delay(50);
  calib();
}

//Stops robot
void calib(){
  wheelOne.writeMicroseconds(1500);
  wheelTwo.writeMicroseconds(1500);
  wheelThree.writeMicroseconds(1500);
}

//Prints servo values
void printSer(){
  Serial.print("Ser1: " );
  Serial.print(ser1);
  Serial.print("\tSer2: " );
  Serial.print(ser2);
  Serial.print("\tSer3: " );
  Serial.println(ser3);
}

//Takes value v in range [a,b] and maps to corresponding value in range [c,d]
float map(float v, float a, float b, float c, float d){
  return (v-a) * (d-c) / (b-a) + c;
}