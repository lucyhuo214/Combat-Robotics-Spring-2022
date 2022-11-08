#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h> 

#define I2C_SDA A4
#define I2C_SCL A5

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo wheelOne;
Servo wheelTwo;
Servo wheelThree;
int thetaPin = 7;
int forwardPin = 8;
int sidePin = 9;

float m1;
float m2;
float m3;

int ser1;
int ser2;
int ser3; 

float xVal;

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SDA);
  Wire.begin(I2C_SCL);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  //initialize wheels
  wheelOne.attach(2);
  wheelTwo.attach(3);
  wheelThree.attach(4);
  calib();
  delay(1000);

  sensorEvent();

  while(xVal>5 && xVal<355){
    rotateRight();
    sensorEvent();
    printXVal();
  }
  calib();
  delay(1000);
  Serial.print("done with setup");
}

void loop() {
  sensorEvent();
  printXVal();
  adjust();
}

void adjust(){
  while(xVal>3 && xVal<357){
  
  if(xVal>180)
    rotateRight(xVal);
  else
    rotateLeft(xVal);
    
  sensorEvent();
  printXVal();
  }
  calib();
  //delay(1000);
}

//generic rotation functions
void rotateRight(){
  wheelOne.writeMicroseconds(1300);
  wheelTwo.writeMicroseconds(1300);
  wheelThree.writeMicroseconds(1300);
  //delay(50);
  calib();
}

void rotateLeft(){
  wheelOne.writeMicroseconds(1700);
  wheelTwo.writeMicroseconds(1700);
  wheelThree.writeMicroseconds(1700);
  //delay(50);
  calib();
}

//rotation functions with speed
void rotateRight(float angle){
  float offset = 360-angle;
  int adj = (int)(map(offset, 3.0, 180.0, 1450.0, 1350.0));
  wheelOne.writeMicroseconds(adj);
  wheelTwo.writeMicroseconds(adj);
  wheelThree.writeMicroseconds(adj);
//  Serial.print("adjust");
//  Serial.println(adj);
  delay(10);
  calib();
}

void rotateLeft(float angle){
  float offset = angle;
  int adj = (int)(map(offset, 3.0, 180.0, 1560.0, 1660.0));
  wheelOne.writeMicroseconds(adj);
  wheelTwo.writeMicroseconds(adj);
  wheelThree.writeMicroseconds(adj);
//  Serial.print("adjust");
//  Serial.println(adj);
  delay(10);
  calib();
}

void calib(){
  wheelOne.writeMicroseconds(1500);
  wheelTwo.writeMicroseconds(1500);
  wheelThree.writeMicroseconds(1500);
}

void printSer(){
  Serial.print("Ser1: " );
  Serial.println(ser1);
  Serial.print("Ser2: " );
  Serial.println(ser2);
  Serial.print("Ser3: " );
  Serial.println(ser3);
}

/* Get a new sensor event */ 
void sensorEvent(){
  sensors_event_t event; 
  bno.getEvent(&event);
  xVal = event.orientation.x;
}

/* Display sensor floating point data */
void printXVal(){
  Serial.print("xVal: " );
  Serial.println(xVal, 4);
}

//takes value v in range [a,b] and maps to corresponding value in range [c,d]
float map(float v, float a, float b, float c, float d){
  return (v-a) * (d-c) / (b-a) + c;
}
