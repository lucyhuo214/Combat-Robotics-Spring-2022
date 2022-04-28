/* This Arduino script enables a kiwi function drive system that is moderated by closed loop control using a BNO055 accelerometer. 
It enables the robot to always reorient itself to a set absolute orientation value. 
This was used as a proof-of-concept to show the goal-seeking ability of the robot using PID control. */

//Libraries
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h> 

//Initialize BNO sensor
#define I2C_SDA A4
#define I2C_SCL A5

Adafruit_BNO055 bno = Adafruit_BNO055(55);

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

//Initialize x-orientation variables
float yaw;
int angleOffset;

//Initialize derivative time values
float angle_before;
long time_before = micros();
int angle_derivative; 

//Initialize PID control constants
float p_constant = 1.5;
float d_constant = 3;

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

//Initialize wheels
  wheelOne.attach(2);
  wheelTwo.attach(3);
  wheelThree.attach(4);
  calib();
  delay(3000);

//Initialize remote signals
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


  //Implements goal-seeking  
  sensorEvent();  //Reads x-orientation from BNO
  adjust();   //Corrects robot orientation with proportional adjustment
  updateDer();    //Updates angle_derivative value
  
  kiwiFunct(fwdVal, sideVal, thetaVal);   //Calculate servo values with kiwi drive math


  ser1+=angleOffset;  //Corrects servo values proportionally
  ser2+=angleOffset;
  ser3+=angleOffset;

  ser1+=angle_derivative;   //Dampens adjustment of servo values
  ser2+=angle_derivative;
  ser3+=angle_derivative;
  
  wheelOne.writeMicroseconds(ser1);   //Write values to drive servos
  wheelTwo.writeMicroseconds(ser2);
  wheelThree.writeMicroseconds(ser3);
  delay(10);
}


//Reorients robot to face forward while driving
void adjust(){
  if(yaw!=180){
    if(yaw<180) angleOffset = rotateRight(yaw);   //Rotates right if robot is shifted left
    else angleOffset = rotateLeft(yaw);  //Rotates left if robot is shifted right
  }
}

//Calculates derivative value in PID to dampen adjustment
void updateDer(){
  float angle_now = yaw;   //Records current x-orientation value
  
  float angle_delta = angle_now - angle_before;  //Calculates change in angle

  long time_now = micros();
  long time_delta = time_now - time_before;   //Calculates change in time

  angle_derivative = (int)(d_constant * angle_delta / (time_delta / 100000.0));  //Calculates angle derivative
  
  angle_before = angle_now;   //Updates current angle value
  time_before = time_now;   //Updates current time value
  }

//Rotation functions with goal-seeking
int rotateRight(float angle){
  float offset = 180-angle;
  return (int)(-1 * p_constant * offset);   //Return scaled value for rightward adjustment
}

int rotateLeft(float angle){
  float offset = angle-180;
  return (int)(p_constant * offset);  //Return scaled value for leftward adjustment
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

/* Get a new sensor event */ 
void sensorEvent(){
  sensors_event_t event;   //Reads absolute x-orientation from BNO
  bno.getEvent(&event);
  yaw = event.orientation.x;   //Updates yaw value
}

/* Display sensor floating point data */
void printYaw(){
  Serial.print("\tyaw:" );
  Serial.println(yaw, 4);
}

//Takes value v in range [a,b] and maps to corresponding value in range [c,d]
float map(float v, float a, float b, float c, float d){
  return (v-a) * (d-c) / (b-a) + c;
}
