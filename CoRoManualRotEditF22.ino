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

//Initialize motors
#define motorPin1L 2
#define motorPin1R 3
#define motorPin2L 5
#define motorPin2R 6
#define motorPin3L A7
#define motorPin3R A6

//Initialize input pins
int thetaPin = 10;
int forwardPin = 7;
int sidePin = 12;

//Initialize kiwi drive variables
int ser1;
int ser2;
int ser3; 
int an1L;
int an1R;
int an2L;
int an2R;
int an3L;
int an3R;

//Initialize x-orientation variables
float yaw;  //Absolute x-orientation value from BNO
int error;  //Error relative to setpoint
float setpoint = 180; //Absolute orientation goal of robot

//Initialize derivative time values
float error_before;
long time_before = micros();
int error_derivative; 

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
  pinMode(motorPin1L, OUTPUT);
  pinMode(motorPin1R, OUTPUT);
  pinMode(motorPin2L, OUTPUT);
  pinMode(motorPin2R, OUTPUT);
  pinMode(motorPin3L, OUTPUT);
  pinMode(motorPin3R, OUTPUT);
  calib();
  delay(5000);

  analogWrite(motorPin1L, 255);
  analogWrite(motorPin1R, 0);
  analogWrite(motorPin2L, 255);
  analogWrite(motorPin2R, 0);
  delay(1000);

  analogWrite(motorPin1L, 0);
  analogWrite(motorPin1R, 255);
  analogWrite(motorPin2L, 0);
  analogWrite(motorPin2R, 255);
  delay(1000);

//Initialize remote signals
  pinMode(forwardPin, INPUT); 
  pinMode(sidePin, INPUT);
  pinMode(thetaPin, INPUT);

  Serial.println("Done with setup");
}

void loop() {
  long thetaDuration = pulseIn(thetaPin, HIGH);  //Read PWM for theta input
  float thetaVal = map((float)thetaDuration, 1750.0, 950.0, -0.5, 0.5);
   
  long fwdDuration = pulseIn(forwardPin, HIGH);  //Read PWM for fwd input
  float fwdVal = map((float)fwdDuration, 1000.0, 1980.0, -1.0, 1.0);

  long sideDuration = pulseIn(sidePin, HIGH);  //Read PWM for side input
  float sideVal = map((float)sideDuration, 2030.0, 1080.0, -1.0, 1.0);
  Serial.println(thetaDuration);
  Serial.println(fwdDuration);
  Serial.println(sideDuration);
  // Serial.println(thetaVal);
  // Serial.println(fwdVal);
  // Serial.println(sideVal);

  if(thetaDuration<1350 || thetaDuration>1550){   //If driver is manually changing theta input
    delay(100);
    sensorEvent();
    setpoint = yaw;   //Adjusts robot setpoint
  }

  //Implements goal-seeking  
  sensorEvent();  //Reads x-orientation from BNO
  adjust();   //Corrects robot orientation with proportional adjustment
  updateDer();    //Updates error_derivative value
  printYaw();
  
  kiwiFunct(fwdVal, sideVal, thetaVal);   //Calculate servo values with kiwi drive math


  ser1+=error;  //Corrects servo values proportionally
  ser2+=error;
  ser3+=error;

  ser1+=error_derivative;   //Dampens adjustment of servo values
  ser2+=error_derivative;
  ser3+=error_derivative;
  printSer();
  
  convertServoToAnalog();
  
  analogWrite(motorPin1L, an1L);
  analogWrite(motorPin1R, an1R);
  analogWrite(motorPin2L, an2L);
  analogWrite(motorPin2R, an2R);
  analogWrite(motorPin3L, an3L);
  analogWrite(motorPin3R, an3R);
  
  printAn();

  delay(10);
}

//Cacluates error relative to robot setpoint
void calcError(){ 
  if(setpoint<=180){ //Calculates error for setpoint less than 180 degrees
    if (yaw>setpoint && yaw<setpoint+180) 
      error = rotateLeft(yaw-setpoint);
    else if(yaw>0 && yaw<setpoint)
      error = rotateRight(setpoint-yaw);
    else if(yaw>setpoint+180 && yaw<360)
      error = rotateRight(360-yaw+setpoint);
  }
  else if(setpoint>180){ //Calculates error for setpoint greater than 180 degrees
    if (yaw>setpoint-180 && yaw<setpoint) 
      error = rotateRight(setpoint-yaw);
    else if(yaw>setpoint && yaw<360)
      error = rotateLeft(yaw-setpoint);
    else if(yaw>0 && yaw<setpoint-180)
      error = rotateLeft(360-setpoint+yaw);
  }
}


//Reorients robot to face forward while driving
void adjust(){
  if(yaw!=setpoint){
    calcError();
  }
}

//Calculates derivative value in PID to dampen adjustment
void updateDer(){
  float error_now = error;   //Records current robot orientation error
  float error_delta = error_now - error_before;  //Calculates change in error

  long time_now = micros();
  long time_delta = time_now - time_before;   //Calculates change in time

  error_derivative = (int)(d_constant * error_delta / (time_delta / 100000.0));  //Calculates error derivative
  
  error_before = error_now;   //Updates current error value
  time_before = time_now;   //Updates current time value
  }

//Rotation functions with goal-seeking
int rotateRight(float angle){
  float offset = angle;
  return (int)(-1 * p_constant * offset);   //Return scaled value for rightward adjustment
}

int rotateLeft(float angle){
  float offset = angle;
  return (int)(p_constant * offset);  //Return scaled value for leftward adjustment
}

//Stops robot
void calib(){
  analogWrite(motorPin1L, 0);
  analogWrite(motorPin1R, 0);
  analogWrite(motorPin2L, 0);
  analogWrite(motorPin2R, 0);
  analogWrite(motorPin3L, 0);
  analogWrite(motorPin3R, 0);
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

void printAn(){
  Serial.print("An1L: " );
  Serial.print(an1L);
  Serial.print("\tAn1R: " );
  Serial.print(an1R);

  Serial.print("\tAn2L: " );
  Serial.print(an2L);
  Serial.print("\tAn2R: " );
  Serial.print(an2R);

  Serial.print("\tAn3L: " );
  Serial.println(an3L);
  Serial.print("\tAn3R: " );
  Serial.println(an3R);
}

//Calculates servo values
void kiwiFunct(float fwdVal, float sideVal, float thetaVal){
  float m1 = .5 * sideVal -sqrt(3)/2 * fwdVal + thetaVal;
  float m2 = .5 * sideVal + sqrt(3)/2 * fwdVal + thetaVal;
  float m3 = -1 * sideVal + thetaVal;

  float avg1 = (m1)/3.0;
  float avg2 = (m2)/3.0;
  float avg3 = (m3)/3.0;
  
  ser1 = (int)(map(avg1, -1, 1, 0, 3000));
  ser2 = (int)(map(avg2, -1, 1, 000, 3000));
  ser3 = (int)(map(avg3, -1, 1, 000, 3000));
}

//Calculates analog values from servo values
void convertServoToAnalog(){
  if (ser1<1500){
    an1L = (int)(map((float)ser1, 1400, 1500, 255, 0));
    an1R = 0;
  }
  else {
    an1L = 0;
    an1R = (int)(map((float)ser1, 1500, 1600, 0, 255));
  }

  if (ser2<1500){
    an2L = (int)(map((float)ser2, 1400, 1500, 255, 0));
    an2R = 0;
  }
  else {
    an2L = 0;
    an2R = (int)(map((float)ser2, 1500, 1600, 0, 255));
  }

  if (ser3<1500){
    an3L = (int)(map((float)ser3, 1400, 1500, 255, 0));
    an3R = 0;
  }
  else {
    an3L = 0;
    an3R = (int)(map((float)ser3, 1500, 1600, 0, 255));
  }
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
