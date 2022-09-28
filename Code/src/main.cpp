/**
 * Pin ports
 * Left Reflector: 21
 * Right Reflector: 22
 * 
 * Echo: 19
 * Trigger: 18
 * 
 */
#include <Arduino.h>
#include <Romi32U4.h>
#include <Chassis.h>
#include <BlueMotor.h>
void lineFollowDistance(int speed, float distance, float kp);
void lineFollow(int speed, float kp);
boolean lineFound();
boolean crossFound();
boolean distanceFromObject(float distance);
float getDistance();
void blueDriveTo(long target);

BlueMotor motor;
Chassis chassis;
int leftReflecentance = 21;
int rightReflecentance = 22;
int echo = 5;
int trigger = 12;
float cmPerEncoderTick = (7 * 3.14)/ 1440;
void setup()
{
  Serial.begin(9600);
  chassis.init();
  motor.setup();
  motor.reset();
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
}

void loop()
{
 motor.moveTo(0);
}



boolean distanceFromObject(float distance){
  if(getDistance() <= distance){
    return true;
  }
  else{
    return false;
  }
}

float getDistance(){
digitalWrite(trigger, LOW);
delayMicroseconds(2);
digitalWrite(trigger, HIGH);
delayMicroseconds(10);
digitalWrite(trigger, LOW);
float duration = pulseIn(echo, HIGH);
float currDistance = (duration/2) / 29.1;
return currDistance;
}


/**
 * @brief Line follows for distance given in cm
 * 
 * @param speed 
 * @param distance 
 * @param kp 
 */
void lineFollowDistance(int speed, float distance, float kp)
{
  float startEncoder = chassis.getRightEncoderCount();
  float distanceInTicks = distance / cmPerEncoderTick;
  while(abs(chassis.getRightEncoderCount() - startEncoder) < abs(distanceInTicks))
  {
  float left = analogRead(leftReflecentance);
  float right = analogRead(rightReflecentance);
  float error = left - right;
  float turn = error * kp;
  chassis.setMotorEfforts(speed + turn, speed - turn);
  }
  chassis.setMotorEfforts(0, 0);
}

/**
 * @brief Line follows forever
 * 
 * @param speed 
 * @param kp 
 */
void lineFollow(int speed, float kp)
{
  float left = analogRead(leftReflecentance);
  float right = analogRead(rightReflecentance);
  float error = left - right;
  float turn = error * kp;
  chassis.setMotorEfforts(speed + turn, speed - turn);
}

/**
 * @brief Return true if line is found
 * 
 * @return boolean
 */
boolean lineFound(){
  float right = analogRead(leftReflecentance);
  if(right > 150){
    return true;
  }
  else{
    return false;
  }
}

/**
 * @brief Returns true if cross is found
 * 
 * @return boolean 
 */
boolean crossFound(){
  float right = analogRead(rightReflecentance);
  float left = analogRead(leftReflecentance);
  if(left > 500 && right > 500){
    return true;
  }
  else{
    return false;
  }
}



