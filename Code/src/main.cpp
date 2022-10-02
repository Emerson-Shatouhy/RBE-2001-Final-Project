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
#include <servo32u4.h>
#include <IRdecoder.h>
#include <RemoteConstants.h>

void lineFollowDistance(int speed, float distance, float kp);
void lineFollow(int speed, float kp);
void liftArm(int pos);
boolean lineFound();
boolean crossFound();
boolean distanceFromObject(float distance);
boolean servoOpenMax();
boolean zeroArm();
void placePiece(int pos);
float getDistance();
void blueDriveTo(long target);
void safeCloseServo(int robot);
int irRemotePin = 14;
BlueMotor motor;
Chassis chassis;
Romi32U4ButtonB buttonB;
Romi32U4ButtonA buttonA;
IRDecoder decoder(irRemotePin);
Servo32U4 servo;
int leftReflecentance = 21;
int rightReflecentance = 22;

int echo = 17;
int trigger = 12;
float cmPerEncoderTick = (7 * 3.14)/ 1440;
float forwardSpeed = 0;
float turnSpeed = 0;

void setup()
{
  Serial.begin(9600);
  chassis.init();
  decoder.init();
  motor.setup();
  zeroArm();
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  servo.setMinMaxMicroseconds(750, 1600);
}


void loop()
{
  if(Serial){
  delay(100); 
  
  }
}

//45 degree = 4500

void placePiece(int pos){
  switch(pos){
    case 0:
    motor.moveTo(498);
delay(5000);
servo.writeMicroseconds(1500);
delay(5000);
motor.moveTo(350);
delay(5000);
chassis.setMotorEfforts(-100, -100);
delay(5000);
chassis.setMotorEfforts(0, 0);
break;
    case 1:
    safeCloseServo(1);
    motor.moveTo(4300);
    servo.writeMicroseconds(1500);
     delay(1000);
    chassis.setMotorEfforts(-100, -100);
    delay(1000);
    chassis.setMotorEfforts(0, 0);
    break;
    case 2:
        safeCloseServo(1);
motor.moveTo(5751);
servo.writeMicroseconds(1500);
delay(1000);
chassis.setMotorEfforts(-100, -100);
delay(1000);
chassis.setMotorEfforts(0, 0);
break;
  }
}

void pickPiece(int pos){
  switch(pos){
        case 0: 
    safeCloseServo(1);
    motor.moveTo(4000);
    break;
    case 1:
    servo.writeMicroseconds(1500);
    motor.moveTo(3853);
    delay(2000);
    safeCloseServo(1);
    motor.moveTo(4500);
    delay(2000);
    chassis.setMotorEfforts(-100, -100);
    delay(1000);
    chassis.setMotorEfforts(0, 0);
    delay(1000);
    break;
    case 2:
    servo.writeMicroseconds(1600);
motor.moveTo(5791);
delay(1000);
safeCloseServo(1);
delay(1000);
motor.moveTo(5285);
safeCloseServo(1);
chassis.setMotorEfforts(-100, -100);
delay(1000);
chassis.setMotorEfforts(0, 0);
break;
  }
}
boolean zeroArm(){
    motor.setEffort(200);
    long oldPos = 0;
    delay(500);
    while(motor.getPosition() != oldPos){
        oldPos = motor.getPosition();
        delay(250);
    }
    motor.setEffort(-400);
    delay(50);
    motor.setEffort(0);
    delay(100);
    motor.reset();
}


//331 is close
//147 is open

void safeCloseServo(int robot){
  switch(robot){
    case 1:
      servo.writeMicroseconds(790);
      delay(350);
      if(analogRead(A0) > 170){
      servo.writeMicroseconds(1600);
      }
      break;
    case 2:
    int last = 0;
    int current = analogRead(A0);
    servo.writeMicroseconds(1750);
    while(current < 905){
      Serial.println(current);
      if((current - last)/2 < 2){
        Serial.println((current-last)/2);
        servo.writeMicroseconds(1500);
        Serial.println("FAIL");
        break;
      } else {
        last = current;
        current = analogRead(A0);
      }

    }
  }
  
}
/**
 * @brief Distance from object from ultrasonic sensor
 * 
 * @param distance 
 * @return boolean 
 */
boolean distanceFromObject(float distance){
  if(getDistance() <= distance){
    return true;
  }
  else{
    return false;
  }
}


/**
 * @brief Get distance from ultrasonic sensor
 * 
 * @param speed 
 * @param kp 
 */
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



