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
unsigned long previousMillis = 0; 
void lineFollowDistance(int speed, float distance, float kp);
void lineFollow(int speed, float kp);
void liftArm(int pos);
void stopRobot();
void pickPiece(int pos);
void handleKeyPress(int keyPress);
void checkRemote();
boolean lineFound(boolean (*function)());
boolean crossFound(boolean (*function)());
boolean distanceFromObject(float distance, boolean (*function)());
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
int leftReflecentance = 22;
int rightReflecentance = 21;
enum stateChoices
{
  STARTUP,
  READY,
  PLACE45,
  PICKUP45,
  PLACE25,
  PICKUP25,
  RETURNHOME,
  STOP,
  TEST
} romiState,
    nextRomiState;
int echo = 17;
int trigger = 12;
float cmPerEncoderTick = (7 * 3.14) / 1440;
float forwardSpeed = 0;
float turnSpeed = 0;

void setup()
{
  Serial.begin(9600);
  romiState = STARTUP;
  chassis.init();
  decoder.init();
  motor.setup();
  zeroArm();
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  servo.setMinMaxMicroseconds(750, 1600);
  romiState = READY;
}

boolean remoteCheck()
{
  checkRemote();
  return (romiState == STOP);
}

void loop()
{
  checkRemote();
  switch (romiState)
  {
  case STOP:
    stopRobot();
    break;
  case PLACE45:
    pickPiece(0);
    chassis.driveFor(-6, 100, true, remoteCheck);
    chassis.turnFor(-90, 100, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(50, -50);
    }
    while (!crossFound(remoteCheck))
    {
      lineFollow(100, 0.1);
    }
    chassis.driveFor(7.5, 50, true, remoteCheck);
    chassis.turnFor(-45, 50, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(50, -50);
    }
    while (!distanceFromObject(10, remoteCheck))
    {
      lineFollow(100, 0.1);
    }
    chassis.setMotorEfforts(0, 0);
    delay(1000);
    placePiece(1);
    chassis.driveFor(-5, 50, true, remoteCheck);
    romiState = STOP;
    break;
  case PICKUP45:
    motor.moveTo(3553, remoteCheck);
    servo.writeMicroseconds(1500);
    while (!distanceFromObject(10, remoteCheck))
    {
      lineFollow(100, 0.1);
    }
    chassis.setMotorEfforts(0, 0);
    pickPiece(1);
    while (getDistance() < 17)
    {
      chassis.setMotorEfforts(-50, -50);
    }
    chassis.setMotorEfforts(0, 0);
    chassis.turnFor(-90, 50, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(50, -50);
    }
    while (!crossFound(remoteCheck))
    {
      lineFollow(50, 0.1);
    }
    chassis.driveFor(7.5, 50, true, remoteCheck);
    chassis.turnFor(45, 40, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(-50, 50);
    }
    while (!distanceFromObject(9, remoteCheck))
    {
      lineFollow(50, 0.1);
    }
    chassis.setMotorEfforts(0, 0);
    placePiece(0);
    romiState = STOP;
    break;
  case PLACE25:
    pickPiece(0);
    chassis.driveFor(-6, 100, true, remoteCheck);
    chassis.turnFor(-90, 100, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(50, -50);
    }
    while (!crossFound(remoteCheck))
    {
      lineFollow(100, 0.1);
    }
    chassis.driveFor(7.5, 50, true, remoteCheck);
    chassis.turnFor(45, 50, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(-50, 50);
    }
    while (!distanceFromObject(10, remoteCheck))
    {
      lineFollow(100, 0.1);
    }
    chassis.setMotorEfforts(0, 0);
    placePiece(2);
    chassis.driveFor(-5, 420, true, remoteCheck);
    romiState = STOP;
    break;
  case PICKUP25:
    motor.moveTo(5100, remoteCheck);
    servo.writeMicroseconds(1500);
    while (!distanceFromObject(12, remoteCheck))
    {
      lineFollow(100, 0.1);
    }
    chassis.setMotorEfforts(0, 0);
    pickPiece(1);
    while (getDistance() < 20)
    {
      chassis.setMotorEfforts(-50, -50);
    }
    chassis.setMotorEfforts(0, 0);
    chassis.turnFor(-90, 50, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(50, -50);
    }
    while (!crossFound(remoteCheck))
    {
      lineFollow(50, 0.1);
    }
    chassis.driveFor(7.5, 40, true, remoteCheck);
    chassis.turnFor(-45, 40, true, remoteCheck);
    while (!lineFound(remoteCheck))
    {
      chassis.setMotorEfforts(50, -50);
    }
    while (!distanceFromObject(10, remoteCheck))
    {
      lineFollow(50, 0.1);
    }
    chassis.setMotorEfforts(0, 0);
    placePiece(0);
    romiState = STOP;
    break;

    case TEST:
    pickPiece(1);
    romiState = STOP;
    break;
  }
}




/**
 * @brief E STOP for the robot
 *
 */
void stopRobot()
{
  chassis.setWheelSpeeds(0, 0);
  motor.setEffort(0);
}

/**
 * @brief State machine to place collector
 *
 * @param int
 *    0 = Station
 *    1 = 45 degree
 *    2 = 25 degree
 * 3 = Stop
 */
void placePiece(int pos)
{
  if (remoteCheck())
  {
    pos = 3;
  }
  chassis.setMotorEfforts(0, 0);
  switch (pos)
  {
  case 0:
    motor.moveTo(498, remoteCheck);
    servo.writeMicroseconds(1500);
    motor.moveTo(350, remoteCheck);
    chassis.driveFor(-10, 100, true, remoteCheck);
    break;
  case 1:
    motor.moveTo(4000, remoteCheck);
    servo.writeMicroseconds(1500);
    chassis.driveFor(-10, 100, true, remoteCheck);
    break;
  case 2:
    motor.moveTo(5751, remoteCheck);
    servo.writeMicroseconds(1500);
    chassis.setMotorEfforts(-100, -100);
    chassis.setMotorEfforts(0, 0);
    break;
  case 3:
    break;
  }
}

/**
 * @brief State machine to pickup collector
 *
 * @param int pos
 *    0 = Station
 *    1 = 45 degree
 *    2 = 25 degree
 *   3 = Stop

 */

void pickPiece(int pos)
{
  if (remoteCheck())
  {
    pos = 3;
  }
  switch (pos)
  {
  case 0:
    safeCloseServo(1);
    motor.moveTo(5400, remoteCheck);
    break;
  case 1:
    servo.writeMicroseconds(1500);
    motor.moveTo(3553, remoteCheck);
    safeCloseServo(1);
    motor.moveTo(4500, remoteCheck);
    chassis.setMotorEfforts(-100, -100);
    chassis.setMotorEfforts(0, 0);
    break;
  case 2:
    motor.moveTo(4700, remoteCheck);
    safeCloseServo(1);
    motor.moveTo(5100, remoteCheck);
    break;
  case 3:
    break;
  }
}

/**
 * @brief Zeros the arm to the bottom
 *
 * @return boolean
 */
boolean zeroArm()
{
  motor.setEffort(200);
  long oldPos = 0;
  delay(500);
  while (motor.getPosition() != oldPos)
  {
    oldPos = motor.getPosition();
    delay(250);
  }
  motor.setEffort(-400);
  delay(100);
  motor.setEffort(0);
  motor.reset();
  servo.writeMicroseconds(1600);
  return true;
}

/**
 * @brief Closes servo safely
 *
 * @param robot robot num
 */
void safeCloseServo(int robot)
{
  switch (robot)
  {
  case 1:
    servo.writeMicroseconds(790);
    delay(350);
    if (analogRead(A0) > 180)
    {

      servo.writeMicroseconds(1600);
    }
    break;
  case 2:
    int last = 0;
    int current = analogRead(A0);
    servo.writeMicroseconds(1750);
    while (current < 905)
    {

      if ((current - last) / 2 < 2)
      {
        servo.writeMicroseconds(1500);
        break;
      }
      else
      {
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
boolean distanceFromObject(float distance, boolean (*function)())
{
  if ((*function)())
  {
    stopRobot();
    return true;
  }
  if (getDistance() <= distance || (*function)())
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Get distance from ultrasonic sensor
 *
 * @param speed
 * @param kp
 */
float getDistance()
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  float duration = pulseIn(echo, HIGH);
  float currDistance = (duration / 2) / 29.1;
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
  while (abs(chassis.getRightEncoderCount() - startEncoder) < abs(distanceInTicks))
  {

    lineFollow(speed, kp);
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
  if (speed > 0)
  {
    float left = analogRead(leftReflecentance);
    float right = analogRead(rightReflecentance);
    float error = right - left;
    float turn = error * kp;
    chassis.setMotorEfforts(speed + turn, speed - turn);
  }
  else
  {
    float left = analogRead(leftReflecentance);
    float right = analogRead(rightReflecentance);
    float error = left - right;
    float turn = error * kp;
    chassis.setMotorEfforts(speed + turn, speed - turn);
  }
}

/**
 * @brief Return true if line is found
 *
 * @return boolean
 */
boolean lineFound(boolean (*function)())
{
  if ((*function)())
  {
    stopRobot();
    return true;
  }
  float right = analogRead(leftReflecentance);
  if (right > 300 || !(*function)())
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Returns true if cross is found
 *
 * @return boolean
 */
boolean crossFound(boolean (*function)())
{
  if ((*function)())
  {
    stopRobot();
    return true;
  }
  float right = analogRead(rightReflecentance);
  float left = analogRead(leftReflecentance);
  if (left > 600 && right > 600 || !(*function)())
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Checks remote for button press
 *
 */
void checkRemote()
{
  int keyPress = decoder.getKeyCode();
  if (keyPress != -1)
    handleKeyPress(keyPress);
}

/**
 * @brief Handles IR Remote Inputs
 *
 * @param keyPress
 */
void handleKeyPress(int keyPress)
{
  Serial.println(keyPress);
  if (keyPress == remotePlayPause) // Pause / Resume
  {
    if(romiState != STOP){
    nextRomiState = romiState;
    romiState = STOP;
    } else {
      romiState = nextRomiState;
    }
  }
  if (keyPress == remote1) // Something
  {
    romiState = PLACE45;
  }
  if (keyPress == remote7) // Something
  {
    romiState = PICKUP45;
  }
  if (keyPress == remote2) // Something
  {
    romiState = PLACE25;
  }
  if (keyPress == remote8) // Something
  {
    romiState = PICKUP25;
  }
  if (keyPress == remote0)
  {
    zeroArm();
  }
  if (keyPress == remote3)
  {
    romiState = TEST;
  }
}
