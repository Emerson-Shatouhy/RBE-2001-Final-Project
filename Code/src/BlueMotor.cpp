#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

long oldValue = 0;
long newValue;
long count = 0;
unsigned time = 0;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}

void BlueMotor::isrA()
{
  if (digitalRead(ENCB) == digitalRead(ENCA)) {
    count++;
  }
  else {
    count--;
  }
}
void BlueMotor::isrB()
{
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    count--;
  }
  else {
    count++;
  }
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::setEffortNoDeadband(int effort){
  effort *= .625;
  if(effort == 0){
    setEffort(0);
  } else if(effort < 0){
    setEffort(effort - 150);
  } else {
    setEffort(effort + 140);
  }
}

void BlueMotor::moveTo(long pos)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
  if(pos < 0){
    return;
  }
  float kp = 1.5;
  float error = (pos - getPosition() ) * kp;
  while(abs(error) > 10){ 
    if(error < 400) 
  setEffort(-1*error*4);
  else 
  setEffort(-400 * kp);
  error = (pos - getPosition())*kp ;
}
setEffort(0);
}

