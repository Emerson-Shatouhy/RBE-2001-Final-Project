#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort);
    void setEffortNoDeadband(int effort);
    void moveTo(long position, boolean (*)());
    long getPosition();
    void reset();
    void setup();   

private:
    void setEffort(int effort, bool clockwise);
    static void isrA();
    static void isrB();
    const int tolerance = 3;
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    static const int ENCA = 0;
    static const int ENCB = 1;
};