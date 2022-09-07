#include <Romi32U4.h>
#include <Chassis.h>

Chassis chassis;

void setup() {
  Serial.begin(9600);
  chassis.init();
}

void loop(){

}