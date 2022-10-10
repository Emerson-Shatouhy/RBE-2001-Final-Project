#include <Arduino.h>
#include <Chassis.h>
#include <Romi32U4.h>
#include <RemoteConstants.h>
#include <IRdecoder.h>

int irRemotePin = 14;

enum stateChoices{FOLLOWLINE, TURNTOLINE, MOVETOCOUNT300, STOP} romiState, nextRomiState;

int leftReflectance = 20;   //Left analog input pin
int rightReflectance = 22;  //Right analog input pin
float Kp = 0.05;            //Line following proportionality constant
float forwardWheels = 8.0;  //Speed when moving forward (cm/s)
float turnWheels  = 4.0;    //Speed when turning (cm/s)
float stopWheels = 0.0;     //Speed when stopped (cm/s)
int RR = 200; //Right reflectance sensor ADC value
int LR = 200; //Left reflectance sensor ADC value
int16_t leftEncoderValue;


Chassis chassis;
Romi32U4ButtonB buttonB;
IRDecoder decoder(irRemotePin);

void setup()
{
  chassis.init();
  decoder.init();
  romiState = FOLLOWLINE; //Initial State
  Serial.begin(9600);
  // Wait for the user to press button B.
  buttonB.waitForButton();
  Serial.println("Button B Pressed");
   
  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(2000);
}

// Handles a key press on the IR remote
void handleKeyPress(int keyPress)
{
  
  if (keyPress == remotePlayPause)  //This is the emergency stop button
  {
    nextRomiState = romiState; //Save the current state so you can pick up where you left off
    romiState = STOP;
    Serial.println("Emergency Stop");
  }

  if (keyPress == remoteUp) //This is the proceed button
  {
    romiState = nextRomiState;
    Serial.println("Proceed");
  }

}
void loop()
{
  // Check for a key press on the remote
  //int16_t keyPress = decoder.getKeyCode();  //true allows the key to repeat if held down
  //Sometimes int16_t is used instead of int to make it cross platform. On the arduino an 
  //int is 16 bits, but on another computer it might be 32 bits.
  int keyPress = decoder.getKeyCode();
  if(keyPress != -1) handleKeyPress(keyPress); //If it returns -1 then no button has been pressed
  
 switch(romiState){
    case FOLLOWLINE:
       
      RR = analogRead(rightReflectance);
      LR = analogRead(leftReflectance);
      chassis.setTwist(forwardWheels,Kp*(LR-RR));
        
      if(RR<70 && LR<70){   //True if the Romi has driven past the end of the black line
        chassis.setWheelSpeeds(stopWheels, stopWheels); 
        nextRomiState = MOVETOCOUNT300;
        romiState = STOP;
        Serial.println("Stop");
        chassis.getLeftEncoderCount(true);//the true sets the count to zero, but returns the previous
        //value so we call it with the true parameter first, and then call it again to get the reset value.
        leftEncoderValue = chassis.getLeftEncoderCount();
        Serial.print("leftEncoderValue =  ");
        Serial.println(leftEncoderValue);
        Serial.println("End of FOLLOWLINE");
      }
    break;

    case MOVETOCOUNT300:

      chassis.setWheelSpeeds(forwardWheels/2, forwardWheels/2);  //Move forward at half speed for 300 counts (1440 per rev).
      //This will be about 1/5 of a revolution.  Do this so that you don't pick up the end of the line when you start the turn.
      leftEncoderValue = chassis.getLeftEncoderCount();
      if(leftEncoderValue > 300){
        chassis.setWheelSpeeds(stopWheels, stopWheels);
        nextRomiState = TURNTOLINE;
        romiState = STOP;
        Serial.println("Stop");
        Serial.print("leftEncoderValue =  "); //Check the encoder count
        Serial.println(leftEncoderValue);
        Serial.println("End of MOVETOCOUNT300");
      }
    break;

    case TURNTOLINE:
      
      RR = analogRead(rightReflectance);
      LR = analogRead(leftReflectance);
      chassis.setWheelSpeeds(-turnWheels, turnWheels);
      if(RR>200 || LR>200){   //Check to see if you have picked up the line
          nextRomiState = FOLLOWLINE;
          romiState = STOP;
          Serial.println("Stop");
          Serial.println("End of TURNTOLINE");
      }
    break;

    case STOP:  //Remain stopped until the proceed remote button moves your Romi to the next state

      chassis.setWheelSpeeds(stopWheels, stopWheels);        
  
    break;

  }
 
}