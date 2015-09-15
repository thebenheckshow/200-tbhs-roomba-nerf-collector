
#include <SPI.h>  
#include <Pixy.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *motor_01 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *motor_02 = AFMS.getStepper(200, 2);

Pixy pixy;

int deadZone = 15;

void setup()
{
  Serial.begin(9600);
  Serial.println("PIXY GO START!");
  
  AFMS.begin(12000);  // create with the default frequency 1.6KHz
  pixy.init();
}

void loop()
{ 

  if (pixy.getBlocks()) {
    
    if (pixy.blocks[0].signature == 1) {
      
      int xx = pixy.blocks[0].x;
     
      if (xx < (160 - deadZone)) {
        motor_01->step(1, FORWARD, SINGLE);  
      }
      if (xx > (160 + deadZone)) {
        motor_01->step(1, BACKWARD, SINGLE);  
      }     
      if (xx > (159 - deadZone) and xx < (161 + deadZone)) {
        motor_01->release(); 
      }      
  
    }
    else {
      motor_01->release(); 
    }    
  }  
}

void testLoop() {

  if (pixy.getBlocks())
  {        
    if (pixy.blocks[0].signature == 1); { 
      Serial.print(pixy.blocks[0].x);
      Serial.print("\t");
      Serial.println(pixy.blocks[0].y);    
    }      
  }  
  
  
}



