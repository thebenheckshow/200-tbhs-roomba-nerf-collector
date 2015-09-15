//==========================================================================
//
//  Pixy Pet Robot
//
//   Adafruit invests time and resources providing this open source code, 
//  please support Adafruit and open-source hardware by purchasing 
//  products from Adafruit!
//
// Written by: Bill Earl for Adafruit Industries
//
//==========================================================================
// begin license header
//
// All Pixy Pet source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
//
// end license header
//
//==========================================================================
//
// Portions of this code are derived from the Pixy CMUcam5 pantilt example code. 
//
//==========================================================================
#include <SoftwareSerial.h>
#include <SPI.h>  
#include <Pixy.h>
#include "iRobot_Create.h"

#define rxPin 10
#define txPin 9

#define start  128
#define reset  7
#define stop  173
#define safe  131
#define full  132

#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)

#define primary 0
#define secondary 1
#define dump 2

// set pin numbers:
const int buttonPin_0 = 2;     // the number of the pushbutton pin
const int buttonPin_1 = 3;     // the number of the pushbutton pin
const int buttonPin_2 = 5;     // the number of the pushbutton pin
const int buttonPin_3 = 6;     // the number of the pushbutton pin
int buttonState_0 = 0;         // variable for reading the pushbutton status
int buttonState_1 = 0;         // variable for reading the pushbutton status
int buttonState_2 = 0;         // variable for reading the pushbutton status
int buttonState_3 = 0;         // variable for reading the pushbutton status

// pixy variables
int deadZone = 30;
int turning = 0;

// vacuum and brush motor variables
signed short mainBrush = 0;
signed short sideBrush = 0;
signed short vacuum = 0;
int16_t brushCurrent = 0;
int lowSpeed = -30;
int midSpeed = -70;
int highSpeed = -100;
int setSpeed = 0;

// interrupt and button variables
// for keeping track of button presses
// also variables for handeling debounce
int action_0=0;
int action_1=0;
int action_2=0;
int action_3=0;
volatile int state_0 = LOW;
volatile int state_1 = LOW;
volatile int state_2 = LOW;
volatile int state_3 = LOW;
long debouncing_time = 40; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop
{
public:
   ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
   
   void update(int32_t error);
   
   int32_t m_pos;
   int32_t m_prevError;
   int32_t m_proportionalGain;
   int32_t m_derivativeGain;
};

// ServoLoop Constructor
ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
   m_pos = RCS_CENTER_POS;
   m_proportionalGain = proportionalGain;
   m_derivativeGain = derivativeGain;
   m_prevError = 0x80000000L;
}

// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
   long int velocity;
   char buf[32];
   if (m_prevError!=0x80000000)
   {	
      velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
      
      m_pos += velocity;
      if (m_pos>RCS_MAX_POS) 
      {
         m_pos = RCS_MAX_POS; 
      }
      else if (m_pos<RCS_MIN_POS) 
      {
         m_pos = RCS_MIN_POS;
      }
   }
   m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------



// setup pixy
Pixy pixy;


// pin assignments
int statusLED = 4;

//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup()
{
   // define pin modes for status led and direct device/baud:
   pinMode(statusLED, OUTPUT); 
   
   // set the data rate for the SoftwareSerial port, this is the
   // iRobot's default rate of 115200 baud:
   roombaSerial.begin(115200);
   
   // start hardware serial port 
   Serial.begin(115200);
   
   // define pin modes for software tx, rx pins:
   pinMode(rxPin, INPUT);
   pinMode(txPin, OUTPUT);
   
   Serial.print("Starting...\n");
   
   // initalize the pixy
   pixy.init();
   
   roombaMode(start);
   roombaMode(safe);
}

//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
uint32_t lastBlockTime = 0;
void loop()
{ 
   uint16_t blocks;
   blocks = pixy.getBlocks();
   
   // If we have blocks in sight, track and follow them
   if (blocks)
   {
      int trackedBlock = TrackBlock(blocks);
//       FollowBlock(trackedBlock);
      lastBlockTime = millis();
   }  
   else if (millis() - lastBlockTime > 100)
   {
      // motors.setLeftSpeed(0);
      // motors.setRightSpeed(0);
      ScanForBlocks();
   }
}

//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int oldX, oldY, oldSignature;
int TrackBlock(int blockCount)
{
   int trackedBlock = 0;
   long maxSize = 0;
   
   Serial.print("blocks =");
   Serial.println(blockCount);
   
   for (int i = 0; i < blockCount; i++)
   {
      if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
      {
         long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
         if (newSize > maxSize)
         {
            trackedBlock = i;
            maxSize = newSize;
         }
      }
   }
   
   int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
   int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
   
   panLoop.update(panError);
   tiltLoop.update(tiltError);
   
   pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
   
   oldX = pixy.blocks[trackedBlock].x;
   oldY = pixy.blocks[trackedBlock].y;
   oldSignature = pixy.blocks[trackedBlock].signature;
   return trackedBlock;
}

// //---------------------------------------
// // Follow blocks via the roomba drive
// //
// // This code makes the roomba base turn 
// // and move to follow the pan/tilt tracking
// // of the head.
// // This code was adapted from Zumo
// //---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
   int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?
   
   // Size is the area of the object.
   // We keep a running average of the last 8.
   size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
   size -= size >> 3;
   
   // Forward speed decreases as we approach the object (size is larger)
   int forwardSpeed = constrain(400 - (size/256), -100, 400);  
   
   // Steering differential is proportional to the error times the forward speed
   int32_t differential = (followError + (followError * forwardSpeed))>>8;
   
   // Adjust the left and right speeds by the steering differential.
   int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
   int rightSpeed = constrain(forwardSpeed - differential, -400, 400);
   
   // And set the motor speeds
//    motors.setLeftSpeed(leftSpeed);
//    motors.setRightSpeed(rightSpeed);
   directDrive(leftSpeed, rightSpeed);
}

//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;
void ScanForBlocks()
{
   if (millis() - lastMove > 20)
   {
      lastMove = millis();
      panLoop.m_pos += scanIncrement;
      if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
      {
         tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
         scanIncrement = -scanIncrement;
         if (scanIncrement < 0)
         {
//             motors.setLeftSpeed(-250);
//             motors.setRightSpeed(250);
         }
         else
         {
//             motors.setLeftSpeed(+180);
//             motors.setRightSpeed(-180);
         }
         delay(random(250, 500));
      }
      
      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
   }
}

// function to drive motors with opcode direct drive 145
// the function takes 2 parameters velocity of left motor and velocity of right motor range -500 to 500 mm/s
void directDrive(signed short leftV, signed short rightV) {
   roombaSerial.write(145); // Opcode number for DIRECT DRIVE
   roombaSerial.write(rightV >> 8); 
   roombaSerial.write(rightV & 0xFF);  
   roombaSerial.write(leftV >> 8); 
   roombaSerial.write(leftV & 0xFF); 
}

// extension of direct drive to halt roomba
void halt() {
   directDrive(0,0);
}

// extension of direct drive to turn roomba left
void left(signed short velocity) {
   directDrive(0-velocity,velocity);
}

// extension of direct drive to turn roomba right
void right(signed short velocity) {
   directDrive(velocity,0-velocity);
}

// extension of direct drive to prompt forward motion
void forward(signed short velocity) {
   directDrive(velocity, velocity);
}

// extension of direct drive to prompt reverse motion
void reverse(signed short velocity) {
   directDrive(0-velocity, 0-velocity);
}

// send mode request to roomba: start, reset, safe, full, stop
void roombaMode(signed short mode) {
   
   // light up status led   
   //digitalWrite(statusLED, HIGH); 
   
   // this instruction starts the OI, roomba must
   // always recieve the Start instruction before
   // it will acknowladge any other operations
   // roombaSerial.write(start);  // Start the roomba OI
   roombaSerial.write(mode);
   
   //blinkLED();
}

// run cleaning motors with pwm instruction
void cleaningMotors(signed short motor, signed short pulseWidthModulation) {
   //    this function takes 2 perameters.  
   //    first is which motor to actuate (primary, secondary, or dump)
   //    second is for setting the motor rate
   //       primary -127 to 127
   //       secondary -127 to 127
   //       dump 0 to 127
   // assign global variables
   switch (motor) {
      case primary:
         mainBrush = pulseWidthModulation;    // range from -127 to 127
         break;
      case secondary:
         sideBrush = pulseWidthModulation;    // range from -127 to 127
         break;
      case dump:
         vacuum = pulseWidthModulation;       // range from 0 to 127
         break;
      default: 
         break;
   }
   // send instructions
   roombaSerial.write(144);        // OI opcode for PWM Motors
   roombaSerial.write((int8_t)mainBrush);  
   roombaSerial.write((int8_t)sideBrush);
   roombaSerial.write((int8_t)vacuum);
   
}

// this test is to give the roomba instruction to look for dart, turn on motor and drive over it.
void  testVisionCollector() {
   
   // read the state of the pushbutton value:
   buttonState_0 = digitalRead(buttonPin_0);
   buttonState_1 = digitalRead(buttonPin_1);
   buttonState_2 = digitalRead(buttonPin_2);
   buttonState_3 = digitalRead(buttonPin_3);
   
   // check if the pushbutton is pressed.
   // if it is, the buttonState is HIGH:
   // if button is high and not moving forward move forward
   // if alreading moving forward stop
   if (buttonState_0 == HIGH) {
      digitalWrite(statusLED, HIGH); 
      
      if (action_0 == 0) {
         // blink the status green status led on microcontroller carrier board
         //blinkLED();
         
         // if button is pressed and action_0 is 0 then start the roomba in full
         // and make action_0 == 1
         // otherwise if action_0 is already 1 then stop the roomba
         roombaMode(start);
         roombaMode(full);
         //halt();
         action_0 = 1;
         action_1 = 0;
         action_2 = 0;
         action_3 = 0;
         setSpeed = 0;
      }
      else {
         halt();
         roombaMode(stop);
         action_0 = 0;
         action_1 = 0;
         action_2 = 0;
         action_3 = 0;
         setSpeed = 0;
      }
      
      while(digitalRead(buttonPin_0)) {
         delay(10);
      }   
      
      //delay(debouncing_time);
      
      digitalWrite(statusLED, LOW); 
      
   }
   
   // check if the pushbutton is pressed.
   // if it is, the buttonState is HIGH:
   // if button is high change brush speed
   // cycle through off low med high
   if (buttonState_1 == HIGH) {
      digitalWrite(statusLED, HIGH); 
      
      //if (action_1 == 0) {
      // blink the status green status led on microcontroller carrier board
      //blinkLED();
      
      // do somthing
      // the brus will have three speeds
      // the speeds are given by global variables: lowSpeed, midSpeed, highSpeed
      // with each button press action_1 starting from 0 is incrimented to 3
      // at 3 action_1 is reset to 0
      // a switch case checks what action_1 is and sets the primary motor speed
      if (setSpeed < 3){
         setSpeed++;
      }
      else {
         setSpeed = 0;
      }
      
      switch (setSpeed) {
         case 0:
            cleaningMotors(primary, 0);   
            break;
         case 1:
            cleaningMotors(primary, lowSpeed);  
            break;
         case 2:
            cleaningMotors(primary, midSpeed);  
            break;
         case 3:
            cleaningMotors(primary, highSpeed); 
            break;
         default: 
            break;
      }
      //  action_1 = 1;
      //       }
      //       else{
      //          action_1 = 0;
      //       }
      
      while(digitalRead(buttonPin_1)) {
         delay(10);
      }   
      
      //delay(debouncing_time);
      digitalWrite(statusLED, LOW); 
      
   }
   
   
   // check if the pushbutton is pressed.
   // if it is, the buttonState is HIGH:
   // if button is high and not moving in reverse move in reverse
   // if alreading moving in reverse stop
   if (buttonState_2 == HIGH) {
      digitalWrite(statusLED, HIGH); 
      
      if (action_2 != -1) {
         reverse(100);
         action_2 = -1;
      }
      else {
         halt();
         action_2 = 0;
      }
      while(digitalRead(buttonPin_2)) {
         delay(10);
      }
      digitalWrite(statusLED, LOW); 
      
   }   
   
   // check if the pushbutton is pressed.
   // if it is, the buttonState is HIGH:
   // if button is high and not moving forward move forward
   // if alreading moving forward stop
   if (buttonState_3 == HIGH) {
      digitalWrite(statusLED, HIGH); 
      
      
      if (action_2 != 1) {
         forward(100);
         action_2 = 1;
      }
      else {
         halt();
         action_2 = 0;
      }
      while(digitalRead(buttonPin_3)) {
         delay(10);
      }   
      
      //delay(debouncing_time);
      digitalWrite(statusLED, LOW); 
      
   }
   
//    if (pixy.getBlocks()) {
//       int deadZone = 16;
//       int xx = pixy.blocks[0].x;
//       
//       if (xx < (160 - deadZone)) {
//          if (turning != 1) {
//             turning = 1;
//             left(100);
//          }
//       }
//       if (xx > (160 + deadZone)) {
//          if (turning != 2) {
//             turning = 2;
//             right(100);
//          }
//       }     
//       if (xx > (159 - deadZone) and xx < (161 + deadZone)) {
//          if (turning != 0) {
//             turning = 0;
//             forward(100);
//          }
//       }   
//       else {
//          if (turning == 0) {
//             halt();
//          }
//       }      
//    }
}

// // this interrupt is for starting and stopping the roomba
void interrupt_0() { 
   // debounce
   if((long)(micros() - last_micros) >= debouncing_time * 1000) {
      // Do Something
      
      // state_0 = !state_0;
      
      // blink the status green status led on microcontroller carrier board
      //blinkLED();
      
      // if button is pressed and action_0 is 0 then start the roomba in full
      // and make action_0 == 1
      // otherwise if action_0 is already 1 then stop the roomba
      if (action_0 == 0) {
         roombaMode(start);
         roombaMode(full);
         action_0 = 1;
      }
      else {
         roombaMode(stop);
         action_0 = 0;
      }
      
      while(digitalRead(buttonPin_0)) {
         delay(10);
      }
      
      // debounce
      last_micros = micros();
   }
}