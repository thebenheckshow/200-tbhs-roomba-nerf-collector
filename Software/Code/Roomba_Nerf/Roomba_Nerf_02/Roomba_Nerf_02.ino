// TBHS 2015 Episode 200 - 202 - 205 Roomba Nerf
//
// 2015.08.03
//    This code is intended for an arduino connected to a pixy and a roomba
//    it will search for nerf darts turn towards them and drive over them 
//    addded code for connecting to roomba and getting sensor data
//    added a function to return to charge base
//
// 2015.08.03
//    added pixy code
//
// 2015.08.20
//    added brush and vacuum motor controls
//    more code needed for collecting sensor data
//    better code needed for debounce
//    code for motor current sense not complete
//    I was going to put the functions into different header files, but for some reason the compiler can not find them.

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <SPI.h>  
#include <Pixy.h>
//#include "tests.h"

#define rxPin 10
#define txPin 9

#define start  128
#define reset  7
#define stop  173
#define safe  131
#define full  132

#define  bumpRight  1 << 0
#define  bumpLeft   1 << 1
#define  dropRight  1 << 2
#define  dropLeft   1 << 3

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

// byte response[16];

// setup pixy
Pixy pixy;

// set up a new software serial port
SoftwareSerial roombaSerial = SoftwareSerial(rxPin, txPin); 

// pin assignments
int statusLED = 4;

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

// program setup
void setup() { 
   
   // attach interrupt buttons:
   // attachInterrupt(0, interrupt_0, RISING); // RISING FALLING CHANGING
   // attachInterrupt(1, interrupt_1, RISING);
   
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
   
   // initalize the pixy
   pixy.init();
   Serial.println("PIXY GO START!");
   
   pinMode(buttonPin_0, INPUT);
   pinMode(buttonPin_1, INPUT);
   pinMode(buttonPin_2, INPUT);
   pinMode(buttonPin_3, INPUT);
   
   // begin roomba mode
   // roombaMode(safe);
   // roombaMode(full);
} 

// program loop
void loop() {
   // testPixy_01();
   // testPixy_02();
   // testMotors_00();
   // testMotors_01();
   // testDartCollector();
   // testBrushMotorCurrent_00();
   testVisionCollector();
   
}

// function to drive motors with opcode direct drive 145
void directDrive(signed short leftV, signed short rightV) {
   // the function takes 2 parameters velocity of left motor and velocity of right motor range -500 to 500 mm/s
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

// // this interrupt is for starting and stopping the roomba
// void interrupt_0() { 
//    // debounce
//    if((long)(micros() - last_micros) >= debouncing_time * 1000) {
//       // Do Something
//       
//       // state_0 = !state_0;
//       
//       // blink the status green status led on microcontroller carrier board
//       blinkLED();
//       
//       // if button is pressed and action_0 is 0 then start the roomba in full
//       // and make action_0 == 1
//       // otherwise if action_0 is already 1 then stop the roomba
//       if (action_0 == 0) {
//          roombaMode(start);
//          roombaMode(full);
//          action_0 = 1;
//       }
//       else {
//          roombaMode(stop);
//          action_0 = 0;
//       }
//       
//       // debounce
//       last_micros = micros();
//    }
// }
// 
// // this interrupt is for changing the pwm speed of mainBrush
// void interrupt_1() {
//    // debounce
//    if((long)(micros() - last_micros) >= debouncing_time * 1000) {
//       // Do Something
//       
//       // state_1 = !state_1;
//       
//       // blink the status green status led on microcontroller carrier board
//       blinkLED();
//       
//       // the brus will have three speeds
//       // the speeds are given by global variables: lowSpeed, midSpeed, highSpeed
//       // with each button press action_1 starting from 0 is incrimented to 3
//       // at 3 action_1 is reset to 0
//       // a switch case checks what action_1 is and sets the primary motor speed
//       if (action_1 < 3){
//          action_1++;
//       }
//       else {
//          action_1 = 0;
//       }
//       
//       switch (action_1) {
//          case 0:
//             cleaningMotors(primary, 0);   
//             break;
//          case 1:
//             cleaningMotors(primary, lowSpeed);  
//             break;
//          case 2:
//             cleaningMotors(primary, midSpeed);  
//             break;
//          case 3:
//             cleaningMotors(primary, highSpeed); 
//             break;
//          default: 
//             break;
//       }
//       
//       last_micros = micros();
//    }
// }

// int checkSensors(byte whichSensor, byte howManyBytes){
//    
//    int timeOut = 40;
//    
//    roombaSerial.write(149);
//    roombaSerial.write((byte)howManyBytes);
//    
//    for (int x = 0; x < howManyBytes ; x++) {
//       
//       roombaSerial.write((byte)whichSensor++);
//       
//    }
//    
//    int bufferPos = 0;
//    
//    while(howManyBytes) {
//       
//       while (roombaSerial.available() == 0) {
//          delayMicroseconds(1);     
//       }  
//       
//       response[bufferPos++] = roombaSerial.read();
//       
//       howManyBytes -= 1;
//       
//    }
//    
// }

// byte response[16];

// int checkSensor(byte whichSensor, byte howManyBytes) {
//    
//    int timeOut = 40;
//    int bufferPos = 0;
//    
//    roombaSerial.write(142);
//    roombaSerial.write((byte)whichSensor);
//    
//    while(howManyBytes and timeOut) {
//       
//       while (roombaSerial.available() == 0 and timeOut) {  
//          timeOut -= 1;   
//          delayMicroseconds(1);    
//       } 
//       
//       response[bufferPos++] = roombaSerial.read();
//       
//       howManyBytes -= 1;
//       
//    }
//    
//    if (timeOut) {
//       return 1;
//    }
//    
// }

// int checkSensors(byte whichSensor, byte howManyBytes){
//    
//    int timeOut = 40;
//    
//    roombaSerial.write(149);
//    roombaSerial.write((byte)howManyBytes);
//    
//    for (int x = 0; x < howManyBytes ; x++) {
//       
//       roombaSerial.write((byte)whichSensor++);
//       
//    }
//    
//    int bufferPos = 0;
//    
//    while(howManyBytes) {
//       
//       while (roombaSerial.available() == 0) {
//          delayMicroseconds(1);     
//       }  
//       
//       response[bufferPos++] = roombaSerial.read();
//       
//       howManyBytes -= 1;
//       
//    }
//    
// }

// void testBrushMotorCurrent_00(){
//    
// //    int current = word(byte[0], byte[1])
// //    short int16 = (short)(((bytes[0] & 0xFF) << 8) | (bytes[1] & 0xFF)); 
//    
//    cleaningMotors(primary, 30);
//    checkSensors(56, 16);
// /*   for (i=0; i=16; i++){
//       Serial.println(response[i]);
//    } */
//    //int16_t s = (int16_t) (((response[0] & 0xFF) << 8) | (response[1] & 0xFF));
//    brushCurrent = (((uint16_t)response[0]) << 8) | response[1];
//    
//    Serial.print("Current: ");
//    Serial.println(brushCurrent);
//    
// }


// function to get sensor data
int checkSensor(byte whichSensor, byte howManyBytes, byte* response) {
   // This function will return the value of a sensor
   // The first perameter inidcates the sensor code to querry
   // The second paremeter is for keeping track of the number of bytes that the querry will return
   // The third paremeter is a pointer, so it can copy the data into the buffer pointed to by that pointer
   
   int timeOut = 40;
   int bufferPos = 0;
   
   if(!response) return 0;
   
   roombaSerial.write(142); // instruction to prompt roomba for sensor data
   roombaSerial.write((int8_t)whichSensor); // send sensor code to roomba for data querry
   
   for(unsigned bufferPos=0; bufferPos < howManyBytes && timeOut; bufferPos++) {  // get the data or timeout
      while (roombaSerial.available() == 0 and timeOut) {  
         timeOut -= 1;   
         delayMicroseconds(1);    
      } 
      response[bufferPos] = roombaSerial.read();
      bufferPos += 1;
      howManyBytes -= 1;
   }
   
   if (timeOut) { // timeout if no response
      return 1;
   }
   
}

// this test is to give the roomba instruction for moving forward stopping and reversing
// also for this test the interrupts are set to start or stop the roomba and set the speed of main brush
void testDartCollector() {
   
   // debounce routine
   // if((long)(micros() - last_micros) >= debouncing_time * 1000) {
   //    // Do Something
   //    last_micros = micros();
   // }
   
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
         halt();
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
   // if button is high and not moving forward move forward
   // if alreading moving forward stop
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
   
}

// test to get motor current
void testBrushMotorCurrent_00(){
   
   byte response[16];
   int incomingByte = 0;   // for incoming serial data
   
   int timeOut = 40;
   int bufferPos = 0;
   byte howManyBytes = 2;
   
   cleaningMotors(primary, 30); // spin the primary motor
   
   roombaSerial.write(142); // ask for sensor data
   roombaSerial.write((int8_t)56); // ask for primary motor current, return 2 signed data bytes high byte first, -32768 to 32767 mA
   
   // I think I need to wait until I am sure that data is avliable
   while (roombaSerial.available() < 2) { 
      // delayMicroseconds(1); 
   }
   //    if (roombaSerial.available() > 0) {
   //       // read the incoming byte:
   //       incomingByte = roombaSerial.read();
   //    }
   
   response[0] = roombaSerial.read();
   response[1] = roombaSerial.read();
   
   Serial.print("Brush Motor Current: \t");
   Serial.print((signed int)response[0]);
   Serial.print(" ");
   Serial.print((signed int)response[1]);
   Serial.print("\t   - - - check two bytes\n");
   
   delay(2000);
   
   
   // code I tried that did not work
   //    
   //    checkSensor(56, 16, response);
   //    for (unsigned i=0; i=16; i++){
   //       //Serial.print(response[i]);
   //       Serial.print("Now What?");
   //    }
   //    Serial.print("\n");
   //    Serial.print("Now What?");
   
   //    for(unsigned bufferPos=0; bufferPos < howManyBytes; bufferPos++) {  // get the data
   //       delayMicroseconds(1);
   //       response[bufferPos] = roombaSerial.read();
   //       howManyBytes -= 1;
   //    }
   //send data only when you receive data:
   //    if (roombaSerial.available() > 0) {
   //       // read the incoming byte:
   //       incomingByte = roombaSerial.read();
   //    }
   //Serial.print(roombaSerial.read());   
   //Serial.print(incomingByte);
   
   
}

// simple brush and vaccum motor test
void testMotors_00() {
   
   // primary on forward and reverse then stop
   cleaningMotors(primary, 31);
   delay(2000);
   cleaningMotors(primary, 0);
   delay(1000);
   cleaningMotors(primary, -31);
   delay(2000);
   cleaningMotors(primary, 0);
   
   // secondary on forward and reverse then stop
   cleaningMotors(secondary, 31);
   delay(2000);
   cleaningMotors(secondary, 0);
   delay(1000);
   cleaningMotors(secondary, -31);
   delay(1000);
   cleaningMotors(secondary, 0);
   
   // dump on and off
   cleaningMotors(dump, 31);
   delay(2000);
   cleaningMotors(dump, 0);
}

// simple pixy test to turn roomba towards signature if the signature x position is left or right of camera
void testPixy_02() {
   
   if (pixy.getBlocks()) {
      
      int xx = pixy.blocks[0].x;
      
      if (xx < (160 - deadZone)) {
         if (turning != 1) {
            turning = 1;
            Serial.print("\tLeft");
            Serial.print("\t");
            Serial.print(turning);
            Serial.print("\n");
            left(100);
         }
      }
      if (xx > (160 + deadZone)) {
         if (turning != 2) {
            turning = 2;
            Serial.print("\tRight");
            Serial.print("\t");
            Serial.print(turning);
            Serial.print("\n");
            right(100);
         }
      }     
      if (xx > (159 - deadZone) and xx < (161 + deadZone)) {
         if (turning != 0) {
            turning = 0;
            Serial.print("\tHalt");
            Serial.print("\t");
            Serial.print(turning);
            Serial.print("\n");
            halt();
         }
      }   
      else {
         if (turning == 0) {
            Serial.print("\t--H");
            Serial.print("\t");
            Serial.print(turning);
            Serial.print("\n");
         }
      }      
   }
   
}

// simple pixy test to ouptut to serial if the signature x position is left or right of camera
void testPixy_01() {
   
   if (pixy.getBlocks()) {
      
      if (pixy.blocks[0].signature == 1) {
         
         int xx = pixy.blocks[0].x;
         
         if (xx < (160 - deadZone)) {
            Serial.print("\tLeft\n"); 
         }
         if (xx > (160 + deadZone)) {
            Serial.print("\tRight\n");
         }     
         if (xx > (159 - deadZone) and xx < (161 + deadZone)) {
            Serial.print("\tHalt\n");  
         }      
         
      }
      else {
         Serial.print("\tHalt\n");  
      }    
   } 
}

// simple pixy test to ouptut serial x and y data of signature
void testPixy_00() {
   
   if (pixy.getBlocks())
   {        
      if (pixy.blocks[0].signature == 1); { 
         Serial.print("\t");
         Serial.print(pixy.blocks[0].x);
         Serial.print("\t\t");
         Serial.println(pixy.blocks[0].y);    
      }      
   }  
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
   
   if (pixy.getBlocks()) {
      int deadZone = 16;
      int xx = pixy.blocks[0].x;
      
      if (xx < (160 - deadZone)) {
         if (turning != 1) {
            turning = 1;
            left(100);
         }
      }
      if (xx > (160 + deadZone)) {
         if (turning != 2) {
            turning = 2;
            right(100);
         }
      }     
      if (xx > (159 - deadZone) and xx < (161 + deadZone)) {
         if (turning != 0) {
            turning = 0;
            forward(100);
         }
      }   
      else {
         if (turning == 0) {
            halt();
         }
      }      
   }
}