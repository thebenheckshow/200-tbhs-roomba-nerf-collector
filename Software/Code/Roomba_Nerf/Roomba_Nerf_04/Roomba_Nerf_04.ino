#include <SPI.h>  
#include <Pixy.h>
#include <SoftwareSerial.h>

#define rxPin 10
#define txPin 9

#define start  128
#define reset  7
#define stop  173
#define safe  131
#define full  132
#define clean 135
#define dock 143

#define  bumpRight  1 << 0
#define  bumpLeft   1 << 1
#define  dropRight  1 << 2
#define  dropLeft   1 << 3

char sensorbytes[10];
 
#define bumpright (sensorbytes[0] & 0x01)
#define bumpleft  (sensorbytes[0] & 0x02)

// cleaning motors
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

byte response[16];

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


int cycleNum = 0;

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
  //roombaMode(safe);
  // roombaMode(full);
	roombaMode(stop);  
	roombaMode(start);  
	roombaMode(safe);
  
} 

// program loop
void loop() {


	//testPixy_01();
	//testPixy_02();

	/*
	checkSensor(46, 1);
	
	for (int x = 0 ; x < 1 ; x++) {
	
		Serial.println(response[x], DEC);
	
	}
		
	if (digitalRead(buttonPin_0) == 1) {
	
		Serial.println("STOP ROOMBA");
	
		roombaMode(stop);
	
	}
	
	delay(500);
	*/
	
	updateSensors();
   
   if(bumpleft) {
     Serial.println("Bump Left");
     delay(1000);
   }
   else if(bumpright) {
     Serial.println("Bump Right");
     delay(1000);
   }
		
}

void updateSensors() {
   roombaSerial.write(142);
   roombaSerial.write(1);  // sensor packet 1, 10 bytes
   delay(100); // wait for sensors 
   char i = 0;
   while(Serial.available()) {
     int c = Serial.read();
     if( c==-1 ) {
       for( int i=0; i<5; i ++ ) {   // say we had an error via the LED
       
       }
     }
     sensorbytes[i++] = c;
   }    
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
  //roombaSerial.write(start);  // Start the roomba OI
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

// function to get sensor data
int checkSensorXX(byte whichSensor, byte howManyBytes, byte* response) {
  // This function will return the value of a sensor
  // The first perameter inidcates the sensor code to querry
  // The second paremeter is for keeping track of the number of bytes that the querry will return
  // The third paremeter is a pointer, so it can copy the data into the buffer pointed to by that pointer
  
  int timeOut = 40;
  int bufferPos = 0;
  
  if(!response) return 0;
  
  roombaSerial.write(142); // instruction to prompt roomba for sensor data
  roombaSerial.write((int8_t)whichSensor); // send sensor code to roomba for data query
  
  for(unsigned bufferPos=0; bufferPos < howManyBytes && timeOut; bufferPos++) {  // get the data or timeout
    while (roombaSerial.available() == 0 and timeOut) {  
      timeOut -= 1;   
      delayMicroseconds(1);    
    } 
    response[bufferPos++] = roombaSerial.read();
    howManyBytes -= 1;
  }
  
  if (timeOut) { // timeout if no response
    return 1;
  }
  
}


int checkSensor(unsigned char whichSensor, unsigned char howManyBytes) {
   
   int timeOut = 4000;
   int bufferPos = 0;
   
   roombaSerial.write(142);
   roombaSerial.write(whichSensor);
   
   while(howManyBytes and timeOut) {
      
      while (roombaSerial.available() == 0 and timeOut) {  
         timeOut -= 1;   
         delayMicroseconds(1);    
      } 
      
      response[bufferPos] = roombaSerial.read();
	  
	  bufferPos += 1;
      
      howManyBytes -= 1;
      
   }
   
   if (timeOut) {
      return 1;
   }
   
}

int checkSensors(byte whichSensor, byte howManyBytes){
   
   int timeOut = 40;
   
   roombaSerial.write(149);
   roombaSerial.write((byte)howManyBytes);
   
   for (int x = 0; x < howManyBytes ; x++) {
      
      roombaSerial.write((byte)whichSensor++);
      
   }
   
   int bufferPos = 0;
   
   while(howManyBytes) {
      
      while (roombaSerial.available() == 0) {
         delayMicroseconds(1);     
      }  
      
      response[bufferPos++] = roombaSerial.read();
      
      howManyBytes -= 1;
      
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

void testPixy_01() {
  
	int numBlock = pixy.getBlocks();

	Serial.print(cycleNum++);
	Serial.print(" ");

	if (numBlock) {

		Serial.print(numBlock);
		Serial.print("> ");

		for (int x = 0 ; x < numBlock; x++) {
			Serial.print("\t");	
			Serial.print(pixy.blocks[x].signature);
			Serial.print("\t");
			Serial.print(pixy.blocks[x].x);
			Serial.print("\t\t");
			Serial.println(pixy.blocks[x].y); 	
		}

	} 
  
	Serial.println("");  

}

void testPixy_02() {
  
	int blocks = pixy.getBlocks();

	if (blocks) {

		cycleNum++;

		if (cycleNum == 20) {
			cycleNum = 0;
			for (int x = 0; x < blocks ; x++)
			{
				Serial.print(pixy.blocks[x].signature);
				Serial.print("\t");
				Serial.print(pixy.blocks[x].x);
				Serial.print("\t");
				Serial.println(pixy.blocks[x].y); 	
			}
			Serial.println("-------------");
		}
	}

}




