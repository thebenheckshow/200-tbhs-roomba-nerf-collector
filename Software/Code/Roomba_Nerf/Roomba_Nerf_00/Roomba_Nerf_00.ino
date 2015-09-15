#include <SoftwareSerial.h>
// RX is digital pin 10 
// (connect to TX of other device - iRobot DB25 pin 2)
// TX is digital pin 11 
// (connect to RX of other device - iRobot DB25 pin 1)

#define rxPin 10
#define txPin 9

//#define bumpright (sensorbytes[0] & 0x01)
//#define bumpleft  (sensorbytes[0] & 0x02)

#define startSafe  131
#define startFull  132

#define  bumpRight  1 << 0
#define  bumpLeft   1 << 1
#define  dropRight  1 << 2
#define  dropLeft   1 << 3

// set up a new software serial port:
SoftwareSerial roombaSerial = SoftwareSerial(rxPin, txPin);

byte response[16];

// pin assignments
int statusLED = 4;
// int roombaDD = ;

//Value holders
int action_0=0;
int action_1=0;
volatile int state_0 = LOW;
volatile int state_1 = LOW;
int inByte = 0; // incoming serial byte
unsigned char docked = 0x02; // hex for 0000 0010
int searchingForDock = 0;
char sensorbytes[10];

signed short getAngle = 0;

// debounce
long debouncing_time = 40; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

void setup() { 
   delay(2000); // NEEDED!!!! To let the robot initialize 
   
   // attach interrupt buttons:
   attachInterrupt(0, interrupt_0, RISING); // RISING FALLING CHANGING
   attachInterrupt(1, interrupt_1, RISING);
   
   // define pin modes for status led and direct device/baud:
   //pinMode(roombaDD, OUTPUT); 
   pinMode(statusLED, OUTPUT); 
   
   // set the data rate for the SoftwareSerial port, this is the
   // iRobot's default rate of 115200 baud:
   roombaSerial.begin(115200);
   
   // start hardware serial port 
   Serial.begin(115200);
   
   // define pin modes for software tx, rx pins:
   pinMode(rxPin, INPUT);
   pinMode(txPin, OUTPUT);
   
   safeMode();
} 

void loop() {
  
  
  
  int g = checkSensor(7, 1); // bumps and wheel drops

  if (g & bumpRight) {
    
    Serial.print("\tRB");   
  }
  if (g & bumpLeft) {
    
    Serial.print("\tLB");   
  }
  
  if (g & dropRight) {
    
    Serial.print("\tRD");   
  }
  if (g & dropLeft) {
    
    Serial.print("\tLD");   
  }  
  
  if (g) {
  
	Serial.println(" ");
  }



}

void stopTest() {

	for (int x = 0 ; x < 250 ; x += 10) {
	
		directDrive(x * 5, x * 5);
	
		delay(20);
	
	}
	for (int x = 250 ; x > 0 ; x -= 10) {
	
		directDrive(x * 5, x * 5);
	
		delay(20);
	
	}
	
	directDrive(0,0);

	delay(1000);
	
	for (int x = 0 ; x < 250 ; x += 10) {
	
		directDrive(0 - (x * 5), 0 - (x * 5));
	
		delay(20);
	
	}
	for (int x = 250 ; x > 0 ; x -= 10) {
	
		directDrive(0 - (x * 5), 0 - (x * 5));
	
		delay(20);
	
	} 
 

}



void checkFront() {

  checkSensors(9, 4);
  
  for (int x = 0 ; x < 4 ; x++) {
  
    if (response[x]) {
      Serial.print("X");
    }
    else {
      Serial.print(".");    
    }  
    
  }
  
  Serial.println(" ");
  
}

void safeMode() {
   // light up status led   
   digitalWrite(statusLED, HIGH); 
   
   roombaSerial.write(128); // This command starts the OI. You must
   // always send the Start command before
   // sending any other commands to the OI
   roombaSerial.write(startSafe); // safe mode

   //roombaSerial.write(startFull);   
   digitalWrite(statusLED, HIGH); 
   delay(100);
   digitalWrite(statusLED, LOW);  
   delay(100);
   digitalWrite(statusLED, HIGH); 
   delay(100);
   digitalWrite(statusLED, LOW);  
   delay(100);
   
   // turn off status led
   digitalWrite(statusLED, LOW);
}

void directDrive(signed short leftV, signed short rightV) {

	roombaSerial.write(145); // Opcode number for DIRECT DRIVE
   roombaSerial.write(rightV >> 8); 
   roombaSerial.write(rightV & 0xFF);  
   roombaSerial.write(leftV >> 8); 
   roombaSerial.write(leftV & 0xFF); 
	
}

void halt() {
   roombaSerial.write(145); // Opcode number for DIRECT DRIVE
   roombaSerial.write((byte)0); 
   roombaSerial.write((byte)0);  
   roombaSerial.write((byte)0); 
   roombaSerial.write((byte)0); 
}

void beep(){
   roombaSerial.write(140); // Opcode number for SONG
   roombaSerial.write((byte)3); 
   roombaSerial.write((byte)1);  
   roombaSerial.write((byte)64); 
   roombaSerial.write((byte)16);
   roombaSerial.write((byte)141);
   //delay(5000); 
}

void interrupt_0() {
   // debounce
   if((long)(micros() - last_micros) >= debouncing_time * 1000) {
      // Do Something
      last_micros = micros();
   }
   //    action_0 = 0;
   //    Serial.print(action_0);
   //    Serial.print("\n");
   //    goForward();
   // state_0 = !state_0;
   digitalWrite(statusLED, HIGH); 
   delay(50);
   digitalWrite(statusLED, LOW);  
   delay(50);
   
   action_0 = 0;
   updateSensors();
//    Serial.print(action_0);
//    Serial.print("\nhalt\n");
   
  // halt();
}

void interrupt_1() {
   // debounce
   if((long)(micros() - last_micros) >= debouncing_time * 1000) {
      // Do Something
      last_micros = micros();
   }
   
   // state_1 = !state_1;
   digitalWrite(statusLED, HIGH); 
   delay(50);
   digitalWrite(statusLED, LOW);  
   delay(50);
   
   if (action_0 < 7){
      action_0++;
   }
   else {
      action_0 = 1;
   }
//    Serial.print(action_0);
//    Serial.print("\n");
   
}

void test_0(){
   
   digitalWrite(statusLED, state_1);
   if (action_0 != action_1){ // only print if updated value
      Serial.print(action_0);
      //Serial.print("\n");
      action_1 = action_0;
      switch (action_0) {
         case 0:
            // stop roomba
            //safeMode();
            halt();
            Serial.print("\nhalt\n");
         break;
         case 1:
            // forward
            //forward();
            Serial.print("\nforward\n");
            break;
         case 2:
            // reverse
            //reverse();
            Serial.print("\nreverse\n");
            break;
         case 3:
            // left
            //left();
            Serial.print("\nleft\n");
//             digitalWrite(statusLED, HIGH); 
//             delay(100);
//             digitalWrite(statusLED, LOW);  
//             delay(100);
//             digitalWrite(statusLED, HIGH); 
//             delay(100);
//             digitalWrite(statusLED, LOW);  
//             delay(100);
            break;
         case 4:
            // right
            //right();
            Serial.print("\nright\n");
            break;
         case 5:
            // dock
            seekDock();
            Serial.print("\ndock\n");
            break;
         case 6:
            // beep
            beep();
            Serial.print("\nbeep\n");
            break;
         case 7:
            // bump and turn
            // bumpAndTurn();
            Serial.print("\nbump and turn\n");
            break;
         default: 
            // if nothing else matches, do the default
            halt();
      }
   }
   updateSensors();
}

int checkSensor(byte whichSensor, byte howManyBytes) {

  int timeOut = 40;
  int bufferPos = 0;
  
   roombaSerial.write(142);
   roombaSerial.write((byte)whichSensor);

  while(howManyBytes and timeOut) {
    
	while (roombaSerial.available() == 0 and timeOut) {  
		timeOut -= 1;   
		delayMicroseconds(1);    
	} 

    response[bufferPos++] = roombaSerial.read();
  
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

/*
void bumpTest(){
   updateSensors();
   if(bumpleft) {
      //right();
      delay(1000);
   }
   else if(bumpright) {
      //left();
      delay(1000);
   }
   //forward();   
}

*/

void seekDock(){
   
   roombaSerial.write(142); // requests the OI to send a packet of 
   // sensor data bytes
   roombaSerial.write((byte)34); // request cliff sensor value specifically
   delay(250); // poll sensor 4 times a second
   if (roombaSerial.available() > 0) {
      inByte = roombaSerial.read();
   } 
   Serial.println(inByte, BIN);
   if ((inByte & docked) != docked) {
      if (searchingForDock == 0) {
         searchingForDock = 1;
         roombaSerial.write(143); // Opcode number for SEEK DOCK
         Serial.println("Roomba is looking for dock.");
         //delay(4000);
      }
      else {
         if (searchingForDock == 1) {
            Serial.println("Roomba is still searching for the dock.");
            //delay(2000);
         }
         else {
            roombaSerial.write(143); // Opcode number for SEEK DOCK
         }  
      } 
   }
   else {
      searchingForDock = 0;
      Serial.println("Roomba is docked.");
      //delay(2000);
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
            
            delay(50);
            
            delay(50);
         }
      }
      sensorbytes[i++] = c;
      Serial.print(c);
   }
//    for(int i=0; i<9; i++){
//       Serial.print(
//   }
}
