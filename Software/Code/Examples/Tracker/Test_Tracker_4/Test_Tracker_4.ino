
#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;
int deadZone = 10;

int deadZoneY = 8;

#define xStep    2
#define xDir     3
#define xEnable  8
#define yStep    4
#define yDir     5
#define yEnable  9
#define goSwitch A5

#define right    1
#define left     0
#define down     1
#define up       0

void setup()
{
  Serial.begin(9600);
  Serial.println("PIXY GO START!");
  pixy.init();
  
  pinMode(xStep, 1);
  pinMode(xDir, 1);
  pinMode(xEnable, 1);
  pinMode(yStep, 1);
  pinMode(yDir, 1);
  pinMode(yEnable, 1);
  
  pinMode(goSwitch, 0);      //The "action" button
  digitalWrite(goSwitch, 1); //Internal pullup
  
}

void loop()
{ 
 
  digitalWrite(yEnable, 0);          //Y always enabled to hold camera

  checkPixy();
 
}

void checkPixy() {

  if (pixy.getBlocks()) {
        
    if (pixy.blocks[0].signature == 1) {
      
      int xx = pixy.blocks[0].x;
      int yy = pixy.blocks[0].y;

      if (xx > (159 - deadZone) and xx < (161 + deadZone)) {
        digitalWrite(xEnable, 1);  
      }
      else {
        digitalWrite(xEnable, 0);   
      }       

      if (xx < (160 - deadZone)) {
        xStepper(left, 50);   
      }
      if (xx > (160 + deadZone)) {
        xStepper(right, 50);  
      }     

      if (yy < (125 - deadZoneY)) {
        yStepper(up, 2);   
      }
      if (yy > (125 + deadZoneY)) {
        yStepper(down, 2);  
      }         
           
    } 
 
  }
  else {
    digitalWrite(xEnable, 1);        //Turn off X when not in use    
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

void xStepper(unsigned char whichWay, unsigned short howLong) {

  digitalWrite(xDir, whichWay);
  
  for (int x = 0 ; x < howLong ; x++) {
    digitalWrite(xStep, 1);
    delayMicroseconds(800);
    digitalWrite(xStep, 0);        
  }
 
}

void yStepper(unsigned char whichWay, unsigned short howLong) {

  digitalWrite(yDir, whichWay);
  
  for (int x = 0 ; x < howLong ; x++) {
    digitalWrite(yStep, 1);
    delay(1);
    digitalWrite(yStep, 0);        
  }
 
}





void stepperTest() {
  
 
  Serial.println("Forward");
  for (int x = 0 ; x < 1000 ; x++) {    
    xStepper(right, 1);  
  }
  
  delay(1000);


  Serial.println("Back");  
  for (int x = 0 ; x < 1000 ; x++) {    
    xStepper(left, 1);  
  }

  delay(1000);
 
  Serial.println("Forward");
  for (int x = 0 ; x < 100 ; x++) {    
    yStepper(right, 1);  
  }
  
  delay(1000);


  Serial.println("Back");  
  for (int x = 0 ; x < 100 ; x++) {    
    yStepper(left, 1);  
  }

  delay(1000); 
   
  
}




