
#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

void setup()
{
  Serial.begin(9600);

  pixy.init();
}

void loop()
{ 

  if (pixy.getBlocks())
  {        
    if (pixy.blocks[0].signature == 1); { 
      Serial.print(pixy.blocks[0].x);
      Serial.print("\t");
      Serial.println(pixy.blocks[0].y);    
    }      
  }  
}
