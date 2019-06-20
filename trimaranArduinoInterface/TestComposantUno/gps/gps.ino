
#include <SoftwareSerial.h>



SoftwareSerial SoftSerial(10, 11);
unsigned char buffer[200];
int count=0;   

void setup() {
  SoftSerial.begin(9600);
  Serial.begin(115200);
  
}

void loop() {
  getGPS();
  
}

void getGPS() {
  
  if (SoftSerial.available())                    
    {
        //char currentchar = '.';
        
        /*while(SoftSerial.available())
        {
          
          char currentchar = SoftSerial.read();
          
          if(currentchar == '$') 
           {
            
            buffer[count++]='$';
            
            break;
            }
          }
        */

        char currentchar = SoftSerial.read();
        if (currentchar ==  '$'){
          buffer[count++]='$';
          currentchar = '.';
          while( currentchar != '*')       
          {
             if (SoftSerial.available())
             {
              currentchar=SoftSerial.read();
              buffer[count++]=currentchar;
              if(count == 200)break;
             }
          }
          delay(10);
          if(isGPSGPGGA(buffer) == 1) 
          {
            Serial.write(buffer, count);
            Serial.println("");
          }
        }
        clearBufferArray();                      
        count = 0;      
                                
    }
    else{
      //Serial.println("NoSignal");
    }
}

int isGPSGPGGA(unsigned char* trameGPS) {
  if(trameGPS[0] == '$' && trameGPS[1] == 'G' && trameGPS[2] == 'P' && trameGPS[3] == 'G' && trameGPS[4] == 'G' && trameGPS[5] == 'A')
    return 1;
  else
    return 0;
  }

void clearBufferArray()                     
{
    for (int i=0; i<count;i++)
    { buffer[i]=NULL;}                      
}
