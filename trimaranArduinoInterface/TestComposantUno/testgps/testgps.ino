#include "TinyGPS++.h"

String buf;
int count = 0;
unsigned long t0;

TinyGPSPlus gps;

void setup() {
  Serial2.begin(9600);
  Serial.begin(57600);
  Serial2.setTimeout(10);
  /*
  Serial2.print("$PMTK000*32\r\n");
  while(Serial2.available()<0);
  Serial.println(Serial2.readString());
  */
  
}

void loop() {
  t0 = millis();
  
  if (Serial2.available()){
    //buf = Serial2.readString();
    buf = Serial2.readStringUntil('\n');
    
    if (isTrame(buf)){
      if (isGPSGPGGA(buf) == 1 || isGPSGPRMC(buf) == 1){
        Serial.println(buf);
        for(int i = 0; i<buf.length(); i++){
          gps.encode(buf[i]);
        }
        Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
        Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");  Serial.println(gps.altitude.meters());

        

      }
      
    }
    
    
  
  }
  //Serial.print("temps : ");
  //Serial.println(millis()-t0);
  
}

int isTrame(String trameGPS){
  int count = 1;
  int res = 0;
  char currentchar = trameGPS[count];
  int len = trameGPS.length();
  while (currentchar != '*' && count<len)
  {
    if (currentchar != ','){
      res = res ^ currentchar;
    }
    count++;
    currentchar = trameGPS[count];
  }
  char check[2];
  check[0] = trameGPS[count+1];
  check[1] = trameGPS[count+2];
  int test  = strtol(check,NULL,16);
  /*Serial.print(" res : ");
  Serial.print(res,HEX);
  Serial.print("Check : ");
  Serial.println(test,HEX);
  */
  if (test == res){
    return 1;
  }
  else
  {
    return 0;
  }
}

int isGPSGPRMC( String trameGPS){
  if(trameGPS[0] == '$' && trameGPS[1] == 'G' && trameGPS[2] == 'P' && trameGPS[3] == 'R' && trameGPS[4] == 'M' && trameGPS[5] == 'C')
    {
    return 1;
    }
  else
    return 0;
}


int isGPSGPGGA( String trameGPS) {
  if(trameGPS[0] == '$' && trameGPS[1] == 'G' && trameGPS[2] == 'P' && trameGPS[3] == 'G' && trameGPS[4] == 'G' && trameGPS[5] == 'A')
    {
    return 1;
    }
  else
    return 0;
  }
