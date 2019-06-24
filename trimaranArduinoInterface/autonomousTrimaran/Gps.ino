
void setupGps(){
   nh.advertise(pubGps);

}


void updateGps(){
    getGPS();
    publishGps();
}

void publishGps(){
  //char buf[10] = "hello";
  //gpsMsg.data = buf;
  pubGps.publish(&gpsMsg);
  Serial.println(gpsMsg.data);
}

void getGPS() {
  clearBufferArray(); 
  count = 0;  
  if (SoftSerial.available())                    
    {
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
       
        strcpy(position, buffer);
        gpsMsg.data = position;
      //Serial.write(buffer, count);
      //Serial.println("");
      }          
    }
  }                   
  else{
    //Serial.println("nosignal");
  }
}

int isGPSGPGGA( char* trameGPS) {
  if(trameGPS[0] == '$' && trameGPS[1] == 'G' && trameGPS[2] == 'P' && trameGPS[3] == 'G' && trameGPS[4] == 'G' && trameGPS[5] == 'A')
    {
    return 1;
    }
  else
    return 0;
  }

void clearBufferArray()                     
{
    for (int i=0; i<count;i++)
    { buffer[i]=NULL;}                      
}
