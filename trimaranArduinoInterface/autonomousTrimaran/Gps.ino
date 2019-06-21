
void setupGps(){
   nh.advertise(pubGps);

}


void updateGps(){
  if (wait == wait_max){
    getGPS();
    publishGps();
    wait = 0;
  }
  else{
    wait = wait +1;
  }
  
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
      while(SoftSerial.available())               // reading data into char array
      {
          buffer[count++]=SoftSerial.read();      // writing data into array
          if(count == 64)break;
      }
          
      if(isGPSGPGGA(buffer) == 1) 
      {
       
        strcpy(position, buffer);
        gpsMsg.data = position;
      //Serial.write(buffer, count);
      //Serial.println("");
      }          
    }                   
    else{
      //Serial.println("nosignal");
    }
}

int isGPSGPGGA( char* trameGPS) {
  if(trameGPS[0] == '$' && trameGPS[1] == 'G' && trameGPS[2] == 'P' && trameGPS[3] == 'G' && trameGPS[4] == 'G' && trameGPS[5] == 'A')
    {
    Serial.print("ok  ");
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
