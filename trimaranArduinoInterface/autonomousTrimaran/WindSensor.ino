

void setupWind(){
  pinMode(PIN_WIND,INPUT);
  pinMode(pinAnemo,INPUT);
  nh.advertise(pubWind);
  nh.advertise(pubWindSpeed);
  t0 = millis();
  validWind = 0;

  attachInterrupt(digitalPinToInterrupt(pinAnemo), anemoInterrupt,FALLING);
}


void updateWind(){
  sensorValue = analogRead(A1);
  angleWind = ((sensorValue-ref-MIN_WIND)/(MAX_WIND-MIN_WIND))*2*PI;
  angleWind = 2*atan(tan(angleWind/2));
  
  publishWind();
}

void publishWind(){
  windMsg.data = angleWind;
  pubWind.publish(&windMsg);
}

/*******anemo**********/

void publishWindSpeed(){
  windSpeedMsg.data = windSpeed;
  pubWindSpeed.publish(&windSpeedMsg);
}

void anemoInterrupt(){
  if (validWind == 1){
    counterAnemo++;
    //Serial.println(counterAnemo);
    if (counterAnemo > 9) {
      counterAnemo = 0;
      t1 = millis();
      windSpeed =  10*(2.25/(t1-t0)) * 1608.34/3600; 
      //https://www.meteo-shopping.fr/Station-meteo/Anemometre-girouette-Vantage-Pro-6410-Davis-Instruments
      //Serial.print(" vitesse m/s : ");
      //Serial.println(windSpeed);
      t0 = t1;
      publishWindSpeed();
      }
  }
  else{
    validWind =1;
    t0= millis();
  }
}
